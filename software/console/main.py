"""FastAPI application factory (REST + WebSocket) for the RPI companion.

By default only the API/WebSocket are served (nginx serves the UI in the
docker setup). When ``UI_DIR`` points at a directory (or ``../ui/static``
exists next to this repository, which it does in a normal checkout) the
static console is mounted as well - handy for running without docker:

    ROBOT_TRANSPORT=mock uvicorn software.console.main:app --host 0.0.0.0 --port 8090

File:    software/console/main.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import asyncio
import os
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from software import __version__
from software.config import settings
from software.core.mission import watcher

from .api.routes import router
from .api.ws import hub, vision_endpoint
from .runtime.state import state


@asynccontextmanager
async def lifespan(_: FastAPI):
    """Lifespan: build the robot link, start the watcher loops, close cleanly."""
    state.event_sink = hub.broadcast
    # build the link chosen by ROBOT_TRANSPORT (i2c = real board, mock = sim);
    # the console can switch it at runtime through POST /api/mode. An absent
    # board must never stop the console: the session logs once and retries in
    # the background (routes answer 503 until the link is up).
    try:
        await state.switch_transport(settings.transport)
    except Exception:
        await state.session.start()
    # expire camera obstacle reports whose TTL ran out (also mid-run)
    vision_sweep = asyncio.create_task(watcher.sweep_loop(state))
    # re-check while driving: an obstacle too far to matter becomes close as
    # the robot advances (a parked robot sends no new report)
    vision_proximity = asyncio.create_task(watcher.proximity_loop(state))
    await state.log("info", f"RPI service started (transport={settings.transport})")
    try:
        yield
    finally:
        vision_sweep.cancel()
        vision_proximity.cancel()
        # stop the sensor tasks, telemetry and close the link in one place
        await state.session.stop()


app = FastAPI(title="Charara RPI companion", version=__version__, lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)
app.include_router(router, prefix="/api")
app.add_api_websocket_route("/ws", hub.endpoint)
app.add_api_websocket_route("/ws/vision", vision_endpoint)

# Optional: serve the web console directly (docker-less mode / local dev).
_ui_candidates = [
    Path(os.environ["UI_DIR"]) if os.environ.get("UI_DIR") else None,
    Path(__file__).resolve().parents[2] / "ui" / "static",
]
for _candidate in _ui_candidates:
    if _candidate is not None and _candidate.is_dir():
        app.mount("/", StaticFiles(directory=str(_candidate), html=True), name="ui")
        break
