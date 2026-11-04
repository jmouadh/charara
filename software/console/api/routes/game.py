"""Game-clock routes: status, start and reset of the match timer.

The clock CONFIGURATION (duration, red-warning window, timed actions) lives
in the persisted game/map document, so it is saved with ``PUT /api/map``
(``game`` field) and survives restarts. These routes only drive the runtime
clock:

* ``GET /api/game``          - status (also embedded in ``GET /api/state``),
* ``POST /api/game/start``   - start the configured clock now (idempotent),
* ``POST /api/game/reset``   - rewind and clear the fired-action history.

``Execute`` starts the clock automatically when the mission begins. In REAL
mode the service stops the run at the limit; MOCK keeps running and shows the
overtime warning on the header timer (Docs/11-game-time.md).

File:    software/console/api/routes/game.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 1.0.0
Date:    2026-11-03
"""

from __future__ import annotations

from typing import Dict

from fastapi import APIRouter

from ...runtime.state import state

router = APIRouter()


@router.get("/game")
async def get_game() -> Dict:
    """Current match clock: config, elapsed/remaining and fired actions."""
    return state.game_status()


@router.post("/game/start")
async def start_game() -> Dict:
    """Start the match clock (no-op while it is already running)."""
    return await state.game_start()


@router.post("/game/reset")
async def reset_game() -> Dict:
    """Reset the match clock and allow a new run after a REAL timeout."""
    return await state.game_reset()
