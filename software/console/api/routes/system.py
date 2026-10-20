"""System routes: health, link mode, state, field, pose, board log, display, I2C trace.

File:    software/console/api/routes/system.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import time
from typing import Dict

from fastapi import APIRouter, HTTPException
from fastapi.responses import PlainTextResponse

from software import __build__, __version__
from software.core.mission.teams import check_team
from software.robot import protocol
from ...runtime.state import state
from .deps import call, client, require_no_execution, require_no_faults
from .models import DisplayModeIn, DisplayTextIn, FieldIn, ModeIn, PoseIn, TeamIn

router = APIRouter()


@router.get("/health")
async def health() -> Dict:
    """Service, link and board-firmware health (SW and FW versions)."""
    board = state.board_status
    return {
        "ok": True,
        "transport": state.transport,
        "connected": state.connected,
        "error": state.last_error,
        "version": __version__,
        "build": __build__,
        "firmware": (
            "unknown" if board is None else f"{board.fw_major}.{board.fw_minor}"
        ),
    }


@router.post("/mode")
async def set_mode(body: ModeIn) -> Dict:
    """Force the robot link mode: 'i2c' (real board) or 'mock' (simulator)."""
    mode = body.transport.strip().lower()
    if mode not in ("i2c", "mock"):
        raise HTTPException(status_code=422, detail="transport must be 'i2c' or 'mock'")
    require_no_execution()
    try:
        await state.switch_transport(mode)
    except Exception as exc:  # bus busy / smbus2 missing / board absent
        raise HTTPException(status_code=503, detail=f"could not switch to '{mode}': {exc}") from exc
    await state.log(
        "info",
        "robot link switched to i2c (real board)" if mode == "i2c" else "robot link switched to mock (simulator)",
    )
    return {"ok": True, "transport": mode}


@router.get("/state")
async def get_state() -> Dict:
    """Telemetry snapshot plus vision status (the console poll)."""
    return {**state.snapshot(), "vision": state.vision.status()}


@router.get("/field")
async def get_field() -> Dict:
    """Current field size and the planning defaults."""
    return state.field


@router.post("/field")
async def set_field(body: FieldIn) -> Dict:
    """Update the field size (minimum 500 x 500 mm)."""
    if body.width_mm < 500 or body.height_mm < 500:
        raise HTTPException(status_code=422, detail="field must be at least 500 x 500 mm")
    state.field["width_mm"] = body.width_mm
    state.field["height_mm"] = body.height_mm
    # the mirrored default start of a team follows the table's centre line
    state.resolve_team_starts()
    return {"ok": True, "field": state.field}


# ------------------------------------------------------------------ team --
@router.get("/team")
async def get_team() -> Dict:
    """Which side we play (Yellow/Blue) + both teams' match-start poses.

    A Eurobot edition names its two sides differently - the labels come from
    configuration (``TEAM_LABEL_YELLOW``/``TEAM_LABEL_BLUE``), the code only
    knows the two sides ``yellow``/``blue``. Our start pose and the
    opponent's follow from the side; strategies authored for the other side
    are mirrored automatically when loaded.
    """
    return state.team_info()


@router.post("/team")
async def set_team(body: TeamIn) -> Dict:
    """Switch sides; our start pose (and the opponent's) follow the team.

    In MOCK the simulator is placed at the new start right away, so a
    rehearsal of the other side begins from the right corner. On the REAL
    robot the console never moves the robot: it tells you where the side
    starts and you place the robot there (Set Pose).
    """
    require_no_execution()
    try:
        team = check_team(body.team)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    if team == state.team:
        return state.team_info()
    state.team = team
    state.redo_restore_pose = None  # the start pose changed: drop a pending redo
    info = state.team_info()
    start = info["start"]
    if state.transport == "mock":
        link = client()
        await call(lambda: link.set_pose(start[0], start[1], start[2]))
        await state.log(
            "info",
            f"team switched to {info['label']} - mock robot placed at its match start "
            f"({start[0]:.0f}, {start[1]:.0f}) mm, {start[2]:.0f} deg"
        )
    else:
        await state.log(
            "info",
            f"team switched to {info['label']} - place the robot at "
            f"({start[0]:.0f}, {start[1]:.0f}) mm, {start[2]:.0f} deg and press Set Pose "
            f"(opponent starts at ({info['opponent']['start'][0]:.0f}, {info['opponent']['start'][1]:.0f}) mm)"
        )
    return info


@router.post("/pose")
async def set_pose(body: PoseIn) -> Dict:
    """SET_POSE: adopt the operator-placed robot position."""
    link = client()
    require_no_execution()
    await require_no_faults()
    await call(lambda: link.set_pose(body.x_mm, body.y_mm, body.theta_deg))
    state.vision_waiting = None  # an operator placed the robot: the wait is over
    state.redo_restore_pose = None  # a manual placement supersedes a pending redo
    await state.log("info", f"pose set to ({body.x_mm:.0f}, {body.y_mm:.0f}) mm, {body.theta_deg:.0f} deg")
    return {"ok": True}


@router.post("/odometry/reset")
async def reset_odometry() -> Dict:
    """Zero the board odometry."""
    link = client()
    require_no_execution()
    await require_no_faults()
    await call(lambda: link.reset_odometry())
    state.vision_waiting = None  # odometry was zeroed: a previous wait is void
    state.redo_restore_pose = None  # ...and so is a pending redo
    return {"ok": True}


@router.get("/errors")
async def board_errors(after: int = 0) -> Dict:
    """Board error/event log: decoded events with index > after (up to 5)."""
    link = client()
    result = await call(lambda: link.events(after))
    events = [
        {**event, "name": protocol.event_name(event["code"]), "severity": protocol.event_severity(event["code"])}
        for event in result["events"]
    ]
    return {"latest": result["latest"], "faults": result["faults"], "events": events}


@router.post("/display/text")
async def display_text(body: DisplayTextIn) -> Dict:
    """Write one line of text on the OLED (manual mode)."""
    link = client()
    await call(lambda: link.display_text(body.line, body.text, body.offset))
    return {"ok": True}


@router.post("/display/mode")
async def display_mode(body: DisplayModeIn) -> Dict:
    """Select the OLED screen mode (0 off, 1 auto, 2 manual)."""
    link = client()
    if body.mode not in (0, 1, 2):
        raise HTTPException(status_code=422, detail="mode must be 0 (off), 1 (auto) or 2 (manual)")
    await call(lambda: link.display_mode(body.mode))
    return {"ok": True}


# ------------------------------------------------------------- I2C trace ---
@router.get("/trace")
async def get_trace(after: int = 0, limit: int = 500) -> Dict:
    """I2C monitor entries, incremental via ``after``."""
    return {
        "entries": state.trace.recent(after, limit),
        "last_id": state.trace.last_id,
        "count": state.trace.count,
    }


@router.get("/trace/download")
async def download_trace() -> PlainTextResponse:
    """Download the whole I2C trace as a timestamped text file."""
    filename = time.strftime("charara_i2c_%Y%m%d_%H%M%S.log")
    return PlainTextResponse(
        state.trace.to_text(),
        headers={"Content-Disposition": f'attachment; filename="{filename}"'},
    )


@router.post("/trace/clear")
async def clear_trace() -> Dict:
    """Empty the I2C trace buffer."""
    state.trace.clear()
    await state.log("info", "I2C trace cleared")
    return {"ok": True}
