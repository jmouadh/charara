"""Settings routes: speed limits, watchdog, board settings register map.

File:    software/console/api/routes/settings.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

from typing import Dict, Optional

from fastapi import APIRouter, HTTPException

from software.robot import robot_config
from software.robot.client import RobotError
from software.robot.link import LinkError
from ...runtime.state import state
from .deps import call, client, require_no_execution
from .models import ConfigWriteIn, DeadmanIn, LimitsIn

router = APIRouter()


@router.post("/config/limits")
async def set_limits(body: LimitsIn) -> Dict:
    """Update the board motion limits (validated; SAVE_CONFIG persists them)."""
    link = client()
    try:
        robot_config.validate_engineering(
            {
                "max_speed_mm_s": body.max_speed_mm_s,
                "accel_mm_s2": body.accel_mm_s2,
                "decel_mm_s2": body.decel_mm_s2,
            }
        )
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    await call(lambda: link.set_limits(body.max_speed_mm_s, body.accel_mm_s2, body.decel_mm_s2))
    return {"ok": True}


@router.post("/config/deadman")
async def set_deadman(body: DeadmanIn) -> Dict:
    """Arm or clear the board command watchdog (0 = off)."""
    link = client()
    await call(lambda: link.set_deadman(body.timeout_ms))
    return {"ok": True}


@router.get("/config/robot")
async def get_robot_config() -> Dict:
    """Full settings register image from the board (calibration tab)."""
    link = client()
    raw_values = await call(lambda: link.get_all_config())
    registers = []
    for register in robot_config.REGISTERS:
        raw = raw_values[register.index]
        registers.append({**register.as_meta(), "raw": raw, "value": register.decode(raw)})
    dirty: Optional[bool] = None
    try:  # live dirty flag: an unsaved edit must be visible immediately
        dirty = bool((await link.status()).cfg_dirty)
    except (RobotError, LinkError):
        dirty = bool(state.board_status.cfg_dirty) if state.board_status is not None else None
    return {
        "map_version": robot_config.MAP_VERSION,
        "speed_limit_mm_s": robot_config.MAX_SPEED_LIMIT_MM_S,
        "registers": registers,
        "dirty": dirty,
    }


@router.post("/config/robot")
async def set_robot_config(body: ConfigWriteIn) -> Dict:
    """Write settings registers (engineering units, validated, idle only)."""
    link = client()
    require_no_execution()
    try:
        raw = robot_config.validate_engineering(body.values)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    if not raw:
        raise HTTPException(status_code=422, detail="no settings values given")
    for offset, values in robot_config.chunk_writes(raw):
        await call(lambda offset=offset, values=values: link.set_config(offset, values))
    applied = {key: robot_config.REGISTER_BY_KEY[key].decode(value) for key, value in raw.items()}
    # a calibrated odometry track must reach the running estimator at once
    await state.session.sync_board_geometry()
    return {"ok": True, "applied": applied}


@router.post("/config/robot/save")
async def save_robot_config() -> Dict:
    """Persist the current settings to the board flash."""
    link = client()
    require_no_execution()
    await call(lambda: link.save_config())
    await state.log("info", "robot settings saved to the board flash")
    return {"ok": True}


@router.post("/config/robot/load")
async def load_robot_config() -> Dict:
    """Reload the settings from the board flash (discard RAM edits)."""
    link = client()
    require_no_execution()
    await call(lambda: link.load_config())
    await state.session.sync_board_geometry()
    return {"ok": True}


@router.post("/config/robot/defaults")
async def reset_robot_config() -> Dict:
    """Restore the compile-time defaults in RAM (not saved)."""
    link = client()
    require_no_execution()
    await call(lambda: link.reset_config())
    await state.log("warn", "robot settings restored to the compile-time defaults (not saved)")
    await state.session.sync_board_geometry()
    return {"ok": True}
