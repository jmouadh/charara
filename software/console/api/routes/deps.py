"""Shared plumbing for the REST routes: client access and error mapping.

File:    software/console/api/routes/deps.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

from typing import Any, Awaitable, Callable, Dict

from fastapi import HTTPException
from pydantic import BaseModel

from software.robot.client import RobotError
from software.robot.link import LinkError
from ...runtime.state import state


def client():
    """The robot client, or HTTP 503 when the link is not initialised."""
    if state.client is None or state.executor is None:
        raise HTTPException(status_code=503, detail="robot link not initialised")
    return state.client


def require_no_execution() -> None:
    """Refuse with HTTP 409 while a mission is executing."""
    if state.executor is not None and state.executor.is_running():
        raise HTTPException(status_code=409, detail="a path is executing - stop it first")

async def require_no_faults() -> None:
    """Refuse a pose/motion change while the board has latched faults.

    E-STOP latches on the board and silently blocks SET_POSE and every drive
    command: the operator only saw "robot: FAULT" and thought the Pose tool
    was broken (2026-10-01). Say what to do instead. Best effort - when the
    odometry link itself is down, let the actual action report it.
    """
    if state.client is None:
        return
    try:
        odometry = await state.client.odometry()
    except Exception:
        return
    if odometry.faults != 0:
        names = ", ".join(odometry.describe_faults()) or "fault"
        raise HTTPException(
            status_code=409,
            detail=f"the board has latched faults ({names}) - press Clear Faults before moving or re-placing the robot",
        )

async def call(action: Callable[[], Awaitable[Any]]) -> Any:
    """Run a robot action, mapping its errors to HTTP status codes."""
    try:
        return await action()
    except RobotError as exc:
        raise HTTPException(status_code=409, detail=f"robot: {exc.name}") from exc
    except LinkError as exc:
        raise HTTPException(status_code=503, detail=f"link: {exc}") from exc
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


def to_dict(model: BaseModel) -> Dict[str, Any]:
    """pydantic v1 (``.dict()``) and v2 (``.model_dump()``) compatible."""
    dump = getattr(model, "model_dump", None)
    return dump() if callable(dump) else model.dict()
