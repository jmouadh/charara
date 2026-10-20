"""Planning routes: plan (and mid-run re-plan) plus the last computed plan.

File:    software/console/api/routes/planning.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

from typing import Dict

from fastapi import APIRouter, HTTPException

from software.core.mission.orchestrator import execute_plan

from ...runtime.state import state
from .deps import to_dict
from .models import PlanIn

router = APIRouter()


@router.post("/plan")
async def make_plan(body: PlanIn) -> Dict:
    """Plan the mission (or re-plan its remainder while the robot executes).

    While the robot is executing, the request's ``elements`` are sliced at
    the robot's stop position and the remainder is re-routed around the
    current obstacle map (console + camera); the execution restarts on it -
    a paused run stays paused.
    See :func:`software.core.mission.orchestrator.execute_plan`.
    """
    if state.odometry is None:
        raise HTTPException(status_code=503, detail="no odometry yet - is the board connected?")
    elements = list(body.elements)
    if not elements and body.strokes:
        elements = [{"kind": "stroke", "points": stroke} for stroke in body.strokes]
    goal = to_dict(body.goal) if body.goal else None
    obstacles = [to_dict(obstacle) for obstacle in body.obstacles]
    try:
        return await execute_plan(state, elements, goal, obstacles, body.options, body.deadman_ms)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@router.get("/plan")
async def get_last_plan() -> Dict:
    """The last computed plan (segments, preview path, stats, warnings)."""
    if state.last_plan is None:
        raise HTTPException(status_code=404, detail="no plan yet")
    return state.last_plan
