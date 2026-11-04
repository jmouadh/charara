"""Motion routes: manual moves, execution control, E-STOP, motor release, fault clearing.

File:    software/console/api/routes/motion.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

from typing import Dict

from fastapi import APIRouter, HTTPException

from software.core.mission.executor import (
    EXEC_PAUSED,
    needs_game_element_actions,
    needs_in_place_actions,
)
from software.core.mission.orchestrator import (
    apply_game_element,
    follow_bounds,
    resume_from_wait,
)
from software.core.navigation.options import PlanOptions

from ...runtime.state import state
from .deps import call, client, require_no_execution, require_no_faults
from .models import ExecuteIn, MoveArcIn, MoveDriveIn, MoveGotoIn, MoveStrafeIn, MoveTurnIn

router = APIRouter()


@router.post("/move/drive")
async def move_drive(body: MoveDriveIn) -> Dict:
    """Manual move: drive ``distance_mm`` forward (negative = backward)."""
    link = client()
    require_no_execution()
    await require_no_faults()
    await call(lambda: link.drive(body.distance_mm, body.speed_mm_s))
    return {"ok": True}


@router.post("/move/turn")
async def move_turn(body: MoveTurnIn) -> Dict:
    """Manual move: turn ``angle_deg`` in place (CCW positive)."""
    link = client()
    require_no_execution()
    await require_no_faults()
    await call(lambda: link.turn(body.angle_deg, body.speed_mm_s))
    return {"ok": True}


@router.post("/move/strafe")
async def move_strafe(body: MoveStrafeIn) -> Dict:
    """Manual move: strafe sideways (dead-reckoned on the board)."""
    link = client()
    require_no_execution()
    await require_no_faults()
    await call(lambda: link.strafe(body.distance_mm, body.speed_mm_s))
    return {"ok": True}


@router.post("/move/goto")
async def move_goto(body: MoveGotoIn) -> Dict:
    """Manual move: one absolute GOTO to (x, y, theta)."""
    link = client()
    require_no_execution()
    await require_no_faults()
    await call(lambda: link.goto(body.x_mm, body.y_mm, body.theta_deg, body.speed_mm_s))
    return {"ok": True}


@router.post("/move/arc")
async def move_arc(body: MoveArcIn) -> Dict:
    """Manual arc: + radius / + sweep = left (CCW); signs must agree."""
    link = client()
    require_no_execution()
    await require_no_faults()
    await call(lambda: link.arc(body.radius_mm, body.sweep_deg, body.speed_mm_s))
    return {"ok": True}


@router.post("/execute")
async def execute(body: ExecuteIn) -> Dict:
    """Start a run: the given segments, or the last computed plan."""
    client()
    executor = state.executor
    assert executor is not None
    if not state.game_allows_motion():
        raise HTTPException(
            status_code=409,
            detail=(
                "game time is over - reset the game clock in Game > Game time "
                "before running again"
            ),
        )
    if state.vision_waiting is not None:
        # The displayed plan can be the pre-obstacle route: a wait deliberately
        # does not replace it with an unsafe candidate. Never replay that stale
        # segment list from the stopped pose; retry the remaining mission using
        # current odometry and the latest obstacle snapshot instead.
        if await resume_from_wait(state):
            await state.game_start_if_configured()
            return {"ok": True, "resumed": True}
        waiting = state.vision_waiting or {}
        reason = str(waiting.get("reason") or "the obstacle still blocks the route")
        raise HTTPException(
            status_code=409,
            detail=f"robot is still stopped: {reason}; clear or move the obstacle and retry Execute",
        )
    segments = body.segments
    if segments is None:
        if state.last_plan is None:
            raise HTTPException(status_code=422, detail="no plan available - press Plan first")
        segments = state.last_plan["segments"]
    # Track B: the plan's polyline drives the run with the streaming follower
    # when the operator enabled it (STREAMING_EXEC). The executor itself
    # REFUSES the start when the board cannot stream - gate off means the
    # segment backend runs, exactly like before (no workflow change).
    follow_path = None
    if body.follow_path is not None and len(body.follow_path) >= 2:
        if state.session.settings.streaming_exec_enabled:
            follow_path = [[float(point[0]), float(point[1])] for point in body.follow_path]
    skipped_stream = False
    if follow_path is not None and needs_in_place_actions(segments):
        # the polyline cannot perform an explicit rotation / pause / mid-route
        # arrival turn: the segment backend does (never silently skip a step)
        follow_path = None
        skipped_stream = True
    # remember where this run starts: the mock "Redo" teleports back here so
    # the same plan can be replayed from the very same conditions
    pose = state.pose()
    if pose is not None:
        state.last_run_start = (float(pose[0]), float(pose[1]), float(pose[2]))
    state.redo_restore_pose = None  # a new run supersedes a pending redo
    bounds = None
    if follow_path is not None:
        options = PlanOptions.from_dict((state.last_plan_request or {}).get("options"))
        bounds = follow_bounds(state.field["width_mm"], state.field["height_mm"], options)
    hook_kwargs = {}
    if needs_game_element_actions(segments):
        # the plan carries erase/spawn actions: update the live map and tell
        # the console as the run reaches each one
        hook_kwargs["game_element_hook"] = lambda action: apply_game_element(state, action)
    try:
        await executor.start(
            segments, body.deadman_ms, follow_path=follow_path, follow_bounds=bounds,
            **hook_kwargs,
        )
    except (RuntimeError, ValueError) as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    state.vision_waiting = None  # a manual run supersedes a waiting state
    state.skipped_waypoints = set()  # ...and gives every checkpoint a new chance
    if skipped_stream:
        await state.log(
            "warn",
            "streaming skipped: this plan performs an in-place rotation, pause, "
            "motor release, arrival turn or close-waypoint final leg - the segment "
            "backend drives it (the polyline cannot encode it safely)",
        )
    if follow_path is not None:
        await state.log(
            "info",
            f"execution started: streaming the path ({len(follow_path)} points, "
            f"{len(segments)} planned segment(s) blended)",
        )
    else:
        await state.log("info", f"execution started: {len(segments)} segments")
    # Execute begins (or continues) the match: the clock starts once and keeps
    # running across re-plans; an explicit Pause freezes it (Resume continues).
    await state.game_start_if_configured()
    return {"ok": True, "n_segments": len(segments), "streaming": follow_path is not None}


@router.post("/mission/redo")
async def mission_redo() -> Dict:
    """Put the robot back EXACTLY at the last run's start - also when paused.

    After a run: the simulator is teleported (no driving) to the pose the
    robot held when the last run started - press Execute again to replay the
    same plan from the same conditions, or edit the drawing and re-plan.

    WHILE PAUSED: the paused run is ended and the same redo happens, so the
    operator can replay the whole sequence from its beginning (the requested
    "Pause then Redo"). Real-board mode refuses: the console never moves the
    real robot by itself, and nothing is stopped before that answer.

    While the redo is armed, SAVING the strategy puts the robot back at the
    pose it had BEFORE the redo (the run's end): a committed step keeps the
    mission continuing from where the validated run actually finished.
    """
    executor = state.executor
    paused = executor is not None and executor.is_running() and executor.state == EXEC_PAUSED
    if executor is not None and executor.is_running() and not paused:
        raise HTTPException(
            status_code=409,
            detail="the mission is running - pause it first, then REDO ends it and "
            "goes back to the start of the sequence",
        )
    if state.transport != "mock":
        raise HTTPException(
            status_code=409,
            detail="redo is a mock rehearsal tool - it teleports the robot; on the real "
            "robot place it back and press Set Pose",
        )
    if paused:
        # the paused mission is over: replaying the sequence starts it again
        await executor.stop("ended by REDO - replay the path from its start")
        await executor.wait()
    require_no_execution()
    if state.last_run_start is None:
        raise HTTPException(status_code=409, detail="nothing to redo yet - execute a plan first")
    start = state.last_run_start
    if state.redo_restore_pose is None:
        pose = state.pose()
        if pose is not None:
            state.redo_restore_pose = (float(pose[0]), float(pose[1]), float(pose[2]))
    link = client()
    await call(lambda: link.set_pose(float(start[0]), float(start[1]), float(start[2])))
    # Redo restarts the counters: the match clock goes back to its full
    # duration and the next Execute starts it again (the task timer is reset
    # by the console at the same moment).
    await state.game_reset()
    await state.log(
        "info",
        f"redo: robot placed back at the start of the last run "
        f"({start[0]:.0f}, {start[1]:.0f}) mm, {start[2]:.0f} deg - press Execute to replay",
    )
    restore = state.redo_restore_pose
    return {
        "ok": True,
        "start": [round(float(v), 1) for v in start],
        "restore": [round(float(v), 1) for v in restore] if restore else None,
    }


@router.post("/pause")
async def pause() -> Dict:
    """Pause the running mission and both counters (stop in place; resume continues)."""
    if state.executor is None or not state.executor.is_running():
        raise HTTPException(status_code=409, detail="nothing is executing")
    state.executor.pause()
    # the console shows one Pause button: the task timer is paused by the UI
    # and the server-authoritative match clock here
    await state.game_pause()
    return {"ok": True}


@router.post("/resume")
async def resume() -> Dict:
    """Resume a paused mission and continue the frozen match clock."""
    if state.executor is None:
        raise HTTPException(status_code=503, detail="robot link not initialised")
    if not state.game_allows_motion():
        raise HTTPException(
            status_code=409,
            detail=(
                "game time is over - reset the game clock in Game > Game time "
                "before resuming"
            ),
        )
    state.executor.resume()
    await state.game_resume()
    return {"ok": True}


@router.post("/stop")
async def stop() -> Dict:
    """Abort the run (or stop the idle board); the robot holds position."""
    link = client()
    executor = state.executor
    assert executor is not None
    state.vision_waiting = None  # a manual STOP cancels a waiting state
    if executor.is_running():
        await executor.stop()
    else:
        await call(lambda: link.stop())
    await state.log("warn", "STOP requested")
    return {"ok": True}


@router.post("/estop")
async def estop() -> Dict:
    """Latch the emergency stop (clear faults before moving again)."""
    link = client()
    executor = state.executor
    assert executor is not None
    state.vision_waiting = None
    if executor.is_running():
        await executor.estop()
    else:
        await call(lambda: link.estop())
    await state.log("error", "E-STOP requested")
    return {"ok": True}


@router.post("/release_motors")
async def release_motors() -> Dict:
    """Release the stepper drivers, only after every movement has stopped.

    This is a normal release (no E-STOP latch): the board keeps its pose and
    the next motion command re-energizes the drivers. The board enforces the
    same idle-only rule, so a request racing the end of a move is rejected
    instead of dropping the drivers while the wheels still turn.
    """
    link = client()
    require_no_execution()
    await require_no_faults()
    odometry = await call(lambda: link.odometry())
    if not odometry.is_idle:
        raise HTTPException(
            status_code=409,
            detail=(
                f"the robot is still moving (board state {odometry.state_name}) - "
                "release the motors only after the movement has completely finished"
            ),
        )
    await call(lambda: link.release_motors())
    await state.log(
        "info",
        "stepper motors released (drivers off) - the next move re-energizes them",
    )
    return {"ok": True}


@router.post("/clear_faults")
async def clear_faults() -> Dict:
    """Clear latched board faults."""
    link = client()
    await call(lambda: link.clear_faults())
    await state.log("info", "faults cleared")
    return {"ok": True}
