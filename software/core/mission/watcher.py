"""Camera-driven obstacle reactions (mission layer).

A vision client - the camera / detector process - connects to ``/ws/vision``
and keeps pushing the obstacles it currently sees. The message protocol
(parsing, acks, disconnects) lives in :mod:`app.api.ws`; THIS module reacts
to store changes:

* every change is broadcast to the console (``{"type": "vision", ...}`` -
  drawn as dashed red-orange outlines), and
* when the robot is executing, turned into the same stop -> re-plan the
  remaining mission -> continue cycle as a dynamic obstacle drawn by hand
  (coalesced: at most one re-plan per ``REPLAN_DEBOUNCE_S``).

The obstacles are merged into every plan next to the console-drawn ones, so
they count even if nobody re-plans by hand.

File:    software/core/mission/watcher.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Dict, Optional

from ..navigation.clearance import body_clear_at, body_geometry
from ..navigation.options import PlanOptions
from . import orchestrator as mission
from .interfaces import MissionContext

_log = logging.getLogger(__name__)

REPLAN_DEBOUNCE_S = 0.15
# the proximity watchdog re-checks while driving: an obstacle that was too far
# to matter can become close as the robot advances (no new camera message).
# 0.2 s (and a 75 mm movement cell below) keeps the dynamic safety bubble
# ahead of a fast opponent: with a 0.5 s / 150 mm check the gap could jump
# from "creep" straight into the stop band (2026-10-27).
PROXIMITY_PERIOD_S = 0.2
_proximity_signature: Optional[tuple] = None
# dragging an opponent robot must not flood the console log with one line per
# camera message: the obstacle preview is logged at most once per second
PREVIEW_LOG_PERIOD_S = 1.0
_preview_log_at = 0.0


# ------------------------------------------------------------- plumbing ----
async def broadcast_state(ctx: MissionContext, reason: str) -> None:
    """Publish the current vision obstacle set to every console."""
    data = ctx.vision.status()
    data["reason"] = reason
    await ctx.emit({"type": "vision", "data": data})


def _obstacle_signature(obstacle: Dict) -> tuple:
    """Rounded, hashable shape of one obstacle - polygons included.

    The proximity watchdog only needs "did the nearby obstacles change?": a
    custom shape has no x/y/w/h, so its vertex list is quantised instead.
    """
    kind = str(obstacle.get("kind", "rect"))
    if kind == "polygon":
        points = []
        for point in obstacle.get("points") or []:
            try:
                points.append(
                    (round(float(point[0]) / 50.0), round(float(point[1]) / 50.0))
                )
            except (TypeError, ValueError, IndexError):
                continue
        return (kind, tuple(points))
    return (
        kind,
        round(float(obstacle.get("x", 0.0)) / 50.0),
        round(float(obstacle.get("y", 0.0)) / 50.0),
        round(float(obstacle.get("w", obstacle.get("r", 0.0))) / 50.0),
        round(float(obstacle.get("h", 0.0)) / 50.0),
    )


def _opponent_cell(pose: Optional[Dict]) -> Optional[tuple]:
    """Quantize opponent movement so camera jitter does not churn replans."""
    if pose is None:
        return None
    return (round(float(pose["x_mm"]) / 50.0), round(float(pose["y_mm"]) / 50.0))


def notify_opponent_change(
    ctx: MissionContext, previous: Optional[Dict], current: Optional[Dict]
) -> None:
    """Schedule a quiet recheck when the opponent moves by a meaningful amount."""
    if _opponent_cell(previous) != _opponent_cell(current):
        schedule_replan(ctx, quiet=True)


async def _stop_on_dynamic_contact(ctx: MissionContext) -> bool:
    """Emergency hold when a moving obstacle enters the robot's safety bubble.

    A dragged/opponent robot can appear ON the robot between two camera
    frames. Stopping normally happens after the reaction planner runs, which
    is too late: the body is already overlapping. Stop immediately (also when
    the obstacle is merely close, ``DYNAMIC_CONTACT_PAD_MM`` around the body)
    and leave a wait record, so the next camera frame resumes the mission.
    An obstacle entirely BEHIND the direction of travel is exempt: the robot
    drives away from it and stopping would only keep it there.
    """
    executor = ctx.executor
    if executor is None or not executor.is_running():
        return False
    if getattr(executor, "state", "") == "paused":
        return False
    pose = ctx.pose()
    request = ctx.last_plan_request
    if pose is None or request is None:
        return False
    # vision obstacles PLUS a fresh opponent pose (a camera may report only
    # ArUco poses, not a rectangle)
    dynamic = mission.active_obstacles(ctx, [], request.get("options") or {})
    if not dynamic:
        return False
    # A trailing obstacle cannot be driven into while the robot continues
    # forward: ignore anything entirely behind the direction the remaining
    # route is driving (a following robot used to retrigger this bubble on
    # every camera frame and freeze the mission).
    direction = mission.travel_direction(
        (ctx.last_plan or {}).get("path") or [], (pose[0], pose[1]), pose[2]
    )
    dynamic = [
        obstacle for obstacle in dynamic
        if not mission.obstacle_behind_robot(obstacle, (pose[0], pose[1]), direction)
    ]
    if not dynamic:
        return False
    options = PlanOptions.from_dict(request.get("options") or {})
    width, height = body_geometry(options)
    field = ctx.field
    touching = [
        obstacle for obstacle in dynamic
        if not body_clear_at(
            (pose[0], pose[1]), pose[2], [obstacle], width, height,
            field["width_mm"], field["height_mm"],
            pad_mm=mission.DYNAMIC_CONTACT_PAD_MM, border_pad_mm=0.0,
        )
    ]
    if not touching:
        return False
    # a re-plan may already be running: cancel it before latching the wait,
    # otherwise it can restart the run or clear vision_waiting
    task = getattr(ctx, "vision_replan_task", None)
    if task is not None and not task.done():
        task.cancel()
    await executor.stop(reason="waiting for the moving obstacle - too close to the robot")
    await executor.wait_idle()
    mission.begin_moving_obstacle_wait(
        ctx,
        elements=request.get("elements") or [],
        goal=request.get("goal"),
        obstacles=request.get("obstacles") or [],
        options=request.get("options") or {},
        deadman_ms=request.get("deadman_ms"),
        paused=False,
        reason="the moving obstacle is too close to the robot",
    )
    await ctx.log(
        "warn",
        "the moving obstacle is inside the robot's safety bubble - stopped "
        "immediately; it resumes by itself when the obstacle moves away",
    )
    return True


async def notify_change(ctx: MissionContext, reason: str) -> None:
    """Log, broadcast and (mid-run only) schedule a re-plan."""
    global _preview_log_at
    obstacles = ctx.vision.active()
    now = time.monotonic()
    if now - _preview_log_at >= PREVIEW_LOG_PERIOD_S:
        _preview_log_at = now
        if obstacles:
            preview = ", ".join(f"({o['x']:.0f}, {o['y']:.0f})" for o in obstacles[:4])
            more = "" if len(obstacles) <= 4 else f" +{len(obstacles) - 4} more"
            await ctx.log("warn", f"vision: {len(obstacles)} obstacle(s) - {preview}{more}")
        else:
            await ctx.log("info", f"vision: {reason} - no obstacles")
    await broadcast_state(ctx, reason)
    if await _stop_on_dynamic_contact(ctx):
        return  # the wait deadline / next camera frame retries
    schedule_replan(ctx)


# -------------------------------------------------------------- re-plan ----
def schedule_replan(ctx: MissionContext, *, quiet: bool = False) -> bool:
    """Coalesced mid-run re-plan: one at a time, 0.15 s after the change.

    While the robot waits for a moving obstacle to clear the run is STOPPED
    (not running), so the same slot schedules the RESUME attempt instead.
    ``quiet`` (the proximity watchdog) suppresses the "trajectory unchanged"
    log - driving beside an irrelevant robot stays silent. Returns True when a
    task was actually queued; a proximity check that could not be scheduled
    must be retried on the next watchdog pass (2026-10-27).
    """
    task = ctx.vision_replan_task
    if task is not None and not task.done():
        return False  # a re-plan is already queued; it will pick up the newest state
    if ctx.vision_waiting is not None:
        ctx.vision_replan_task = asyncio.create_task(_resume_soon(ctx))
        return True
    executor = ctx.executor
    if executor is None or not executor.is_running() or ctx.last_plan_request is None:
        return False
    ctx.vision_replan_task = asyncio.create_task(_replan_soon(ctx, quiet))
    return True


async def _resume_soon(ctx: MissionContext) -> None:
    """Coalesced resume attempt for a robot waiting on a moving obstacle."""
    try:
        await asyncio.sleep(REPLAN_DEBOUNCE_S)
        await mission.resume_from_wait(ctx)
    except Exception as exc:  # never let the watcher die
        await ctx.log("error", f"waiting resume error: {exc}")
    finally:
        ctx.vision_replan_task = None


async def _replan_soon(ctx: MissionContext, quiet: bool = False) -> None:
    """Debounced mid-run re-plan with the newest request and obstacles."""
    try:
        await asyncio.sleep(REPLAN_DEBOUNCE_S)
        request = ctx.last_plan_request
        executor = ctx.executor
        if request is None or executor is None or not executor.is_running():
            return
        result = await mission.execute_plan(
            ctx,
            request["elements"],
            request.get("goal"),
            request.get("obstacles") or [],
            request.get("options") or {},
            request.get("deadman_ms"),
            quiet_unchanged=quiet,
        )
        if result.get("unchanged") or result.get("waiting"):
            return  # the outcome logged itself where it matters
        await ctx.log(
            "warn",
            f"vision re-plan: the remaining mission was updated ({result['stats']['n_segments']} segments)",
        )
    except ValueError as exc:
        await ctx.log("warn", f"vision re-plan failed: {exc}")
    except Exception as exc:  # never let the watcher die
        # keep the traceback in the service log; the console only gets the
        # short message
        _log.exception("vision re-plan error")
        await ctx.log("error", f"vision re-plan error: {exc}")
    finally:
        ctx.vision_replan_task = None


async def proximity_loop(ctx: MissionContext, period_s: float = PROXIMITY_PERIOD_S) -> None:
    """Re-check the remaining route as the robot ADVANCES.

    A parked opponent robot sends no new camera report, so the report-driven
    path alone would never notice when a far obstacle becomes close. Every
    pass: if an obstacle is now within the reaction distance of the robot,
    schedule the usual affected check - it speaks only when something is
    actually blocked (quiet otherwise, so driving beside a robot is silent).
    """
    global _proximity_signature
    while True:
        await asyncio.sleep(period_s)
        try:
            executor = ctx.executor
            if executor is None or not executor.is_running():
                _proximity_signature = None
                continue
            request = ctx.last_plan_request
            pose = ctx.pose()
            if request is None or pose is None:
                continue
            merged = mission.active_obstacles(
                ctx, request.get("obstacles") or [], request.get("options") or {}
            )
            position = (pose[0], pose[1])
            near = mission.nearby_obstacles(merged, position, request.get("options") or {})
            if not near:
                _proximity_signature = None
                continue
            signature = (
                int(position[0] // 75.0),
                int(position[1] // 75.0),
                tuple(sorted(_obstacle_signature(o) for o in near)),
            )
            if signature == _proximity_signature:
                continue
            # consume the signature only when the check was really queued: a
            # busy re-plan task must not swallow the next movement check
            if schedule_replan(ctx, quiet=True):
                _proximity_signature = signature
        except asyncio.CancelledError:
            raise
        except Exception:
            pass


# ------------------------------------------------------- websocket side ----
# The message-level protocol (parsing, acks, disconnect handling) lives in
# the API layer (``software.console.api.ws``); this module only reacts to
# STORE changes through :func:`notify_change`.


async def sweep_loop(ctx: MissionContext, period_s: float = 0.5) -> None:
    """Expire stale reports (also mid-run) so old obstacles self-clear."""
    while True:
        await asyncio.sleep(period_s)
        try:
            if ctx.vision.prune():
                await notify_change(ctx, "stale report expired")
        except asyncio.CancelledError:
            raise
        except Exception:
            pass
