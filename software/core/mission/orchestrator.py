"""Mission orchestration - plan computation and mid-run dynamic swaps.

``execute_plan`` is the single entry point used by both the console
(``POST /api/plan``) and the vision watcher (a camera report that changes the
obstacle map while the robot executes). Behaviour:

* robot idle        -> plain plan of the request's elements;
* robot executing   -> **dynamic re-plan**: only when the new obstacle map
  affects the REMAINING route (see :func:`_remaining_route_affected`) the
  robot is stopped and the run is re-planned. The re-plan continues on the
  **current route** (the not-yet-executed program, i.e. what the robot is
  actually driving, detours included) so a robot that is mid-detour is never
  sent back to the drawing; only when the MISSION itself was edited does the
  re-plan go back to the drawing (sliced at the robot's position). A paused
  run restarts paused.

The obstacle map a plan is built from is ``console obstacles (this request)``
**plus** the camera's live reports (:class:`~app.world.vision.VisionStore`), so
both sources are always respected.

File:    software/core/mission/orchestrator.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import asyncio
import json
import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

from ..navigation import geometry as g
from ..navigation.clearance import (
    BODY_PAD_MM,
    body_geometry,
    clearances,
    first_body_clip,
    rotation_clearance,
    waypoint_clearance,
)
from ..navigation.options import PlanOptions
from ..navigation.replan import (
    WAYPOINT_VISIT_TOL_MM,
    advance_cursor,
    continue_route_elements,
    drop_skipped_waypoints,
    executed_path,
    path_fraction,
    replan_route_elements,
    remaining_path,
    route_blocked,
    route_distance_mm,
    route_remainder_elements,
    slice_leading_stroke_at_robot,
    slice_remaining_elements,
)
from ..navigation.trajectory import plan
from .executor import EXEC_ABORTED, EXEC_PAUSED, needs_in_place_actions
from .interfaces import MissionContext

# a moving obstacle (another robot) that blocks the route costs nothing to
# wait for: a detour that is much longer than the remaining route - or one
# that cannot avoid the obstacle at all - is not worth driving
WAIT_DETOUR_RATIO = 1.6
WAIT_DETOUR_SLACK_MM = 500.0

# mid-run reactions have a horizon: an obstacle far ahead of the robot is
# only ACTED on when it actually clips the remaining route (the clip gate
# below); within half-diagonal + REACT_GAP_MM of our robot it is worth
# re-planning early - the detour is then generated while the robot still has
# room for it (the robot used to react ~0.7 m before the blockage and had to
# swerve at the last moment). The proximity watchdog re-checks automatically
# as the robot advances.
REACT_GAP_MM = 800.0

# An obstacle ON the remaining route is reacted to by DISTANCE, not by the
# mere fact that it blocks:
#
# * farther than CAUTION_GAP_MM along the route: nothing happens. Another
#   robot usually moves on before we get there, and the watchdog re-checks
#   every 0.5 s as we advance - re-planning from far away stops the robot for
#   a detour it may not need (2026-10-09);
# * inside CAUTION_GAP_MM: the robot CREEPS (CAUTION_SCALE of its own plan
#   speeds). That buys the time the (expensive) re-plan needs and is exactly
#   the "be careful when you get close" behaviour;
# * inside STOP_GAP_MM: stop and resolve - wait <= WAIT_MAX_S for a moving
#   obstacle, detour when there is nowhere to wait, never a forever stop;
# * any distance: when the BLOCKAGE POINT itself is tight (a border, a static
#   obstacle closer than the body clearance + TIGHT_ROOM_MM), act at once -
#   re-planning from there would have no room left to manoeuvre.
CAUTION_GAP_MM = 700.0
STOP_GAP_MM = 250.0
CAUTION_SCALE = 0.35
TIGHT_ROOM_MM = 150.0
#: how long a restarted run must survive before it counts as running: an
#: immediate abort (a latched fault) used to be invisible to the caller
REPLAN_VERIFY_S = 0.5
#: a resume that keeps aborting gives up with a clear reason instead of
#: sitting on "waiting" forever
WAIT_RESUME_MAX_FAILS = 3

# the next WAYPOINT is the mission's next checkpoint: when a camera obstacle
# parks on it, the robot waits for it to move instead of driving a compromise
# approach that "visits" it 27 cm short - or worse, ignoring it. The wait
# only STARTS when the robot is this close to the checkpoint (stopping from
# far away is pointless - the obstacle usually moves on before the robot
# gets there) and never lasts longer than WAIT_MAX_S: after that the robot
# tries an alternative path to the checkpoint and, when there is none, gives
# it up and continues with the NEXT waypoint (a stop is never forever).
WAYPOINT_WAIT_REACH_MM = 600.0
WAIT_MAX_S = 1.0


def obstacle_distance_mm(obstacle: Dict, position: Tuple[float, float]) -> float:
    """Distance from ``position`` to the obstacle boundary (0 when inside)."""
    px, py = float(position[0]), float(position[1])
    if str(obstacle.get("kind", "rect")) == "circle":
        cx, cy = float(obstacle.get("x", 0.0)), float(obstacle.get("y", 0.0))
        return max(0.0, math.hypot(px - cx, py - cy) - float(obstacle.get("r", 0.0)))
    x0, y0 = float(obstacle.get("x", 0.0)), float(obstacle.get("y", 0.0))
    x1, y1 = x0 + float(obstacle.get("w", 0.0)), y0 + float(obstacle.get("h", 0.0))
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    return math.hypot(dx, dy)


def nearby_obstacles(
    obstacles: Sequence[Dict],
    position: Optional[Tuple[float, float]],
    options_dict: Optional[Dict],
) -> List[Dict]:
    """The subset of ``obstacles`` close enough to the robot to react to.

    Only these take part in the mid-run decisions (stop / detour / wait): a
    robot parked on the route 1 m ahead must not change anything yet. A
    missing position (no odometry) keeps everything - conservative.
    """
    if position is None:
        return list(obstacles)
    options = PlanOptions.from_dict(options_dict or {})
    border, _ = clearances(options)
    reach = border + REACT_GAP_MM
    return [o for o in obstacles if obstacle_distance_mm(o, position) <= reach]


def elements_signature(elements: Sequence[Dict]) -> str:
    """Stable identity of a mission so an edit can be told from a re-send."""
    return json.dumps(list(elements), sort_keys=True, separators=(",", ":"))


def _head_waypoint(elements: Sequence[Dict]) -> Optional[Tuple[float, float]]:
    """The first motion target of a remainder when it is a WAYPOINT."""
    for element in elements:
        kind = str(element.get("kind", "stroke"))
        if kind in ("pause", "rotate", "drive", "release"):
            continue
        if kind == "waypoint" and "x" in element and "y" in element:
            return (float(element["x"]), float(element["y"]))
        return None
    return None


def _next_waypoint(elements: Sequence[Dict], cursor: int) -> Optional[Tuple[float, float]]:
    """The first waypoint of the mission at/after ``cursor`` (None when none)."""
    start = max(0, min(int(cursor), len(elements)))
    for element in list(elements)[start:]:
        if str(element.get("kind", "stroke")) == "waypoint":
            return (float(element.get("x", 0.0)), float(element.get("y", 0.0)))
    return None


def _previous_waypoint(elements: Sequence[Dict], cursor: int) -> Optional[Tuple[float, float]]:
    """The last waypoint BEFORE ``cursor`` (the one just visited/approached)."""
    for element in reversed(list(elements)[: max(0, min(int(cursor), len(elements)))]):
        if str(element.get("kind", "stroke")) == "waypoint":
            return (float(element.get("x", 0.0)), float(element.get("y", 0.0)))
    return None


def _waypoint_blocked(
    point: Tuple[float, float], obstacles: Sequence[Dict], options_dict: Optional[Dict]
) -> bool:
    """Does a camera obstacle sit inside the body clearance of ``point``?

    This is exactly the situation that makes the planner approach short of a
    waypoint instead of reaching it - both the blocked-waypoint wait and the
    wait-resume gate key off it.
    """
    options = PlanOptions.from_dict(options_dict or {})
    if options.waypoint_margin_mm is None:
        border, _ = clearances(options)
        limit = border + 8.0
    else:
        # a configured close waypoint is legal at its stop clearance; only
        # an obstacle INSIDE that configured distance (plus the body pad)
        # blocks it. Using the routing margin here would wait forever next
        # to a static obstacle the waypoint was intentionally placed near.
        limit = waypoint_clearance(options) + BODY_PAD_MM
    return any(obstacle_distance_mm(obstacle, point) < limit for obstacle in obstacles)


def clip_gap_mm(
    path: Sequence,
    obstacles: Sequence[Dict],
    options_dict: Optional[Dict],
    field_width_mm: float = 3000.0,
    field_height_mm: float = 2000.0,
) -> Tuple[float, Optional[Tuple[float, float]]]:
    """Distance ALONG ``path`` to the first point that physically clips an obstacle.

    ``(inf, None)`` when the route stays clear. This is the cheap half of the
    reaction ladder: pure geometry along the already-planned route, no
    re-plan of anything - the expensive candidate plan only runs once the
    ladder decides to stop. The path starts at the robot, so the arc length
    walked IS the distance still to drive before entering the obstacle's
    body clearance.

    The check is on the ROTATED body at the local travel heading (not the
    conservative half-diagonal circle): a close-waypoint straight final leg
    intentionally enters the routing margin and must not make the robot
    stop/re-plan against the very obstacle it is approaching.
    """
    points = [(float(p[0]), float(p[1])) for p in path if len(p) >= 2]
    if len(points) < 2 or not obstacles:
        return (math.inf, None)
    options = PlanOptions.from_dict(options_dict or {})
    width, height = body_geometry(options)
    clip = first_body_clip(
        points, obstacles, width, height, field_width_mm, field_height_mm,
    )
    if clip is None:
        return (math.inf, None)
    return (clip[0], clip[1])


def room_mm(
    point: Tuple[float, float], obstacles: Sequence[Dict], options_dict: Optional[Dict], field: Dict
) -> float:
    """Free room at ``point``: distance to the nearest STATIC obstacle or border.

    Used to tell a blockage in the open (wait, watch it move) from one in a
    corner or a corridor, where stopping late would leave no room to re-plan:
    there the robot acts while it still has space.
    """
    field_w = float(field.get("width_mm", 3000.0))
    field_h = float(field.get("height_mm", 2000.0))
    px, py = float(point[0]), float(point[1])
    room = min(px, py, field_w - px, field_h - py)
    for obstacle in obstacles:
        room = min(room, obstacle_distance_mm(obstacle, (px, py)))
    return room


def tight_blockage(
    point: Optional[Tuple[float, float]],
    static_obstacles: Sequence[Dict],
    options_dict: Optional[Dict],
    field: Dict,
) -> bool:
    """Is the blockage point too tight to re-plan from? (act early there)"""
    if point is None:
        return False
    options = PlanOptions.from_dict(options_dict or {})
    border, _ = clearances(options)
    return room_mm(point, static_obstacles, options_dict, field) < border + TIGHT_ROOM_MM


async def _plan_off_loop(**kwargs) -> Dict:
    """Run the (CPU bound) planner in a worker thread.

    On the robot's Pi a full re-plan takes a second or two; run synchronously
    it stalls the whole event loop - telemetry polling included. A stalled
    poller is what let the board's deadman lapse while the wheels were still
    winding down after the stop, which latched DEADMAN and made the next
    restart abort (the robot "stopped for good", 2026-10-09).
    """
    return await asyncio.to_thread(plan, **kwargs)


def _note_ladder(ctx: MissionContext, band: str) -> bool:
    """True when the ladder should speak: once per band, not per camera report.

    Dragging an obstacle around sends a burst of reports (coalesced to 1/0.25 s
    upstream): the "watching it" / "creeping" lines must not turn the console
    log into a ticker, so each band (250 mm bucket of the clip distance, or
    "caution") speaks once.
    """
    if getattr(ctx, "ladder_band", None) == band:
        return False
    ctx.ladder_band = band
    return True


def follow_bounds(field_width_mm: float, field_height_mm: float,
                  options: PlanOptions) -> Tuple[float, float, float]:
    """Streaming geofence: the play area shrunk by the robot body inset.

    The planner keeps the robot centre inside the half-diagonal + 12 mm
    border, so the follower aborts if the live pose leaves the same region
    (with a few millimetres of tolerance): the robot can never drive out of
    the game area, even when a stale plan or a localization jump says it may.
    """
    half = math.hypot(max(50.0, options.robot_width_mm), max(50.0, options.robot_height_mm)) * 0.5
    return (float(field_width_mm), float(field_height_mm), half + 6.0)


async def _start_verified(
    ctx: MissionContext,
    segments: Sequence[Dict],
    deadman_ms: Optional[int],
    paused: bool,
    what: str,
    follow: Optional[Sequence[Sequence[float]]] = None,
    follow_bounds: Optional[Tuple[float, float, float]] = None,
) -> Optional[str]:
    """Start a run and make sure it SURVIVED its first moments.

    ``executor.start`` returns as soon as the runner task exists, so a run that
    aborts immediately (a latched board fault, a rejected first command) was
    invisible to the caller: it logged "resuming the mission", dropped the wait
    state and left the robot standing with nothing driving it - "stopped for
    good" (2026-10-09). Returns ``None`` while the run is alive, else the
    executor's abort message.

    ``follow`` continues a STREAMING mission in the same backend: the caller
    passes the re-planned polyline only when the interrupted run was streaming
    (``executor.last_stream``) - a re-plan restart must never silently switch
    backends, or the mission's mode would flip mid-run. ``follow_bounds`` is
    the streaming geofence (field width/height + body inset).
    """
    executor = ctx.executor
    assert executor is not None
    if follow is not None and needs_in_place_actions(segments):
        # the polyline cannot encode a mid-mission rotation / arrival turn:
        # the segment backend performs the whole program instead
        await ctx.log(
            "warn",
            "the re-planned remainder needs an in-place action - the segment backend takes over",
        )
        follow = None
    try:
        if follow is None:
            # simple executor doubles predate the streaming kwarg
            await executor.start(segments, deadman_ms, paused=paused)
        else:
            await executor.start(
                segments, deadman_ms, paused=paused,
                follow_path=follow, follow_bounds=follow_bounds,
            )
    except (RuntimeError, ValueError) as exc:
        return f"{what}: {exc}"
    await asyncio.sleep(REPLAN_VERIFY_S)
    if executor.state == EXEC_ABORTED:
        return str(executor.progress.get("message") or "aborted")
    return None


def _skipped_waypoints(ctx: MissionContext) -> set:
    """The checkpoints the mission gave up on (created on first use)."""
    skipped = getattr(ctx, "skipped_waypoints", None)
    if skipped is None:
        skipped = set()
        ctx.skipped_waypoints = skipped
    return skipped


def _arm_wait_deadline(ctx: MissionContext) -> None:
    """Start the bounded-wait timer for the CURRENT ``ctx.vision_waiting``.

    A wait never lasts longer than ``WAIT_MAX_S``: the timer calls the normal
    resume path (which then sees the elapsed time and resolves the wait by
    skipping a still-blocked checkpoint or accepting a long detour). While
    the wait persists the timer retries every ``WAIT_MAX_S``, so even an
    impassable situation keeps being re-evaluated instead of freezing.
    """
    token = ctx.vision_waiting
    if token is None:
        return

    async def _run() -> None:
        """Resolve the wait every WAIT_MAX_S until it is superseded."""
        while ctx.vision_waiting is token:
            await asyncio.sleep(WAIT_MAX_S)
            if ctx.vision_waiting is not token:
                return
            try:
                await resume_from_wait(ctx)
            except Exception as exc:  # never let the timer die silently
                await ctx.log("error", f"wait deadline error: {exc}")

    ctx.wait_deadline_task = asyncio.create_task(_run())


def begin_moving_obstacle_wait(
    ctx: MissionContext,
    *,
    elements: Sequence[Dict],
    goal: Optional[Dict],
    obstacles: Sequence[Dict],
    options: Dict,
    deadman_ms: Optional[int],
    paused: bool,
    direct_mm: float = 0.0,
    reason: str = "the moving obstacle blocks the remaining route",
) -> None:
    """Record a recoverable stop caused by a MOVING obstacle.

    Unlike a failed re-plan, this leaves ``ctx.vision_waiting`` set: the next
    camera change (or the bounded deadline) calls :func:`resume_from_wait`, so
    the robot continues by itself once the obstacle has moved instead of
    staying stopped forever.
    """
    ctx.vision_waiting = {
        "reason": reason,
        "elements": list(elements),
        "goal": goal,
        "obstacles": list(obstacles),
        "options": dict(options),
        "deadman_ms": deadman_ms,
        "paused_by_user": bool(paused),
        "direct_mm": float(direct_mm),
        "started_at": time.monotonic(),
    }
    _arm_wait_deadline(ctx)


def active_obstacles(
    ctx: MissionContext,
    request_obstacles: Sequence[Dict] = (),
    options_dict: Optional[Dict] = None,
) -> List[Dict]:
    """Merge requested, camera-detected, and fresh opponent obstacles."""
    merged = list(request_obstacles) + ctx.vision.active()
    opponent = ctx.opponent_pose()
    if opponent is not None:
        options = PlanOptions.from_dict(options_dict or {})
        radius = math.hypot(options.robot_width_mm, options.robot_height_mm) * 0.5
        merged.append({
            "kind": "circle",
            "x": float(opponent["x_mm"]),
            "y": float(opponent["y_mm"]),
            "r": radius,
        })
    return merged


def _route_progress(
    ctx: MissionContext,
    elements: Sequence[Dict],
    position: Optional[Tuple[float, float]] = None,
    preserve_detours: bool = True,
) -> Tuple[int, List[Dict]]:
    """(cursor, remainder) of the drawn mission along the current route.

    The cursor is the first mission element the DRIVEN route has not
    completed (waypoints count as visited only when the route really swept
    them, so a waypoint an obstacle made the robot compromise past stays
    ahead; skipped checkpoints - given up after a bounded wait - count as
    done); the remainder is what the robot still has to do - either the
    mission from the cursor verbatim (next target: a checkpoint) or the
    current stroke's continuation with the route's detours kept.
    """
    if not elements:
        return 0, []
    stored = ctx.last_plan_request or {}
    cursor_prev = int(stored.get("cursor") or 0)
    last_path = (ctx.last_plan or {}).get("path") or []
    if position is None:
        pose = ctx.pose()
        position = None if pose is None else (pose[0], pose[1])
    skipped = _skipped_waypoints(ctx)
    if position is None or len(last_path) < 2:
        cursor = max(0, min(cursor_prev, len(elements)))
        return cursor, drop_skipped_waypoints(list(elements)[cursor:], skipped)
    driven = executed_path(last_path, position)
    cursor = advance_cursor(elements, cursor_prev, driven, skipped)
    progress = path_fraction(last_path, position)
    if preserve_detours:
        remainder = route_remainder_elements(elements, cursor, last_path, position, progress)
    else:
        remainder = replan_route_elements(elements, cursor, position, progress)
    return cursor, drop_skipped_waypoints(remainder, skipped)


def _apply_resolved_waypoints(
    elements: Sequence[Dict], resolved: Sequence[Sequence[float]]
) -> List[Dict]:
    """Rewrite waypoint coordinates with the planner's effective positions.

    A waypoint standing inside a clearance zone is moved to the closest legal
    point. The stored plan request must use the MOVED points, otherwise a
    later camera re-plan would still target the unreachable original and the
    cursor would never count it as visited.
    """
    if not resolved:
        return list(elements)
    out: List[Dict] = []
    index = 0
    for element in elements:
        if str(element.get("kind")) != "waypoint" or index >= len(resolved):
            out.append(element)
            continue
        point = resolved[index]
        index += 1
        if math.dist((float(element.get("x", 0.0)), float(element.get("y", 0.0))),
                     (float(point[0]), float(point[1]))) <= 0.5:
            out.append(element)
            continue
        moved = dict(element)
        moved["x"] = float(point[0])
        moved["y"] = float(point[1])
        out.append(moved)
    return out


async def execute_plan(
    ctx: MissionContext,
    elements: Sequence[Dict],
    goal: Optional[Dict],
    obstacles: Sequence[Dict],
    options_dict: Optional[Dict] = None,
    deadman_ms: Optional[int] = None,
    quiet_unchanged: bool = False,
) -> Dict:
    """Plan the mission (or re-plan its remainder while executing).

    ``ctx`` is the mission context (:class:`~.interfaces.MissionContext`):
    the executor, obstacle store, last plan/request, pose source and log
    sink - the console's AppState, the brain's runner, or a test double.
    Raises ``ValueError`` when the mission cannot be planned (no geometry,
    nothing left to re-plan, ...). The caller maps that to HTTP 422 or logs it.
    ``quiet_unchanged`` suppresses the "trajectory unchanged" log line - the
    proximity watchdog re-checks while driving and must not spam the console.
    """
    request_elements = list(elements)
    request_goal = goal
    request_obstacles = [dict(obstacle) for obstacle in obstacles]
    request_options = dict(options_dict or {})

    def remember_request(cursor: int = 0) -> None:
        """Store this request as the base for future camera re-plans."""
        ctx.last_plan_request = {
            "elements": request_elements,
            "goal": request_goal,
            "obstacles": request_obstacles,
            "options": request_options,
            "deadman_ms": deadman_ms,
            "signature": elements_signature(request_elements),
            "cursor": int(cursor),
        }

    async with ctx.plan_lock:
        executor = ctx.executor
        mid_run = executor is not None and executor.is_running()
        was_paused = bool(
            mid_run and executor is not None
            and getattr(executor, "state", "") == EXEC_PAUSED
        )
        resume = False
        mode = "route"
        cursor = 0
        route_elements: List[Dict] = []
        if not mid_run and ctx.vision_waiting is not None:
            # a fresh plan (or a manual restart) supersedes a previous wait
            ctx.vision_waiting = None
        if not mid_run:
            # ... and gives every checkpoint a new chance: the obstacle map
            # may have changed since a previous bounded wait skipped one
            ctx.skipped_waypoints = set()

        if mid_run:
            assert executor is not None
            goal_only = request_goal is not None and not request_elements
            # An edited mission goes back to the drawing; a plain obstacle
            # change continues on the route the robot is driving.
            mission_edited = (
                ctx.last_plan_request is None
                or elements_signature(request_elements) != ctx.last_plan_request.get("signature")
            )
            mode = "drawing" if mission_edited else "route"
            # Where is the mission right now? The cursor is the first element
            # the DRIVEN route has not completed; the remainder is what the
            # robot still has to do (the next checkpoint verbatim, or the
            # current stroke's continuation).
            pose_now = ctx.pose()
            if mode == "route":
                cursor, route_elements = _route_progress(
                    ctx, request_elements, preserve_detours=False
                )
                head_wp = _head_waypoint(route_elements)
                previous_wp = _previous_waypoint(request_elements, cursor)
                if (
                    head_wp is not None
                    and pose_now is not None
                    and math.dist(head_wp, (pose_now[0], pose_now[1])) <= WAYPOINT_WAIT_REACH_MM
                    # let the just-visited waypoint finish its arrival turn
                    # and drive off before stopping the robot
                    and not (
                        previous_wp is not None
                        and math.dist(previous_wp, (pose_now[0], pose_now[1])) < 250.0
                    )
                    and _waypoint_blocked(
                        head_wp, active_obstacles(ctx, request_obstacles, request_options), request_options
                    )
                ):
                    # THE next checkpoint is occupied by a moving obstacle:
                    # wait a moment for it to move on instead of "visiting"
                    # the waypoint 27 cm short - the bounded wait resumes by
                    # itself and skips the checkpoint when it stays blocked
                    paused_by_user = executor.state == EXEC_PAUSED
                    ctx.vision_waiting = {
                        "reason": (
                            f"waypoint ({head_wp[0]:.0f}, {head_wp[1]:.0f}) is blocked by the "
                            "moving obstacle"
                        ),
                        "elements": route_elements,
                        "goal": request_goal,
                        "obstacles": request_obstacles,
                        "options": request_options,
                        "deadman_ms": deadman_ms,
                        "paused_by_user": paused_by_user,
                        "direct_mm": 0.0,
                        "started_at": time.monotonic(),
                    }
                    await executor.stop(reason="waiting for the moving obstacle to clear")
                    await executor.wait_idle()
                    remember_request(cursor)
                    _arm_wait_deadline(ctx)
                    await ctx.log(
                        "warn",
                        f"the next waypoint ({head_wp[0]:.0f}, {head_wp[1]:.0f}) mm is blocked by the "
                        f"moving obstacle - waiting up to {WAIT_MAX_S:.0f} s for it to move; the "
                        "robot continues by itself (skipping the waypoint if it stays blocked)",
                    )
                    assert ctx.last_plan is not None
                    return {**ctx.last_plan, "waiting": True}
            # Only interrupt the run when an obstacle NEAR the robot actually
            # affects what is still ahead. Anything farther than the reaction
            # distance is ignored - it may be gone by the time the robot gets
            # there (it usually is another robot); the proximity watchdog
            # re-checks automatically as the robot advances.
            near = nearby_obstacles(
                active_obstacles(ctx, request_obstacles, request_options),
                (pose_now[0], pose_now[1]) if pose_now is not None else None,
                request_options,
            )
            # ---- reaction ladder (cheap, no re-plan yet) -------------------
            # How far along the route is the first REAL blockage (an obstacle
            # the robot's body would drive into)? Far away it is only watched:
            # another robot usually moves on, and re-planning from a distance
            # used to stop the robot for a detour it never needed. Close by the
            # robot creeps instead, which buys the re-plan its time; a blockage
            # in a tight spot (border/static obstacle) is acted on at any
            # distance because re-planning from there has no room left.
            # An EDITED mission is honoured at once (mode "drawing"): the
            # ladder only gates obstacle reactions.
            current_path = remaining_path(
                (ctx.last_plan or {}).get("path") or [],
                (pose_now[0], pose_now[1]) if pose_now is not None else (0.0, 0.0),
            )
            clip_gap, clip_point = clip_gap_mm(
                current_path, near, request_options,
                ctx.field["width_mm"], ctx.field["height_mm"],
            )
            early = clip_gap < math.inf and tight_blockage(
                clip_point, request_obstacles, request_options, ctx.field
            )
            if mode == "route" and clip_gap < math.inf and not early and clip_gap > CAUTION_GAP_MM:
                remember_request(cursor)
                if executor.speed_scale < 1.0:
                    executor.set_speed_scale(1.0)
                    await ctx.log("info", "the route ahead is clear again - full speed")
                if not quiet_unchanged and _note_ladder(ctx, f"far{(clip_gap // 250):.0f}"):
                    await ctx.log(
                        "info",
                        f"obstacle {clip_gap:.0f} mm ahead on the route - watching it "
                        "(no stop until it gets close)",
                    )
                assert ctx.last_plan is not None
                return {**ctx.last_plan, "unchanged": True}
            if mode == "route" and clip_gap < math.inf and not early and clip_gap > STOP_GAP_MM:
                remember_request(cursor)
                if executor.speed_scale > CAUTION_SCALE + 1e-9:
                    executor.set_speed_scale(CAUTION_SCALE)
                    if _note_ladder(ctx, "caution"):
                        await ctx.log(
                            "warn",
                            f"obstacle {clip_gap:.0f} mm ahead on the route - creeping at "
                            f"{CAUTION_SCALE * 100:.0f}% until it either moves away or the "
                            "robot gets close enough to stop and decide",
                        )
                assert ctx.last_plan is not None
                return {**ctx.last_plan, "unchanged": True, "caution": True}
            if executor.speed_scale < 1.0:
                executor.set_speed_scale(1.0)
            ctx.ladder_band = None  # a new approach speaks again
            affected, candidate, current_length = await _assess_remaining_route(
                ctx, executor, request_elements, request_goal, near, request_options, mode,
                cursor=cursor,
                planned_elements=[] if goal_only else (route_elements or None),
            )
            if not affected:
                remember_request(cursor)
                if not quiet_unchanged:
                    await ctx.log(
                        "info",
                        "obstacle change does not affect the remaining route - trajectory unchanged",
                    )
                assert ctx.last_plan is not None
                return {**ctx.last_plan, "unchanged": True}
            wait_allowed = (
                not goal_only
                and not request_obstacles
                and bool(ctx.vision.active())
            )
            wait_reason = (
                _wait_instead_of_detour(ctx, candidate, current_length, near, request_options)
                if wait_allowed else None
            )
            if wait_reason:
                # a MOVING obstacle blocks the way and every detour is much
                # longer than waiting: hold the robot and continue on its own
                # when the route ahead clears (next camera report)
                paused_by_user = executor.state == EXEC_PAUSED
                ctx.vision_waiting = {
                    "reason": wait_reason,
                    "elements": route_elements or executor.remaining_elements(),
                    "goal": request_goal,
                    "obstacles": request_obstacles,
                    "options": request_options,
                    "deadman_ms": deadman_ms,
                    "paused_by_user": paused_by_user,
                    "direct_mm": current_length,
                    "started_at": time.monotonic(),
                }
                await executor.stop(reason="waiting for the moving obstacle to clear")
                await executor.wait_idle()
                remember_request(cursor)
                _arm_wait_deadline(ctx)
                await ctx.log(
                    "warn",
                    "moving obstacle blocks the remaining route - waiting up to "
                    f"{WAIT_MAX_S:.0f} s ({wait_reason}); the robot takes the detour "
                    "by itself when the wait is over",
                )
                assert ctx.last_plan is not None
                return {**ctx.last_plan, "waiting": True}
            resume = executor.state == EXEC_PAUSED
            request_options["resume_mid_mission"] = True
            await executor.stop(reason="re-planning the remaining route (dynamic obstacle)")
            await executor.wait_idle()

        pose = ctx.pose()
        if pose is None:
            raise ValueError("no odometry yet - is the board connected?")
        px, py, pth = pose

        if mid_run and mode == "route":
            # Route slicing above happens before STOP so we can assess the
            # obstacle. The robot may coast while the executor brakes; rebuild
            # the continuation at its final pose or the new plan can drive
            # back to that stale pre-stop cut point.
            if ctx.last_plan_request is not None:
                ctx.last_plan_request["cursor"] = cursor
            cursor, route_elements = _route_progress(
                ctx, request_elements, (px, py), preserve_detours=False
            )

        planned_elements = request_elements
        if not mid_run:
            # the robot may ALREADY stand on the route the drawing starts with
            # (rehearsing a strategy from wherever the last run left it):
            # continue from its position - driving back to the stroke's first
            # point meant two 180 deg pivots and a double pass of the same
            # line ("why is it turning around for a straight line?")
            sliced_at_robot = slice_leading_stroke_at_robot(request_elements, (px, py))
            if sliced_at_robot is not request_elements:
                planned_elements = sliced_at_robot
                await ctx.log(
                    "info",
                    "the robot already stands on this route - the plan continues from "
                    "its current position (not from the drawing's start)",
                )
        if mid_run:
            assert executor is not None
            goal_only = request_goal is not None and not request_elements
            if mode == "drawing":
                # the mission was edited mid-run: slice the new drawing at the
                # robot's position and re-route from there
                progress = 0.5
                last_path = (ctx.last_plan or {}).get("path") or []
                if len(last_path) >= 2:
                    progress = path_fraction(last_path, (px, py))
                sliced = slice_remaining_elements(
                    request_elements, (px, py), progress
                )
                planned_elements = sliced if sliced else executor.remaining_elements()
            else:
                # A close-waypoint DEPARTURE may be interrupted by this
                # re-plan. Finish the reverse first: the executor tail starts
                # with the remaining signed DRIVE to the turn-safe point.
                # Routing the drawing from the half-reversed pose turned the
                # body right next to the wall it had just stopped beside
                # (2026-10-19).
                departure_left = 0.0
                remaining_departure = getattr(executor, "remaining_departure_mm", None)
                if callable(remaining_departure):
                    departure_left = float(remaining_departure((px, py)) or 0.0)
                if abs(departure_left) >= 1.0 and not goal_only:
                    planned_elements = executor.remaining_elements((px, py))
                else:
                    # continue on the mission from its CURSOR: the next checkpoint
                    # verbatim (shortest route to it - a waypoint a detour
                    # bypassed is picked up again) or the current stroke cut at
                    # the robot, with the route's detours kept
                    planned_elements = [] if goal_only else route_elements
                    if not planned_elements and not goal_only:
                        planned_elements = drop_skipped_waypoints(
                            continue_route_elements(
                                request_elements, (ctx.last_plan or {}).get("path") or [], (px, py), 0.5,
                                start_index=cursor,
                            ),
                            _skipped_waypoints(ctx),
                        )
                    if not planned_elements and not goal_only:
                        planned_elements = (
                            executor.remaining_elements((px, py))
                            if callable(remaining_departure)
                            else executor.remaining_elements()
                        )
                    if not planned_elements and not goal_only:
                        planned_elements = slice_remaining_elements(
                            request_elements, (px, py), 0.5
                        )
            if not planned_elements and request_goal is None:
                raise ValueError(
                    "nothing left to re-plan (the mission is almost over) - obstacle ignored"
                )

        options = PlanOptions.from_dict(request_options)
        merged_obstacles = active_obstacles(ctx, request_obstacles, request_options)

        def enter_moving_obstacle_wait(reason: str, direct_mm: float = 0.0) -> None:
            """Stop here, but let a camera change retry by itself."""
            begin_moving_obstacle_wait(
                ctx,
                elements=planned_elements,
                goal=request_goal,
                obstacles=request_obstacles,
                options=request_options,
                deadman_ms=deadman_ms,
                paused=was_paused,
                direct_mm=direct_mm,
                reason=reason,
            )

        try:
            result = await _plan_off_loop(
                elements=planned_elements,
                start_pose=(px, py, pth),
                goal=request_goal,
                obstacles=merged_obstacles,
                options=options,
                field_width_mm=ctx.field["width_mm"],
                field_height_mm=ctx.field["height_mm"],
            )
        except ValueError:
            if mid_run and executor is not None and ctx.vision.active():
                enter_moving_obstacle_wait(
                    "the moving obstacle blocks the remaining route"
                )
                await ctx.log(
                    "warn",
                    "the moving obstacle blocks the remaining route - waiting for "
                    "it to move; the robot resumes by itself",
                )
                return {"waiting": True}
            raise
        clip = first_body_clip(
            result.get("path") or [], merged_obstacles,
            *body_geometry(options),
            ctx.field["width_mm"], ctx.field["height_mm"],
        ) if mid_run else None
        if clip is not None:
            # Is the first overlap caused by a MOVING obstacle (vision)?
            # Then do not dead-end the mission: wait and retry when the
            # camera reports the obstacle gone. This is the case where the
            # opponent drove onto the robot: every immediate re-plan still
            # starts inside its clearance and used to leave the robot
            # stopped forever ("robot remains stopped").
            dynamic_hit = [
                obstacle for obstacle in ctx.vision.active()
                if obstacle_distance_mm(obstacle, clip[1]) <= rotation_clearance(options)
            ]
            if dynamic_hit and executor is not None:
                enter_moving_obstacle_wait(
                    "the moving obstacle blocks the robot",
                    float(result["stats"]["length_mm"]),
                )
                await ctx.log(
                    "warn",
                    "the moving obstacle is on the remaining route and no clear "
                    "re-plan exists - waiting for it to move; the robot resumes "
                    "by itself",
                )
                return {**result, "waiting": True}
            raise ValueError(
                "replanned path still intersects an obstacle's robot-clearance area; "
                "the robot remains stopped"
            )
        # a waypoint the planner moved out of a clearance zone is the point
        # the cursor and later re-plans must use (the original is unreachable)
        request_elements = _apply_resolved_waypoints(
            request_elements, result.get("resolved_waypoints") or []
        )
        ctx.last_plan = result
        # remember the console request: a camera report arriving mid-run
        # re-plans from exactly these elements, not from the previous plan
        remember_request(cursor)

        if mid_run:
            assert executor is not None
            if getattr(executor, "last_stream", False) and len(result.get("path") or []) < 2:
                # the remaining route collapsed to the robot's own position:
                # there is nothing left to drive. Restarting the follower on a
                # one-point path used to raise "follow_path needs at least two
                # points" and the camera re-plan retried forever (2026-10-18).
                await ctx.log(
                    "info",
                    "re-plan: the remaining route is already at the robot - nothing left to drive",
                )
                return {**result, "done": True}
            failure = await _start_verified(
                ctx,
                result["segments"],
                deadman_ms,
                resume,
                "re-plan restart",
                follow=result.get("path") if getattr(executor, "last_stream", False) else None,
                follow_bounds=follow_bounds(
                    ctx.field["width_mm"], ctx.field["height_mm"], options
                ),
            )
            if failure is not None:
                raise ValueError(f"re-plan computed but not restarted: {failure}")
            await ctx.log(
                "warn",
                "dynamic obstacle - the robot stopped at "
                f"({px:.0f}, {py:.0f}) mm and the remaining route was re-planned "
                f"({result['stats']['n_segments']} segments): "
                + ("PAUSED - press Resume when the table is clear" if resume else "it re-routes and continues"),
            )
            # the console map must show the REGENERATED route the robot will
            # actually drive (not the old one): push the new plan document
            await ctx.emit({"type": "plan", "data": result})

    await ctx.log(
        "info",
        f"plan ready: {result['stats']['n_segments']} segments "
        f"({result['stats']['n_gotos']} GOTO, {result['stats']['n_actions']} action), "
        f"{result['stats']['length_mm'] / 1000.0:.2f} m, ~{result['stats']['est_s']:.0f} s",
    )
    return result


async def _assess_remaining_route(
    ctx: MissionContext,
    executor,
    elements: Sequence[Dict],
    goal: Optional[Dict],
    obstacles: Sequence[Dict],
    options_dict: Optional[Dict],
    mode: str,
    cursor: int = 0,
    planned_elements: Optional[Sequence[Dict]] = None,
) -> Tuple[bool, Optional[Dict], float]:
    """Would the new obstacle map actually change what the robot has to drive?

    Returns ``(affected, candidate_plan_or_None, current_remaining_length)``.
    A candidate re-plan of the remainder - the mission from its ``cursor``
    (mode ``route``: the next uncompleted elements, the next checkpoint
    verbatim or the current stroke's continuation) or the drawing sliced at
    the robot (mode ``drawing``: the mission was edited) - is built on the
    NEW obstacle map. The route is re-planned when the current route
    physically drives through an obstacle, when the candidate proves the
    blockage, or when the mission's NEXT WAYPOINT is no longer really
    reached (the candidate reaches it, the current route misses it: a
    compromised approach is picked up again as soon as the waypoint frees
    up). A mere LENGTH difference without a clip or a missed waypoint is
    re-fit noise and leaves the robot driving.
    """
    last_plan = ctx.last_plan or {}
    path = last_plan.get("path") or []
    pose = ctx.pose()
    if pose is None or len(path) < 2:
        return (True, None, 0.0)  # no ruler to compare against: be conservative
    position = (pose[0], pose[1])
    options = PlanOptions.from_dict({**(options_dict or {}), "resume_mid_mission": True})
    merged = list(obstacles)  # the caller already merged + proximity-filtered
    field_w, field_h = ctx.field["width_mm"], ctx.field["height_mm"]

    if mode == "drawing":
        progress = path_fraction(path, position)
        candidate_elements = slice_remaining_elements(elements, position, progress)
    elif planned_elements is not None:
        candidate_elements = [dict(element) for element in planned_elements]
    else:
        candidate_elements = drop_skipped_waypoints(
            continue_route_elements(
                elements, path, position, path_fraction(path, position), start_index=cursor
            ),
            _skipped_waypoints(ctx),
        )
        if not candidate_elements:
            candidate_elements = executor.remaining_elements()
    if not candidate_elements and goal is None:
        return (False, None, 0.0)  # nothing left to re-plan anyway
    current_path = remaining_path(path, position)
    current_length = g.polyline_length(current_path)
    try:
        candidate = await _plan_off_loop(
            elements=candidate_elements,
            start_pose=pose,
            goal=goal,
            obstacles=merged,
            options=options,
            field_width_mm=field_w,
            field_height_mm=field_h,
        )
    except ValueError:
        # cannot even plan what is left: stop only when it is physically blocked
        affected = route_blocked(current_path, merged, options, field_w, field_h)
        return (affected, None, current_length)
    candidate_length = candidate["stats"]["length_mm"]
    tolerance = max(100.0, 0.02 * current_length)
    # does the CURRENT route physically drive through an obstacle? That must
    # never stay unnoticed, whatever the length difference says: a small
    # obstacle can be avoided with a detour only a few cm longer than the
    # drawn line (below the tolerance) and the robot would otherwise drive
    # straight into it until it reaches the clearance zone.
    if _clips_obstacle(current_path, merged, options, field_w, field_h):
        return (True, candidate, current_length)
    # A length difference alone matters only when the CANDIDATE clips: a
    # mid-run re-plan re-fits the SAME corridor (ARCs are re-derived from
    # sampled points) and changes the drawn length by a few percent without
    # changing what the robot drives. Without this gate the proximity
    # watchdog swapped the whole program ~1 s after every start and the
    # executed path drifted away from the planned one.
    if abs(candidate_length - current_length) > tolerance and _clips_obstacle(
        candidate.get("path") or [], merged, options, field_w, field_h
    ):
        return (True, candidate, current_length)
    # the mission's NEXT WAYPOINT must still be really visited: while an
    # obstacle blocks it, both routes miss it (quiet - the blocked-waypoint
    # wait handles that case); as soon as the candidate reaches it again,
    # the stale compromise is replaced
    next_wp = _next_waypoint(elements, cursor)
    if next_wp is not None:
        current_gap = route_distance_mm(current_path, next_wp)
        candidate_gap = route_distance_mm(candidate.get("path") or [], next_wp)
        if current_gap > WAYPOINT_VISIT_TOL_MM and candidate_gap <= WAYPOINT_VISIT_TOL_MM:
            return (True, candidate, current_length)
    return (False, candidate, current_length)


def _wait_instead_of_detour(
    ctx: MissionContext,
    candidate: Optional[Dict],
    current_length: float,
    obstacles: Sequence[Dict],
    options_dict: Optional[Dict],
) -> Optional[str]:
    """Why is standing still better than re-routing? ``None`` = drive around.

    A MOVING obstacle (another robot) is different from a box: it costs
    nothing to let it pass. So the robot only re-routes when the candidate
    plan avoids the obstacle at body clearance AND the detour is not much
    longer than the route it replaces; otherwise it stops and waits for the
    next camera report.
    """
    if candidate is None:
        return None  # let the normal re-plan path speak (it decides/reports)
    options = PlanOptions.from_dict({**(options_dict or {}), "resume_mid_mission": True})
    merged = list(obstacles)  # the caller already merged + proximity-filtered
    field_w, field_h = ctx.field["width_mm"], ctx.field["height_mm"]
    if _clips_obstacle(candidate.get("path") or [], merged, options, field_w, field_h):
        return "there is no way around it (every detour still drives through it)"
    length = float(candidate["stats"]["length_mm"])
    if current_length > 0.0 and length > max(
        WAIT_DETOUR_RATIO * current_length, current_length + WAIT_DETOUR_SLACK_MM
    ):
        return (
            f"the shortest detour is {length / current_length:.1f}x the remaining route "
            f"({length:.0f} mm vs {current_length:.0f} mm)"
        )
    return None


def _clips_obstacle(
    path: Sequence, merged: Sequence[Dict], options: PlanOptions, field_w: float, field_h: float
) -> bool:
    """Does the route drive THROUGH an obstacle (physical clearance)?

    Pure geometry, sampled along the route - NOT a grid line-of-sight test.
    The body is checked at the local travel heading, so a close-waypoint
    straight final leg is legal even though its centre is inside the
    half-diagonal circle; the check still catches a real overlap.
    """
    points = [(float(p[0]), float(p[1])) for p in path if len(p) >= 2]
    if len(points) < 2 or not merged:
        return False
    width, height = body_geometry(options)
    return first_body_clip(
        points, merged, width, height, field_w, field_h,
    ) is not None


async def resume_from_wait(ctx: MissionContext) -> bool:
    """Continue a mission that was waiting for a moving obstacle to clear.

    Called on every camera change (and by the bounded-wait timer) while
    ``ctx.vision_waiting`` is set. Re-plans the route captured at the stop
    point on the CURRENT obstacle map; when the route is free again - or the
    detour is finally reasonable - the run restarts (still paused if the
    user had paused it). ``WAIT_MAX_S`` after the wait began the wait is
    RESOLVED no matter what: a checkpoint that is still blocked and has no
    alternative path is skipped (the mission continues with the next
    waypoint) and a long detour is accepted (waiting is never forever).
    """
    async with ctx.plan_lock:
        waiting = ctx.vision_waiting
        executor = ctx.executor
        if waiting is None or executor is None or executor.is_running():
            return False
        elements = waiting.get("elements") or []
        request = ctx.last_plan_request or {}
        if waiting.get("goal") is not None and not request.get("elements"):
            # A goal-only run has no route constraints between the robot and
            # its target. The wait record may contain executor.remaining_elements()
            # captured from the old route; combining that stale stroke with the
            # goal makes every retry retrace the same blocked path.
            elements = []
            waiting["elements"] = []
        if not elements and waiting.get("goal") is None:
            ctx.vision_waiting = None
            return False
        pose = ctx.pose()
        if pose is None:
            return False
        options = PlanOptions.from_dict({**(waiting.get("options") or {}), "resume_mid_mission": True})
        merged = nearby_obstacles(
            active_obstacles(ctx, waiting.get("obstacles") or [], waiting.get("options") or {}),
            (pose[0], pose[1]),
            waiting.get("options") or {},
        )
        field_w, field_h = ctx.field["width_mm"], ctx.field["height_mm"]
        forced = time.monotonic() - float(waiting.get("started_at") or 0.0) >= WAIT_MAX_S
        candidate: Optional[Dict] = None
        # the waypoint this wait is about must have FREED UP: an opponent
        # still parked on it keeps the wait - until the bounded wait expires,
        # then the checkpoint is either reached by another path or skipped
        head_wp = _head_waypoint(elements)
        if head_wp is not None and _waypoint_blocked(
            head_wp,
            active_obstacles(ctx, waiting.get("obstacles") or [], waiting.get("options") or {}),
            waiting.get("options") or {},
        ):
            if not forced:
                return False  # give the moving obstacle a moment to move off
            reachable = False
            try:
                probe = await _plan_off_loop(
                    elements=elements,
                    start_pose=pose,
                    goal=waiting.get("goal"),
                    obstacles=merged,
                    options=options,
                    field_width_mm=field_w,
                    field_height_mm=field_h,
                )
                probe_path = probe.get("path") or []
                reachable = (
                    not _clips_obstacle(probe_path, merged, options, field_w, field_h)
                    and route_distance_mm(probe_path, head_wp) <= WAYPOINT_VISIT_TOL_MM
                )
            except ValueError:
                reachable = False
            if reachable:
                candidate = probe  # another path reaches it: go for it now
            else:
                # no alternative path to the checkpoint in a reasonable time:
                # give it up and continue with the NEXT waypoint
                _skipped_waypoints(ctx).add((float(head_wp[0]), float(head_wp[1])))
                elements = drop_skipped_waypoints(elements, _skipped_waypoints(ctx))
                await ctx.log(
                    "warn",
                    f"waypoint ({head_wp[0]:.0f}, {head_wp[1]:.0f}) mm is still blocked after "
                    f"{WAIT_MAX_S:.0f} s and no alternative path reaches it - skipping it "
                    "and continuing with the next waypoint",
                )
                if not elements and waiting.get("goal") is None:
                    ctx.vision_waiting = None
                    await ctx.log("warn", "nothing left after the skipped waypoint - mission ends here")
                    return False
        if candidate is None:
            try:
                candidate = await _plan_off_loop(
                    elements=elements,
                    start_pose=pose,
                    goal=waiting.get("goal"),
                    obstacles=merged,
                    options=options,
                    field_width_mm=field_w,
                    field_height_mm=field_h,
                )
            except ValueError:
                return False  # still nothing plannable: keep waiting
        if _clips_obstacle(candidate.get("path") or [], merged, options, field_w, field_h):
            return False  # physically impassable right now: keep waiting
        direct = float(waiting.get("direct_mm") or 0.0)
        length = float(candidate["stats"]["length_mm"])
        if (
            not forced
            and direct > 0.0
            and length > max(WAIT_DETOUR_RATIO * direct, direct + WAIT_DETOUR_SLACK_MM)
        ):
            return False  # a huge detour is still worse than waiting - until the deadline
        ctx.last_plan = candidate
        executor.set_speed_scale(1.0)  # the wait is over: drive the plan as planned
        # clear the wait BEFORE the restart: the state must not keep saying
        # "waiting" while the resumed run is already moving (the caller polls
        # for exactly that); a failed start puts it back below
        ctx.vision_waiting = None
        failure = await _start_verified(
            ctx,
            candidate["segments"],
            waiting.get("deadman_ms"),
            bool(waiting.get("paused_by_user")),
            "waiting resume",
            follow=candidate.get("path") if getattr(executor, "last_stream", False) else None,
            follow_bounds=follow_bounds(field_w, field_h, options),
        )
        if failure is not None:
            # the resume DID NOT survive: keep the wait state - the bounded
            # timer retries, and after WAIT_RESUME_MAX_FAILS it says so
            # instead of sitting on "waiting" forever with nothing driving
            fails = int(waiting.get("fails") or 0) + 1
            waiting["fails"] = fails
            ctx.vision_waiting = waiting
            if fails >= WAIT_RESUME_MAX_FAILS:
                ctx.vision_waiting = None
                await ctx.log(
                    "error",
                    f"the mission stays stopped: the resumed run keeps aborting "
                    f"({failure}) - fix the board (clear the fault), then press Execute",
                )
            else:
                await ctx.log("error", f"the resumed run aborted ({failure}) - still waiting")
            return False
        ctx.vision_waiting = None
        await ctx.log(
            "info",
            f"the route is clear again - resuming the mission ({candidate['stats']['n_segments']} segments)"
            + (" (stays PAUSED - press Resume)" if waiting.get("paused_by_user") else ""),
        )
        # the console map shows the resumed route like every re-plan
        await ctx.emit({"type": "plan", "data": candidate})
        return True
