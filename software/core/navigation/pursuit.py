"""Pure Pursuit path following: geometry only, no I/O (Track B, Docs/10).

The follower backend of the streaming executor: given the planned path (a
polyline in field millimetres) and the live pose, it computes the next
velocity command (v forward, omega yaw rate) towards a lookahead point.
Everything here is a pure function so the whole controller is unit-testable
against :mod:`.sim` without a robot.

Units and conventions:
* positions/radii in mm, speeds in mm/s, poses ``(x_mm, y_mm, theta_deg)``
  with theta CCW positive (the project convention);
* curvature in 1/mm; ``omega = v * kappa`` comes out in rad/s;
* the speed-dependent lookahead and the lateral-acceleration budget follow
  the classic formulation::

      L      = clamp(k_l * v, L_min, L_max)
      kappa  = 2 * sin(alpha) / L        (alpha = angle to the lookahead)
      omega  = v * kappa
      v      = min(v_cruise, sqrt(a_lat / |kappa|))

  and near the path end ``v`` additionally honours the braking profile
  ``sqrt(2 * decel * remaining)`` so the robot arrives stopped, not flying.

File:    software/core/navigation/pursuit.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-09
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]


@dataclass(frozen=True)
class PursuitParams:
    """Tuning of the pure-pursuit law (all optional in the callers)."""

    #: lookahead time constant: L = k_l * v (s); 0.5 keeps the steady-state
    #: cut-in on a 300 mm circle at 300 mm/s around 10 mm (L^2 / 8R)
    k_l: float = 0.5
    #: lookahead bounds [mm]
    l_min: float = 100.0
    l_max: float = 350.0
    #: lateral acceleration budget on curves [mm/s^2]
    lat_accel: float = 300.0
    #: never command less than this while moving [mm/s]
    v_min: float = 100.0
    #: braking deceleration used near the end of the path [mm/s^2]
    decel: float = 700.0
    #: arrival tolerance: closer than this to the last point = done [mm];
    #: the executor's final correction handles anything above 12 mm
    stop_tol_mm: float = 10.0
    #: a lookahead this far off the robot heading means "turn first": driving
    #: forward would take the robot AWAY from the path (the real-robot bug of
    #: 2026-10-18: the plan started behind the robot, the follower drove
    #: straight out of the field). The follower rotates in place until the
    #: target is inside this cone, then streams normally.
    align_deg: float = 30.0
    #: in-place alignment law: omega = clamp(align_gain * error, +-max)
    align_gain: float = 2.5
    align_max_omega_rad_s: float = 2.5  # ~143 deg/s (firmware caps at 3.0)
    #: a run whose live pose is this far from the whole planned path is not
    #: started: re-plan from where the robot really stands instead of driving
    #: across the table (follower safety check)
    start_off_path_mm: float = 300.0


DEFAULT_PARAMS = PursuitParams()


@dataclass(frozen=True)
class PursuitCommand:
    """One controller output (what the follower sends on the wire)."""

    v_mm_s: float
    omega_rad_s: float
    target: Point
    kappa: float
    remaining_mm: float
    #: True while the follower is rotating in place to face the path
    turn_only: bool = False


def distance_to_path(points: Sequence[Point], pose: Sequence[float]) -> float:
    """Shortest distance from ``pose`` to the path polyline [mm]."""
    px, py = float(pose[0]), float(pose[1])
    if not points:
        return math.inf
    if len(points) == 1:
        return math.hypot(px - points[0][0], py - points[0][1])
    best = math.inf
    for a, b in zip(points, points[1:]):
        seg = math.dist(a, b)
        if seg <= 1e-9:
            continue
        t = ((px - a[0]) * (b[0] - a[0]) + (py - a[1]) * (b[1] - a[1])) / (seg * seg)
        t = max(0.0, min(1.0, t))
        best = min(best, math.hypot(px - (a[0] + (b[0] - a[0]) * t),
                                    py - (a[1] + (b[1] - a[1]) * t)))
    return best


def path_length(points: Sequence[Point]) -> float:
    """Total polyline length [mm]."""
    return sum(math.dist(points[i], points[i + 1]) for i in range(len(points) - 1))


def _project_on_segment(pose: Point, a: Point, b: Point) -> Tuple[float, float, float]:
    """Projection of ``pose`` on segment a->b.

    Returns ``(along_mm, distance_mm, t)``: the arc length from ``a`` to the
    projection, the perpendicular distance and the segment parameter in
    [0, 1].
    """
    ax, ay = a
    bx, by = b
    seg = math.dist(a, b)
    if seg <= 1e-9:
        return 0.0, math.dist(pose, a), 0.0
    t = ((pose[0] - ax) * (bx - ax) + (pose[1] - ay) * (by - ay)) / (seg * seg)
    t = max(0.0, min(1.0, t))
    px = ax + (bx - ax) * t
    py = ay + (by - ay) * t
    return seg * t, math.hypot(pose[0] - px, pose[1] - py), t


def nearest_projection(points: Sequence[Point], pose: Point) -> Tuple[int, float]:
    """Closest point on the path to ``pose``.

    Returns ``(index, along_mm)``: the index of the segment START and the
    arc length from the path start to the projection, so the caller can
    measure the remaining distance and walk the lookahead from there.
    """
    best_index = 0
    best_dist = math.inf
    best_along = 0.0
    walked = 0.0
    for i in range(len(points) - 1):
        along, dist, _t = _project_on_segment(pose, points[i], points[i + 1])
        if dist < best_dist:
            best_dist = dist
            best_index = i
            best_along = walked + along
        walked += math.dist(points[i], points[i + 1])
    if len(points) == 1:
        return 0, 0.0
    return best_index, best_along


def remaining_distance(points: Sequence[Point], pose: Point) -> float:
    """Distance still to drive from ``pose`` to the end of the path [mm]."""
    if len(points) < 2:
        return math.dist(pose, points[-1]) if points else 0.0
    _, along = nearest_projection(points, pose)
    return max(0.0, path_length(points) - along)


def lookahead_target(points: Sequence[Point], pose: Point, lookahead_mm: float) -> Point:
    """First path point at least ``lookahead_mm`` ahead of the robot.

    The robot is projected on the path, then the lookahead distance is
    walked forward segment by segment (interpolating inside one segment).
    Clamps to the last point when the lookahead overruns the path.
    """
    if len(points) == 1:
        return points[0]
    index, along = nearest_projection(points, pose)
    needed = along + max(0.0, lookahead_mm)
    walked = 0.0
    for i in range(len(points) - 1):
        seg = math.dist(points[i], points[i + 1])
        if walked + seg >= needed:
            t = 0.0 if seg <= 1e-9 else (needed - walked) / seg
            ax, ay = points[i]
            bx, by = points[i + 1]
            return (ax + (bx - ax) * t, ay + (by - ay) * t)
        walked += seg
    return points[-1]


def lookahead_distance(v_mm_s: float, params: PursuitParams = DEFAULT_PARAMS) -> float:
    """Speed-dependent lookahead L = clamp(k_l * v, L_min, L_max) [mm]."""
    return max(params.l_min, min(params.l_max, params.k_l * abs(v_mm_s)))


def max_curvature_ahead(
    points: Sequence[Point], pose: Sequence[float], horizon_mm: float
) -> float:
    """Largest path curvature magnitude within ``horizon_mm`` ahead [1/mm].

    The feed-forward of the speed law: a purely reactive pursuit only slows
    AT a curve and cuts the corner; scanning the upcoming window lets the
    follower brake BEFORE the curve, which is what keeps the tracking error
    of a 90 deg fillet inside the acceptance budget (Docs/10 section 6.3).
    """
    from .velocity import curvature_at

    if len(points) < 3:
        return 0.0
    index, along = nearest_projection(points, (pose[0], pose[1]))
    lengths = [math.dist(points[i], points[i + 1]) for i in range(len(points) - 1)]
    walked_before = sum(lengths[:index])
    budget = horizon_mm - max(0.0, lengths[index] - max(0.0, along - walked_before))
    best = 0.0
    i = index + 1
    while i < len(points) and budget > 0.0:
        best = max(best, curvature_at(points, i))
        if i < len(lengths):
            budget -= lengths[i]
        i += 1
    return best


def pursuit_curvature(pose: Sequence[float], target: Point) -> Tuple[float, float]:
    """Curvature command towards ``target``: returns ``(kappa [1/mm], d [mm])``.

    ``kappa = 2 * sin(alpha) / d`` with ``alpha`` the angle from the robot
    heading to the target direction and ``d`` the actual target distance
    (using the true distance instead of the nominal lookahead keeps the law
    stable when the target clamps to the path end).
    """
    theta = math.radians(float(pose[2]))
    dx = target[0] - float(pose[0])
    dy = target[1] - float(pose[1])
    d = math.hypot(dx, dy)
    if d <= 1e-9:
        return 0.0, 0.0
    # signed lateral component of the target direction in the robot frame
    sin_alpha = (math.cos(theta) * dy - math.sin(theta) * dx) / d
    return 2.0 * sin_alpha / d, d


def speed_for_curvature(
    kappa: float,
    v_cruise: float,
    params: PursuitParams = DEFAULT_PARAMS,
) -> float:
    """Largest speed whose lateral acceleration fits the budget [mm/s]."""
    if abs(kappa) <= 1e-12:
        return v_cruise
    v_curve = math.sqrt(params.lat_accel / abs(kappa))
    return max(params.v_min, min(v_cruise, v_curve))


def pursuit_step(
    points: Sequence[Point],
    pose: Sequence[float],
    v_cruise: float,
    params: PursuitParams = DEFAULT_PARAMS,
) -> Optional[PursuitCommand]:
    """One Pure Pursuit step: None when the path end is reached.

    Combines the braking profile, the curvature feed-forward (brake before
    a curve, not on it), the speed-dependent lookahead and the curvature
    law. Call it every control tick with the latest fused pose.

    When the lookahead target sits outside the ``align_deg`` cone around the
    robot heading, the step becomes a TURN IN PLACE (``v = 0``, omega towards
    the target, ``turn_only = True``): a pure-pursuit law with a target
    almost behind the robot commands a tiny curvature and full forward speed,
    which drove the real robot straight ahead - out of the field - instead of
    turning around (2026-10-18). Rotating first is always the safe answer.
    """
    if not points:
        return None
    remaining = remaining_distance(points, pose)
    if remaining <= params.stop_tol_mm:
        return None
    # braking profile: arrive slowed down, then stop_tol ends the run
    v_plan = min(abs(v_cruise), math.sqrt(2.0 * params.decel * remaining))
    v_plan = max(params.v_min, v_plan)
    # feed-forward: slow down for the sharpest curve inside the combined
    # braking + lookahead window, so the corner is entered at curve speed
    horizon = v_plan * v_plan / (2.0 * params.decel) + lookahead_distance(v_plan, params)
    kappa_ahead = max_curvature_ahead(points, pose, horizon)
    v_plan = speed_for_curvature(kappa_ahead, v_plan, params)
    look = lookahead_target(points, pose, lookahead_distance(v_plan, params))
    kappa, distance_mm = pursuit_curvature(pose, look)
    if distance_mm > 1e-9:
        theta = math.radians(float(pose[2]))
        dx = look[0] - float(pose[0])
        dy = look[1] - float(pose[1])
        cos_alpha = (math.cos(theta) * dx + math.sin(theta) * dy) / distance_mm
        alpha = math.atan2(kappa * distance_mm / 2.0, max(-1.0, min(1.0, cos_alpha)))
        if abs(math.degrees(alpha)) > params.align_deg:
            omega = max(-params.align_max_omega_rad_s,
                        min(params.align_max_omega_rad_s, params.align_gain * alpha))
            return PursuitCommand(
                v_mm_s=0.0,
                omega_rad_s=omega,
                target=look,
                kappa=kappa,
                remaining_mm=remaining,
                turn_only=True,
            )
    v = speed_for_curvature(kappa, v_plan, params)
    return PursuitCommand(
        v_mm_s=v,
        omega_rad_s=v * kappa,
        target=look,
        kappa=kappa,
        remaining_mm=remaining,
    )


def resample(points: Sequence[Point], step_mm: float) -> List[Point]:
    """Uniformly resample a polyline every ``step_mm`` (helpers for tests).

    Walks the polyline emitting a point every ``step_mm`` of arc length and
    always keeps the final vertex; used to build dense reference paths
    (circles, S-curves) for the simulator.
    """
    if len(points) < 2 or step_mm <= 0.0:
        return list(points)
    out: List[Point] = [(float(points[0][0]), float(points[0][1]))]
    acc = 0.0  # distance walked since the last emitted point
    for i in range(len(points) - 1):
        ax, ay = float(points[i][0]), float(points[i][1])
        bx, by = float(points[i + 1][0]), float(points[i + 1][1])
        seg = math.hypot(bx - ax, by - ay)
        if seg <= 1e-9:
            continue
        pos = 0.0
        while acc + (seg - pos) >= step_mm:
            pos += step_mm - acc
            t = pos / seg
            out.append((ax + (bx - ax) * t, ay + (by - ay) * t))
            acc = 0.0
        acc += seg - pos
    last = (float(points[-1][0]), float(points[-1][1]))
    if math.hypot(out[-1][0] - last[0], out[-1][1] - last[1]) > 1e-6:
        out.append(last)
    return out
