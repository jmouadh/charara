"""Robot footprint clearances and the occupancy grid (pure geometry).

The centre follows the path, so the body sweeps a circle of half-diagonal
radius: that radius keeps the robot inside the field and obstacles get an
extra ``obstacle_margin_mm``. The full margin is the routing clearance; the
body clearance is only the last-resort tier.

Waypoints are STOP points with their own, configurable clearance
(``waypoint_margin_mm``): 0 means the robot may stop as close as half its
larger body side. Because a close waypoint cannot be reached by the routing
grid (it lies inside the routing margin) and turning in place next to an
obstacle sweeps the body corners, the planner approaches those points along
a straight final leg after performing the arrival rotation at a safe
stand-off. :func:`body_clear_at` / :func:`straight_approach_clear` check the
real rotated body, not just its bounding circle.

File:    software/core/navigation/clearance.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-19
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

from .grid import OccupancyGrid
from .options import PlanOptions

#: Small pad kept around the rotated body at a close waypoint [mm]. The
#: configured clearance is the hard centre-distance floor; 0 is intentional:
#: `waypoint_margin_mm = 0` means "reach the physical limit", and a positive
#: pad silently pulled the operator's exact stop 0.5 mm inwards.
BODY_PAD_MM = 0.0
#: Path samples are serialised to 0.1 mm. A waypoint placed exactly on the
#: stop clearance can read back a few hundredths of a millimetre inside the
#: obstacle/border; tolerate that in the audit, but never a real overlap.
CLIP_TOLERANCE_MM = 0.25
#: Clearance needed to rotate in place: the body sweeps its half-diagonal.
ROTATION_PAD_MM = 6.0
#: Steps used when the rotated body still clips at the minimum clearance.
_CLEARANCE_STEP_MM = 1.0


def body_geometry(options: PlanOptions) -> Tuple[float, float]:
    """Validated ``(width, height)`` of the robot body [mm]."""
    return (
        max(50.0, float(options.robot_width_mm)),
        max(50.0, float(options.robot_height_mm)),
    )


def half_diagonal(options: PlanOptions) -> float:
    """Radius of the circle the body sweeps when it turns in place [mm]."""
    width, height = body_geometry(options)
    return math.hypot(width, height) * 0.5


def waypoint_clearance(options: PlanOptions) -> float:
    """Minimum centre-to-obstacle / centre-to-border distance for a waypoint.

    ``waypoint_margin_mm is None`` keeps the historical behaviour: a stop
    point is projected all the way out of the routing clearance
    (``half_diagonal + obstacle_margin_mm``). A configured value (including
    0) switches to the physical stop-point rule: half the robot's larger
    body side plus the configured extra margin. With a 300 x 300 mm body,
    0 mm therefore allows a waypoint 150 mm from an obstacle or the border.
    """
    width, height = body_geometry(options)
    half_body = 0.5 * max(width, height)
    configured = options.waypoint_margin_mm
    if configured is None:
        return half_diagonal(options) + max(0.0, float(options.obstacle_margin_mm))
    return half_body + max(0.0, float(configured))


def rotation_clearance(options: PlanOptions) -> float:
    """Clearance needed around a point to turn the robot in place [mm]."""
    return half_diagonal(options) + ROTATION_PAD_MM


def obstacle_clearance(point: Sequence[float], obstacles: Sequence[Dict]) -> float:
    """Free distance from ``point`` to the nearest obstacle surface [mm]."""
    x, y = float(point[0]), float(point[1])
    distance = math.inf
    for obstacle in obstacles:
        kind = obstacle.get("kind", "rect")
        if kind == "rect":
            ox, oy = float(obstacle.get("x", 0.0)), float(obstacle.get("y", 0.0))
            ow, oh = float(obstacle.get("w", 0.0)), float(obstacle.get("h", 0.0))
            if ow <= 0.0 or oh <= 0.0:
                continue
            dx = max(ox - x, 0.0, x - (ox + ow))
            dy = max(oy - y, 0.0, y - (oy + oh))
            distance = min(distance, math.hypot(dx, dy))
        elif kind == "circle":
            cx, cy = float(obstacle.get("x", 0.0)), float(obstacle.get("y", 0.0))
            radius = max(0.0, float(obstacle.get("r", 0.0)))
            distance = min(distance, math.hypot(x - cx, y - cy) - radius)
    return distance


def point_clearance(
    point: Sequence[float],
    obstacles: Sequence[Dict],
    width: float,
    height: float,
) -> float:
    """Free distance from ``point`` to the nearest border or obstacle [mm].

    The obstacle distance is measured to its real shape (not the inflated
    grid), so it can be compared directly with the physical body dimensions.
    """
    x, y = float(point[0]), float(point[1])
    distance = min(x, y, width - x, height - y)
    return max(0.0, min(distance, obstacle_clearance(point, obstacles)))


def _body_corners(
    x: float, y: float, theta_deg: float, width: float, height: float
) -> List[Tuple[float, float]]:
    """Corners of the robot rectangle at ``(x, y)`` facing ``theta_deg``."""
    rad = math.radians(theta_deg)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    hw, hh = width * 0.5, height * 0.5
    corners: List[Tuple[float, float]] = []
    for lx, ly in ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)):
        corners.append((x + lx * cos_a - ly * sin_a, y + lx * sin_a + ly * cos_a))
    return corners


def body_clear_at(
    point: Sequence[float],
    theta_deg: float,
    obstacles: Sequence[Dict],
    width: float,
    height: float,
    field_width_mm: float,
    field_height_mm: float,
    pad_mm: float = BODY_PAD_MM,
) -> bool:
    """Does the ROTATED body at ``point``/``theta`` stay clear?

    Borders are checked on the four corners (the rectangle is convex), rect
    obstacles with a separating-axis test and circles by distance to the
    rotated rectangle. This is the check that makes a close approach safe:
    the routing grid only knows the half-diagonal circle, which is too
    conservative for a waypoint and exactly right for an in-place turn.
    """
    x, y = float(point[0]), float(point[1])
    rad = math.radians(float(theta_deg))
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    hw, hh = width * 0.5, height * 0.5
    if not (0.0 <= x <= field_width_mm and 0.0 <= y <= field_height_mm):
        return False
    for cx, cy in _body_corners(x, y, theta_deg, width, height):
        if (
            cx < pad_mm
            or cy < pad_mm
            or cx > field_width_mm - pad_mm
            or cy > field_height_mm - pad_mm
        ):
            return False
    # The body axes and the world axes are the only separating axes for an
    # oriented rectangle vs an axis-aligned rectangle.
    axes = ((cos_a, sin_a), (-sin_a, cos_a), (1.0, 0.0), (0.0, 1.0))
    for obstacle in obstacles:
        kind = obstacle.get("kind", "rect")
        if kind == "rect":
            ox, oy = float(obstacle.get("x", 0.0)), float(obstacle.get("y", 0.0))
            ow, oh = float(obstacle.get("w", 0.0)), float(obstacle.get("h", 0.0))
            if ow <= 0.0 or oh <= 0.0:
                continue
            x0, y0 = ox - pad_mm, oy - pad_mm
            x1, y1 = ox + ow + pad_mm, oy + oh + pad_mm
            separated = False
            for ax, ay in axes:
                centre = x * ax + y * ay
                radius = hw * abs(cos_a * ax + sin_a * ay) + hh * abs(-sin_a * ax + cos_a * ay)
                projections = (x0 * ax + y0 * ay, x1 * ax + y0 * ay,
                               x0 * ax + y1 * ay, x1 * ax + y1 * ay)
                if centre + radius <= min(projections) or centre - radius >= max(projections):
                    separated = True
                    break
            if not separated:
                return False
        elif kind == "circle":
            cx, cy = float(obstacle.get("x", 0.0)), float(obstacle.get("y", 0.0))
            radius = max(0.0, float(obstacle.get("r", 0.0))) + pad_mm
            dx, dy = cx - x, cy - y
            # circle centre in the robot frame
            lx = dx * cos_a + dy * sin_a
            ly = -dx * sin_a + dy * cos_a
            nearest_x = min(max(lx, -hw), hw)
            nearest_y = min(max(ly, -hh), hh)
            if math.hypot(lx - nearest_x, ly - nearest_y) <= radius:
                return False
    return True


def straight_approach_clear(
    start: Sequence[float],
    end: Sequence[float],
    theta_deg: float,
    obstacles: Sequence[Dict],
    width: float,
    height: float,
    field_width_mm: float,
    field_height_mm: float,
    step_mm: float = 5.0,
) -> bool:
    """Is the whole straight final leg clear for the body held at ``theta``?"""
    ax, ay = float(start[0]), float(start[1])
    bx, by = float(end[0]), float(end[1])
    length = math.hypot(bx - ax, by - ay)
    steps = max(1, int(math.ceil(length / max(1.0, step_mm))))
    for index in range(steps + 1):
        t = index / steps
        point = (ax + (bx - ax) * t, ay + (by - ay) * t)
        if not body_clear_at(
            point, theta_deg, obstacles, width, height,
            field_width_mm, field_height_mm,
        ):
            return False
    return True


def first_body_clip(
    points: Sequence[Sequence[float]],
    obstacles: Sequence[Dict],
    width: float,
    height: float,
    field_width_mm: float,
    field_height_mm: float,
    pad_mm: float = BODY_PAD_MM,
    sample_step_mm: float = 25.0,
) -> Optional[Tuple[float, Tuple[float, float]]]:
    """First point of a polyline whose ROTATED body touches something.

    Returns ``(distance_along, point)`` or ``None`` when the whole path is
    clear. The body heading at each sample is the local travel direction -
    the pose a differential-drive robot holds while following the route - so
    a close-waypoint final leg (straight into an obstacle, body aligned with
    it) is legal even though its centre is inside the half-diagonal circle.
    At a vertex with a real heading change the robot may rotate in place
    there; that is only accepted when the centre has full half-diagonal
    clearance, which is exactly what the planner guarantees for stand-offs
    and turn-safe stops.
    """
    pts = [(float(p[0]), float(p[1])) for p in points if len(p) >= 2]
    if len(pts) < 2:
        return None
    half = 0.5 * math.hypot(width, height)
    # negative pad: shrink obstacles and borders by the serialisation
    # tolerance before the body test
    check_pad = pad_mm - CLIP_TOLERANCE_MM
    walked = 0.0
    previous_heading: Optional[float] = None
    for index in range(1, len(pts)):
        a, b = pts[index - 1], pts[index]
        span = math.dist(a, b)
        if span <= 1e-9:
            continue
        heading = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        if previous_heading is not None:
            turn = abs((heading - previous_heading + 180.0) % 360.0 - 180.0)
            # A path-direction REVERSAL is the close-waypoint departure:
            # the robot keeps its body heading and drives backwards, so
            # there is no in-place rotation to check. (A real U-turn at a
            # tight point is planned as an explicit TURN, handled by the
            # planner itself.)
            if 1.0 < turn < 170.0 and point_clearance(
                a, obstacles, field_width_mm, field_height_mm,
            ) < half - 1e-6:
                # A small heading change on a sampled curve is a MOVING turn:
                # accept it when the body fits at both headings. A big break
                # means the robot would rotate in place there.
                if turn > 30.0:
                    return (walked, a)
                if not body_clear_at(
                    a, previous_heading, obstacles, width, height,
                    field_width_mm, field_height_mm, pad_mm=check_pad,
                ) or not body_clear_at(
                    a, heading, obstacles, width, height,
                    field_width_mm, field_height_mm, pad_mm=check_pad,
                ):
                    return (walked, a)
        previous_heading = heading
        steps = max(1, int(math.ceil(span / max(1.0, sample_step_mm))))
        for step in range(steps + 1):
            t = step / steps
            sample = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            if not body_clear_at(
                sample, heading, obstacles, width, height,
                field_width_mm, field_height_mm, pad_mm=check_pad,
            ):
                return (walked + span * t, sample)
        walked += span
    return None


def closest_body_clear_point(
    point: Sequence[float],
    theta_deg: float,
    obstacles: Sequence[Dict],
    minimum_mm: float,
    width: float,
    height: float,
    field_width_mm: float,
    field_height_mm: float,
) -> Tuple[float, float]:
    """Closest point satisfying the configured clearance AND the rotated body.

    ``closest_clear_point`` first applies the raw centre-distance rule; when
    the body is diagonal to an obstacle or border that is not enough (a
    corner reaches farther than half the body side). The point is then
    pushed out with a slightly larger effective clearance until the real
    rectangle fits. The search is bounded by the half-diagonal: beyond that
    the bounding circle is clear for every heading.
    """
    best = closest_clear_point(
        point, obstacles, minimum_mm, minimum_mm, field_width_mm, field_height_mm,
    )
    if body_clear_at(
        best, theta_deg, obstacles, width, height, field_width_mm, field_height_mm,
    ):
        return best
    limit = minimum_mm
    maximum = 0.5 * math.hypot(width, height) + ROTATION_PAD_MM
    while limit < maximum - 1e-9:
        limit = min(maximum, limit + _CLEARANCE_STEP_MM)
        candidate = closest_clear_point(
            point, obstacles, limit, limit, field_width_mm, field_height_mm,
        )
        if body_clear_at(
            candidate, theta_deg, obstacles, width, height,
            field_width_mm, field_height_mm,
        ):
            return candidate
    return best


def clearances(options: PlanOptions) -> Tuple[float, float]:
    """(border clearance, obstacle inflation) from the robot footprint.

    The centre follows the path, so the body sweeps a circle of half-diagonal
    radius: that radius keeps the robot inside the field, and obstacles get an
    extra ``obstacle_margin_mm``. ``inflate_mm > 0`` overrides the inflation.
    """
    width = max(50.0, float(options.robot_width_mm))
    height = max(50.0, float(options.robot_height_mm))
    half_diagonal = math.hypot(width, height) * 0.5
    if options.inflate_mm and options.inflate_mm > 0:
        return half_diagonal, float(options.inflate_mm)
    margin = max(0.0, float(options.obstacle_margin_mm))
    return half_diagonal, half_diagonal + margin


def body_inflation(options: PlanOptions) -> float:
    """The physical obstacle clearance: half-diagonal + a small buffer.

    Last-resort tier only: the router prefers the full configured margin and
    falls back to this physical clearance when no full route exists.
    """
    border, _ = clearances(options)
    return border + max(30.0, options.grid_mm * 0.6)


def closest_clear_point(
    point: Sequence[float],
    obstacles: Sequence[Dict],
    inflate_mm: float,
    border_mm: float,
    width: float,
    height: float,
    iterations: int = 4,
) -> Tuple[float, float]:
    """Closest point that keeps the FULL clearance around every obstacle.

    Used for STOP points (waypoints, goals): the robot comes to rest there
    and may turn in place, so the configured obstacle margin must hold - a
    point inside the inflated zone is pushed out to its boundary exactly
    (Euclidean, rounded corners), then the field border is re-applied.
    Overlapping obstacles are resolved by a few iterations; a residual
    violation (a pocket between boxes) is left to the caller, which can fall
    back to the occupancy grid.
    """
    low_x, high_x = border_mm, max(border_mm, width - border_mm)
    low_y, high_y = border_mm, max(border_mm, height - border_mm)
    x = min(max(float(point[0]), low_x), high_x)
    y = min(max(float(point[1]), low_y), high_y)
    limit = max(0.0, float(inflate_mm))
    for _ in range(max(1, iterations)):
        moved = False
        for obstacle in obstacles:
            kind = obstacle.get("kind", "rect")
            if kind == "rect":
                ox, oy = float(obstacle.get("x", 0.0)), float(obstacle.get("y", 0.0))
                ow, oh = float(obstacle.get("w", 0.0)), float(obstacle.get("h", 0.0))
                if ow <= 0.0 or oh <= 0.0:
                    continue
                nx = min(max(x, ox), ox + ow)
                ny = min(max(y, oy), oy + oh)
                dx, dy = x - nx, y - ny
                distance = math.hypot(dx, dy)
                if distance >= limit:
                    continue
                if distance > 1e-6:
                    x = nx + dx / distance * limit
                    y = ny + dy / distance * limit
                else:
                    # inside the box: leave through the nearest side
                    left, right = x - ox, ox + ow - x
                    bottom, top = y - oy, oy + oh - y
                    nearest = min(left, right, bottom, top)
                    if nearest == left:
                        x = ox - limit
                    elif nearest == right:
                        x = ox + ow + limit
                    elif nearest == bottom:
                        y = oy - limit
                    else:
                        y = oy + oh + limit
                moved = True
            elif kind == "circle":
                cx, cy = float(obstacle.get("x", 0.0)), float(obstacle.get("y", 0.0))
                radius = max(0.0, float(obstacle.get("r", 0.0)))
                reach = radius + limit
                dx, dy = x - cx, y - cy
                distance = math.hypot(dx, dy)
                if distance >= reach:
                    continue
                if distance <= 1e-6:
                    dx, dy, distance = 1.0, 0.0, 1.0
                x = cx + dx / distance * reach
                y = cy + dy / distance * reach
                moved = True
        if not moved:
            break
    return (
        min(max(x, low_x), high_x),
        min(max(y, low_y), high_y),
    )


def build_grid(
    width: float,
    height: float,
    obstacles: Sequence[Dict],
    options: PlanOptions,
    inflation_mm: Optional[float] = None,
) -> OccupancyGrid:
    """Occupancy grid of the merged obstacles with the robot clearances."""
    border, inflation = clearances(options)
    if inflation_mm is not None:
        inflation = max(border, min(float(inflation_mm), inflation))
    grid = OccupancyGrid(width, height, options.grid_mm, inflation, border_mm=border)
    for obstacle in obstacles:
        kind = obstacle.get("kind", "rect")
        if kind == "rect":
            grid.add_rect(float(obstacle["x"]), float(obstacle["y"]), float(obstacle["w"]), float(obstacle["h"]))
        elif kind == "circle":
            grid.add_circle(float(obstacle["x"]), float(obstacle["y"]), float(obstacle["r"]))
    return grid

