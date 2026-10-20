"""Re-plan helpers for a running mission (pure).

Slicing the drawn mission at the robot position, cutting the driven part
off a path, and the physical-clearance mapping used by the mid-run obstacle
reactions. These functions operate on plain dicts/lists - no service, no
I/O - so the re-plan logic stays portable and unit-testable.

File:    software/core/navigation/replan.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

from . import geometry as g
from .clearance import body_geometry, first_body_clip
from .geometry import Point
from .options import PlanOptions

def slice_remaining_elements(
    elements: Sequence[Dict],
    position: Tuple[float, float],
    progress: float = 0.5,
) -> List[Dict]:
    """The part of a mission still ahead of ``position`` (mid-run re-plan).

    The DRAWN mission is the source of truth: the stroke the robot is on is
    trimmed at the point nearest to it, everything before that stroke is
    dropped and every following element (strokes, waypoints, rotations,
    pauses) is kept verbatim. A re-plan therefore re-routes the SAME drawing
    around whatever appeared on the table - and repeated re-plans can never
    drift away from it (they always come back to the drawing, not to the
    previous plan's resampled nodes). ``progress`` (0..1, how much of the
    mission is already driven) only decides between candidates when the
    mission crosses itself. Returns ``[]`` when there is nothing to slice.
    """
    px, py = float(position[0]), float(position[1])
    strokes: List[Dict] = []
    cum = 0.0
    previous_end: Optional[Point] = None
    for index, element in enumerate(elements):
        if element.get("kind", "stroke") != "stroke":
            continue
        pts = [(float(p[0]), float(p[1])) for p in element.get("points", []) if len(p) >= 2]
        if len(pts) < 2:
            continue
        if previous_end is not None:
            cum += math.dist(previous_end, pts[0])  # connector leg to this stroke
        strokes.append({"index": index, "pts": pts, "cum": cum, "length": g.polyline_length(pts)})
        cum += strokes[-1]["length"]
        previous_end = pts[-1]
    if not strokes:
        return []
    want = max(0.0, min(1.0, float(progress))) * cum
    best: Optional[Tuple[float, Dict, float]] = None
    for entry in strokes:
        nearest = _nearest_on_polyline(entry["pts"], (px, py))
        if nearest is None:
            continue
        distance, along = nearest
        # a small progress prior breaks ties on self-crossing missions while
        # still favouring the branch the robot is physically closest to
        cost = distance + 0.2 * abs(entry["cum"] + along - want)
        if best is None or cost < best[0]:
            best = (cost, entry, along)
    if best is None:
        return []
    _, entry, along = best
    sliced: List[Dict] = []
    remainder = _polyline_cut_at(entry["pts"], along)
    if len(remainder) >= 2:
        sliced.append({"kind": "stroke", "points": [[p[0], p[1]] for p in remainder]})
    sliced.extend(dict(element) for element in elements[entry["index"] + 1 :])
    return sliced


#: The robot this close to its route counts as STANDING ON it.
ROUTE_ON_TOL_MM = 60.0
#: A projection closer than this to the route start counts as "starts here".
ROUTE_EARLY_MM = 25.0


def slice_leading_stroke_at_robot(
    elements: Sequence[Dict],
    position: Optional[Tuple[float, float]],
    tol_mm: float = ROUTE_ON_TOL_MM,
    early_mm: float = ROUTE_EARLY_MM,
) -> List[Dict]:
    """Continue the drawing from the robot when it already STANDS on it.

    A fresh mission is planned from the robot's position - normally that is a
    connector leg to the first stroke's first point. But when the robot is
    already ON that stroke (rehearsing a saved strategy from wherever the
    last run left it) driving back to the start and re-driving the line is
    absurd: it showed up as "the robot turns around twice for a straight
    line" (two 180 deg pivots plus a double pass of the same segment).

    In that case the leading stroke is cut at the robot's projection and the
    plan continues from there; every following element is kept verbatim, and
    a robot at the very end of the leading stroke leaves only those. Only a
    stroke as the FIRST element is considered, and a projection within
    ``early_mm`` of the stroke start is left alone (the mission starts at
    the robot anyway). Returns the SAME object (identity) when nothing
    changes, so callers can detect a slice cheaply.
    """
    if not elements or position is None:
        return elements
    first = elements[0]
    if str(first.get("kind", "stroke")) != "stroke":
        return elements
    pts = _stroke_points(first)
    if len(pts) < 2:
        return elements
    nearest = _nearest_on_polyline(pts, (float(position[0]), float(position[1])))
    if nearest is None:
        return elements
    distance, along = nearest
    if distance > tol_mm or along <= early_mm:
        return elements
    remainder = _polyline_cut_at(pts, along)
    sliced: List[Dict] = []
    if len(remainder) >= 2 and g.polyline_length(remainder) >= 2.0:
        # (a robot at the very end leaves no drivable stroke - just the rest)
        cut_stroke = dict(first)
        cut_stroke["points"] = [[float(p[0]), float(p[1])] for p in remainder]
        sliced.append(cut_stroke)
    sliced.extend(dict(element) for element in elements[1:])
    return sliced


def path_fraction(points: Sequence[Point], position: Tuple[float, float]) -> float:
    """Where ``position`` sits along a path, as a fraction (0..1) of its length."""
    if len(points) < 2:
        return 0.5
    nearest = _nearest_on_polyline(
        [(float(p[0]), float(p[1])) for p in points], (float(position[0]), float(position[1]))
    )
    total = g.polyline_length(points)
    if nearest is None or total <= 1e-6:
        return 0.5
    return max(0.0, min(1.0, nearest[1] / total))


REJOIN_TOL_MM = 25.0
WAYPOINT_VISIT_TOL_MM = 80.0   # a waypoint is VISITED when the driven route swept this close
STROKE_END_TOL_MM = 60.0       # a stroke is DONE when the driven route reached its end


def _stroke_points(element: Dict) -> List[Point]:
    """The point list of one stroke element."""
    return [(float(p[0]), float(p[1])) for p in element.get("points", []) if len(p) >= 2]


EXECUTED_SNAP_TOL_MM = 25.0   # tie window when a route passes near itself


def executed_path(points: Sequence[Point], position: Tuple[float, float]) -> List[Point]:
    """The part of a path already driven: from its start to the robot's projection.

    Projections are ambiguous where a route crosses itself (or passes near a
    later leg): the EARLIEST candidate within ``EXECUTED_SNAP_TOL_MM`` of the
    closest distance wins. Under-cutting only re-schedules a little work,
    while cutting late would mark unvisited waypoints as visited.
    """
    if len(points) < 2:
        return [(float(p[0]), float(p[1])) for p in points]
    cleaned = [(float(p[0]), float(p[1])) for p in points]
    px, py = float(position[0]), float(position[1])
    best_dist = float("inf")
    projections: List[Tuple[float, float]] = []  # (along, distance)
    cum = 0.0
    for i in range(1, len(cleaned)):
        ax, ay = cleaned[i - 1]
        bx, by = cleaned[i]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 <= 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
        qx, qy = ax + dx * t, ay + dy * t
        dist = math.hypot(px - qx, py - qy)
        segment = math.sqrt(l2)
        projections.append((cum + segment * t, dist))
        best_dist = min(best_dist, dist)
        cum += segment
    along = min(
        (a for a, d in projections if d <= best_dist + EXECUTED_SNAP_TOL_MM),
        default=0.0,
    )
    driven: List[Point] = [cleaned[0]]
    remaining = along
    for i in range(1, len(cleaned)):
        ax, ay = cleaned[i - 1]
        bx, by = cleaned[i]
        segment = math.dist((ax, ay), (bx, by))
        if segment <= 1e-9:
            continue
        if remaining <= segment:
            t = remaining / segment
            driven.append((ax + (bx - ax) * t, ay + (by - ay) * t))
            return driven
        driven.append((bx, by))
        remaining -= segment
    return driven


def route_distance_mm(points: Sequence[Point], point: Tuple[float, float]) -> float:
    """Distance from ``point`` to a route polyline (inf when the route is empty)."""
    if len(points) < 2:
        if len(points) == 1:
            return math.dist((float(points[0][0]), float(points[0][1])), point)
        return float("inf")
    nearest = _nearest_on_polyline(
        [(float(p[0]), float(p[1])) for p in points], (float(point[0]), float(point[1]))
    )
    return float("inf") if nearest is None else nearest[0]


def advance_cursor(
    elements: Sequence[Dict],
    cursor: int,
    driven: Sequence[Point],
    skipped: Sequence[Point] = (),
) -> int:
    """Index of the first mission element the driven route has NOT completed.

    Waypoints count as visited when the driven polyline swept within
    ``WAYPOINT_VISIT_TOL_MM``. A robot blocked ON a waypoint makes the plan
    approach short of it - that does NOT visit it, so a later re-plan picks
    it up again instead of ignoring it forever. ``skipped`` holds waypoints
    the mission logic GAVE UP on after a bounded wait (still blocked, no
    alternative path): they count as completed so the cursor moves past
    them and the next checkpoint becomes the target. Strokes are done when
    their end was reached; a pause / rotation / manual step is done when the
    next MOTION element is done. The cursor only moves forward.
    """

    def is_skipped(point: Point) -> bool:
        return any(math.dist(point, (float(s[0]), float(s[1]))) <= 1.0 for s in skipped)

    def completed(index: int) -> bool:
        """Has the driven route completed the mission element at ``index``?"""
        while index < len(elements):
            element = elements[index]
            kind = str(element.get("kind", "stroke"))
            if kind == "waypoint":
                point = (float(element.get("x", 0.0)), float(element.get("y", 0.0)))
                if is_skipped(point):
                    return True  # given up after the bounded wait: move on
                return route_distance_mm(driven, point) <= WAYPOINT_VISIT_TOL_MM
            if kind == "stroke":
                pts = _stroke_points(element)
                if len(pts) < 2:
                    index += 1
                    continue
                return route_distance_mm(driven, pts[-1]) <= STROKE_END_TOL_MM
            index += 1  # actions: done when the next motion element is done
        return True

    index = max(0, min(int(cursor), len(elements)))
    while index < len(elements) and completed(index):
        index += 1
    return index


def drop_skipped_waypoints(
    elements: Sequence[Dict], skipped: Sequence[Point]
) -> List[Dict]:
    """The mission elements without the waypoints given up on.

    A bounded wait can declare a checkpoint unreachable (still blocked, no
    alternative path): it must then never be re-targeted by a later
    remainder, so every remainder passes through this filter.
    """
    if not skipped:
        return [dict(element) for element in elements]
    out: List[Dict] = []
    for element in elements:
        if str(element.get("kind", "stroke")) == "waypoint":
            point = (float(element.get("x", 0.0)), float(element.get("y", 0.0)))
            if any(math.dist(point, (float(s[0]), float(s[1]))) <= 1.0 for s in skipped):
                continue
        out.append(dict(element))
    return out


def route_remainder_elements(
    elements: Sequence[Dict],
    cursor: int,
    path: Sequence[Point],
    position: Tuple[float, float],
    progress: float = 0.5,
) -> List[Dict]:
    """The mission elements still ahead of ``cursor``, continuing the route.

    ``cursor`` is the index of the first element the driven route has NOT
    completed (see :func:`advance_cursor`). Two shapes:

    * the next element is a waypoint / pause / rotation - the mission's
      immediate target is a POINT: the rest of the mission is returned
      verbatim, so the planner heads for that checkpoint directly (the
      shortest sensible route around the CURRENT obstacles) instead of
      preserving a stale detour - a waypoint bypassed by an earlier detour
      is picked up again here instead of being ignored forever;
    * the next element is a STROKE: continue on the route actually driven -
      the planned path is cut at the robot and the CURRENT stroke (and only
      it) takes over again where the route rejoins it, so a crossing with a
      LATER stroke of the mission can never make the remainder jump over
      unvisited waypoints.

    Returns ``[]`` when there is nothing left to continue.
    """
    if not elements:
        return []
    cursor = max(0, min(int(cursor), len(elements)))
    if cursor >= len(elements):
        return []
    if str(elements[cursor].get("kind", "stroke")) != "stroke":
        return [dict(element) for element in elements[cursor:]]
    continued = continue_route_elements(elements, path, position, progress, start_index=cursor)
    return continued


def replan_route_elements(
    elements: Sequence[Dict],
    cursor: int,
    position: Tuple[float, float],
    progress: float = 0.5,
) -> List[Dict]:
    """Rebuild the remaining mission from its drawing, not the old detour.

    Dynamic obstacle replans should route the current unfinished stroke anew
    from the robot's live position. Rejoining the prior executed polyline
    preserves its old detours and can create a long loop when a new obstacle
    blocks that route. Only the current stroke is sliced; later mission
    elements remain verbatim so waypoints and actions are not skipped.
    """
    if not elements:
        return []
    cursor = max(0, min(int(cursor), len(elements)))
    if cursor >= len(elements):
        return []
    remaining = [dict(element) for element in elements[cursor:]]
    if str(remaining[0].get("kind", "stroke")) != "stroke":
        return remaining
    current = slice_remaining_elements(remaining[:1], position, progress)
    return current + remaining[1:]


def continue_route_elements(
    elements: Sequence[Dict],
    path: Sequence[Point],
    position: Tuple[float, float],
    progress: float = 0.5,
    start_index: int = 0,
) -> List[Dict]:
    """Planner elements that CONTINUE the route the robot is driving.

    The last plan's path - detours included - is cut at the robot; where the
    cut rejoins the CURRENT mission stroke (the element at ``start_index``,
    which must be a stroke) the drawing takes over again (its remaining
    elements, actions included). Only THAT stroke may match: a route that
    merely CROSSES a later stroke must not make the remainder jump over
    unvisited waypoints (the "waypoints ignored one by one" bug). Never
    routes backwards: the cut starts at the robot's projection. Returns
    ``[]`` when there is nothing to continue.
    """
    if len(path) < 2 or not elements:
        return []
    start_index = max(0, min(int(start_index), len(elements) - 1))
    head = elements[start_index]
    if str(head.get("kind", "stroke")) != "stroke":
        return []
    stroke_pts = _stroke_points(head)
    if len(stroke_pts) < 2:
        return []
    cut = remaining_path(path, position)
    if len(cut) < 2:
        return []
    join = _rejoin_index(cut, [stroke_pts])
    if join is not None:
        resume_at = cut[join]
        prefix = cut[: join + 1]
    else:
        # the route never rides the stroke again: restart the stroke at the
        # robot's projection onto the drawing (a plain connector)
        nearest = _nearest_on_polyline(stroke_pts, (float(position[0]), float(position[1])))
        if nearest is None:
            return []
        resume_at = _polyline_cut_at(stroke_pts, nearest[1])[0]
        prefix = []
    along = _nearest_on_polyline(stroke_pts, resume_at)
    if along is None:
        return []
    stroke_rest = _polyline_cut_at(stroke_pts, along[1])
    out: List[Dict] = []
    if len(prefix) >= 2:
        out.append({"kind": "stroke", "points": [[float(x), float(y)] for x, y in prefix]})
    if len(stroke_rest) >= 2:
        out.append({"kind": "stroke", "points": [[float(x), float(y)] for x, y in stroke_rest]})
    out.extend(dict(element) for element in elements[start_index + 1 :])
    return out


def _rejoin_index(cut: Sequence[Point], strokes: Sequence[Sequence[Point]]) -> Optional[int]:
    """Where does the cut get back on the drawn mission? (index in ``cut``)

    ``None`` when the cut never runs along the drawing. One sample near a
    stroke is not enough: the point AND the next one must both lie on the
    drawing, so a route that merely CROSSES it on its way around an obstacle
    does not count as a rejoin.
    """
    for index, point in enumerate(cut):
        if not _near_strokes(point, strokes):
            continue
        if index + 1 < len(cut):
            if _near_strokes(cut[index + 1], strokes):
                return index
        else:
            return index
    return None


def _near_strokes(point: Point, strokes: Sequence[Sequence[Point]]) -> bool:
    """Is ``point`` within the rejoin tolerance of any drawn stroke?"""
    for pts in strokes:
        nearest = _nearest_on_polyline(pts, (float(point[0]), float(point[1])))
        if nearest is not None and nearest[0] <= REJOIN_TOL_MM:
            return True
    return False


def remaining_path(points: Sequence[Point], position: Tuple[float, float]) -> List[Point]:
    """The part of a path still ahead of ``position`` (cut at its nearest point)."""
    if len(points) < 2:
        return [(float(p[0]), float(p[1])) for p in points]
    cleaned = [(float(p[0]), float(p[1])) for p in points]
    nearest = _nearest_on_polyline(cleaned, (float(position[0]), float(position[1])))
    if nearest is None:
        return cleaned
    return _polyline_cut_at(cleaned, nearest[1])


def route_blocked(
    points: Sequence[Point],
    obstacles: Sequence[Dict],
    options: "PlanOptions",
    field_width_mm: float,
    field_height_mm: float,
) -> bool:
    """True when a leg of ``points`` violates the physical body clearance.

    Used to decide whether a new obstacle map actually affects what the robot
    still has to drive: obstacles behind the robot or far from the remaining
    route leave it unblocked.
    """
    if len(points) < 2:
        return False
    width, height = body_geometry(options)
    return first_body_clip(
        points, obstacles, width, height, field_width_mm, field_height_mm,
    ) is not None


def _nearest_on_polyline(points: Sequence[Point], p: Point) -> Optional[Tuple[float, float]]:
    """Distance from ``p`` to a polyline and the arc length at that point."""
    if len(points) < 2:
        return None
    best_d = float("inf")
    best_a = 0.0
    cum = 0.0
    for i in range(1, len(points)):
        ax, ay = points[i - 1]
        bx, by = points[i]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 <= 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2))
        qx, qy = ax + dx * t, ay + dy * t
        d = math.hypot(p[0] - qx, p[1] - qy)
        segment = math.sqrt(l2)
        if d < best_d:
            best_d = d
            best_a = cum + segment * t
        cum += segment
    return (best_d, best_a)


def _polyline_cut_at(points: Sequence[Point], along: float) -> List[Point]:
    """Polyline points at/after arc length ``along`` (the cut point interpolated)."""
    remaining = max(0.0, along)
    for i in range(1, len(points)):
        ax, ay = points[i - 1]
        bx, by = points[i]
        segment = math.dist((ax, ay), (bx, by))
        if segment <= 1e-9:
            continue
        if remaining <= segment:
            t = remaining / segment
            cut = (ax + (bx - ax) * t, ay + (by - ay) * t)
            return [cut] + list(points[i:])
        remaining -= segment
    return [points[-1]]


