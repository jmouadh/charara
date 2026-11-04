"""Polygon obstacles - the custom shape drawn point by point in the console.

The console closes the operator's points automatically (the last vertex is
joined to the first), so the service always receives an explicit, closed
vertex list. Everything the planner needs about such a shape lives here: the
inside test, the distance to the outline and the rectangle/polygon distance
behind the rotated-body check. Keeping them in one module means the clearance
code and the occupancy grid cannot disagree about a drawn shape.

File:    software/core/navigation/polygon.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-23
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

Point = Tuple[float, float]

#: A polygon needs at least three vertices to enclose an area.
MIN_VERTICES = 3
#: Below this area (mm^2) the points are a mis-click, not an obstacle.
MIN_AREA_MM2 = 25.0


def vertices(obstacle: Dict) -> List[Point]:
    """Usable ``(x, y)`` vertices of a polygon obstacle ([] when invalid)."""
    points: List[Point] = []
    for raw in obstacle.get("points") or []:
        try:
            points.append((float(raw[0]), float(raw[1])))
        except (TypeError, ValueError, IndexError):
            continue
    return points if len(points) >= MIN_VERTICES else []


def clean_vertices(points: Sequence[Sequence[float]]) -> List[Point]:
    """Parsed, de-duplicated vertices (consecutive duplicates removed)."""
    cleaned: List[Point] = []
    for raw in points or []:
        try:
            point = (float(raw[0]), float(raw[1]))
        except (TypeError, ValueError, IndexError):
            continue
        if not (math.isfinite(point[0]) and math.isfinite(point[1])):
            continue
        if cleaned and math.dist(cleaned[-1], point) < 0.5:
            continue
        cleaned.append(point)
    while len(cleaned) > 1 and math.dist(cleaned[0], cleaned[-1]) < 0.5:
        cleaned.pop()
    return cleaned


def area_mm2(points: Sequence[Point]) -> float:
    """Shoelace area of the closed polygon [mm^2]."""
    if len(points) < MIN_VERTICES:
        return 0.0
    total = 0.0
    for index, (x, y) in enumerate(points):
        nx, ny = points[(index + 1) % len(points)]
        total += x * ny - nx * y
    return abs(total) * 0.5


def is_usable(points: Sequence[Point]) -> bool:
    """True when the vertices really enclose an area."""
    return len(points) >= MIN_VERTICES and area_mm2(points) >= MIN_AREA_MM2


def centroid(points: Sequence[Point]) -> Point:
    """Mean of the vertices - only used to pick a push-out direction."""
    if not points:
        return (0.0, 0.0)
    return (
        sum(p[0] for p in points) / len(points),
        sum(p[1] for p in points) / len(points),
    )


def point_in_polygon(x: float, y: float, points: Sequence[Point]) -> bool:
    """Ray casting inside test (the outline itself counts as inside)."""
    count = len(points)
    if count < MIN_VERTICES:
        return False
    inside = False
    for index in range(count):
        ax, ay = points[index]
        bx, by = points[(index + 1) % count]
        if _point_on_segment(x, y, ax, ay, bx, by):
            return True
        if (ay > y) != (by > y):
            crossing = ax + (y - ay) * (bx - ax) / (by - ay)
            if crossing > x:
                inside = not inside
    return inside


def point_segment_distance(
    x: float, y: float, ax: float, ay: float, bx: float, by: float
) -> float:
    """Shortest distance from a point to a segment [mm]."""
    dx, dy = bx - ax, by - ay
    span = dx * dx + dy * dy
    if span <= 1e-12:
        return math.hypot(x - ax, y - ay)
    t = ((x - ax) * dx + (y - ay) * dy) / span
    t = min(1.0, max(0.0, t))
    return math.hypot(x - (ax + t * dx), y - (ay + t * dy))


def distance_to_polygon(x: float, y: float, points: Sequence[Point]) -> float:
    """Distance from a point to the shape (0 when inside or on the outline)."""
    if len(points) < MIN_VERTICES:
        return math.inf
    if point_in_polygon(x, y, points):
        return 0.0
    best = math.inf
    count = len(points)
    for index in range(count):
        ax, ay = points[index]
        bx, by = points[(index + 1) % count]
        best = min(best, point_segment_distance(x, y, ax, ay, bx, by))
    return best


def nearest_on_polygon(x: float, y: float, points: Sequence[Point]) -> Point:
    """Closest point of the outline to ``(x, y)``."""
    best: Point = (points[0][0], points[0][1]) if points else (x, y)
    best_distance = math.inf
    count = len(points)
    for index in range(count):
        ax, ay = points[index]
        bx, by = points[(index + 1) % count]
        distance = point_segment_distance(x, y, ax, ay, bx, by)
        if distance >= best_distance:
            continue
        best_distance = distance
        dx, dy = bx - ax, by - ay
        span = dx * dx + dy * dy
        if span <= 1e-12:
            best = (ax, ay)
            continue
        t = ((x - ax) * dx + (y - ay) * dy) / span
        t = min(1.0, max(0.0, t))
        best = (ax + t * dx, ay + t * dy)
    return best


def segments_intersect(
    ax: float, ay: float, bx: float, by: float,
    cx: float, cy: float, dx: float, dy: float,
) -> bool:
    """Do the two closed segments cross or touch?"""

    def orient(px: float, py: float, qx: float, qy: float, rx: float, ry: float) -> float:
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)

    o1 = orient(ax, ay, bx, by, cx, cy)
    o2 = orient(ax, ay, bx, by, dx, dy)
    o3 = orient(cx, cy, dx, dy, ax, ay)
    o4 = orient(cx, cy, dx, dy, bx, by)
    if (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0):
        return True
    if abs(o1) <= 1e-9 and _point_on_segment(cx, cy, ax, ay, bx, by):
        return True
    if abs(o2) <= 1e-9 and _point_on_segment(dx, dy, ax, ay, bx, by):
        return True
    if abs(o3) <= 1e-9 and _point_on_segment(ax, ay, cx, cy, dx, dy):
        return True
    if abs(o4) <= 1e-9 and _point_on_segment(bx, by, cx, cy, dx, dy):
        return True
    return False


def segment_distance(
    ax: float, ay: float, bx: float, by: float,
    cx: float, cy: float, dx: float, dy: float,
) -> float:
    """Shortest distance between two segments (0 when they cross)."""
    if segments_intersect(ax, ay, bx, by, cx, cy, dx, dy):
        return 0.0
    return min(
        point_segment_distance(ax, ay, cx, cy, dx, dy),
        point_segment_distance(bx, by, cx, cy, dx, dy),
        point_segment_distance(cx, cy, ax, ay, bx, by),
        point_segment_distance(dx, dy, ax, ay, bx, by),
    )


def polygon_rect_distance(corners: Sequence[Point], points: Sequence[Point]) -> float:
    """Distance between a (convex) rectangle and the polygon, 0 on overlap.

    Works for convex AND concave polygons: overlap is decided by containment
    (a corner of one inside the other) or by a real edge crossing, never by a
    separating-axis assumption.
    """
    if len(points) < MIN_VERTICES or len(corners) < 3:
        return math.inf
    min_x = min(c[0] for c in corners)
    max_x = max(c[0] for c in corners)
    min_y = min(c[1] for c in corners)
    max_y = max(c[1] for c in corners)
    for x, y in corners:
        if point_in_polygon(x, y, points):
            return 0.0
    for x, y in points:
        if min_x <= x <= max_x and min_y <= y <= max_y:
            return 0.0
    best = math.inf
    rect_count = len(corners)
    poly_count = len(points)
    for i in range(rect_count):
        ax, ay = corners[i]
        bx, by = corners[(i + 1) % rect_count]
        for j in range(poly_count):
            cx, cy = points[j]
            dx, dy = points[(j + 1) % poly_count]
            if segments_intersect(ax, ay, bx, by, cx, cy, dx, dy):
                return 0.0
            best = min(best, segment_distance(ax, ay, bx, by, cx, cy, dx, dy))
    return best


def _point_on_segment(
    x: float, y: float, ax: float, ay: float, bx: float, by: float
) -> bool:
    """Is ``(x, y)`` on the closed segment (within the serialisation noise)?"""
    if point_segment_distance(x, y, ax, ay, bx, by) > 1e-6:
        return False
    return (
        min(ax, bx) - 1e-6 <= x <= max(ax, bx) + 1e-6
        and min(ay, by) - 1e-6 <= y <= max(ay, by) + 1e-6
    )


def bounding_box(points: Sequence[Point]) -> Optional[Tuple[float, float, float, float]]:
    """``(min_x, min_y, max_x, max_y)`` of the vertices."""
    if not points:
        return None
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs), min(ys), max(xs), max(ys))
