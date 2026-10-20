"""Path geometry: RDP simplification, line/arc fitting, Bezier smoothing and
arc-length resampling. Pure Python - no third-party dependencies.

File:    software/core/navigation/geometry.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import math
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

Point = Tuple[float, float]

EPS = 1e-9


# --------------------------------------------------------------- basics ----
def polyline_length(points: Sequence[Point]) -> float:
    """Total length of a polyline in millimetres."""
    return sum(math.dist(points[i], points[i + 1]) for i in range(len(points) - 1))


def clamp(value: float, low: float, high: float) -> float:
    """Clamp ``value`` into the inclusive range [lo, hi]."""
    return max(low, min(high, value))


def deg_norm180(angle_deg: float) -> float:
    """Normalize an angle to (-180, 180]."""
    wrapped = math.fmod(angle_deg + 180.0, 360.0)
    if wrapped < 0.0:
        wrapped += 360.0
    return wrapped - 180.0


def point_at_angle(center: Point, radius: float, angle_deg: float) -> Point:
    """Point at ``distance`` from (x, y) along ``angle_deg``."""
    rad = math.radians(angle_deg)
    return (center[0] + radius * math.cos(rad), center[1] + radius * math.sin(rad))


# ------------------------------------------------------------------ RDP ----
def rdp(points: Sequence[Point], eps: float) -> List[Point]:
    """Ramer-Douglas-Peucker simplification (iterative, stack based)."""
    if len(points) < 3:
        return list(points)
    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack: List[Tuple[int, int]] = [(0, len(points) - 1)]
    while stack:
        start, end = stack.pop()
        if end <= start + 1:
            continue
        a, b = points[start], points[end]
        max_dist, max_idx = -1.0, -1
        for i in range(start + 1, end):
            d = _point_line_distance(points[i], a, b)
            if d > max_dist:
                max_dist, max_idx = d, i
        if max_dist > eps and max_idx > 0:
            keep[max_idx] = True
            stack.append((start, max_idx))
            stack.append((max_idx, end))
    return [pt for pt, k in zip(points, keep) if k]


def _point_line_distance(p: Point, a: Point, b: Point) -> float:
    """Distance from a point to the segment a-b (mm)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    norm = math.hypot(dx, dy)
    if norm < EPS:
        return math.dist(p, a)
    return abs(dy * (p[0] - a[0]) - dx * (p[1] - a[1])) / norm


# ------------------------------------------------------------ circle fit ---
def fit_circle(points: Sequence[Point]) -> Tuple[Point, float]:
    """Kasa algebraic circle fit: returns (center, radius)."""
    n = len(points)
    if n < 3:
        a, b = points[0], points[-1]
        return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0), math.dist(a, b) / 2.0
    sx = sy = sxx = syy = sxy = sxz = syz = sz = 0.0
    for x, y in points:
        z = x * x + y * y
        sx += x; sy += y; sz += z
        sxx += x * x; syy += y * y; sxy += x * y
        sxz += x * z; syz += y * z
    # Solve the 3x3 normal equations for (D, E, F) in x^2+y^2 + Dx + Ey + F = 0
    m = [
        [sxx, sxy, sx],
        [sxy, syy, sy],
        [sx, sy, float(n)],
    ]
    rhs = [-sxz, -syz, -sz]
    det = _det3(m)
    if abs(det) < EPS:
        a, b = points[0], points[-1]
        return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0), math.dist(a, b) / 2.0
    d = _det3([(rhs[0], m[0][1], m[0][2]), (rhs[1], m[1][1], m[1][2]), (rhs[2], m[2][1], m[2][2])]) / det
    e = _det3([(m[0][0], rhs[0], m[0][2]), (m[1][0], rhs[1], m[1][2]), (m[2][0], rhs[2], m[2][2])]) / det
    f = _det3([(m[0][0], m[0][1], rhs[0]), (m[1][0], m[1][1], rhs[1]), (m[2][0], m[2][1], rhs[2])]) / det
    cx, cy = -d / 2.0, -e / 2.0
    r = math.sqrt(max(cx * cx + cy * cy - f, 0.0))
    return (cx, cy), r


def _det3(m: Sequence[Sequence[float]]) -> float:
    """3x3 determinant (Kasa circle-fit helper)."""
    return (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )


def _circle_error(points: Sequence[Point], center: Point, radius: float) -> Tuple[float, int]:
    """Algebraic circle-fit residual of the three points."""
    worst, worst_idx = 0.0, 0
    for i, pt in enumerate(points):
        err = abs(math.dist(pt, center) - radius)
        if err > worst:
            worst, worst_idx = err, i
    return worst, worst_idx


def _point_polyline_distance(p: Point, polyline: Sequence[Point]) -> float:
    """Distance from a point to a polyline (mm)."""
    best = float("inf")
    for i in range(len(polyline) - 1):
        ax, ay = polyline[i]
        bx, by = polyline[i + 1]
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        if length2 < EPS:
            t = 0.0
        else:
            t = clamp(((p[0] - ax) * dx + (p[1] - ay) * dy) / length2, 0.0, 1.0)
        best = min(best, math.dist(p, (ax + t * dx, ay + t * dy)))
    return best


def _arc_follows_polyline(
    polyline: Sequence[Point], center: Point, radius: float, a0_deg: float, sweep_deg: float, tol: float
) -> bool:
    """True when the whole fitted arc hugs the drawn polyline (not only its
    vertices - a sparse corner is exactly a semicircle through 3 points and
    would otherwise be accepted while bulging far away from the drawing)."""
    steps = max(8, min(40, int(abs(sweep_deg) / 10.0) + 1))
    for k in range(steps + 1):
        probe = point_at_angle(center, radius, a0_deg + sweep_deg * k / steps)
        if _point_polyline_distance(probe, polyline) > tol:
            return False
    return True


def robust_circle_fit(
    points: Sequence[Point], tol: float, max_trim_ratio: float = 0.15
) -> Optional[Tuple[Point, float]]:
    """Circle fit that tolerates hand jitter and a few outliers.

    Fits, drops the points whose radial error sticks out (a shaky hand, a
    stray sample), refits, and returns the circle when the *kept* points are
    within ``tol``. Returns ``None`` when the span is not arc-like (too many
    outliers or the residual never converges) - the caller then splits.
    """
    idx = list(range(len(points)))
    center, radius = fit_circle(points)
    for _ in range(4):
        residuals = [abs(math.dist(points[i], center) - radius) for i in idx]
        if max(residuals) <= tol:
            return center, radius
        median = sorted(residuals)[len(residuals) // 2]
        threshold = max(tol, 2.0 * median + 1.0)
        keep = [i for i, err in zip(idx, residuals) if err <= threshold]
        min_keep = max(3, int(len(points) * (1.0 - max_trim_ratio)))
        if len(keep) < min_keep or len(keep) == len(idx):
            return None
        center, radius = fit_circle([points[i] for i in keep])
        idx = keep
    residuals = [abs(math.dist(points[i], center) - radius) for i in idx]
    return (center, radius) if max(residuals) <= tol else None


# ------------------------------------------------- line / arc segmentation --
def segment_line_arc(points: Sequence[Point], tol: float, max_arc_tol_ratio: float = 0.25) -> List[Dict]:
    """Split a polyline into line and arc primitives within ``tol``.

    Recursive strategy: try a circle fit on the whole span; if the maximum
    radial error fits the tolerance (and the arc is not almost closed) it is
    an arc. Otherwise try a line; if that fails, split at the point that
    sticks out most from the span chord (RDP-style - a residual-based split
    degenerates on mixed straight+curve spans). ``max_arc_tol_ratio`` guards
    degenerate fits: the radius must not be tiny compared to the tolerance
    (else it is a corner, not an arc).

    Every accepted arc starts and ends on the radial projection of its span
    endpoints (the regenerated sweep LANDS on the end angle instead of
    drifting up to 10 deg away), adjacent primitives are snapped onto one
    shared junction point, and a robust fit may only drop INTERIOR outliers:
    the regenerated path has no seams, so a chain of fitted arcs comes out
    continuous and can be driven as consecutive ARC commands.
    """
    items: List[Tuple[Dict, int, int]] = []  # (primitive, span start, span end)
    stack: List[Tuple[int, int]] = [(0, len(points) - 1)]
    while stack:
        start, end = stack.pop(0)
        span = list(points[start : end + 1])
        if len(span) < 2:
            continue
        if len(span) == 2 or _is_line(span, tol):
            items.append(({"type": "line", "a": span[0], "b": span[-1]}, start, end))
            continue
        (cx, cy), r = fit_circle(span)
        err, err_idx = _circle_error(span, (cx, cy), r)
        arc_ok = err <= tol and r > tol / max(max_arc_tol_ratio, 1e-6)
        if not arc_ok:
            # hand jitter: drop the few outlier samples, refit, and keep the
            # corrected circle when the trimmed fit is clean and every drawn
            # point stays reasonably close to it
            robust = robust_circle_fit(span, tol, max_trim_ratio=0.15)
            if robust is not None:
                (rcx, rcy), rr = robust
                # honest endpoints: the span start AND the run end anchor the
                # primitive - a run origin must stay on the drawing, junctions
                # must be real, and the stroke must finish where the operator
                # ended it (a robust fit may only drop interior samples; the
                # old finger-lift exemption silently trimmed up to 20 mm off
                # the drawn tail)
                first_ok = abs(math.dist(span[0], (rcx, rcy)) - rr) <= tol
                last_ok = abs(math.dist(span[-1], (rcx, rcy)) - rr) <= tol
                if rr > tol / max(max_arc_tol_ratio, 1e-6) and first_ok and last_ok:
                    err_all = max(abs(math.dist(p, (rcx, rcy)) - rr) for p in span)
                    if err_all <= tol * 3.0:
                        (cx, cy), r = (rcx, rcy), rr
                        arc_ok = True
        if arc_ok:
            sweep = _arc_sweep(span, (cx, cy))
            a0 = math.degrees(math.atan2(span[0][1] - cy, span[0][0] - cx))
            a1 = math.degrees(math.atan2(span[-1][1] - cy, span[-1][0] - cx))
            landing = _landing_sweep(a0, a1, sweep)
            follows_drawing = landing is not None and _arc_follows_polyline(
                span, (cx, cy), r, a0, landing, tol * 4.0
            )
            if landing is not None and abs(landing) <= 340.0 and follows_drawing:
                primitive = {
                    "type": "arc",
                    "center": (cx, cy),
                    "r": r,
                    "a_start_deg": a0,
                    "a_end_deg": a0 + landing,
                    "sweep_deg": landing,
                }
                items.append((primitive, start, end))
                continue
        split = start + _worst_chord_deviation_index(span)
        # Process the left half first so the primitives come out in drawing
        # order (the stack pops from the front).
        stack.insert(0, (split, end))
        stack.insert(0, (start, split))
    _snap_primitive_junctions(items, points)
    return [item[0] for item in items]


def _landing_sweep(a0_deg: float, a1_deg: float, accumulated_deg: float, max_drift_deg: float = 15.0) -> Optional[float]:
    """Sweep from ``a0`` to ``a1`` closest to the accumulated rotation.

    The regenerated arc must END on the projection of the span's last point
    (``a1``); the accumulated sweep of the noisy samples only provides the
    winding, so the closest same-direction candidate wins. ``None`` means the
    two disagree beyond ``max_drift_deg`` (a wandering span) - the caller
    then splits instead of walking a wrong part of the circle.
    """
    base = deg_norm180(a1_deg - a0_deg)
    best: Optional[float] = None
    for k in (-2, -1, 0, 1, 2):
        candidate = base + 360.0 * k
        if candidate * accumulated_deg < 0.0:
            continue  # opposite winding
        drift = abs(candidate - accumulated_deg)
        if drift <= max_drift_deg and (best is None or drift < abs(best - accumulated_deg)):
            best = candidate
    return best


def _snap_primitive_junctions(items: Sequence[Tuple[Dict, int, int]], points: Sequence[Point]) -> None:
    """Make neighbouring primitives share ONE junction point exactly.

    The fitted circles/lines are independent, so their regenerated endpoints
    sit a few millimetres apart at every span boundary - sampled back, that
    reads as a kink (node headings swing by 10-20 deg) and blocks the chain
    from being driven as consecutive ARCs. Each junction is moved onto the
    point where the two primitives really meet (circle-circle / line-circle
    intersection, the candidate nearest to the drawn boundary point).
    """
    for index in range(len(items) - 1):
        prim = items[index][0]
        nxt = items[index + 1][0]
        raw = points[items[index][2]]
        shared = _common_junction_point(prim, nxt, raw)
        if shared is None or math.dist(shared, raw) > 60.0:
            continue
        _set_primitive_end(prim, shared)
        _set_primitive_start(nxt, shared)


def _common_junction_point(prim: Dict, nxt: Dict, raw: Point) -> Optional[Point]:
    """The meeting point of two adjacent primitives, nearest to ``raw``."""
    if prim["type"] == "arc" and nxt["type"] == "arc":
        candidates = _circle_circle_points(prim["center"], prim["r"], nxt["center"], nxt["r"])
    elif prim["type"] == "arc" and nxt["type"] == "line":
        candidates = _line_circle_points(nxt["a"], nxt["b"], prim["center"], prim["r"])
    elif prim["type"] == "line" and nxt["type"] == "arc":
        candidates = _line_circle_points(prim["a"], prim["b"], nxt["center"], nxt["r"])
    else:
        return tuple(raw)  # type: ignore[return-value]
    if not candidates:
        return None
    return min(candidates, key=lambda point: math.dist(point, raw))


def _circle_circle_points(c1: Point, r1: float, c2: Point, r2: float) -> List[Point]:
    """Intersection points of two circles (0, 1 or 2)."""
    d = math.dist(c1, c2)
    if d < 1e-6 or d > r1 + r2 + 1e-6 or d < abs(r1 - r2) - 1e-6:
        return []
    a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d)
    h = math.sqrt(max(r1 * r1 - a * a, 0.0))
    bx = c1[0] + a * (c2[0] - c1[0]) / d
    by = c1[1] + a * (c2[1] - c1[1]) / d
    ox = -(c2[1] - c1[1]) * h / d
    oy = (c2[0] - c1[0]) * h / d
    return [(bx + ox, by + oy), (bx - ox, by - oy)]


def _line_circle_points(a: Point, b: Point, center: Point, radius: float) -> List[Point]:
    """Intersection points of a segment and a circle."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    if length2 < 1e-9:
        return []
    t0 = ((center[0] - a[0]) * dx + (center[1] - a[1]) * dy) / length2
    dist2 = math.dist(center, a) ** 2
    h2 = (radius * radius - dist2 + t0 * t0 * length2) / length2
    if h2 < 0.0:
        return []
    h = math.sqrt(h2)
    return [(a[0] + (t0 + h) * dx, a[1] + (t0 + h) * dy), (a[0] + (t0 - h) * dx, a[1] + (t0 - h) * dy)]


def _set_primitive_start(prim: Dict, point: Point) -> None:
    """Move a primitive's start point to ``p``."""
    if prim["type"] == "line":
        prim["a"] = tuple(point)
        return
    prim["a_start_deg"] = math.degrees(
        math.atan2(point[1] - prim["center"][1], point[0] - prim["center"][0])
    )
    _resync_arc_sweep(prim)


def _set_primitive_end(prim: Dict, point: Point) -> None:
    """Move a primitive's end point to ``p``."""
    if prim["type"] == "line":
        prim["b"] = tuple(point)
        return
    prim["a_end_deg"] = math.degrees(
        math.atan2(point[1] - prim["center"][1], point[0] - prim["center"][0])
    )
    _resync_arc_sweep(prim)


def _resync_arc_sweep(prim: Dict) -> None:
    """Recompute ``sweep_deg`` after an endpoint angle was moved.

    Keeps the winding and picks the candidate closest to the previous sweep,
    so a snapped junction only nudges the sweep (never flips the arc).
    """
    base = deg_norm180(prim["a_end_deg"] - prim["a_start_deg"])
    previous = prim["sweep_deg"]
    best: Optional[float] = None
    for k in (-2, -1, 0, 1, 2):
        candidate = base + 360.0 * k
        if candidate * previous < 0.0:
            continue
        if best is None or abs(candidate - previous) < abs(best - previous):
            best = candidate
    if best is not None:
        prim["sweep_deg"] = best
        prim["a_end_deg"] = prim["a_start_deg"] + best


def _worst_chord_deviation_index(span: Sequence[Point]) -> int:
    """Index (1..len-2) of the interior point farthest from the chord."""
    a, b = span[0], span[-1]
    worst, worst_idx = -1.0, 1
    for i in range(1, len(span) - 1):
        deviation = _point_line_distance(span[i], a, b)
        if deviation > worst:
            worst, worst_idx = deviation, i
    return max(1, min(worst_idx, len(span) - 2))


def _is_line(points: Sequence[Point], tol: float) -> bool:
    """True when the primitive is a line (not an arc)."""
    a, b = points[0], points[-1]
    return all(_point_line_distance(pt, a, b) <= tol for pt in points[1:-1])


def _arc_sweep(points: Sequence[Point], center: Point) -> float:
    """Total signed sweep of the polyline around ``center`` (degrees)."""
    total = 0.0
    prev = None
    for x, y in points:
        ang = math.degrees(math.atan2(y - center[1], x - center[0]))
        if prev is not None:
            total += deg_norm180(ang - prev)
        prev = ang
    return total


def arc_to_points(arc: Dict, max_step_deg: float = 10.0) -> List[Point]:
    """Convert an arc primitive into a polyline (endpoints included)."""
    center, r = arc["center"], arc["r"]
    a0, sweep = arc["a_start_deg"], arc["sweep_deg"]
    steps = max(2, int(math.ceil(abs(sweep) / max_step_deg)))
    return [point_at_angle(center, r, a0 + sweep * i / steps) for i in range(steps + 1)]


def primitive_to_points(primitive: Dict, max_step_deg: float = 10.0) -> List[Point]:
    """Sample a primitive into points with arc-length spacing."""
    if primitive["type"] == "arc":
        return arc_to_points(primitive, max_step_deg)
    return [tuple(primitive["a"]), tuple(primitive["b"])]  # type: ignore[list-item]


# --------------------------------------------------------------- smoothing --
def catmull_rom(points: Sequence[Point], samples_per_segment: int = 12) -> List[Point]:
    """Uniform Catmull-Rom spline through ``points`` (C1, locally cubic).

    Equivalent to chained cubic Beziers; used as corner rounding before the
    velocity profile. Endpoints are duplicated so the spline starts/ends at
    the first/last waypoint.
    """
    pts = list(points)
    if len(pts) < 3:
        return pts
    out: List[Point] = []
    for i in range(len(pts) - 1):
        p0 = pts[i - 1] if i > 0 else pts[i]
        p1, p2 = pts[i], pts[i + 1]
        p3 = pts[i + 2] if i + 2 < len(pts) else pts[i + 1]
        for s in range(samples_per_segment):
            t = s / samples_per_segment
            t2, t3 = t * t, t * t * t
            x = 0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t
                       + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
                       + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3)
            y = 0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t
                       + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
                       + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
            out.append((x, y))
    out.append(pts[-1])
    return out


def resample(points: Sequence[Point], step_mm: float) -> List[Point]:
    """Uniform arc-length resampling (endpoints preserved)."""
    if len(points) < 2 or step_mm <= 0:
        return list(points)
    step = max(step_mm, 1.0)
    out: List[Point] = [tuple(points[0])]  # type: ignore[list-item]
    carry = 0.0
    for i in range(len(points) - 1):
        a, b = points[i], points[i + 1]
        seg = math.dist(a, b)
        if seg < EPS:
            continue
        pos = carry
        while pos + step <= seg:
            pos += step
            t = pos / seg
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
        carry = pos - seg
    if math.dist(out[-1], points[-1]) > EPS:
        out.append(tuple(points[-1]))  # type: ignore[list-item]
    return out


def point_headings(points: Sequence[Point]) -> List[float]:
    """Heading (degrees, CCW) of the outgoing direction at every point."""
    out: List[float] = []
    for i in range(len(points) - 1):
        dx = points[i + 1][0] - points[i][0]
        dy = points[i + 1][1] - points[i][1]
        out.append(math.degrees(math.atan2(dy, dx)))
    out.append(out[-1] if out else 0.0)
    return out


# ------------------------------------------------------- corner rounding ---
def dedupe_consecutive(points: Sequence[Point], tol: float = 0.5) -> List[Point]:
    """Drop consecutive points closer than ``tol`` (keeps first and last)."""
    out: List[Point] = []
    for point in points:
        if out and math.dist(out[-1], point) < tol:
            continue
        out.append((float(point[0]), float(point[1])))
    return out


def round_corners(
    points: Sequence[Point],
    radius_mm: float,
    min_turn_deg: float = 10.0,
    max_turn_deg: float = 170.0,
    arc_step_deg: float = 4.0,
    veto: Optional[Callable[[Sequence[Point]], bool]] = None,
    radius_scales: Sequence[float] = (1.0,),
    skipped: Optional[List[int]] = None,
    radius_limit_for: Optional[Callable[[Point, float], Optional[float]]] = None,
) -> List[Point]:
    """Replace sharp vertices by tangent circular fillets (corner rounding).

    Every vertex whose deflection exceeds ``min_turn_deg`` is replaced by an
    arc of up to ``radius_mm`` tangent to both legs, so the robot starts
    turning *before* the corner instead of stopping on it. The fillet radius
    is reduced automatically when the adjacent legs are too short.

    ``veto(points)->bool`` is offered every candidate fillet (per
    ``radius_scales``, largest first) and may reject it (e.g. blocked cells):
    the corner then keeps its vertex **while the other corners are still
    rounded** - one impossible corner must not flatten the whole path.
    Rejected corner indices are appended to ``skipped``.

    ``radius_limit_for(vertex, half_turn_rad) -> max_radius|None`` caps the
    fillet radius per vertex (waypoints: the sweep stays within a few cm of
    the exact point while still rounding the corner).
    """
    pts = [(float(x), float(y)) for x, y in points]
    if radius_mm <= 0.0 or len(pts) < 3:
        return pts
    out: List[Point] = [pts[0]]
    prev = pts[0]
    for i in range(1, len(pts) - 1):
        vertex = pts[i]
        nxt = pts[i + 1]
        d1x, d1y = vertex[0] - prev[0], vertex[1] - prev[1]
        l1 = math.hypot(d1x, d1y)
        d2x, d2y = nxt[0] - vertex[0], nxt[1] - vertex[1]
        l2 = math.hypot(d2x, d2y)
        if l1 < EPS or l2 < EPS:
            continue
        u1 = (d1x / l1, d1y / l1)
        u2 = (d2x / l2, d2y / l2)
        turn = math.degrees(math.atan2(u1[0] * u2[1] - u1[1] * u2[0], u1[0] * u2[0] + u1[1] * u2[1]))
        if abs(turn) < min_turn_deg or abs(turn) > max_turn_deg:
            out.append(vertex)
            prev = vertex
            continue
        half = math.radians(abs(turn)) / 2.0
        # sharper corners get a wider blend (up to 1.8x) so the robot sweeps
        # through the turn like a drawn arc instead of nicking the vertex
        sharpness = clamp(1.0 + (abs(turn) - 60.0) / 120.0, 1.0, 1.8)
        limit = radius_limit_for(vertex, half) if radius_limit_for is not None else None
        fillet: Optional[List[Point]] = None
        for scale in radius_scales:
            tangent = min(radius_mm * scale * sharpness * math.tan(half), 0.45 * l1, 0.45 * l2)
            if limit is not None:
                tangent = min(tangent, limit * math.tan(half))
            if tangent < 2.0:
                continue
            radius = tangent / math.tan(half)
            t1 = (vertex[0] - u1[0] * tangent, vertex[1] - u1[1] * tangent)
            t2 = (vertex[0] + u2[0] * tangent, vertex[1] + u2[1] * tangent)
            bx, by = u2[0] - u1[0], u2[1] - u1[1]
            blen = math.hypot(bx, by)
            if blen < EPS:
                break
            center = (
                vertex[0] + (bx / blen) * radius / math.cos(half),
                vertex[1] + (by / blen) * radius / math.cos(half),
            )
            a0 = math.atan2(t1[1] - center[1], t1[0] - center[0])
            a1 = math.atan2(t2[1] - center[1], t2[0] - center[0])
            sweep = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
            if (sweep >= 0.0) != (turn >= 0.0):
                sweep -= math.copysign(2.0 * math.pi, sweep)
            steps = max(2, int(math.ceil(abs(sweep) / math.radians(max(1.0, arc_step_deg)))))
            candidate = [t1]
            for k in range(1, steps):
                angle = a0 + sweep * k / steps
                candidate.append((center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle)))
            candidate.append(t2)
            if veto is None or not veto(candidate):
                fillet = candidate
                break
        if fillet is None:
            if skipped is not None:
                skipped.append(i)
            out.append(vertex)
            prev = vertex
            continue
        out.extend(fillet)
        prev = fillet[-1]
    out.append(pts[-1])
    return dedupe_consecutive(out)


# ------------------------------------------------------ adaptive sampling ---
def path_resample(
    points: Sequence[Point],
    base_step_mm: float,
    chord_tol_mm: float = 6.0,
    max_turn_deg: float = 15.0,
    min_step_mm: float = 40.0,
) -> List[Point]:
    """Arc-length resampling with curvature-aware node spacing.

    Straight sections keep ``base_step_mm``; on curves the chord shrinks
    until both limits hold on the polyline itself:

    * every skipped point stays within ``chord_tol_mm`` of the chord
      (checked point by point, not from an estimated radius), and
    * each node turns by at most ``max_turn_deg``.

    Measuring the chord on the real geometry keeps the spacing LOCAL: a tight
    fillet only densifies its own nodes and can never bleed fine steps onto
    the long straights around it (a window-based radius estimate did exactly
    that, turning a 1.3 m straight into ~16 nodes of 3-80 mm). The generated
    points double as GOTO nodes for the robot, which is how an arc is
    followed accurately.
    """
    pts = dedupe_consecutive(points, 0.5)
    if len(pts) < 2:
        return pts
    n = len(pts)
    cum = [0.0]
    for i in range(n - 1):
        cum.append(cum[-1] + math.dist(pts[i], pts[i + 1]))
    total = cum[-1]
    if total < 1.0:
        return pts
    base = max(min_step_mm, base_step_mm)
    tol = max(0.5, chord_tol_mm)
    max_turn = math.radians(clamp(max_turn_deg, 4.0, 90.0))

    def locate(s: float) -> int:
        """Point and tangent at ``distance`` along the primitive."""
        lo, hi = 0, n - 2
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if cum[mid] <= s:
                lo = mid
            else:
                hi = mid - 1
        return lo

    def point_at(s: float) -> Point:
        """The point at ``distance`` along the primitive."""
        i = locate(s)
        span = cum[i + 1] - cum[i]
        t = 0.0 if span < EPS else (s - cum[i]) / span
        return (pts[i][0] + (pts[i + 1][0] - pts[i][0]) * t, pts[i][1] + (pts[i + 1][1] - pts[i][1]) * t)

    def heading(a: Point, b: Point) -> float:
        """Travel heading (degrees) at ``distance`` along the primitive."""
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def tangent_heading(s: float) -> float:
        """Tangent heading at the primitive's start or end."""
        index = locate(min(max(s, 0.0), total))
        return heading(pts[index], pts[index + 1])

    def chord_dev(a: Point, b: Point, s_from: float, s_to: float) -> float:
        """Worst distance of the skipped polyline points to the chord a->b."""
        abx, aby = b[0] - a[0], b[1] - a[1]
        length = math.hypot(abx, aby)
        if length < EPS:
            return 0.0
        worst = 0.0
        index = locate(s_from) + 1
        while index < n - 1 and cum[index] < s_to - 1e-9:
            px, py = pts[index]
            worst = max(worst, abs((px - a[0]) * aby - (py - a[1]) * abx) / length)
            index += 1
        return worst

    def longest_chord(a: Point, s: float, limit: float) -> float:
        """Farthest step <= limit whose skipped geometry stays within tol."""
        lo, hi = 1.0, max(1.0, limit)
        best = lo
        for _ in range(16):
            mid = 0.5 * (lo + hi)
            if chord_dev(a, point_at(s + mid), s, s + mid) <= tol:
                best = mid
                lo = mid
            else:
                hi = mid
        return best

    def longest_turn(s: float, limit: float) -> float:
        """Farthest step <= limit whose span turns by <= max_turn end to end.

        Measuring the tangent change INSIDE the chord (not chord-vs-chord)
        keeps the nodes off the steepest part of a curve: consecutive spans
        of <=15 deg leave each node turning ~15 deg too.
        """
        start = tangent_heading(s)
        lo, hi = 1.0, max(1.0, limit)
        best = lo
        for _ in range(16):
            mid = 0.5 * (lo + hi)
            turn = abs(deg_norm180(math.degrees(tangent_heading(s + mid) - start)))
            if turn <= math.degrees(max_turn):
                best = mid
                lo = mid
            else:
                hi = mid
        return best

    def longest_align(a: Point, s: float, limit: float) -> float:
        """Farthest step <= limit whose chord direction stays within half the
        turn budget of the tangent it ends on.

        A chord is allowed to end INSIDE a curve (up to the deviation
        tolerance); without this the next chord could not satisfy the turn
        cap from the skewed previous heading and collapsed into 1 mm crawl
        nodes. Half the budget each side guarantees consecutive chords differ
        by <= ``max_turn_deg``.
        """
        half = math.degrees(max_turn) * 0.5
        lo, hi = 1.0, max(1.0, limit)
        best = lo
        for _ in range(16):
            mid = 0.5 * (lo + hi)
            skew = abs(deg_norm180(math.degrees(heading(a, point_at(s + mid)) - tangent_heading(s + mid))))
            if skew <= half:
                best = mid
                lo = mid
            else:
                hi = mid
        return best

    def longest_chord_turn(a: Point, s: float, previous: float, limit: float) -> float:
        """Farthest step <= limit turning <= max_turn from the previous chord.

        The robot turns between two DRIVE chords, so the chord-vector turn is
        what the firmware actually executes at a node - bound it too.
        """
        lo, hi = 1.0, max(1.0, limit)
        best = lo
        for _ in range(16):
            mid = 0.5 * (lo + hi)
            turn = abs(deg_norm180(math.degrees(heading(a, point_at(s + mid)) - previous)))
            if turn <= math.degrees(max_turn):
                best = mid
                lo = mid
            else:
                hi = mid
        return best

    samples: List[Point] = [pts[0]]
    s = 0.0
    previous: Optional[float] = None
    guard = 0
    while s < total - 0.5 and guard < 100000:
        guard += 1
        a = point_at(s)
        step = longest_chord(a, s, min(base, total - s))
        step = longest_turn(s, step)
        step = longest_align(a, s, step)
        if previous is not None:
            step = longest_chord_turn(a, s, previous, step)
        step = max(step, 1.0)
        s = min(s + step, total)
        b = point_at(s)
        previous = heading(a, b)
        samples.append(b)
    if math.dist(samples[-1], pts[-1]) > 0.5:
        samples.append(pts[-1])
    return dedupe_consecutive(samples, 0.5)
