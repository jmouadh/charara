"""Shortest legal arrival maneuver for a waypoint that carries a heading.

A waypoint with an arrival angle must be reached **on the point** facing
``theta``. The old implementation wove the turn into the last ~100 mm of the
approach as a post-processing step: it only looked at the emitted GOTO tail,
could not route to an intermediate point, and its legality test only covered
the robot body (half-diagonal) - so near a wall or an obstacle it either
refused the weave (a stop-and-turn) or swung the arc outside the field.

This module plans the maneuver BEFORE the run is emitted:

* a circular arc ending on the waypoint with an exact exit tangent ``theta``
  is fully described by its start ``Q`` (the manoeuvre replaces the tail of
  the drawn run, or a fresh route is planned to ``Q``);
* every candidate is sampled and tested against the FULL obstacle clearance
  (robot half-diagonal + the configured obstacle margin) and the legal field
  border, so neither the arc nor the route can leave the table or cut a
  clearance zone;
* the candidate with the SHORTEST total (route to ``Q`` + arc) wins, with an
  in-place turn costed as the alternative - "combine with the approach" only
  pays when it really is the short way round;
* when no legal arc exists the caller gets a ``turn`` choice: reach the
  (legal) waypoint and turn in place there, which is always safe because the
  waypoint itself was moved out of every clearance zone first.

Pure geometry: no I/O and no planner state, so it is unit-testable on its own.

File:    software/core/navigation/arrival.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-18
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from . import geometry as g
from .grid import NoPathError, OccupancyGrid, theta_star

Point = Tuple[float, float]

#: firmware window for a woven arrival arc (radius / sweep / length)
ARC_MIN_RADIUS_MM = 60.0
ARC_MAX_RADIUS_MM = 2000.0
ARC_MIN_SWEEP_DEG = 12.0
ARC_MAX_SWEEP_DEG = 180.0
ARC_MAX_LENGTH_MM = 2900.0

#: how far back the manoeuvre may replace the drawn approach [mm]
MAX_APPROACH_CUT_MM = 700.0
#: spacing of the candidate points along that approach [mm]
APPROACH_STEP_MM = 25.0
#: ring of radii tried for a fresh approach (routed to the arc start)
RADII_MM = (60.0, 80.0, 105.0, 135.0, 170.0, 215.0, 270.0, 340.0,
            430.0, 540.0, 680.0, 860.0, 1080.0, 1360.0, 1700.0, 2000.0)
SWEEPS_DEG = (20.0, 35.0, 50.0, 65.0, 80.0, 95.0, 110.0, 125.0, 140.0, 155.0, 170.0)
#: routed candidates evaluated with Theta* (the rest only by their estimate)
MAX_ROUTES = 14
#: cost of an in-place turn, expressed as an equivalent path length:
#: ~200 mm/s cruise / 90 deg/s turn = 2.2 mm per degree
TURN_COST_MM_PER_DEG = 2.2
#: an arc wins a tie: it is the smooth version of the same manoeuvre
ARC_TIE_BREAK_MM = 0.5
#: "combine" is a request for a smooth arrival, not for the absolute shortest
#: line: a legal arc may cost this much more than stopping to turn in place
#: (25 % + 30 mm), beyond it the in-place turn is the honest answer
ARC_MAX_OVERHEAD = 0.25
ARC_OVERHEAD_MM = 30.0
#: replacing drawn geometry costs a little: a real angle change may shorten
#: the approach, but a long drawn curve is not cut just to save millimetres
APPROACH_CUT_PENALTY = 0.25


@dataclass(frozen=True)
class ArrivalArc:
    """One circular arc that ends on ``point`` facing ``theta``."""

    start: Point
    centre: Point
    radius: float
    sweep_deg: float
    entry_deg: float
    points: Tuple[Point, ...]

    @property
    def length_mm(self) -> float:
        """Driven length of the arc [mm]."""
        return abs(math.radians(self.sweep_deg)) * self.radius

    def as_segment(self, speed: int) -> Dict:
        """Emit the firmware ARC segment for this manoeuvre."""
        return {
            "kind": "arc",
            "x0": round(self.start[0], 1),
            "y0": round(self.start[1], 1),
            "x1": round(self.points[-1][0], 1),
            "y1": round(self.points[-1][1], 1),
            "radius_mm": round(math.copysign(self.radius, self.sweep_deg), 1),
            "sweep_deg": round(self.sweep_deg, 1),
            "th0_deg": round(g.deg_norm180(self.entry_deg), 1),
            "v": int(speed),
            "points": [[round(x, 1), round(y, 1)] for x, y in self.points],
            "arrival": True,
        }


@dataclass(frozen=True)
class ArrivalChoice:
    """What to drive into a heading-constrained waypoint.

    ``kind == "arc"``:  the arc ends on the waypoint. ``from_run`` says
    whether ``route`` is the REPLACEMENT for the run already planned (its
    last point is the arc start) or a connector from the anchor to the arc
    start. The caller appends the arc as an explicit ARC segment, so the
    sampler can never flatten it.
    ``kind == "turn"``: drive to ``point`` and turn in place there.
    """

    kind: str
    point: Point
    theta_deg: float
    arc: Optional[ArrivalArc] = None
    route: Tuple[Point, ...] = ()
    from_run: bool = False
    cost_mm: float = 0.0


# --------------------------------------------------------------- helpers ----
def _left_normal(angle_deg: float) -> Point:
    """Left normal of a travel heading (CCW arcs curve towards it)."""
    rad = math.radians(angle_deg)
    return (-math.sin(rad), math.cos(rad))


def _rotate(vector: Point, angle_rad: float) -> Point:
    """Rotate a 2-D vector by ``angle_rad`` (CCW positive)."""
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return (vector[0] * cos_a - vector[1] * sin_a,
            vector[0] * sin_a + vector[1] * cos_a)


def _heading(from_point: Point, to_point: Point) -> Optional[float]:
    """Travel heading from one point to another (None when they coincide)."""
    dx = to_point[0] - from_point[0]
    dy = to_point[1] - from_point[1]
    if math.hypot(dx, dy) <= 1e-6:
        return None
    return math.degrees(math.atan2(dy, dx))


def _turn_cost(heading_a: Optional[float], heading_b: float) -> float:
    """Equivalent path length of the in-place turn between two headings [mm]."""
    if heading_a is None:
        return 0.0
    return abs(g.deg_norm180(heading_b - heading_a)) * TURN_COST_MM_PER_DEG


def _arc_samples(centre: Point, radius: float, start_rad: float,
                 sweep_rad: float, max_step_deg: float = 5.0) -> List[Point]:
    """Sample an arc every <= ``max_step_deg``, both ends included."""
    steps = max(2, int(math.ceil(abs(math.degrees(sweep_rad)) / max_step_deg)))
    return [
        (centre[0] + radius * math.cos(start_rad + sweep_rad * (step / steps)),
         centre[1] + radius * math.sin(start_rad + sweep_rad * (step / steps)))
        for step in range(steps + 1)
    ]


def arc_from_start(point: Point, start: Point, theta_deg: float) -> Optional[ArrivalArc]:
    """The unique circle through ``start`` that ends on ``point`` facing theta.

    A circle tangent to ``theta`` at ``point`` has its centre on the normal
    there. Through ``start`` two such circles exist (one per turn direction);
    the one whose sweep runs FORWARD towards ``point`` is returned. ``None``
    when neither is a usable arrival (degenerate or more than 180 deg).
    """
    px, py = point
    qx, qy = start
    dx = qx - px
    dy = qy - py
    chord2 = dx * dx + dy * dy
    if chord2 <= 1e-6:
        return None
    nx, ny = _left_normal(theta_deg)  # left normal at the waypoint
    for side in (1.0, -1.0):
        along = dx * nx + dy * ny
        denom = 2.0 * side * along
        if abs(denom) <= 1e-9:
            continue
        radius = chord2 / denom
        if radius <= 0.0:
            continue  # this turn direction cannot pass through start
        centre = (px + side * radius * nx, py + side * radius * ny)
        start_angle = math.atan2(qy - centre[1], qx - centre[0])
        end_angle = math.atan2(py - centre[1], px - centre[0])
        if side > 0.0:  # CCW: walk forward from start to end
            sweep = (end_angle - start_angle) % (2.0 * math.pi)
        else:           # CW: walk backward
            sweep = -((start_angle - end_angle) % (2.0 * math.pi))
        sweep_deg = math.degrees(sweep)
        if abs(sweep_deg) < ARC_MIN_SWEEP_DEG or abs(sweep_deg) > ARC_MAX_SWEEP_DEG + 1e-6:
            continue
        if not (ARC_MIN_RADIUS_MM <= radius <= ARC_MAX_RADIUS_MM):
            continue
        if radius * abs(math.radians(sweep_deg)) > ARC_MAX_LENGTH_MM:
            continue
        entry = theta_deg - sweep_deg
        samples = _arc_samples(centre, radius, start_angle, sweep)
        samples[0] = (qx, qy)
        samples[-1] = (px, py)
        return ArrivalArc(
            start=(qx, qy),
            centre=centre,
            radius=radius,
            sweep_deg=sweep_deg,
            entry_deg=g.deg_norm180(entry),
            points=tuple(samples),
        )
    return None


def _arc_fits(arc: ArrivalArc, grid: Optional[OccupancyGrid],
              legal_margin: float, width: float, height: float) -> bool:
    """Is every millimetre of the arc legal (border + full clearance)?"""
    previous: Optional[Point] = None
    for x, y in arc.points:
        if x < legal_margin or x > width - legal_margin:
            return False
        if y < legal_margin or y > height - legal_margin:
            return False
        if grid is not None:
            if not grid.is_free_mm(x, y):
                return False
            if previous is not None and not grid.los(previous, (x, y)):
                return False
        previous = (x, y)
    return True


def _route_length(route: Sequence[Point]) -> float:
    """Length of a routed polyline [mm]."""
    return g.polyline_length(route)


# -------------------------------------------------------------- planning ----
def choose_arrival(
    anchor: Point,
    point: Point,
    theta_deg: float,
    grid: Optional[OccupancyGrid],
    legal_margin: float,
    width: float,
    height: float,
    previous_run: Optional[Sequence[Point]] = None,
) -> Optional[ArrivalChoice]:
    """Shortest legal way to reach ``point`` facing ``theta_deg``.

    ``previous_run`` is the geometry already planned into the waypoint (its
    last point is ``point`` when the waypoint sits on the end of a drawn
    stroke): the manoeuvre may then REPLACE its tail with the arrival arc.
    Otherwise the caller gets a route to the arc's start (an intermediate
    point chosen by the search), or a route to ``point`` plus the in-place
    turn fallback. All options are costed in millimetres, so the answer is
    the shortest one that respects the field and the obstacle clearance.
    """
    best_arc: Optional[ArrivalChoice] = None
    best_turn: Optional[ArrivalChoice] = None

    def offer(choice: ArrivalChoice) -> None:
        nonlocal best_arc, best_turn
        if choice.cost_mm == math.inf:
            return
        if choice.kind == "arc":
            if best_arc is None or choice.cost_mm < best_arc.cost_mm - 1e-6:
                best_arc = choice
        else:
            if best_turn is None or choice.cost_mm < best_turn.cost_mm - 1e-6:
                best_turn = choice

    # ---- A) the waypoint closes the run already planned -----------------
    if previous_run is not None and len(previous_run) >= 2:
        run = list(previous_run)
        if math.dist(run[-1], point) <= 2.0:
            arc, turn = _arrival_on_run(run, point, theta_deg, grid, legal_margin, width, height)
            if arc is not None:
                offer(arc)
            if turn is not None:
                offer(turn)

    # ---- B) a fresh route to an intermediate point ----------------------
    # (only when the waypoint is not already the end of the planned run: a
    # route from P to Q and back to P would be a loop, and its empty
    # "turn at P" fallback would wrongly beat a smooth arrival weave)
    if math.dist(anchor, point) > 1.0 and grid is not None:
        fallback = _turn_route(anchor, point, theta_deg, grid, legal_margin, width, height)
        if fallback is not None:
            offer(fallback)
        # cheap estimates first, then Theta* only for the most promising
        # starts: the grid search is the expensive part of the choice
        estimates: List[Tuple[float, ArrivalArc]] = []
        for radius in RADII_MM:
            for sweep in SWEEPS_DEG:
                for side in (1.0, -1.0):
                    start = _arc_start(point, theta_deg, radius, sweep, side)
                    if start is None:
                        continue
                    arc = arc_from_start(point, start, theta_deg)
                    if arc is None or not _arc_fits(arc, grid, legal_margin, width, height):
                        continue
                    if not grid.is_free_mm(start[0], start[1]):
                        continue
                    estimate = (
                        math.dist(anchor, start)
                        + arc.length_mm
                        + _turn_cost(_heading(anchor, start), arc.entry_deg)
                        - ARC_TIE_BREAK_MM
                    )
                    estimates.append((estimate, arc))
        seen_start: List[Point] = []
        for _estimate, arc in sorted(estimates, key=lambda item: item[0]):
            if any(math.dist(other, arc.start) <= 20.0 for other in seen_start):
                continue
            seen_start.append(arc.start)
            if len(seen_start) > MAX_ROUTES:
                break
            offer(_routed_arrival(anchor, point, theta_deg, arc, grid,
                                  legal_margin, width, height))
    elif previous_run is None and math.dist(anchor, point) > 1.0:
        # no obstacle map: a direct route is always possible
        for radius in RADII_MM:
            for sweep in SWEEPS_DEG:
                for side in (1.0, -1.0):
                    start = _arc_start(point, theta_deg, radius, sweep, side)
                    if start is None:
                        continue
                    arc = arc_from_start(point, start, theta_deg)
                    if arc is None or not _arc_fits(arc, None, legal_margin, width, height):
                        continue
                    cost = (
                        math.dist(anchor, start)
                        + arc.length_mm
                        + _turn_cost(_heading(anchor, start), arc.entry_deg)
                        - ARC_TIE_BREAK_MM
                    )
                    offer(ArrivalChoice(
                        kind="arc", point=point, theta_deg=theta_deg, arc=arc,
                        route=((anchor[0], anchor[1]), start), from_run=False, cost_mm=cost,
                    ))
        cost = math.dist(anchor, point) + _turn_cost(_heading(anchor, point), theta_deg)
        offer(ArrivalChoice(
            kind="turn", point=point, theta_deg=theta_deg, arc=None,
            route=((anchor[0], anchor[1]), point), from_run=False, cost_mm=cost,
        ))
    # "combine" asks for a smooth arrival: a legal arc is preferred even when
    # stopping to turn would be a little shorter. A manoeuvre that costs much
    # more than the turn (a wide loop near a wall) is not smooth, it is wrong.
    if best_arc is None:
        return best_turn
    if best_turn is None:
        return best_arc
    limit = best_turn.cost_mm * (1.0 + ARC_MAX_OVERHEAD) + ARC_OVERHEAD_MM
    return best_arc if best_arc.cost_mm <= limit else best_turn


def _arc_start(point: Point, theta_deg: float, radius: float, sweep_deg: float,
               side: float) -> Optional[Point]:
    """Start ``Q`` of a ``radius`` / ``sweep`` arc ending on ``point``.

    ``side = +1`` is a CCW arrival (centre left of the exit heading), ``-1``
    clockwise. The sweep is the angle travelled from ``Q`` to ``point``.
    """
    sweep = math.radians(sweep_deg)
    nx, ny = _left_normal(theta_deg)
    centre = (point[0] + side * radius * nx, point[1] + side * radius * ny)
    px, py = point[0] - centre[0], point[1] - centre[1]
    qx, qy = _rotate((px, py), -side * sweep)
    start = (centre[0] + qx, centre[1] + qy)
    if math.dist(start, point) <= 1.0:
        return None
    return start


def _arrival_on_run(run: Sequence[Point], point: Point, theta_deg: float,
                    grid: Optional[OccupancyGrid], legal_margin: float,
                    width: float, height: float
                    ) -> Tuple[Optional[ArrivalChoice], Optional[ArrivalChoice]]:
    """Best ARC and the in-place TURN that replace (or keep) a planned run.

    The run is the drawn approach itself; its prefix is legal by
    construction. Every candidate start is taken along the last
    ``MAX_APPROACH_CUT_MM`` of the run, so a huge angle change may shorten the
    drawing but never makes the detour longer than the shortest arc.
    """
    best_arc: Optional[ArrivalChoice] = None
    best_turn: Optional[ArrivalChoice] = None

    # in-place turn on the waypoint: keep the run exactly as drawn
    run_length = g.polyline_length(run)
    final_heading = _heading(run[-2], run[-1])
    best_turn = ArrivalChoice(
        "turn", point, theta_deg, None, tuple(run), True,
        run_length + _turn_cost(final_heading, theta_deg),
    )

    # walk back from the waypoint; stop after MAX_APPROACH_CUT_MM
    walked = 0.0
    index = len(run) - 1
    while index > 0:
        segment_start = run[index - 1]
        segment_end = run[index]
        segment = math.dist(segment_start, segment_end)
        if segment <= 1e-9:
            index -= 1
            continue
        # candidate starts every APPROACH_STEP_MM along this segment, from
        # the waypoint backwards (the cut length is measured from the point)
        max_back = MAX_APPROACH_CUT_MM - walked
        if max_back <= 0.0:
            break
        offset = segment - min(segment, max_back)
        while offset < segment:
            remaining = segment - offset
            start = (
                segment_end[0] + (segment_start[0] - segment_end[0]) * (remaining / segment),
                segment_end[1] + (segment_start[1] - segment_end[1]) * (remaining / segment),
            )
            prefix_length = g.polyline_length(run[:index]) + remaining
            cut_mm = max(0.0, run_length - prefix_length)
            arc = arc_from_start(point, start, theta_deg)
            if arc is not None and _arc_fits(arc, grid, legal_margin, width, height):
                approach = _heading(segment_start, segment_end)
                cost = (
                    prefix_length
                    + arc.length_mm
                    + _turn_cost(approach, arc.entry_deg)
                    - ARC_TIE_BREAK_MM
                    + cut_mm * APPROACH_CUT_PENALTY
                )
                if best_arc is None or cost < best_arc.cost_mm - 1e-6:
                    prefix = tuple(list(run[:index]) + [start])
                    best_arc = ArrivalChoice("arc", point, theta_deg, arc, prefix, True, cost)
            offset += APPROACH_STEP_MM
        walked += segment
        index -= 1
    return best_arc, best_turn


def _routed_arrival(anchor: Point, point: Point, theta_deg: float, arc: ArrivalArc,
                    grid: OccupancyGrid, legal_margin: float, width: float,
                    height: float) -> ArrivalChoice:
    """Cost one arc candidate with its real, obstacle-aware route to ``Q``."""
    try:
        route = theta_star(grid, anchor, arc.start)
    except NoPathError:
        return ArrivalChoice("arc", point, theta_deg, arc, (), False, math.inf)
    # theta_star snaps blocked endpoints to the nearest free cell; only an
    # exact landing on Q is connected to the arc
    if math.dist(route[-1], arc.start) > 1.0:
        return ArrivalChoice("arc", point, theta_deg, arc, tuple(route), False, math.inf)
    heading = _heading(route[-2], route[-1]) if len(route) >= 2 else None
    cost = _route_length(route) + arc.length_mm + _turn_cost(heading, arc.entry_deg) - ARC_TIE_BREAK_MM
    return ArrivalChoice("arc", point, theta_deg, arc, tuple(route), False, cost)


def _turn_route(anchor: Point, point: Point, theta_deg: float, grid: OccupancyGrid,
                legal_margin: float, width: float, height: float) -> Optional[ArrivalChoice]:
    """Route straight to the waypoint and turn in place (the safe fallback)."""
    try:
        route = theta_star(grid, anchor, point)
    except NoPathError:
        return None
    if math.dist(route[-1], point) > 1.0:
        # the waypoint is inside a clearance zone: the caller pre-snaps stop
        # points, so this is only reachable through a stale/illegal request
        return None
    heading = _heading(route[-2], route[-1]) if len(route) >= 2 else None
    cost = _route_length(route) + _turn_cost(heading, theta_deg)
    return ArrivalChoice("turn", point, theta_deg, None, tuple(route), False, cost)
