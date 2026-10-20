"""Planning pipeline (sequence model).

    elements --> line/arc fit (arcs stay arcs) --> routing (Theta*) -->
    corner rounding (tangent fillets) --> curvature-aware node sampling -->
    velocity profile --> native ARC segments for circular chains + GOTO
    nodes elsewhere; rotate / pause actions in sequence.

The input is an ordered ``elements`` list. Each element is one of:

    {"kind": "stroke", "points": [[x, y], ...]}   freehand/polyline geometry
    {"kind": "rotate", "angle_deg": 90.0}         in-place rotation (+ = CCW)
    {"kind": "pause",  "seconds": 2.0}            stop and wait
    {"kind": "release"}                            release the stepper drivers
                                                     (idle-only, no fault latch)

The output is a JSON-friendly program: absolute ``GOTO`` nodes (the firmware
corrects drift and pre-aligns the heading at every node), ``TURN`` segments
for the rotations and ``pause`` segments for the waits. Constant-curvature
chains (drawn arcs, corner fillets) are emitted as single ``ARC`` segments:
the firmware drives motion and rotation together, so the curve is followed
as one smooth move instead of a string of nodes. Contiguous strokes are
merged into one continuous run so corners between them are rounded too.

Smoothness comes from the plan: every sharp vertex is replaced by a tangent
fillet (the robot starts turning *before* the corner) and the node spacing
shrinks on curves so the chord error stays within ``node_chord_tol_mm`` and
each node turns by at most ``max_node_turn_deg`` - this is what makes a drawn
arc be followed as an arc instead of being cut down to a straight line.

File:    software/core/navigation/trajectory.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

from . import geometry as g
from .geometry import Point
from .grid import NoPathError, OccupancyGrid, theta_star
from .velocity import speed_profile

#: Largest speed the protocol carries (mm/s); the board's absolute ceiling.
PROTO_MAX_SPEED = 900
TURN_CHUNK_LIMIT_DEG = 170.0  # firmware TURN accepts at most 180 deg
ARC_JOINT_TOL_DEG = 1.0       # tangent break allowed between consecutive ARCs
ARC_CONNECT_MAX_DEG = 55.0    # bigger tangent breaks keep the in-place TURN
ARC_CONNECT_MAX_DEV_MM = 24.0  # corner cut cap (distance to the ORIGINAL arcs)
ARC_CONNECT_TRIM_MAX_DEG = 60.0  # most of a neighbour arc the connector may eat
ARC_CONNECT_MIN_SWEEP_DEG = 1.5  # neighbour arcs must keep at least this much
ARC_CONNECT_MIN_RADIUS_MM = 60.0  # connector radius scan bounds (firmware min 50)
ARC_CONNECT_MAX_RADIUS_MM = 3000.0
ARC_CONNECT_FAR_TRIM_MM = 60.0  # a near-straight ARC (> ARC_CONNECT_MAX_RADIUS_MM)
#                                  is only trimmed this far in MILLIMETRES: the
#                                  degree caps above would eat hundreds of mm of
#                                  a 0.5 deg sweep
JUNCTION_RELOCATE_MAX_MM = 120.0  # how far a junction may slide along its two
#                                   circles onto their meeting point: a tight-to-wide
#                                   transition of a drawn curve puts the curvature
#                                   step at the junction, where the circles are
#                                   (nearly) tangent and no connector exists
LAUNCH_WEAVE_MAX_LEAD_MM = 250.0   # only a SHORT opening lead is woven (the robot
#                                    stands at the start - a far one keeps driving)
LAUNCH_WEAVE_MIN_REST_DEG = 12.0   # the first drawn arc keeps at least this much
LAUNCH_WEAVE_MAX_SWEEP_DEG = 90.0  # the connector stays a gentle launch
LAUNCH_WEAVE_MAX_DEV_MM = 90.0     # deviation from the drawn opening it replaces
LAUNCH_WEAVE_LEGAL_SKIP_MM = 60.0  # the robot may stand inside the border strip
#                                    (match start y=1800): the first millimetres
#                                    leaving it are exempt, like a lead GOTO is
LEGAL_PAD_MM = 12.0           # keep the body a little clear of the border line
LEGAL_EPS_MM = 1e-3           # boundary points are legal, not a hair outside
WAYPOINT_STANDOFF_STEP_MM = 10.0  # search step of the straight final-leg stand-off
WAYPOINT_STANDOFF_MIN_MM = 25.0   # shortest final leg that still forms a real node
WAYPOINT_STANDOFF_MAX_MM = 1200.0  # farthest safe point worth trying [mm]
DEPARTURE_TURN_TOL_DEG = 0.5      # a continuation this close to the arrival heading
#                                   needs no departure move at all
WEAVE_MAX_DEG = 155.0         # arrival turns above this are not arc-woven
ARRIVAL_MAX_BOW_MM = 10.0     # the arrival weave may bow the approach leg (and
#                               re-aim the near-straight ARC it starts on) no
#                               further than this off the drawn path - past it
#                               the stop-and-turn fallback is the honest answer
ANCHOR_SNAP_MM = 15.0         # closer than this: treat as the same point
DEDUPE_NODE_MM = 2.0          # drop only true duplicate GOTO nodes - a larger
#                               threshold would swallow the dense samples of a
#                               small fillet and turn it into one in-place pivot
CORNER_CUT_MAX_MM = 1200.0    # how far back a detour may start to skip a drawn
#                               dead-end approach (it saves the walk-in AND the
#                               reversal - "the robot did a return")
CORNER_CUT_MIN_SAVE_MM = 120.0  # only cut when it is at least this much shorter
CORNER_CUT_MAX_STEPS = 40     # ...and never look further back than this many points

#: near-straight ARCs (see _near_straight_arcs): the straight runs of a
#: drawing drive as VERY flat arcs, so the whole path chains through
#: QUEUE_ARC instead of stopping at a GOTO node every goto_step_mm (the
#: board brakes to a full stop at every GOTO node - TURN/DRIVE/TURN with a
#: ramp reset per segment, a "stop and turn" between every part of the path)
LINE_ARC_RADIUS_MAX_MM = 32767.0   # protocol cap (radius travels as i16 mm)
LINE_ARC_SWEEP_QUANT_DEG = 0.1     # sweep travels as i16 * 0.1 deg
LINE_ARC_MIN_SPAN_MM = 30.0        # shorter gaps keep their GOTO nodes
LINE_ARC_ONE_PIECE_MM = 120.0      # up to this a span drives as ONE flat arc
LINE_ARC_DIR_SLOPE = 0.00218       # dev <= span * slope  (~0.5 deg at the ends):
#                                    a bowed span is NOT straight and stays GOTO
LINE_ARC_PROTECT_MM = 6.0          # an arrival point (waypoint angle) must keep
#                                    its own GOTO node: a flat ARC may not
#                                    swallow it (the arrival weave needs it)
LINE_ARC_ARRIVAL_CLEAR_MM = 300.0  # ...and the stretch before it stays GOTO
#                                    nodes too: the weave replaces that stub
#                                    with a tangent arrival arc (the node
#                                    spacing on a straight is goto_step_mm,
#                                    so the stub must reach one node back)
CHAIN_MIN_DURATION_S = 0.2         # shortest chained segment lifetime: the
#                                    console refills the board's single pending
#                                    slot from the CAN/odometry poll, a 5 mm
#                                    arc at 130 mm/s would run the slot dry and
#                                    the board braked in the middle of a chain
CHAIN_REANCHOR_MM = 1800.0         # longest run that drives on relative ARCs
#                                    alone: the board blends them without
#                                    stopping, but every ARC ends on a +-0.8 deg
#                                    tolerance and slip is never fed back, so
#                                    past this the plan inserts an ABSOLUTE GOTO
#                                    (one stop, position taken from the plan)
ARC_WIRE_MIN_RADIUS_MM = 50.0      # firmware floor (motion.c), 55 mm of margin
#                                    was found unnecessary: the board accepts 50
ARC_WIRE_MAX_RADIUS_MM = 32767.0   # protocol i16 mm
ARC_WIRE_MIN_LENGTH_MM = 2.0       # motion.c rejects arc length < 1 mm (RANGE)
ARC_WIRE_MAX_LENGTH_MM = 2900.0    # firmware limit is 3000 mm
ARC_WIRE_FLAT_MM = 20000.0         # above this radius an ARC is a NEAR-STRAIGHT
#                                    one (the fitted-arc cap is 20000): its
#                                    sweep is derived from its chord, not from
#                                    the sampled polyline
ARC_NODE_SNAP_MM = 5.0             # a GOTO node within this distance of the
#                                    arc that precedes it is the SAME junction
#                                    (the node comes from the reference path,
#                                    the arc from the fit): the node is moved
#                                    onto the arc's true exit - a shorter miss
#                                    is a 0.5 mm back-step, a stop and a creep
#                                    for nothing (see _sanitize_arcs)
ARC_CHAIN_TANGENT_DEG = 5.0        # a node that faces within this of the arc
#                                    that arrives at it was NOT meant as a
#                                    pivot: when the sanitiser moves that
#                                    arc's exit heading the node follows it,
#                                    so no "touch-up" turn appears there

#: a hand drawn switchback (cusp) has an equivalent radius far below the
#: firmware's 50 mm ARC minimum; it drives as a stop-go stutter of tiny GOTO
#: nodes - see _collapse_cusps
CUSP_MIN_NODES = 2          # a cusp run needs at least this many GOTO nodes
CUSP_MIN_TURN_DEG = 70.0    # ... turns at least this much end to end ...
CUSP_MAX_RADIUS_MM = 60.0   # ... with an equivalent radius below the ARC floor
CUSP_MIN_PROGRESS = 0.5     # ... and still makes progress (an out-and-back jab
#                             returns to its start: never collapsed)
CUSP_MAX_SPAN_MM = 150.0    # safety cap on the run's travelled length

from .arrival import choose_arrival
from .clearance import (
    body_clear_at,
    body_geometry,
    body_inflation,
    build_grid,
    clearances,
    closest_body_clear_point,
    closest_clear_point,
    obstacle_clearance,
    point_clearance,
    rotation_clearance,
    straight_approach_clear,
    waypoint_clearance,
)
from .options import PlanOptions


def _clamp_obstacles(
    obstacles: Sequence[Dict], field_width_mm: float, field_height_mm: float
) -> List[Dict]:
    """Keep drawn obstacles inside the playfield.

    A rectangle poking out of the table is clipped to the border; a circle
    is moved/shrunk so its whole disc stays on the table. The planner only
    uses the field grid, but the console and the saved strategies must agree
    with it: an obstacle drawn half outside used to leave a phantom part of
    the clearance zone outside the map and made the drawing lie.
    """
    width = max(0.0, float(field_width_mm))
    height = max(0.0, float(field_height_mm))
    clamped: List[Dict] = []
    for obstacle in obstacles:
        if not isinstance(obstacle, dict):
            continue
        kind = str(obstacle.get("kind", "rect"))
        item = dict(obstacle)
        if kind == "rect":
            x = float(item.get("x", 0.0) or 0.0)
            y = float(item.get("y", 0.0) or 0.0)
            w = abs(float(item.get("w", 0.0) or 0.0))
            h = abs(float(item.get("h", 0.0) or 0.0))
            x0 = min(max(x, 0.0), width)
            y0 = min(max(y, 0.0), height)
            x1 = min(max(x + w, 0.0), width)
            y1 = min(max(y + h, 0.0), height)
            item["x"], item["y"] = x0, y0
            item["w"], item["h"] = max(0.0, x1 - x0), max(0.0, y1 - y0)
        elif kind == "circle":
            r = abs(float(item.get("r", 0.0) or 0.0))
            r = min(r, 0.5 * min(width, height))
            x = min(max(float(item.get("x", 0.0) or 0.0), r), max(r, width - r))
            y = min(max(float(item.get("y", 0.0) or 0.0), r), max(r, height - r))
            item["x"], item["y"], item["r"] = x, y, r
        clamped.append(item)
    return clamped


def plan(
    elements: Sequence[Dict],
    start_pose: Tuple[float, ...],
    goal: Optional[Dict],
    obstacles: Sequence[Dict],
    options: PlanOptions,
    field_width_mm: float,
    field_height_mm: float,
) -> Dict:
    """Run the whole pipeline and return a JSON-friendly program."""
    obstacles = _clamp_obstacles(obstacles, field_width_mm, field_height_mm)
    warnings: List[str] = []
    primitives: List[Dict] = []
    segments: List[Dict] = []
    reference: List[Point] = []
    samples_all: List[Point] = []
    length_mm = 0.0
    est_s = 0.0
    turn_speed = int(max(options.min_speed_mm_s, min(options.turn_speed_mm_s, options.speed_mm_s)))

    grid: Optional[OccupancyGrid] = None
    relaxed_grid: Optional[OccupancyGrid] = None
    body_grid: Optional[OccupancyGrid] = None
    inflation = 0.0
    if options.avoid_obstacles:
        grid = build_grid(field_width_mm, field_height_mm, obstacles, options)
        border, inflation = clearances(options)
        # tier 2 (routing fallback + nothing else): border + half the margin
        soft = border + 0.5 * max(0.0, inflation - border)
        if soft < inflation - 0.01:
            relaxed_grid = build_grid(field_width_mm, field_height_mm, obstacles, options, soft)
        # tier 3 (PHYSICAL clearance: half-diagonal + a small buffer): the drawn
        # geometry is only re-routed where the robot body would hit something -
        # legs that merely use part of the margin are kept as drawn
        body_infl = body_inflation(options)
        body_grid = (
            build_grid(field_width_mm, field_height_mm, obstacles, options, body_infl)
            if body_infl < inflation - 0.01
            else grid
        )

    start = (float(start_pose[0]), float(start_pose[1]))
    # the pose heading is optional: with (x, y) the planner does not know how
    # the robot is oriented, and a run that begins with a turn falls back to
    # plain GOTO nodes (no ARC may start at the run origin without a known
    # entry heading)
    start_heading_deg = float(start_pose[2]) if len(start_pose) > 2 else None
    waypoint_points: List[Point] = []
    #: effective waypoint positions (illegal ones are projected out of the
    #: clearance zones): the console moves its markers there and the mission
    #: cursor uses them, so a moved point is still "visited"
    resolved_waypoints: List[List[float]] = []

    # legal centre region: the body sweeps the half-diagonal circle, so the
    # centre must stay that far from the field border for EVERY heading -
    # drawn geometry that pokes out is clamped back in (with a warning).
    # A small pad keeps the body visibly clear of the border line, also when
    # the robot turns in place at the very end of the mission.
    pose_radius = 0.5 * math.hypot(
        max(50.0, float(options.robot_width_mm)), max(50.0, float(options.robot_height_mm))
    )
    legal_margin = pose_radius + LEGAL_PAD_MM
    border_clamped = False

    def clamp_inside(point: Point) -> Point:
        """Clamp a centre point so the whole body stays on the table."""
        nonlocal border_clamped
        # a point drawn exactly on the legal boundary (the console clamps
        # there) must not be moved by a floating-point hair: that used to
        # inject a zero-length GOTO at the robot pose and force the whole
        # plan onto the stop-and-turn backend
        max_x = max(legal_margin, field_width_mm - legal_margin - LEGAL_EPS_MM)
        max_y = max(legal_margin, field_height_mm - legal_margin - LEGAL_EPS_MM)
        x = min(max(point[0], legal_margin), max_x)
        y = min(max(point[1], legal_margin), max_y)
        if abs(x - point[0]) > 0.5 or abs(y - point[1]) > 0.5:
            border_clamped = True
        return (x, y)

    def arc_stays_legal(points) -> bool:
        """Do the samples of a generated approach arc stay on the table?

        Checked against the FULL clearance (field border + obstacle margin):
        a generated manoeuvre must never leave the game area or clip an
        obstacle's robot-clearance zone, even when the drawn geometry it
        replaces was allowed to use part of the margin.
        """
        for x, y in points:
            if x < legal_margin or x > field_width_mm - legal_margin:
                return False
            if y < legal_margin or y > field_height_mm - legal_margin:
                return False
            if grid is not None and not grid.is_free_mm(x, y):
                return False
        return True

    # ---- configured waypoint stop clearance (Robot settings) ------------
    # When ``waypoint_margin_mm`` is set, a waypoint is a STOP point whose
    # clearance is the physical body rule (half the larger body side + the
    # configured extra margin), not the routing margin. Such a waypoint may
    # stand inside the routing grid; it is then reached by a STRAIGHT final
    # leg from a safe stand-off. When the point is too close to turn in place
    # (less than the half-diagonal), the arrival angle is performed at the
    # stand-off and the robot drives the last stretch already facing it -
    # rotating ON the waypoint would sweep a body corner into the obstacle.
    waypoint_mode = options.waypoint_margin_mm is not None
    waypoint_clear = waypoint_clearance(options)
    waypoint_obstacles = list(obstacles) if options.avoid_obstacles else []
    body_width, body_height = body_geometry(options)
    turn_clearance = rotation_clearance(options)

    def waypoint_clearance_at(point: Point) -> float:
        """Actual free distance of a waypoint from obstacles and borders."""
        return point_clearance(
            point, waypoint_obstacles, field_width_mm, field_height_mm,
        )

    def waypoint_can_turn_at(point: Point) -> bool:
        """Can the robot turn in place on ``point`` without touching?"""
        return waypoint_clearance_at(point) >= turn_clearance

    def waypoint_heading_hint(point: Point, theta: Optional[float]) -> float:
        """Heading used to check the body: the arrival angle, else the leg."""
        if theta is not None:
            return float(theta)
        dx, dy = point[0] - anchor[0], point[1] - anchor[1]
        if math.hypot(dx, dy) > 1.0:
            return math.degrees(math.atan2(dy, dx))
        return float(anchor_heading if anchor_heading is not None else 0.0)

    def place_waypoint(raw: Point, theta: Optional[float]) -> Tuple[Point, float]:
        """Apply the configured stop clearance to a waypoint position.

        The raw centre-distance projection is refined until the REAL rotated
        body fits too: a waypoint hugging a wall at 45 deg needs the corner
        clearance, not just half the body side. The returned heading is the
        one the body was checked with (an approximation of the final leg).
        """
        hint = waypoint_heading_hint(raw, theta)
        # A point the operator placed exactly ON the configured limit is
        # kept EXACTLY: the half-millimetre pad below is for projecting a
        # violating point out, not for pulling a legal stop inwards (the
        # reported "reach the waypoint exactly" case).
        if (
            waypoint_clearance_at(raw) >= waypoint_clear - 1e-6
            and body_clear_at(
                raw, hint, waypoint_obstacles, body_width, body_height,
                field_width_mm, field_height_mm, pad_mm=0.0,
            )
        ):
            return raw, hint
        placed = raw
        for _ in range(3):
            placed = closest_body_clear_point(
                raw, hint, waypoint_obstacles, waypoint_clear,
                body_width, body_height, field_width_mm, field_height_mm,
            )
            new_hint = waypoint_heading_hint(placed, theta)
            if abs(g.deg_norm180(new_hint - hint)) <= 0.5:
                hint = new_hint
                break
            hint = new_hint
        if math.dist(placed, raw) > 0.5:
            warnings.append(
                "waypoint ({:.0f}, {:.0f}) mm sits inside the configured waypoint "
                "clearance ({:.0f} mm) - moved to the closest allowed point "
                "({:.0f}, {:.0f}) mm".format(
                    raw[0], raw[1], waypoint_clear, placed[0], placed[1]
                )
            )
        return placed, hint

    def find_close_approach(
        point: Point, heading_deg: float
    ) -> Optional[Tuple[Point, List[Point]]]:
        """Safe stand-off on the reverse final heading + the route to it.

        The final leg runs from the stand-off to ``point`` along
        ``heading_deg``. The stand-off must be turn-safe (the board finishes
        the GOTO there facing the leg) and fully route-reachable; the leg is
        sampled and checked against the REAL rotated body, so neither the
        final approach nor the in-place rotation can touch the obstacle.
        """
        rad = math.radians(heading_deg)
        ux, uy = math.cos(rad), math.sin(rad)
        max_d = min(
            max(WAYPOINT_STANDOFF_MAX_MM, 2.0 * inflation + pose_radius + 400.0),
            math.hypot(field_width_mm, field_height_mm),
        )
        distance = WAYPOINT_STANDOFF_MIN_MM
        while distance <= max_d:
            standoff = (point[0] - ux * distance, point[1] - uy * distance)
            inside = (
                0.0 <= standoff[0] <= field_width_mm
                and 0.0 <= standoff[1] <= field_height_mm
            )
            if inside:
                route_ok = True
                if grid is not None:
                    border_distance = min(
                        standoff[0], standoff[1],
                        field_width_mm - standoff[0],
                        field_height_mm - standoff[1],
                    )
                    route_ok = (
                        grid.is_free_mm(standoff[0], standoff[1])
                        and obstacle_clearance(standoff, waypoint_obstacles)
                        >= inflation - 1e-6
                        and border_distance >= grid.border - 1e-6
                    )
                if route_ok and waypoint_can_turn_at(standoff):
                    leg_ok = straight_approach_clear(
                        standoff, point, heading_deg, waypoint_obstacles,
                        body_width, body_height, field_width_mm, field_height_mm,
                    )
                    if leg_ok:
                        route: List[Point] = [anchor, standoff]
                        if grid is not None:
                            route = _route(
                                grid, route, warnings, relaxed_grid, body_grid,
                            )
                        elif math.dist(anchor, standoff) > 1.0:
                            # no routing grid: the connector is the straight
                            # line from the robot, so check that body pose too
                            connector_heading = math.degrees(math.atan2(
                                standoff[1] - anchor[1],
                                standoff[0] - anchor[0],
                            ))
                            if not straight_approach_clear(
                                anchor, standoff, connector_heading,
                                waypoint_obstacles, body_width, body_height,
                                field_width_mm, field_height_mm,
                            ):
                                distance += WAYPOINT_STANDOFF_STEP_MM
                                continue
                        if math.dist(route[-1], standoff) <= 1.0:
                            return standoff, route
            distance += WAYPOINT_STANDOFF_STEP_MM
        return None

    def approach_waypoint_straight(
        point: Point, theta: Optional[float], natural_heading: float
    ) -> Optional[Dict]:
        """Straight-in approach for a waypoint inside the routing margin.

        ``theta`` is the commanded arrival angle (None for a pass-through
        stop). When the point is turn-safe the robot may rotate there, so a
        stand-off along the natural approach is enough; otherwise the final
        leg must already run along ``theta`` and the rotation is finished at
        the stand-off (``pre_rotated``).
        """
        can_turn = waypoint_can_turn_at(point)
        preferred: List[float] = []
        if theta is not None:
            preferred.append(float(theta))
        preferred.append(natural_heading)
        if theta is not None and not can_turn:
            # rotating ON this point is unsafe, so the ONLY legal final
            # heading is the commanded one: the rotation must be finished at
            # the stand-off before the straight leg
            candidates = [float(theta)]
        else:
            # The point is turn-safe, or no final orientation is requested:
            # the final leg may come from any direction. The natural line is
            # tried first; when it hugs the obstacle (e.g. a wall parallel to
            # the approach) a compass scan finds the open side.
            candidates = list(preferred)
            candidates.extend(float(angle) for angle in range(0, 360, 45))
        seen: List[float] = []
        for heading in candidates:
            if any(abs(g.deg_norm180(heading - other)) <= 1.0 for other in seen):
                continue
            seen.append(heading)
            found = find_close_approach(point, heading)
            if found is None:
                continue
            standoff, route = found
            return {
                "standoff": standoff,
                "route": route,
                "heading": heading,
                "pre_rotated": (
                    theta is not None
                    and abs(g.deg_norm180(heading - float(theta))) <= 1.0
                ),
            }
        return None

    anchor_heading: Optional[float] = start_heading_deg

    def tail_heading() -> Optional[float]:
        """Travel heading at the end of the last appended run (None when the
        last program entry is a rotation/pause - actions keep their heading)."""
        if program and program[-1]["kind"] == "run" and len(program[-1]["points"]) >= 2:
            a, b = program[-1]["points"][-2], program[-1]["points"][-1]
            if math.dist(a, b) > 1.0:
                return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        return None

    # ------- pass 1: ordered program - continuous runs and action steps ----
    program: List[Dict] = []
    #: waypoints carrying an arrival angle: {point, theta, woven} - the
    #: heading is applied after emission (a woven arc exits at the angle,
    #: a plain approach turns in place on the node)
    arrival_targets: List[Dict] = []

    def append_run(points: Sequence[Point], speed_mm_s: Optional[float] = None) -> None:
        """Add geometry; connected pieces merge into one continuous run.

        The merge tolerance equals the anchor snap: two strokes closer than
        ``ANCHOR_SNAP_MM`` are one path (otherwise the joint would sit on a run
        boundary and never be corner-rounded), while a larger gap is bridged
        by a connector leg before this call. A ``cut`` marker (waypoints)
        always starts a fresh run so the node lands exactly on the point.

        ``speed_mm_s`` is a per-step override: only THIS run cruises at most
        that fast (the system speed governs every other run). When pieces
        merge into an existing run, the first override wins.
        """
        pts = g.dedupe_consecutive(points, 0.5)
        if len(pts) < 2:
            return
        if (
            program
            and program[-1]["kind"] == "run"
            and math.dist(program[-1]["points"][-1], pts[0]) <= ANCHOR_SNAP_MM
        ):
            program[-1]["points"].extend(pts[1:])
            if speed_mm_s is not None and "speed_mm_s" not in program[-1]:
                program[-1]["speed_mm_s"] = float(speed_mm_s)
        else:
            entry: dict = {"kind": "run", "points": list(pts)}
            if speed_mm_s is not None:
                entry["speed_mm_s"] = float(speed_mm_s)
            program.append(entry)

    # ---- close-waypoint DEPARTURE (back up, turn, continue) --------------
    # A close waypoint is reached EXACTLY (that is the operator's stop
    # point). If the mission continues and the body cannot rotate there, the
    # robot first makes a straight longitudinal move (reverse out of the
    # tight spot, or forward when the obstacle is behind it) to a turn-safe,
    # route-free point, rotates there and continues. The move is inserted
    # lazily so a `pause` still happens AT the waypoint.
    pending_departure: Optional[Dict] = None
    departure_join: Optional[Point] = None
    departure_origin: Optional[Point] = None

    def departure_ok(point: Point, heading_deg: float, candidate: Point) -> bool:
        """Can the robot stop at ``candidate`` after a straight departure?"""
        if not (
            0.0 <= candidate[0] <= field_width_mm
            and 0.0 <= candidate[1] <= field_height_mm
        ):
            return False
        border_distance = min(
            candidate[0], candidate[1],
            field_width_mm - candidate[0],
            field_height_mm - candidate[1],
        )
        if grid is not None:
            route_ok = (
                grid.is_free_mm(candidate[0], candidate[1])
                and obstacle_clearance(candidate, waypoint_obstacles)
                >= inflation - 1e-6
                and border_distance >= grid.border - 1e-6
            )
        else:
            route_ok = border_distance >= legal_margin
        if not route_ok or not waypoint_can_turn_at(candidate):
            return False
        return straight_approach_clear(
            candidate, point, heading_deg, waypoint_obstacles,
            body_width, body_height, field_width_mm, field_height_mm,
        )

    def find_departure(point: Point, heading_deg: float) -> Optional[Dict]:
        """Straight move out of a close stop to a turn-safe point.

        Both longitudinal directions are tried, reverse first (the reported
        case: the waypoint hugs the border ahead of the robot, so backing up
        is the natural escape). The move must leave the body clear along its
        whole length, end fully route-reachable and require the true
        half-diagonal clearance to rotate there. ``distance_mm`` is signed in
        the robot frame: negative = reverse, positive = forward.
        """
        rad = math.radians(heading_deg)
        ux, uy = math.cos(rad), math.sin(rad)
        max_d = min(
            max(WAYPOINT_STANDOFF_MAX_MM, 2.0 * turn_clearance + 200.0),
            math.hypot(field_width_mm, field_height_mm),
        )
        for sign in (-1.0, 1.0):
            distance = WAYPOINT_STANDOFF_MIN_MM
            while distance <= max_d:
                candidate = (
                    point[0] + sign * ux * distance,
                    point[1] + sign * uy * distance,
                )
                if departure_ok(point, heading_deg, candidate):
                    return {
                        "point": candidate,
                        "distance_mm": sign * distance,
                    }
                distance += WAYPOINT_STANDOFF_STEP_MM
        return None

    def outgoing_heading(next_element: Dict) -> Optional[float]:
        """Where the mission goes right after the close stop (None = turn)."""
        kind = str(next_element.get("kind", "stroke"))
        if kind == "stroke":
            pts = [
                (float(p[0]), float(p[1]))
                for p in next_element.get("points", [])
                if len(p) >= 2
            ]
            if len(pts) < 2:
                return None
            if math.dist(pts[0], anchor) > max(ANCHOR_SNAP_MM * 2.0, 50.0):
                target = pts[0]  # the connector to the stroke's start
            else:
                target = pts[-1]
                for candidate in pts[1:]:
                    if math.dist(pts[0], candidate) >= 90.0:
                        target = candidate
                        break
            dx, dy = target[0] - pts[0][0], target[1] - pts[0][1]
            if math.hypot(dx, dy) <= 1e-6:
                return None
            return math.degrees(math.atan2(dy, dx))
        if kind in ("waypoint", "goal"):
            dx = float(next_element.get("x", 0.0)) - anchor[0]
            dy = float(next_element.get("y", 0.0)) - anchor[1]
            if math.hypot(dx, dy) <= 1e-6:
                return None
            return math.degrees(math.atan2(dy, dx))
        if kind == "drive":
            # a drive keeps the current heading (forward or reverse)
            return float(anchor_heading if anchor_heading is not None else 0.0)
        return None  # explicit rotation / unknown: a move is required

    def insert_pending_departure(next_element: Dict) -> None:
        """Emit the planned straight move before the continuation."""
        nonlocal pending_departure, anchor, anchor_heading
        nonlocal departure_join, departure_origin
        pending = pending_departure
        if pending is None:
            return
        pending_departure = None
        outgoing = outgoing_heading(next_element)
        if (
            outgoing is not None
            and abs(g.deg_norm180(outgoing - pending["heading"]))
            <= DEPARTURE_TURN_TOL_DEG
        ):
            return  # the robot simply continues along the same line
        departure = pending.get("departure")
        if departure is None:
            return
        point = pending["point"]
        heading = float(pending["heading"])
        target = departure["point"]
        distance = float(departure["distance_mm"])
        if str(next_element.get("kind", "stroke")) == "stroke":
            raw = [
                (float(p[0]), float(p[1]))
                for p in next_element.get("points", [])
                if len(p) >= 2
            ]
            if (
                len(raw) >= 2
                and math.dist(raw[0], point) <= max(ANCHOR_SNAP_MM * 2.0, 50.0)
            ):
                # The stroke starts ON the waypoint. A straight connector
                # from the departure point meets it at a right angle (the
                # departure line is offset from the stroke), so the join
                # point itself must also be turn-safe. Back up a little
                # farther until it is - otherwise the merge turn would touch
                # the very border/obstacle the waypoint hugs.
                def join_ok(join: Point, start: Point) -> bool:
                    travel = math.degrees(math.atan2(
                        join[1] - start[1], join[0] - start[0],
                    ))
                    return (
                        waypoint_can_turn_at(join)
                        and straight_approach_clear(
                            start, join, travel, waypoint_obstacles,
                            body_width, body_height, field_width_mm, field_height_mm,
                        )
                    )

                sliced = stroke_after_departure(raw, target, point)
                if sliced and not join_ok(sliced[0], target):
                    sign = 1.0 if distance > 0.0 else -1.0
                    ux = math.cos(math.radians(heading))
                    uy = math.sin(math.radians(heading))
                    max_d = min(
                        max(WAYPOINT_STANDOFF_MAX_MM, 2.0 * turn_clearance + 200.0),
                        math.hypot(field_width_mm, field_height_mm),
                    )
                    step = abs(distance) + WAYPOINT_STANDOFF_STEP_MM
                    while step <= max_d:
                        candidate = (
                            point[0] + sign * ux * step,
                            point[1] + sign * uy * step,
                        )
                        if departure_ok(point, heading, candidate):
                            sliced = stroke_after_departure(raw, candidate, point)
                            if sliced and join_ok(sliced[0], candidate):
                                target = candidate
                                distance = sign * step
                                break
                        step += WAYPOINT_STANDOFF_STEP_MM
        departure_step = {
            "kind": "depart",
            "distance_mm": distance,
            "points": [
                [round(point[0], 1), round(point[1], 1)],
                [round(target[0], 1), round(target[1], 1)],
            ],
        }
        if pending.get("speed_mm_s"):
            departure_step["speed_mm_s"] = float(pending["speed_mm_s"])
        program.append(departure_step)
        warnings.append(
            "waypoint ({:.0f}, {:.0f}) mm is too close to rotate in place - "
            "reached exactly, then the robot {} {:.0f} mm to ({:.0f}, {:.0f}) mm "
            "before turning to continue".format(
                point[0], point[1],
                "backs up" if distance < 0.0 else "moves forward",
                abs(distance), target[0], target[1],
            )
        )
        anchor = target
        departure_join = target
        departure_origin = point
        anchor_heading = pending["heading"]

    def stroke_after_departure(
        points: Sequence[Point], join: Point, origin: Point
    ) -> List[Point]:
        """Cut a following stroke where it is nearest the departure point.

        The stroke normally starts ON the waypoint; after backing away, the
        robot must not drive back to that tight point. The leading piece is
        replaced by the connector from the departure point to the nearest
        stroke point, and the rest keeps its drawn direction.
        """
        pts = list(points)
        if len(pts) < 2 or math.dist(pts[0], origin) > max(ANCHOR_SNAP_MM * 2.0, 50.0):
            return pts
        best_distance = math.inf
        best_along = 0.0
        walked = 0.0
        for index in range(1, len(pts)):
            ax, ay = pts[index - 1]
            bx, by = pts[index]
            dx, dy = bx - ax, by - ay
            length2 = dx * dx + dy * dy
            if length2 <= 1e-12:
                continue
            t = max(
                0.0,
                min(1.0, ((join[0] - ax) * dx + (join[1] - ay) * dy) / length2),
            )
            qx, qy = ax + dx * t, ay + dy * t
            distance = math.hypot(join[0] - qx, join[1] - qy)
            if distance < best_distance:
                best_distance = distance
                best_along = walked + math.sqrt(length2) * t
            walked += math.sqrt(length2)
        if best_along <= 0.5:
            return pts
        remaining = best_along
        for index in range(1, len(pts)):
            ax, ay = pts[index - 1]
            bx, by = pts[index]
            span = math.hypot(bx - ax, by - ay)
            if span <= 1e-9:
                continue
            if remaining <= span:
                t = remaining / span
                cut = (ax + (bx - ax) * t, ay + (by - ay) * t)
                out = [cut] + pts[index:]
                return out if len(out) >= 2 else pts
            remaining -= span
        return pts

    def later_motion_needs_turn(index: int) -> bool:
        """Does anything after this element still move or rotate the robot?

        A close waypoint may only stay close when it is a TERMINAL stop (a
        following pause is fine). Any later motion needs either a safe
        departure move or, for an explicit rotation, a turn-safe point.
        """
        for later in elements[index + 1:]:
            kind = str(later.get("kind", "stroke"))
            if kind in ("rotate", "drive", "waypoint"):
                return True
            if kind == "stroke" and len(later.get("points", [])) >= 2:
                return True
        return goal is not None

    anchor = start
    first_stroke = True
    for element_index, element in enumerate(elements):
        kind = element.get("kind", "stroke")
        # a pending departure is inserted before the first REAL motion after
        # the close waypoint; a pause or motor release still happens ON the
        # waypoint itself (neither moves the robot)
        if pending_departure is not None and kind not in ("pause", "release"):
            insert_pending_departure(element)
        if kind != "stroke":
            departure_join = None
            departure_origin = None
        if kind == "stroke":
            departure_sliced = False
            pts = [(float(p[0]), float(p[1])) for p in element.get("points", []) if len(p) >= 2]
            if len(pts) < 2:
                departure_join = None
                departure_origin = None
                continue
            if departure_join is not None and departure_origin is not None:
                original = pts
                pts = stroke_after_departure(pts, departure_join, departure_origin)
                departure_sliced = pts != original
                departure_join = None
                departure_origin = None
            stroke_points = _fit_stroke(pts, options, primitives)
            if len(stroke_points) < 2:
                continue
            if grid is not None:
                routed = _route(grid, stroke_points, warnings, relaxed_grid, body_grid)
                if len(routed) >= 2:
                    stroke_points = routed
                else:
                    # the router collapsed the stroke - a short piece lying
                    # fully inside the border/clearance strip (e.g. right at
                    # the top edge) is "moved to the closest allowed point",
                    # which for a 2-point stroke leaves ONE point. Fall back
                    # to the clamped drawing: losing the step silently also
                    # killed whole plans that were just this step + actions
                    # ("no usable stroke - draw a path first").
                    stroke_points = [clamp_inside(p) for p in stroke_points]
            if departure_sliced:
                # the continuation was deliberately cut near the departure
                # point, which is inside the cosmetic border pad but on the
                # router's safe grid: clamping it back would bend the join
                stroke_points = [
                    (
                        min(max(p[0], 0.0), field_width_mm),
                        min(max(p[1], 0.0), field_height_mm),
                    )
                    for p in stroke_points
                ]
            else:
                stroke_points = [clamp_inside(p) for p in stroke_points]
            if math.dist(anchor, stroke_points[0]) > ANCHOR_SNAP_MM:
                connector = [anchor, stroke_points[0]]
                if grid is not None:
                    connector = _route(grid, connector, warnings, relaxed_grid, body_grid)
                if first_stroke and math.dist(anchor, start) < 1.0 and not options.resume_mid_mission:
                    warnings.append(
                        "the first path starts away from the robot - connector leg added from the robot position"
                    )
                append_run(connector + list(stroke_points), element.get("speed_mm_s"))
            else:
                append_run(stroke_points, element.get("speed_mm_s"))
            if (
                departure_sliced
                and program
                and program[-1].get("kind") == "run"
            ):
                # the router already keeps this leg on the safe grid; the
                # 12 mm cosmetic pad must not bend the departure join
                program[-1]["close_connector"] = True
            first_stroke = False
            anchor = stroke_points[-1]
            anchor_heading = tail_heading() or anchor_heading
        elif kind == "rotate":
            program.append(
                {
                    "kind": "rotate",
                    "angle_deg": float(element.get("angle_deg", 0.0)),
                    **(
                        {"speed_mm_s": float(element["speed_mm_s"])}
                        if element.get("speed_mm_s")
                        else {}
                    ),
                }
            )
            anchor_heading = (anchor_heading or 0.0) + float(element.get("angle_deg", 0.0))
        elif kind == "pause":
            program.append({"kind": "pause", "seconds": float(element.get("seconds", 1.0))})
        elif kind == "release":
            # action with no geometry: the executor sends the dedicated
            # RELEASE_MOTORS command once the previous move is fully idle
            program.append({"kind": "release"})
        elif kind == "waypoint":
            # mandatory pass-through position: the path is routed through the
            # point and then SWEPT smoothly (the corner fillet is capped by
            # waypoint_cut_mm), like a drawn corner - not a knife-point turn.
            # With waypoint_cut_mm = 0 the run is CUT around the waypoint so
            # an exact GOTO node lands on it (knife-point mode). A waypoint
            # that carries an arrival angle is always exact, so the heading
            # constraint can land on a real node.
            raw_point = (float(element.get("x", 0.0)), float(element.get("y", 0.0)))
            theta_deg: Optional[float] = None
            if element.get("theta_deg") is not None:
                theta_deg = float(element["theta_deg"])
            combine = bool(element.get("combine"))
            waypoint_speed = (
                float(element["speed_mm_s"])
                if element.get("speed_mm_s") else None
            )
            if waypoint_mode:
                # configured stop clearance (Robot setting): half the body
                # side + waypoint_margin_mm, with the rotated body checked
                # too. This may leave the point inside the ROUTING margin;
                # that is legal for a stop and handled by the close branch.
                point, _placement_heading = place_waypoint(raw_point, theta_deg)
            else:
                point = clamp_inside(raw_point)
                # STOP points respect the FULL clearance: the robot comes to
                # rest here (and may turn in place), so the configured
                # obstacle margin must hold. The route itself is re-routed at
                # the same margin; a waypoint may never be projected only to
                # body clearance - the robot would stop close enough to touch
                # the obstacle.
                if grid is not None and not grid.is_free_mm(point[0], point[1]):
                    snapped = closest_clear_point(
                        point, obstacles, inflation, legal_margin,
                        field_width_mm, field_height_mm,
                    )
                    if not grid.is_free_mm(snapped[0], snapped[1]):
                        try:
                            snapped = grid.nearest_free(point[0], point[1])
                        except NoPathError:
                            snapped = point
                    if math.dist(snapped, point) > 0.5:
                        warnings.append(
                            "waypoint ({:.0f}, {:.0f}) mm sits inside the robot clearance zone "
                            "- moved to the closest allowed point ({:.0f}, {:.0f}) mm".format(
                                point[0], point[1], snapped[0], snapped[1]
                            )
                        )
                    point = snapped
            if not program and not waypoint_points and math.dist(anchor, point) <= ANCHOR_SNAP_MM:
                # the robot already stands on this waypoint (the usual first
                # element: the operator marks the pose it starts from). There
                # is nothing to drive and no arrival manoeuvre to weave: the
                # old code emitted a zero-length GOTO here, whose arrival
                # heading differed from the outgoing leg and forced the whole
                # plan onto the stop-and-turn backend.
                if theta_deg is not None:
                    anchor_heading = theta_deg
                waypoint_points.append(point)
                resolved_waypoints.append([round(point[0], 1), round(point[1], 1)])
                continue
            # ---- close waypoint: straight final leg ----------------------
            # A waypoint allowed inside the routing margin cannot be reached
            # by the normal grid route, and one closer than the half-diagonal
            # cannot turn in place. Route to a turn-safe stand-off, perform
            # the arrival rotation there (the GOTO's final heading), then
            # drive the last stretch STRAIGHT into the point. Rotating on the
            # close waypoint itself is what used to swing a body corner into
            # the obstacle / over the field border.
            if waypoint_mode:
                inside_routing = (
                    grid is not None and not grid.is_free_mm(point[0], point[1])
                )
                can_turn_here = waypoint_can_turn_at(point)
                if inside_routing or not can_turn_here:
                    natural_heading = waypoint_heading_hint(point, None)
                    approach = approach_waypoint_straight(
                        point, theta_deg, natural_heading,
                    )
                    if approach is None and not can_turn_here:
                        # No safe straight approach along the final heading:
                        # safety wins over the requested closeness. Move the
                        # stop point out to the closest spot where an in-place
                        # turn is safe, then perform the angle there.
                        safe = closest_body_clear_point(
                            point,
                            theta_deg if theta_deg is not None else natural_heading,
                            waypoint_obstacles, turn_clearance,
                            body_width, body_height, field_width_mm, field_height_mm,
                        )
                        if math.dist(safe, point) > 0.5:
                            warnings.append(
                                "waypoint ({:.0f}, {:.0f}) mm is too close to rotate in "
                                "place and no safe straight final leg exists - moved to "
                                "the closest turning-safe point ({:.0f}, {:.0f}) mm".format(
                                    point[0], point[1], safe[0], safe[1]
                                )
                            )
                            point = safe
                            inside_routing = (
                                grid is not None
                                and not grid.is_free_mm(point[0], point[1])
                            )
                            can_turn_here = waypoint_can_turn_at(point)
                            # the point is turn-safe now: a straight final leg
                            # along the natural approach (turning in place at
                            # the point is legal there) is enough
                            natural_heading = waypoint_heading_hint(point, None)
                            approach = approach_waypoint_straight(
                                point, theta_deg, natural_heading,
                            )
                    # The stop point is reached EXACTLY. When the mission
                    # continues and the body cannot rotate here, plan a
                    # straight departure move to a turn-safe point instead of
                    # moving the operator's waypoint outwards.
                    departure = None
                    needs_departure = False
                    if (
                        approach is not None
                        and not waypoint_can_turn_at(point)
                        and later_motion_needs_turn(element_index)
                    ):
                        needs_departure = True
                        departure = find_departure(point, approach["heading"])
                        if departure is None:
                            # no legal longitudinal escape (a pocket): last
                            # resort - move the stop to a turn-safe point
                            safe = closest_body_clear_point(
                                point,
                                theta_deg if theta_deg is not None else approach["heading"],
                                waypoint_obstacles, turn_clearance,
                                body_width, body_height,
                                field_width_mm, field_height_mm,
                            )
                            if math.dist(safe, point) > 0.5:
                                warnings.append(
                                    "waypoint ({:.0f}, {:.0f}) mm is too close to rotate "
                                    "in place and no safe departure move exists - moved "
                                    "to the closest turning-safe point ({:.0f}, {:.0f}) mm".format(
                                        point[0], point[1], safe[0], safe[1]
                                    )
                                )
                                point = safe
                                can_turn_here = waypoint_can_turn_at(point)
                                natural_heading = waypoint_heading_hint(point, None)
                                approach = approach_waypoint_straight(
                                    point, theta_deg, natural_heading,
                                )
                                needs_departure = False
                    if approach is not None:
                        standoff = approach["standoff"]
                        route = approach["route"]
                        final_heading = approach["heading"]
                        pre_rotated = bool(approach["pre_rotated"])
                        if math.dist(anchor, standoff) > 0.5:
                            # the connector is routed on the full grid: keep
                            # it apart from the preceding run so the close-
                            # waypoint exemption below only covers THIS leg
                            program.append({"kind": "cut"})
                            append_run(route, waypoint_speed)
                            if program and program[-1].get("kind") == "run":
                                program[-1]["close_connector"] = True
                        # the cut keeps the stand-off a real node: the GOTO
                        # there finishes facing the final leg, so the robot
                        # has performed the angle BEFORE the waypoint
                        program.append({"kind": "cut"})
                        append_run([standoff, point], waypoint_speed)
                        if program and program[-1].get("kind") == "run":
                            # this leg was validated point-by-point against the
                            # ROTATED body; it intentionally enters the routing
                            # margin, so pass 2 must not clamp it back out
                            program[-1]["close_waypoint"] = True
                        program.append({"kind": "cut"})
                        waypoint_points.append(point)
                        if theta_deg is not None:
                            arrival_targets.append(
                                {"point": point, "theta": theta_deg, "woven": pre_rotated}
                            )
                            anchor_heading = theta_deg
                        else:
                            anchor_heading = final_heading
                        resolved_waypoints.append(
                            [round(point[0], 1), round(point[1], 1)]
                        )
                        anchor = point
                        if needs_departure and departure is not None:
                            pending_departure = {
                                "point": point,
                                "heading": float(final_heading),
                                "departure": departure,
                                "speed_mm_s": waypoint_speed,
                            }
                        detail = ""
                        if theta_deg is not None:
                            detail = (
                                "; the arrival angle is performed at the stand-off "
                                "before the straight final leg"
                                if pre_rotated else
                                "; the point is turn-safe, so the arrival angle is "
                                "performed in place there"
                            )
                        warnings.append(
                            "waypoint ({:.0f}, {:.0f}) mm stands inside the routing "
                            "margin - approached straight from a safe stand-off at "
                            "({:.0f}, {:.0f}) mm{}".format(
                                point[0], point[1], standoff[0], standoff[1], detail
                            )
                        )
                        continue
            exact = options.waypoint_cut_mm <= 0.0 or theta_deg is not None
            # a waypoint closing a drawn run gets its arrival woven into THAT
            # run: the shortest legal arc may replace the tail of the approach
            run_entry = None
            previous_run = None
            if theta_deg is not None and combine:
                for candidate in reversed(program):
                    if candidate.get("kind") == "run":
                        run_entry = candidate
                        break
                    if candidate.get("kind") in ("rotate", "pause"):
                        break
                if (
                    run_entry is not None
                    and math.dist(run_entry["points"][-1], point) <= 2.0
                ):
                    previous_run = list(run_entry["points"])
            if exact:
                program.append({"kind": "cut"})
            crossed = True
            woven = False
            if theta_deg is not None and combine:
                choice = choose_arrival(
                    anchor, point, theta_deg, grid, legal_margin,
                    field_width_mm, field_height_mm, previous_run,
                )
                if choice is not None and choice.kind == "arc" and choice.arc is not None:
                    if choice.from_run and run_entry is not None:
                        # replace the drawn tail with the arrival arc: the
                        # shortest legal manoeuvre, still inside the field
                        run_entry["points"] = list(choice.route)
                        run_entry["arrival"] = choice.arc
                        woven = True
                    else:
                        connector = list(choice.route)
                        if grid is not None:
                            connector = _route(
                                grid, [anchor, choice.route[-1]],
                                warnings, relaxed_grid, body_grid,
                            )
                        if math.dist(connector[-1], choice.arc.start) <= 1.0:
                            append_run(connector, waypoint_speed)
                            if program[-1].get("kind") == "run":
                                program[-1]["arrival"] = choice.arc
                                woven = True
                if not woven:
                    warnings.append(
                        "the arrival angle at waypoint ({:.0f}, {:.0f}) mm could not be "
                        "woven into the approach - the robot turns in place there".format(
                            point[0], point[1]
                        )
                    )
            if not woven and math.dist(anchor, point) > 0.5:
                connector = [anchor, point]
                if grid is not None:
                    connector = _route(grid, connector, warnings, relaxed_grid, body_grid)
                    if math.dist(connector[-1], point) > 0.5:
                        # the waypoint sits inside the robot clearance zone:
                        # the router stopped at the closest cell the robot may
                        # stand on. Drive THAT point and continue - driving the
                        # extra stub to the waypoint and reversing out would
                        # hairpin (the path has to come back out anyway), and
                        # the body would sweep outside the clearance.
                        reached = connector[-1]
                        gap = math.dist(reached, point)
                        warnings.append(
                            "waypoint ({:.0f}, {:.0f}) mm sits inside the robot clearance zone "
                            "- closest allowed approach ({:.0f}, {:.0f}) mm, {:.0f} mm short".format(
                                point[0], point[1], reached[0], reached[1], gap
                            )
                        )
                        crossed = False
                        point = reached
                append_run(connector, waypoint_speed)
            if exact:
                program.append({"kind": "cut"})
            if crossed:
                waypoint_points.append(point)
            if theta_deg is not None:
                arrival_targets.append({"point": point, "theta": theta_deg, "woven": woven})
                anchor_heading = theta_deg
            else:
                anchor_heading = tail_heading() or anchor_heading
            resolved_waypoints.append([round(point[0], 1), round(point[1], 1)])
            anchor = point
        elif kind == "drive":
            # manual step: advance (or back up with a negative distance)
            # along the heading the robot holds at this point of the mission -
            # the same primitive the Drive pad uses, embedded in the sequence
            distance = float(element.get("distance_mm", 0.0))
            if abs(distance) < 0.5:
                continue
            heading = anchor_heading if anchor_heading is not None else 0.0
            radians = math.radians(heading)
            target = clamp_inside(
                (anchor[0] + math.cos(radians) * distance, anchor[1] + math.sin(radians) * distance)
            )
            if distance < 0.0:
                # a REVERSE keeps the heading: a GOTO chain would turn the
                # robot around and drive forward to the same point
                actual = -math.dist(anchor, target)
                if abs(actual) >= 0.5:
                    departure_step = {
                        "kind": "depart",
                        "distance_mm": actual,
                        "points": [
                            [round(anchor[0], 1), round(anchor[1], 1)],
                            [round(target[0], 1), round(target[1], 1)],
                        ],
                    }
                    if element.get("speed_mm_s"):
                        departure_step["speed_mm_s"] = float(element["speed_mm_s"])
                    program.append(departure_step)
                    anchor = target
            else:
                append_run([anchor, target], element.get("speed_mm_s"))
                anchor_heading = tail_heading() or heading
                anchor = target
        else:
            warnings.append(f"unknown element kind '{kind}' ignored")

    if pending_departure is not None and goal is not None:
        # the close stop is followed by the goal: leave it with the same
        # straight departure move before routing the goal leg
        insert_pending_departure({"kind": "goal", **goal})
        departure_join = None
        departure_origin = None
    goal_heading: Optional[float] = None
    if goal is not None:
        # the goal leg is routed like every other leg - a goal behind an
        # obstacle gets a Theta* detour; the goal position itself is always
        # reached exactly (a blocked goal is reported and approached directly)
        goal_point = clamp_inside((float(goal["x"]), float(goal["y"])))
        # a goal is a STOP point: keep the FULL obstacle margin there too
        if grid is not None and not grid.is_free_mm(goal_point[0], goal_point[1]):
            snapped = closest_clear_point(
                goal_point, obstacles, inflation, legal_margin,
                field_width_mm, field_height_mm,
            )
            if not grid.is_free_mm(snapped[0], snapped[1]):
                try:
                    snapped = grid.nearest_free(goal_point[0], goal_point[1])
                except NoPathError:
                    snapped = goal_point
            if math.dist(snapped, goal_point) > 0.5:
                warnings.append(
                    "goal ({:.0f}, {:.0f}) mm sits inside the robot clearance zone "
                    "- moved to the closest allowed point ({:.0f}, {:.0f}) mm".format(
                        goal_point[0], goal_point[1], snapped[0], snapped[1]
                    )
                )
            goal_point = snapped
        goal_leg: List[Point] = [anchor, goal_point]
        if grid is not None:
            goal_leg = _route(grid, goal_leg, warnings, relaxed_grid, body_grid)
            if math.dist(goal_leg[-1], goal_point) > 0.5:
                warnings.append(
                    "goal lies inside the robot clearance zone (field border or obstacle) - reached directly"
                )
                goal_leg.append(goal_point)
        append_run(goal_leg)
        if goal.get("theta_deg") is not None:
            goal_heading = float(goal["theta_deg"])

    # the FINAL waypoint may carry the arrival angle too (like the goal):
    # the robot stops exactly on it and faces that angle
    end_run_heading: Optional[float] = goal_heading
    if end_run_heading is None and elements:
        tail_element = elements[-1]
        if str(tail_element.get("kind")) == "waypoint" and tail_element.get("theta_deg") is not None:
            if arrival_targets:
                end_run_heading = float(arrival_targets[-1]["theta"])
                if arrival_targets[-1]["woven"]:
                    # the arrival ARC already exits at the angle: no final
                    # GOTO heading override (that would eat the arc's last
                    # sample and flatten the manoeuvre back into a node)
                    end_run_heading = None

    if not any(entry["kind"] == "run" for entry in program):
        if not any(entry["kind"] in ("rotate", "pause", "release") for entry in program):
            raise ValueError("no usable stroke - draw a path first")
        # an ACTIONS-ONLY mission (a rotation, a pause, a motor release) is
        # legitimate: pass 2 emits those and the plan carries no path

    # ------- pass 2: round corners, sample, emit GOTO/ARC nodes and actions -
    # the FINAL run: the last entry that carries geometry. NOT ``program[-1]``
    # when a waypoint with an arrival angle appended its ``cut`` markers after
    # the run - that made every run look non-final, the last sample was eaten
    # by the native arc and the waypoint arrival angle silently degraded to
    # "passed through - only approximate" (real-robot bug, 2026-10-01).
    final_run = None
    for item in program:
        if item.get("kind") == "run":
            final_run = item
    last_emitted: Optional[Point] = None
    n_gotos = 0
    n_arcs = 0
    prev_kind: Optional[str] = None  # type of the last emitted motion segment
    for entry in program:
        if entry["kind"] == "cut":
            continue
        if entry["kind"] == "depart":
            # straight longitudinal move that keeps the robot's heading:
            # a negative distance is the reverse used to escape a close
            # waypoint before rotating to continue
            distance = float(entry.get("distance_mm", 0.0))
            if abs(distance) < 0.5:
                continue
            declared_speed = entry.get("speed_mm_s")
            speed = int(min(
                PROTO_MAX_SPEED,
                max(
                    options.min_speed_mm_s,
                    float(declared_speed) if declared_speed else turn_speed,
                ),
            ))
            segment: Dict = {
                "kind": "drive",
                "distance_mm": round(distance, 1),
                "v": speed,
                "departure": True,
            }
            points = [
                (float(p[0]), float(p[1]))
                for p in entry.get("points", [])
                if len(p) >= 2
            ]
            if len(points) >= 2:
                segment["points"] = [[round(x, 1), round(y, 1)] for x, y in points]
                reference.extend(points)
                samples_all.extend(points)
            segments.append(segment)
            length_mm += abs(distance)
            est_s += abs(distance) / max(float(speed), 1.0)
            prev_kind = "drive"
            continue
        if entry["kind"] == "rotate":
            # per-step speed override: "turn THIS one slowly", every other
            # step keeps the system turn speed (clamped to the protocol range)
            step_speed = entry.get("speed_mm_s")
            v = turn_speed
            if step_speed:
                v = int(
                    min(
                        PROTO_MAX_SPEED,
                        max(options.min_speed_mm_s, float(step_speed)),
                    )
                )
            for chunk in _split_turn(entry["angle_deg"]):
                segments.append({"kind": "turn", "angle_deg": round(chunk, 1), "v": v,
                                 "explicit": True})
            prev_kind = "turn"
            continue
        if entry["kind"] == "pause":
            seconds = max(0.1, min(entry["seconds"], 600.0))
            segments.append({"kind": "pause", "seconds": round(seconds, 1)})
            est_s += seconds
            continue
        if entry["kind"] == "release":
            segments.append({"kind": "release"})
            continue

        segments_before = len(segments)
        rounded = _smooth_run(entry["points"], grid, options, warnings, None, waypoint_points)
        if entry.get("close_waypoint") or entry.get("close_connector"):
            # validated close-waypoint leg: only the physical field bounds
            # apply (clamping to the half-diagonal inset would move the stop
            # point away from the obstacle it was allowed to approach)
            rounded = [
                (
                    min(max(p[0], 0.0), field_width_mm),
                    min(max(p[1], 0.0), field_height_mm),
                )
                for p in rounded
            ]
        else:
            rounded = [clamp_inside(p) for p in rounded]
        samples = g.path_resample(
            rounded,
            options.goto_step_mm,
            options.node_chord_tol_mm,
            options.max_node_turn_deg,
            options.min_node_step_mm,
        )
        if len(samples) < 2:
            continue
        # per-step speed override: a run carrying ``speed_mm_s`` never cruises
        # faster than that (accel/decel and the lateral arc caps still apply)
        run_speed = float(options.speed_mm_s)
        step_speed = entry.get("speed_mm_s")
        if step_speed:
            run_speed = max(
                float(options.min_speed_mm_s),
                min(run_speed, float(step_speed)),
            )
        speeds, run_time = speed_profile(
            samples,
            run_speed,
            options.accel_mm_s2,
            options.decel_mm_s2,
            options.lat_accel_mm_s2,
            options.min_speed_mm_s,
        )
        headings = g.point_headings(samples)
        length_mm += g.polyline_length(samples)
        est_s += run_time
        is_final_run = entry is final_run
        # circular chains become single ARC segments; everything else keeps
        # being sampled as GOTO nodes
        last_index = len(samples) - 1
        if is_final_run and end_run_heading is not None:
            last_index -= 1  # keep the final node free for the arrival heading
        # an arc starting at the run origin has no entry node to pre-align:
        # allowed when the previous GOTO already faces its tangent, or when
        # the robot heading is known (first motion of the plan) and matches
        arc_map = (
            _native_arcs(
                samples,
                headings,
                speeds,
                options,
                last_index,
                zero_entry_ok=(prev_kind == "goto"),
                entry_heading_deg=(start_heading_deg if prev_kind is None else None),
            )
            if options.native_arcs
            else {}
        )
        if options.native_arcs and options.native_lines:
            # the STRAIGHT gaps between the fitted arcs become very flat ARCs
            # too: a GOTO chain stops at every node, so a straight run drove
            # as stop-and-go and every arc -> straight junction cost a stop
            # plus an in-place turn (2026-10-02)
            protected: List[Point] = [
                target["point"] for target in arrival_targets
            ]
            _near_straight_arcs(
                samples,
                headings,
                speeds,
                options,
                arc_map,
                last_index,
                # a run that starts after an ARC already has a known entry
                # heading (the chain is tangent or the emission inserts the
                # aligning TURN), so it may open on a flat arc too
                zero_entry_ok=(prev_kind in ("goto", "arc")),
                entry_heading_deg=(start_heading_deg if prev_kind is None else None),
                protected=protected,
            )
        i = 1
        if 0 in arc_map:
            end_index, arc = arc_map[0]
            if prev_kind is None and start_heading_deg is not None:
                # the plan opens on an ARC: the board starts it from the
                # CURRENT heading, so align first - a small entry error
                # rotates the whole arc and every segment after it (the real
                # robot inherits the same drift)
                delta = g.deg_norm180(arc["th0_deg"] - start_heading_deg)
                if abs(delta) > ARC_JOINT_TOL_DEG:
                    segments.append(
                        {"kind": "turn", "angle_deg": round(delta, 1), "v": turn_speed}
                    )
            segments.append(arc)
            n_arcs += 1
            last_emitted = samples[end_index]
            i = end_index
        while i < len(samples):
            is_last = i == len(samples) - 1
            if i in arc_map:
                end_index, arc = arc_map[i]
                node = samples[i]
                if not (last_emitted is not None and math.dist(last_emitted, node) < DEDUPE_NODE_MM):
                    segments.append(
                        {
                            "kind": "goto",
                            "x": round(node[0], 1),
                            "y": round(node[1], 1),
                            "th": round(g.deg_norm180(arc["th0_deg"]), 1),
                            "v": arc["v"],
                        }
                    )
                    last_emitted = node
                    n_gotos += 1
                else:
                    # the junction node was just emitted, so consecutive fitted
                    # arcs touch WITHOUT the pre-aligning GOTO between them.
                    # The board drives an ARC from the CURRENT heading, so a
                    # tangent break at the junction would bend the executed
                    # path away from the plan (and cut obstacle corners): turn
                    # in place so the arc entry heading matches the exit of
                    # the arc just driven.
                    previous = None
                    for candidate in reversed(segments):
                        if candidate["kind"] == "pause":
                            continue
                        previous = candidate
                        break
                    if previous is not None and previous["kind"] == "arc":
                        exit_deg = g.deg_norm180(previous["th0_deg"] + previous["sweep_deg"])
                        delta = g.deg_norm180(arc["th0_deg"] - exit_deg)
                        if abs(delta) > ARC_JOINT_TOL_DEG:
                            segments.append(
                                {"kind": "turn", "angle_deg": round(delta, 1), "v": turn_speed}
                            )
                segments.append(arc)
                n_arcs += 1
                last_emitted = samples[end_index]
                i = end_index
                continue
            node = samples[i]
            if last_emitted is not None and math.dist(last_emitted, node) < DEDUPE_NODE_MM:
                # the final arrival node carries the end-of-run heading (the
                # board turns in place there): keep it even when it coincides
                # with the previous sample - dropping it silently lost the
                # waypoint arrival angle ("passed through - only approximate")
                if not (is_last and is_final_run and end_run_heading is not None):
                    i += 1
                    continue
            theta = headings[i]
            if is_last and is_final_run and end_run_heading is not None:
                theta = end_run_heading
            segments.append(
                {
                    "kind": "goto",
                    "x": round(node[0], 1),
                    "y": round(node[1], 1),
                    "th": round(g.deg_norm180(theta), 1),
                    "v": int(min(PROTO_MAX_SPEED, max(options.min_speed_mm_s, round(speeds[i])))),
                }
            )
            last_emitted = node
            n_gotos += 1
            i += 1
        reference.extend(rounded)
        samples_all.extend(samples)
        arrival = entry.get("arrival")
        if arrival is not None:
            # the run ends at Q; the arrival ARC is emitted explicitly so the
            # sampler can never flatten it and the heading really is exact
            node_speed = int(min(PROTO_MAX_SPEED, max(options.min_speed_mm_s, round(run_speed))))
            if last_emitted is None or math.dist(last_emitted, arrival.start) > 3.0:
                segments.append(
                    {
                        "kind": "goto",
                        "x": round(arrival.start[0], 1),
                        "y": round(arrival.start[1], 1),
                        "th": round(g.deg_norm180(arrival.entry_deg), 1),
                        "v": node_speed,
                        "arrival_gate": True,
                    }
                )
                last_emitted = arrival.start
                n_gotos += 1
            previous = None
            for candidate in reversed(segments):
                if candidate["kind"] == "pause":
                    continue
                previous = candidate
                break
            if previous is not None and previous["kind"] == "goto":
                # the GOTO ends facing the arc tangent: exact, no extra turn.
                # Tag it so the wire sanitiser does not drag it onto the
                # previous (flat) arc's exit and tilt the whole arrival
                previous["th"] = round(g.deg_norm180(arrival.entry_deg), 1)
                previous["arrival_gate"] = True
            elif previous is not None and previous["kind"] == "arc":
                exit_deg = g.deg_norm180(previous["th0_deg"] + previous["sweep_deg"])
                delta = g.deg_norm180(arrival.entry_deg - exit_deg)
                if abs(delta) > ARC_JOINT_TOL_DEG:
                    segments.append(
                        {"kind": "turn", "angle_deg": round(delta, 1), "v": turn_speed}
                    )
            segments.append(arrival.as_segment(node_speed))
            n_arcs += 1
            last_emitted = arrival.points[-1]
            length_mm += arrival.length_mm
            est_s += arrival.length_mm / max(float(node_speed), 1.0)
            reference.extend(arrival.points)
            samples_all.extend(arrival.points)
        if entry.get("close_waypoint"):
            # the streaming follower would cut the stand-off corner; a close
            # waypoint approach must run on the segment backend (in-place
            # rotation at the stand-off, then the straight final leg)
            for segment in segments[segments_before:]:
                segment["close_waypoint"] = True
        if entry.get("close_connector"):
            for segment in segments[segments_before:]:
                segment["close_connector"] = True
        if segments:
            prev_kind = segments[-1]["kind"]

    # a near-straight run is ONE straight move, not an alternating S of tiny
    # flat arcs: the real robot read that as acceleration/deceleration and
    # unnecessary turns. Merge before the junction smoother so the remaining
    # corners (the real curves) still get their connector arcs.
    straight_merged = _merge_straight_runs(segments, start)

    # smooth the fitted-arc junctions: a small tangent break is rounded with a
    # connector arc tangent to both neighbours (corner rounding) instead of an
    # in-place TURN - the chain then drives as ONE continuous move (the
    # console executes consecutive ARCs with QUEUE_ARC, no stop in between).
    # Big breaks / explicit rotations keep the turn (fallback, see the doc).
    _smooth_arc_junctions(segments, options, arc_stays_legal)

    # the OPENING: a robot that stands at (or a few cm from) the start of the
    # drawing gets a tangent connector ARC instead of the turn-drive-turn
    # dance into the first curve ("the first turn is annoying, it should be
    # smoother") - one continuous move from standstill onto the drawn circle
    if start_heading_deg is not None and not options.resume_mid_mission:
        if _launch_weave(segments, samples_all, start, start_heading_deg, arc_stays_legal):
            warnings.append(
                "the mission starts with a smooth connector arc onto the first curve "
                "(no stop-and-turn launch)"
            )

    # a hand drawn switchback (a cusp tighter than any drivable arc) came out
    # of the sampler as a run of tiny GOTO nodes: the board stops at every one
    # (real case 2026-10-07: 14 nodes 2-4 mm apart turning 10-13 deg each -
    # "a lot of turning unnecessary and not smooth"). ONE pivot at the exit
    # corner replaces the stutter.
    cusps_collapsed = _collapse_cusps(segments, options, arc_stays_legal)
    if cusps_collapsed:
        warnings.append(
            "a drawn switchback too tight for the wheels became ONE pivot "
            "instead of a stop-and-go node stutter"
        )

    # the board drives ARCs RELATIVE (radius + sweep from the current pose):
    # mid-path TURNs become absolute GOTO re-anchor points (the same single
    # stop, but the position is taken from the plan again) and a long arc run
    # is cut with one more re-anchor - the wheel slip and the per-arc
    # completion tolerance are otherwise never fed back (2026-10-07: the run
    # ended 70 mm short of the plan's own end)
    reanchored = _reanchor_turns(segments)
    capped = _cap_chain_length(segments)

    # every node faces the next one (also across run boundaries and joints);
    # a node before pause(s) already faces the continuation, so the robot is
    # aligned when it resumes instead of turning twice
    for index, segment in enumerate(segments[:-1]):
        if segment["kind"] != "goto":
            continue
        nxt_th: Optional[float] = None
        if segments[index + 1]["kind"] == "goto":
            nxt = segments[index + 1]
            nxt_th = math.degrees(math.atan2(nxt["y"] - segment["y"], nxt["x"] - segment["x"]))
        else:
            for following in segments[index + 1 :]:
                if following["kind"] in ("turn", "drive", "strafe"):
                    # a departure drive keeps the robot's current heading:
                    # the node before it must NOT be re-aimed at the route
                    # after the move
                    nxt_th = None
                    break
                if following["kind"] == "arc":
                    nxt_th = float(following["th0_deg"])  # enter on the arc tangent
                    break
                if following["kind"] == "goto":
                    nxt_th = math.degrees(
                        math.atan2(following["y"] - segment["y"], following["x"] - segment["x"])
                    )
                    break
        if nxt_th is not None:
            segment["th"] = round(g.deg_norm180(nxt_th), 1)

    # waypoint arrival headings: the combined ones were planned into the run
    # geometry above (an explicit ARC exits at the commanded angle); the
    # others turn in place on the waypoint node (the board's GOTO final turn)
    arrival_edited = any(target["woven"] for target in arrival_targets)
    for target in arrival_targets:
        point = target["point"]
        theta = target["theta"]
        applied = _set_arrival_heading(segments, point, theta, turn_speed)
        if not applied:
            warnings.append(
                "waypoint (%.0f, %.0f) mm is not a stop node on the route - "
                "its arrival angle cannot be performed" % (point[0], point[1])
            )
        else:
            arrival_edited = True
            # the arrival angle changed the heading at the node: an ARC that
            # starts right there must still be entered on its tangent
            _align_arc_after_arrival(segments, point, theta, turn_speed)

    # every motion ARC must be something the board accepts and drives exactly
    # (wire rounding + the firmware's radius/sweep/length window): an arc
    # outside it becomes a GOTO to its own exit instead of a RANGE abort
    sanitized = _sanitize_arcs(segments, options)

    blended_reanchors = _blend_tangent_reanchors(segments)

    if (
        arrival_edited
        or cusps_collapsed
        or reanchored
        or capped
        or sanitized
        or blended_reanchors
        or straight_merged
    ):
        # the weave rebuilds the emitted tail AFTER the run samples were
        # collected: rebuild the preview path from the FINAL segments so the
        # map overlay shows the woven arc - with "combine with the approach"
        # the drawn plan looked like a no-op before (2026-10-01); the cusp
        # collapse edits segments the same way
        preview = _preview_path(segments)
        # the first emitted node may be one goto_step ahead of the robot (a
        # GOTO at the exact start pose is redundant on the board): keep the
        # real start at the front or the streaming follower would see a path
        # that begins 250 mm away from its own pose. Only when the start is a
        # legal, nearby pose - an off-table/very stale start must stay out of
        # the plan (the follower's own guards handle that case).
        if (
            preview
            and math.dist(preview[0], start) > 1.0
            and math.dist(preview[0], start) <= max(options.goto_step_mm * 1.5, 400.0)
            and legal_margin <= start[0] <= field_width_mm - legal_margin
            and legal_margin <= start[1] <= field_height_mm - legal_margin
        ):
            preview.insert(0, start)
        samples_all[:] = preview

    # ------- continuity audit: every emitted motion step must start where the
    # previous one ends. The board builds each ARC from the CURRENT pose, so
    # an arc whose ``x0`` is elsewhere is driven from the wrong circle and
    # lands far from the plan (real-robot incident 2026-10-01: the arrival
    # weave left a 300 mm hole and the mission "completed" 450 mm short of
    # the waypoint). This is a bug net, not a feature - warn, never silent.
    audit_prev: Optional[Tuple[float, float]] = None
    for audit_index, segment in enumerate(segments):
        kind = segment.get("kind")
        if kind == "arc":
            start_pt = (float(segment["x0"]), float(segment["y0"]))
            if audit_prev is not None:
                gap_mm = math.dist(audit_prev, start_pt)
                if gap_mm > 3.0:
                    warnings.append(
                        "internal continuity check: the arc at (%.0f, %.0f) mm starts "
                        "%.0f mm from the previous exit - the tail was rebuilt "
                        "inconsistently" % (start_pt[0], start_pt[1], gap_mm)
                    )
            audit_prev = (float(segment["x1"]), float(segment["y1"]))
        elif kind == "goto":
            audit_prev = (float(segment["x"]), float(segment["y"]))
        elif kind in ("drive", "strafe"):
            stored = segment.get("points")
            if stored:
                audit_prev = (float(stored[-1][0]), float(stored[-1][1]))

    if border_clamped:
        warnings.append(
            "the drawn geometry was clamped inside the table border: the robot body "
            "(half-diagonal %.0f mm) must stay on the field - the outer strip is unreachable"
            % pose_radius
        )

    if n_gotos == 0 and n_arcs == 0 and goal is None and not any(
        segment["kind"] in ("turn", "pause", "release") for segment in segments
    ):
        # (an ACTIONS-ONLY mission - just a rotation, a pause or a motor
        # release - is a legitimate program; only a plan with NOTHING to
        # execute is refused)
        raise ValueError("no usable stroke - draw a path first")

    n_gotos = sum(1 for segment in segments if segment["kind"] == "goto")
    n_arcs = sum(1 for segment in segments if segment["kind"] == "arc")
    n_actions = len(segments) - n_gotos - n_arcs
    if len(segments) > options.max_segments:
        warnings.append(
            f"{len(segments)} segments > cap {options.max_segments} - increase goto_step_mm or simplify"
        )

    stats = {
        "length_mm": round(length_mm, 1),
        "est_s": round(est_s, 1),
        "n_segments": len(segments),
        "n_gotos": n_gotos,
        "n_arcs": n_arcs,
        "n_actions": n_actions,
        "n_primitives": len(primitives),
    }
    return {
        "primitives": _json_primitives(primitives),
        "reference": _json_points(reference),
        "path": _json_points(samples_all),
        "segments": segments,
        "stats": stats,
        "warnings": warnings,
        # one [x, y] per waypoint element, in element order: a waypoint that
        # stood inside a clearance zone was moved, and the console/cursor must
        # follow the effective point (not the original, unreachable one)
        "resolved_waypoints": resolved_waypoints,
    }


def _sweep_about(points: Sequence[Point], center: Point) -> float:
    """Signed sweep of a polyline around ``center`` (degrees, unwrapped).

    Uses the central angle of every sample (not the chord headings), so the
    value is the true arc sweep even when the node sampling is coarse.
    """
    total = 0.0
    prev: Optional[float] = None
    for x, y in points:
        angle = math.degrees(math.atan2(y - center[1], x - center[0]))
        if prev is not None:
            total += g.deg_norm180(angle - prev)
        prev = angle
    return total


_ARC_END_ALIGN_DEG = 10.0  # max travel/tangent disagreement on an arc end node


def _circle_tangent_deg(point: Point, center: Point, radius: float, ccw: bool) -> float:
    """Direction of travel of the circle tangent at ``point`` (CCW or CW)."""
    ux = (point[0] - center[0]) / max(radius, 1e-6)
    uy = (point[1] - center[1]) / max(radius, 1e-6)
    tx, ty = (-uy, ux) if ccw else (uy, -ux)
    return math.degrees(math.atan2(ty, tx))


def _grow_arc(points: Sequence[Point], tol: float) -> Optional[Tuple[int, Point, float]]:
    """Longest prefix (>= 3 points) that still hugs one circle within ``tol``.

    Returns ``(last_index, center, radius)`` for the prefix
    ``points[0..last_index]`` or ``None`` when even the first three points
    disagree. Growth is what merges a chain of near-tangent fitted arcs into
    one ARC and what stops exactly at the junction where the next circle
    begins.
    """
    best: Optional[Tuple[int, Point, float]] = None
    for count in range(3, len(points) + 1):
        span = points[:count]
        center, radius = g.fit_circle(span)
        if not (
            math.isfinite(center[0]) and math.isfinite(center[1]) and math.isfinite(radius)
        ) or radius < 1.0:
            break
        if any(abs(math.dist(point, center) - radius) > tol for point in span):
            break
        best = (count - 1, center, radius)
    return best



def _native_arcs(
    samples: Sequence[Point],
    headings: Sequence[float],
    speeds: Sequence[float],
    options: PlanOptions,
    last_index: int,
    zero_entry_ok: bool,
    entry_heading_deg: Optional[float],
) -> Dict[int, Tuple[int, Dict]]:
    """Detect sampled chains that can be driven as one firmware ARC command.

    The samples are walked left to right; at every position the LONGEST span
    that still hugs one circle within ``arc_fit_tol_mm`` is grown and, when it
    passes the firmware gates (radius, sweep, arc length, entry heading), it
    becomes one ARC segment. Scanning resumes at the span END, so consecutive
    arcs share their junction and no GOTO node separates them - a chain of
    fitted arcs drives as consecutive ARC commands. The board drives an ARC
    from the CURRENT heading, so when two spans meet with a tangent break the
    caller inserts a small in-place TURN first (the fitter cannot make every
    junction tangent-continuous on a detoured path). Everything a span cannot
    cover keeps being emitted as GOTO nodes by the caller, so a failed
    conversion is harmless. A span starting at the very beginning of a run
    has no entry node to pre-align the heading: it is accepted only when the
    previous GOTO already faces the arc tangent (``zero_entry_ok``) or the
    robot heading is known (first motion of the plan) and matches
    (``entry_heading_deg``). Returns ``{start_index: (end_index, segment)}``.
    """
    result: Dict[int, Tuple[int, Dict]] = {}
    n = len(samples)
    if n < 4:
        return result
    limit = min(last_index, n - 1)
    index = 0
    while index + 2 <= limit:
        grown = _grow_arc(samples[index : limit + 1], options.arc_fit_tol_mm)
        if grown is not None:
            end_index, center, radius = grown
            lo, hi = index, index + end_index
            sweep = _sweep_about(samples[lo : hi + 1], center)
            arc_len = abs(math.radians(sweep)) * radius
            # entry tangent from the fitted circle (the sweep sign says which
            # way the circle is travelled), used for the pre-align node and
            # for the run-origin heading gate
            tangent_deg = _circle_tangent_deg(samples[lo], center, radius, ccw=sweep > 0.0)
            zero_allowed = zero_entry_ok or (
                entry_heading_deg is not None
                and abs(g.deg_norm180(tangent_deg - entry_heading_deg)) <= 5.0
            )
            if (
                (lo == 0 and not zero_allowed)
                or hi > last_index
                or abs(sweep) < options.arc_min_sweep_deg
                or abs(sweep) > options.arc_max_sweep_deg
                or radius < options.arc_min_radius_mm
                or radius > 20000.0
                or arc_len > 2900.0
            ):
                index += 1
                continue
            # one constant speed for the whole sweep: the firmware ramps up to
            # it and brakes at the arc end by itself; the sampled profile would
            # hand the arc its slowest value (usually the exit ramp to creep)
            span_speed = max(speeds[lo : hi + 1])
            lateral_cap = math.sqrt(max(1.0, options.lat_accel_mm_s2) * max(radius, 1.0))
            speed = min(span_speed, lateral_cap, float(PROTO_MAX_SPEED))
            speed = max(float(options.min_speed_mm_s), speed)
            result[lo] = (
                hi,
                {
                    "kind": "arc",
                    "x0": round(samples[lo][0], 1),
                    "y0": round(samples[lo][1], 1),
                    "x1": round(samples[hi][0], 1),
                    "y1": round(samples[hi][1], 1),
                    "radius_mm": round(math.copysign(radius, sweep), 1),
                    "sweep_deg": round(sweep, 1),
                    "th0_deg": round(g.deg_norm180(tangent_deg), 1),
                    "v": int(speed),
                    "points": [[round(x, 1), round(y, 1)] for x, y in samples[lo : hi + 1]],
                },
            )
            index = hi
            continue
        index += 1
    return result


def _chord_deviation(samples: Sequence[Point], lo: int, hi: int) -> float:
    """Worst distance of the samples between ``lo`` and ``hi`` to their chord."""
    ax, ay = samples[lo]
    bx, by = samples[hi]
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return 0.0
    worst = 0.0
    for index in range(lo + 1, hi):
        px, py = samples[index]
        worst = max(worst, abs((px - ax) * dy - (py - ay) * dx) / length)
    return worst


def _nearest_sample(along: Sequence[float], lo: int, hi: int, target: float) -> int:
    """Sample index closest to ``target`` along the path, clamped to [lo, hi]."""
    if hi < lo:
        return lo
    index = lo
    while index < hi and abs(along[index + 1] - target) <= abs(along[index] - target):
        index += 1
    return index


def _near_straight_arcs(
    samples: Sequence[Point],
    headings: Sequence[float],
    speeds: Sequence[float],
    options: PlanOptions,
    arc_map: Dict[int, Tuple[int, Dict]],
    limit: int,
    zero_entry_ok: bool,
    entry_heading_deg: Optional[float],
    protected: Sequence[Point] = (),
) -> None:
    """Fill the gaps between the fitted ARCs with near-straight ARCs.

    A straight run used to be emitted as a GOTO node every
    ``goto_step_mm`` (250 mm) and the board brakes to a FULL stop at every
    GOTO node (TURN -> DRIVE -> TURN, ramp reset per segment): a long
    straight drove as stop-and-go and every arc -> straight junction cost a
    stop plus an in-place turn. The same geometry driven as an ARC costs
    nothing - the firmware tracks the arc length with the wheels, holds the
    heading with the (gyro aided) curvature loop, and the console blends
    consecutive ARCs through QUEUE_ARC without stopping - so the straight
    samples become very flat arcs:

    * the sweep is an EXACT multiple of the 0.1 deg wire quantum and the
      radius is derived from it (``R = half / (units * 0.1 deg)``): the
      firmware re-derives its arc length as ``sweep * R``, so the span is
      driven at exactly its measured length. Deriving the sweep from a
      rounded radius instead would move the robot tens of mm off the plan
      at 32767 mm (each 0.1 deg quantum is 57 mm of arc length there);
    * a span up to ``line_pair_mm`` drives as ONE arc; longer spans are cut
      into half-arcs that ALTERNATE side (an extremely flat S) so the span
      leaves and re-joins the run direction and the junctions stay tangent
      (the bulge stays below ~2 mm, well inside the fit tolerance);
    * samples straying more than ``line_tol_mm`` from their chord (or bowed
      enough to tilt the ends by more than ~0.5 deg) are NOT straight: the
      circular fit refused them for a reason and they keep their GOTO nodes
      with the stop/turn fallback.

    Mutates ``arc_map``; the caller's emission loop walks it exactly like
    the fitted arcs.
    """
    n = len(samples)
    if n < 3 or limit < 1:
        return
    arc_starts = set(arc_map.keys())
    interior = set()
    for lo, (hi, _arc) in arc_map.items():
        # strictly inside: a span may START on an arc's exit sample and END on
        # the next arc's entry sample (that is how the chain stays tangent
        # with no GOTO node - and no stop - between them)
        interior.update(range(lo + 1, min(hi, limit)))
    forbidden: set = set()
    along = [0.0]
    for index in range(n - 1):
        along.append(along[-1] + math.dist(samples[index], samples[index + 1]))
    for arrival in protected:
        arrival_index = 0
        best = None
        for point_index in range(0, limit + 1):
            distance = math.dist(samples[point_index], arrival)
            if best is None or distance < best:
                best = distance
                arrival_index = point_index
        forbidden.add(arrival_index)
        cursor = arrival_index
        while cursor > 0 and along[arrival_index] - along[cursor - 1] <= LINE_ARC_ARRIVAL_CLEAR_MM:
            cursor -= 1
            forbidden.add(cursor)

    quantum_rad = math.radians(LINE_ARC_SWEEP_QUANT_DEG)
    index = 0
    while index < limit:
        if index in interior or index in arc_starts or index in forbidden:
            index += 1
            continue
        end = index + 1
        while end < limit and (end + 1) not in interior:
            span = along[end + 1] - along[index]
            tolerance = min(options.line_tol_mm, span * LINE_ARC_DIR_SLOPE)
            if _chord_deviation(samples, index, end + 1) > tolerance:
                break
            if (end + 1) in forbidden:
                break  # the arrival stub keeps its GOTO nodes (the weave)
            end += 1
            if end in arc_starts:
                break  # the span stops ON the next arc's entry node (a chain)
        span = along[end] - along[index]
        if span < LINE_ARC_MIN_SPAN_MM:
            index += 1
            continue
        phi = math.degrees(
            math.atan2(
                samples[end][1] - samples[index][1],
                samples[end][0] - samples[index][0],
            )
        )
        if index == 0 and not zero_entry_ok:
            # a run that OPENS the plan has no entry node to pre-align the
            # heading (the launch weave needs that GOTO lead too): only the
            # known heading of the very first motion may open on an ARC
            if entry_heading_deg is None or abs(g.deg_norm180(phi - entry_heading_deg)) > 5.0:
                index = end
                continue
        halves = (
            1
            if span <= LINE_ARC_ONE_PIECE_MM
            else 2 * max(1, int(math.ceil(span / max(1.0, options.line_pair_mm))))
        )
        # a half can never be shorter than one sample step (a two point
        # stroke cannot be split): the sample indices are the real split
        # points and each arc's sweep is derived from ITS OWN measured
        # length, so the firmware re-derives exactly the piece it drives
        # (a uniform "step" would have driven 350 mm over a 250 mm sample
        # span and disagreed with the plan's own x0/x1 chain)
        halves = min(halves, max(1, end - index))
        step = span / halves
        side = 1.0
        if index > 0 and g.deg_norm180(headings[index - 1] - phi) > 0.0:
            side = -1.0
        lo = index
        tangent: Optional[float] = None
        for half in range(halves):
            if half == halves - 1:
                hi = end
            else:
                hi = _nearest_sample(along, lo + 1, end - 1, along[index] + step * (half + 1))
            if hi <= lo:
                break
            # the piece is the CHORD (the flat arc drives a straight line):
            # taking the polyline length instead made the arc overshoot its
            # own entry/exit samples whenever the span ended on a curve (a
            # real 6 mm jump the preview and the board both followed)
            piece = math.dist(samples[lo], samples[hi])
            units = max(1, int(math.ceil(piece / (LINE_ARC_RADIUS_MAX_MM * quantum_rad))))
            radius = piece / (units * quantum_rad)
            sweep = units * LINE_ARC_SWEEP_QUANT_DEG
            sign = side if half % 2 == 0 else -side
            if tangent is None:
                tangent = phi - sign * sweep / 2.0
            th0 = tangent
            # the next piece starts exactly on this tangent: the S only stays
            # a chain because the headings are carried, not recomputed
            tangent = th0 + sign * sweep
            cruise = min(
                max(speeds[lo : hi + 1]),
                math.sqrt(max(1.0, options.lat_accel_mm_s2) * radius),
                float(PROTO_MAX_SPEED),
            )
            cruise = max(
                float(options.min_speed_mm_s),
                min(cruise, max(1.0, piece) / CHAIN_MIN_DURATION_S),
            )
            arc_map[lo] = (
                hi,
                {
                    "kind": "arc",
                    "x0": round(samples[lo][0], 1),
                    "y0": round(samples[lo][1], 1),
                    "x1": round(samples[hi][0], 1),
                    "y1": round(samples[hi][1], 1),
                    "radius_mm": round(sign * radius, 1),
                    "sweep_deg": round(sign * sweep, 1),
                    "th0_deg": round(g.deg_norm180(th0), 1),
                    "v": int(cruise),
                    "points": [[round(x, 1), round(y, 1)] for x, y in samples[lo : hi + 1]],
                },
            )
            lo = hi
        index = end


# --------------------------------------------------------------- helpers ----
def _arrival_anchor(segments: Sequence[Dict], point: Point, tol: float = 2.0) -> Optional[Tuple[int, str]]:
    """The segment that LANDS on ``point``: ``(index, kind)`` or ``None``.

    Either a GOTO node (the usual case) or an ARC whose EXIT is the point -
    the fit joins a straight approach into one near-straight ARC, so a
    waypoint is very often the exit of an arc. Without the arc case there was
    no node to turn at and the commanded arrival angle was dropped silently
    (2026-10-09).
    """
    for index in range(len(segments) - 1, -1, -1):
        segment = segments[index]
        if segment.get("kind") == "goto" and math.dist((segment["x"], segment["y"]), point) <= tol:
            return (index, "goto")
        if segment.get("kind") == "arc" and math.dist((segment["x1"], segment["y1"]), point) <= tol:
            return (index, "arc")
    return None


def _find_arrival_index(segments: Sequence[Dict], point: Point, tol: float = 2.0) -> int:
    """Index of the GOTO node landing on ``point`` (-1 when there is none)."""
    for index in range(len(segments) - 1, -1, -1):
        segment = segments[index]
        if segment.get("kind") == "goto" and math.dist((segment["x"], segment["y"]), point) <= tol:
            return index
    return -1


def _flat_arc(segment: Dict) -> Optional[Tuple[Point, float]]:
    """``(exit point, exit travel heading)`` of a NEAR-STRAIGHT arc.

    Only near-straight ARCs may be re-aimed or dropped by the arrival weave: on
    a real curve that would visibly cut the path. ``None`` for anything else.
    """
    if segment.get("kind") != "arc":
        return None
    if abs(float(segment.get("radius_mm", 0.0))) < ARC_WIRE_FLAT_MM:
        return None
    return (
        (float(segment["x1"]), float(segment["y1"])),
        g.deg_norm180(float(segment["th0_deg"]) + float(segment["sweep_deg"])),
    )


def _weave_base_on_arc(
    base_start: Point,
    base_th0: float,
    point: Point,
    theta_deg: float,
    leg: float,
    stays_legal,
) -> Optional[Tuple[Point, float, float, float]]:
    """``(Q, entry heading, sweep, radius)`` for a weave ON a near-straight ARC.

    The approach arc is RE-AIMED so that it lands on the weave base Q: an arc
    from ``base_start`` with entry ``base_th0`` and sweep ``s`` has its chord
    along ``base_th0 + s / 2`` and leaves Q facing ``base_th0 + s``; the
    arrival arc from Q onto the waypoint must then run with its chord along
    ``(base_th0 + s + theta) / 2`` (a circle's chord lies along the mean of its
    entry and exit tangents). Both of those are RAYS, so Q is simply their
    intersection - no iteration, no approximation: the robot really arrives
    facing the commanded angle.
    The sweep search runs on the 0.1 deg wire the board uses, so the sanitiser
    pass re-derives exactly these numbers and nothing shifts afterwards.
    ``None`` when no sweep leaves a near-straight approach (the caller then
    falls back to the in-place turn at the waypoint).
    """
    total = g.deg_norm180(theta_deg - base_th0)
    approach = math.degrees(math.atan2(point[1] - base_start[1], point[0] - base_start[0]))
    reach = math.dist(base_start, point)
    candidates: List[Tuple[float, float, Point, float, float, float]] = []
    for tenths in range(-int(WEAVE_MAX_DEG * 10.0), int(WEAVE_MAX_DEG * 10.0) + 1):
        sweep = tenths / 10.0
        turn_arrival = g.deg_norm180(total - sweep)
        if abs(sweep) < 0.5 or abs(turn_arrival) < 0.5:
            continue
        chord_dir = math.radians(base_th0 + sweep / 2.0)
        arrival_dir = math.radians(base_th0 + sweep + turn_arrival / 2.0 + 180.0)
        cross = math.sin(arrival_dir - chord_dir)
        if abs(cross) < 1e-6:
            continue
        travelled = (
            (point[0] - base_start[0]) * math.sin(arrival_dir)
            - (point[1] - base_start[1]) * math.cos(arrival_dir)
        ) / cross
        if travelled < 40.0 or travelled > reach + 5.0:
            continue
        start = (
            base_start[0] + math.cos(chord_dir) * travelled,
            base_start[1] + math.sin(chord_dir) * travelled,
        )
        gap = math.dist(start, point)
        if gap < 25.0 or gap > 3.0 * leg:
            continue
        # both arcs must run FORWARD along the approach: the ray pair has a
        # second solution that bends the leg around and passes the waypoint -
        # the robot would drive past it and swing back (a switchback)
        chord_bearing = math.degrees(chord_dir)
        if abs(g.deg_norm180(chord_bearing - approach)) > 45.0:
            continue
        bearing_arrival = math.degrees(math.atan2(point[1] - start[1], point[0] - start[0]))
        if abs(g.deg_norm180(bearing_arrival - approach)) > 60.0:
            continue
        radius = travelled / (2.0 * math.sin(math.radians(abs(sweep)) / 2.0))
        if not (ARC_WIRE_MIN_RADIUS_MM <= radius <= ARC_WIRE_MAX_RADIUS_MM):
            continue
        if radius * math.radians(abs(sweep)) > ARC_WIRE_MAX_LENGTH_MM:
            continue
        if radius * (1.0 - math.cos(math.radians(abs(sweep)) * 0.5)) > ARRIVAL_MAX_BOW_MM:
            continue
        arrival_radius = gap / (2.0 * math.sin(math.radians(abs(turn_arrival)) / 2.0))
        if not (50.0 <= arrival_radius <= 20000.0):
            continue
        candidates.append((abs(gap - leg), abs(sweep), start, base_th0 + sweep, sweep, radius))
    candidates.sort(key=lambda item: (item[0], item[1]))
    for _, _, start, entry, sweep, radius in candidates[:8]:
        centre = _arc_center(base_start, base_th0, math.copysign(radius, sweep))
        if not stays_legal(
            _arc_samples(
                centre,
                radius,
                math.atan2(base_start[1] - centre[1], base_start[0] - centre[0]),
                math.radians(sweep),
            )
        ):
            continue
        return (start, g.deg_norm180(entry), sweep, radius)
    return None


def _previous_motion_point(segments: Sequence[Dict], index: int) -> Optional[Point]:
    """End point of the motion segment right before ``index``."""
    for candidate in reversed(segments[:index]):
        if candidate.get("kind") == "goto":
            return (candidate["x"], candidate["y"])
        if candidate.get("kind") == "arc":
            return (candidate["x1"], candidate["y1"])
    return None


def _set_arrival_heading(
    segments: List[Dict], point: Point, theta_deg: float, turn_speed: int = 150
) -> bool:
    """Make the robot face ``theta_deg`` ON ``point`` (turn in place).

    A GOTO node landing on the point turns in place as part of its final
    phase, so setting its ``th`` is enough. When the point is the EXIT of an
    ARC (the fit joins a straight approach into one near-straight ARC, so a
    waypoint very often sits at an arc exit) there is no node to turn at: an
    explicit pivot from the arc's exit heading is inserted right there - the
    board can only turn to the final heading of a GOTO after driving to it,
    and a zero-length GOTO would first turn to its degenerate travel heading
    (a double turn the user sees as a dance). The pivot goes BEFORE any pause
    following the waypoint, so the wait happens facing the commanded angle
    (2026-10-09). ``False`` when nothing lands on the point at all.
    """
    anchor = _arrival_anchor(segments, point)
    if anchor is None:
        return False
    index, kind = anchor
    if kind == "goto":
        segments[index]["th"] = round(g.deg_norm180(theta_deg), 1)
        return True
    arc = segments[index]
    heading = g.deg_norm180(float(arc["th0_deg"]) + float(arc["sweep_deg"]))
    delta = g.deg_norm180(theta_deg - heading)
    if abs(delta) <= ARC_JOINT_TOL_DEG:
        return True  # already facing the commanded angle
    segments.insert(
        index + 1,
        {
            "kind": "turn",
            "angle_deg": round(delta, 1),
            "v": int(turn_speed),
            "explicit": True,
            "arrival": True,
        },
    )
    return True


def _align_arc_after_arrival(
    segments: List[Dict], point: Point, theta_deg: float, turn_speed: int, tol: float = 2.0
) -> None:
    """Align the continuation after an arrival angle edited the heading.

    The board drives an ARC from the CURRENT heading (firmware parity: the
    arc's circle is built from ``self._th``), so an arc entered right after
    the node that now faces ``theta_deg`` would start on a different circle
    and leave the planned path entirely - the robot used to "get lost and go
    completely to the right" after a waypoint arrival angle. Insert the
    in-place turn from the arrival angle to the arc's entry tangent:
    "perform the angle, then turn toward the next point and continue".
    The turn goes AFTER any pause following the waypoint: the mission order
    is arrive -> face the commanded angle -> wait -> turn away and continue,
    and the old insert at ``index + 1`` turned away before waiting
    (2026-10-09).
    """
    anchor = _arrival_anchor(segments, point, tol)
    if anchor is None:
        return
    index = anchor[0]
    probe = index + 1
    while probe < len(segments) and (
        segments[probe].get("kind") == "pause" or segments[probe].get("arrival")
    ):
        probe += 1
    if probe >= len(segments) or segments[probe].get("kind") != "arc":
        return
    delta = g.deg_norm180(float(segments[probe]["th0_deg"]) - theta_deg)
    if abs(delta) > ARC_JOINT_TOL_DEG:
        segments.insert(probe, {"kind": "turn", "angle_deg": round(delta, 1), "v": turn_speed,
                                "explicit": True})


def _merge_arrival_turn(
    segments: List[Dict],
    point: Point,
    theta_deg: float,
    options: PlanOptions,
    stays_legal,
) -> bool:
    """LEGACY post-hoc weave - no longer called by :func:`plan`.

    The combined arrival is now PLANNED (``navigation/arrival.py``) before the
    run is sampled, which lets it choose a legal intermediate point, route to
    it and keep the arc exact. This helper is kept for reference only: it
    rewrote the last ~100 mm of the emitted tail, could not route to an
    intermediate point and validated arcs against body clearance only.

    Weave the arrival turn into the approach as one small ARC.

    Replaces the final bit into ``point`` with a short ARC whose exit tangent
    is exactly ``theta_deg`` (the arc ends ON the point), so the robot arrives
    already facing the wished angle - no stop-and-turn. Two approach shapes
    are supported:

    * a GOTO base - the node moves to Q and everything between it and the
      waypoint is dropped (that tail must run on GOTO nodes only);
    * a NEAR-STRAIGHT ARC base - the arc is RE-AIMED to end at Q
      (:func:`_weave_base_on_arc` solves the two chord rays on the board's
      0.1 deg wire) and the weave lands on the waypoint. Straights are emitted
      as flat ARCs, so without this "combine with the approach" could never
      apply at all (2026-10-09).

    A real (curved) ARC approach or a pause/rotation in the tail still returns
    False: the rebuild would swallow it and - the board drives every ARC from
    the CURRENT pose - the chain would jump and "complete" hundreds of mm
    short of the waypoint (real-robot bug, 2026-10-01). The caller then falls
    back to the exact heading override (in-place turn).
    The arc is sampled and must stay on the table (and off obstacles) with the
    FULL body clearance; the weave is retried with shorter approach stubs and
    finally falls back to the plain heading override (turn in place) when the
    angle is tiny or no legal arc exists - the caller then warns.
    """
    anchor = _arrival_anchor(segments, point)
    if anchor is None:
        return False
    index, anchor_kind = anchor
    if index <= 0:
        return False
    # the node right before the waypoint is often just the resampling remnant:
    # walk back to the last motion node far enough from the point for the arc
    # to fit. Anything the rebuild would swallow must be a GOTO node or a
    # near-straight ARC: a curve, a pause or a rotation in the tail is refused
    base_index = -1
    base_point = None
    base_arc = None
    base_start = None
    base_th0 = 0.0
    for candidate in range(index - 1, -1, -1):
        segment = segments[candidate]
        if segment.get("kind") == "goto":
            if math.dist((segment["x"], segment["y"]), point) < 110.0:
                continue
            base_index = candidate
            base_point = (float(segment["x"]), float(segment["y"]))
            break
        flat = _flat_arc(segment)
        if flat is None:
            return False
        if math.dist(flat[0], point) < 110.0:
            continue  # too close to weave on: try the arc behind it
        base_index = candidate
        base_point = flat[0]
        base_arc = segment
        base_start = (float(segment["x0"]), float(segment["y0"]))
        base_th0 = float(segment["th0_deg"])
        # the board holds the entry heading the PREVIOUS segment really left:
        # the sanitiser pass re-labels this arc from it (a 0.5 deg stale label
        # there tilted the whole weave and the arrival ended degrees off), and
        # it really stands where the previous segment really ended
        if candidate > 0:
            before = segments[candidate - 1]
            if before.get("kind") == "arc":
                landed = g.deg_norm180(float(before["th0_deg"]) + float(before["sweep_deg"]))
                if abs(g.deg_norm180(landed - base_th0)) <= ARC_CHAIN_TANGENT_DEG:
                    base_th0 = landed
                base_start = (float(before["x1"]), float(before["y1"]))
            elif before.get("kind") == "goto":
                base_start = (float(before["x"]), float(before["y"]))
        break
    if base_index < 0 or base_point is None:
        return False
    # everything the rebuild would swallow has to be a GOTO node or a
    # near-straight ARC: cutting a real curve or dropping a pause/rotation the
    # user asked for is not a weave, it is a different mission
    for dropped in segments[base_index + 1 : index + 1]:
        if dropped.get("kind") != "goto" and _flat_arc(dropped) is None:
            return False
    distance = math.dist(base_point, point)
    if base_arc is None:
        inbound = math.degrees(math.atan2(point[1] - base_point[1], point[0] - base_point[0]))
    else:
        inbound = g.deg_norm180(float(base_arc["th0_deg"]) + float(base_arc["sweep_deg"]))
    delta = g.deg_norm180(theta_deg - inbound)
    if abs(delta) <= 5.0 and anchor_kind == "goto":
        # a few degrees is not worth an arc: the in-place turn is negligible
        segments[index]["th"] = round(g.deg_norm180(theta_deg), 1)
        return True
    if abs(delta) > WEAVE_MAX_DEG:
        return False
    speed = int(segments[index].get("v", options.speed_mm_s))
    # geometry: the arc runs from Q to P; a circle whose entry tangent is
    # ``inbound`` and exit tangent is ``theta`` has its chord along the MEAN
    # direction (inbound + delta/2), so Q = P - chord * unit(mean) - placing Q
    # on the inbound line instead would make the circle miss the waypoint
    for leg in (140.0, 100.0, 70.0, 45.0):
        leg = min(leg, distance - 10.0)
        if leg < 25.0:
            continue
        inbound_here = inbound
        start = None
        base_signed_sweep = 0.0
        base_radius = 0.0
        if base_arc is None:
            mean = math.radians(inbound + delta / 2.0)
            start = (point[0] - math.cos(mean) * leg, point[1] - math.sin(mean) * leg)
        else:
            solution = _weave_base_on_arc(
                base_start, base_th0, point, theta_deg, leg, stays_legal
            )
            if solution is None:
                continue
            start, inbound_here, base_signed_sweep, base_radius = solution
        delta = g.deg_norm180(theta_deg - inbound_here)
        chord = math.dist(start, point)
        if chord < 25.0:
            continue
        radius = chord / (2.0 * math.sin(math.radians(abs(delta)) * 0.5))
        # the arrival weave is a transition maneuver, not a travel arc: only
        # the firmware's physical radius limit applies (arc_min_radius_mm is a
        # quality gate for travel arcs and would block every tight arrival)
        if radius < 50.0 or radius > 20000.0:
            continue
        if base_arc is not None:
            # exactness check: an arc's exit tangent is 2 * chord bearing -
            # entry tangent, so the weave base must sit on the MEAN ray; a
            # gross deviation would arrive at the wrong angle - try the next
            # stub rather than promise something the geometry cannot hold
            bearing = math.degrees(math.atan2(point[1] - start[1], point[0] - start[0]))
            if abs(g.deg_norm180(2.0 * bearing - inbound_here - theta_deg)) > 1.5:
                continue
        # samples of the candidate arc: centre = start + signed_r * left normal
        signed_radius = math.copysign(radius, delta)
        centre = (
            start[0] + signed_radius * -math.sin(math.radians(inbound_here)),
            start[1] + signed_radius * math.cos(math.radians(inbound_here)),
        )
        phi0 = math.atan2(start[1] - centre[1], start[0] - centre[0])
        steps = max(2, int(abs(delta) / 10.0) + 1)
        samples = [
            (
                centre[0] + radius * math.cos(phi0 + math.radians(delta) * (step / steps)),
                centre[1] + radius * math.sin(phi0 + math.radians(delta) * (step / steps)),
            )
            for step in range(steps + 1)
        ]
        samples[-1] = (point[0], point[1])  # the arc ends exactly on the point
        if not stays_legal(samples):
            continue  # the swing would leave the table: try a shorter stub
        # rebuild the tail: drop the small nodes between the base and the
        # waypoint, then land the arrival ARC on the point facing theta
        del segments[base_index + 1 : index + 1]
        if base_arc is not None:
            # the base arc now ENDS on Q, re-aimed so that its exit heading is
            # what the arrival arc is entered with (the same numbers the
            # sanitiser pass derives, so the two agree and nothing shifts)
            base_arc["x1"] = round(start[0], 1)
            base_arc["y1"] = round(start[1], 1)
            base_arc["sweep_deg"] = round(base_signed_sweep, 1)
            base_arc["radius_mm"] = round(math.copysign(base_radius, base_signed_sweep), 1)
            base_arc["points"] = [
                [round(float(base_arc["x0"]), 1), round(float(base_arc["y0"]), 1)],
                [round(start[0], 1), round(start[1], 1)],
            ]
        elif segments[base_index]["kind"] == "goto":
            segments[base_index] = {
                "kind": "goto",
                "x": round(start[0], 1),
                "y": round(start[1], 1),
                "th": round(g.deg_norm180(inbound_here), 1),
                "v": speed,
            }
        segments.insert(
            base_index + 1,
            {
                "kind": "arc",
                "x0": round(start[0], 1),
                "y0": round(start[1], 1),
                "x1": round(point[0], 1),
                "y1": round(point[1], 1),
                "radius_mm": round(signed_radius, 1),
                "sweep_deg": round(delta, 1),
                "th0_deg": round(g.deg_norm180(inbound_here), 1),
                "v": speed,
                "points": [
                    [round(start[0], 1), round(start[1], 1)],
                    [round(point[0], 1), round(point[1], 1)],
                ],
            },
        )
        return True
    return False


def _arc_samples(center: Point, radius: float, start_angle: float, sweep: float) -> List[Point]:
    """Sample an arc (world mm) every <= 10 deg, including both ends."""
    steps = max(1, int(abs(sweep) / math.radians(10.0)) + 1)
    return [
        (
            center[0] + radius * math.cos(start_angle + sweep * (step / steps)),
            center[1] + radius * math.sin(start_angle + sweep * (step / steps)),
        )
        for step in range(steps + 1)
    ]


def _arc_center(start: Point, th0_deg: float, signed_radius: float) -> Point:
    """Circle centre of an arc: start + signed R * left-normal of the entry."""
    heading = math.radians(th0_deg)
    return (start[0] - signed_radius * math.sin(heading), start[1] + signed_radius * math.cos(heading))


def _point_to_polyline(point: Point, polyline: Sequence[Point]) -> float:
    """Distance from ``point`` to the polyline, measured to its SEGMENTS.

    Never to the sample vertices: a fitted ARC keeps the node spacing of the
    sampler (up to ~250 mm on a large radius), so the distance to the nearest
    vertex over-states a local corner cut by up to ~125 mm and used to reject
    perfectly good junction connectors (measured 36.8 mm where the connector
    really cut 1.6 mm - 2026-10-09).
    """
    best = float("inf")
    px, py = point
    for (ax, ay), (bx, by) in zip(polyline, polyline[1:]):
        dx = bx - ax
        dy = by - ay
        length2 = dx * dx + dy * dy
        if length2 <= 1e-9:
            nearest = math.hypot(px - ax, py - ay)
        else:
            along = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
            nearest = math.hypot(px - (ax + along * dx), py - (ay + along * dy))
        if nearest < best:
            best = nearest
    return best


def _arc_point_at(arc: Dict, distance: float, from_start: bool) -> Tuple[Point, float]:
    """Point and travel heading ``distance`` mm along ``arc`` (forward from its
    start or backward from its exit) - exact for any radius, including the
    near-straight ARCs whose sweep is measured in 0.1 deg wire quanta."""
    radius = abs(float(arc["radius_mm"]))
    sense = 1.0 if float(arc["sweep_deg"]) > 0.0 else -1.0
    centre = _arc_center(
        (float(arc["x0"]), float(arc["y0"])), float(arc["th0_deg"]), float(arc["radius_mm"])
    )
    if from_start:
        anchor = (float(arc["x0"]), float(arc["y0"]))
    else:
        anchor = (float(arc["x1"]), float(arc["y1"]))
    angle = math.atan2(anchor[1] - centre[1], anchor[0] - centre[0])
    angle += (1.0 if from_start else -1.0) * sense * distance / max(1.0, radius)
    point = (centre[0] + radius * math.cos(angle), centre[1] + radius * math.sin(angle))
    heading = math.degrees(angle) + (90.0 if sense > 0.0 else -90.0)
    return point, g.deg_norm180(heading)


def _junction_keep_floor(radius: float) -> float:
    """Sweep a neighbour of a rounded junction must keep: degrees, or mm for a flat ARC.

    A near-straight ARC (a straight run driven as a very flat arc) has a
    fraction of a degree of sweep: the degree floor would reject every
    connector next to it, and the degree trim caps would let one eat hundreds
    of mm of the run. Those neighbours are measured in MILLIMETRES instead
    (``ARC_CONNECT_FAR_TRIM_MM``).
    """
    if radius > ARC_CONNECT_MAX_RADIUS_MM:
        return ARC_CONNECT_FAR_TRIM_MM / max(1.0, radius * math.pi / 180.0)
    return ARC_CONNECT_MIN_SWEEP_DEG


def _junction_candidate(
    radius: float,
    centre: Point,
    a_pt: Point,
    b_pt: Point,
    trim1: float,
    trim2: float,
    prev_sweep: float,
    next_sweep: float,
    r1: float,
    r2: float,
    delta: float,
    sc: float,
    prev_pts: Sequence[Point],
    next_pts: Sequence[Point],
    stays_legal,
) -> Optional[Dict]:
    """Gate and score one connector candidate of a rounded junction.

    Shared by the tangent search and by the anchored (near-straight neighbour)
    construction so both are held to the same limits: the trims must leave the
    neighbours drivable, the connector's sweep must fill the break within the
    firmware's window, and the connector may not cut the drawn corner by more
    than ``ARC_CONNECT_MAX_DEV_MM`` nor leave the legal area.
    """
    keep1 = abs(prev_sweep) - trim1
    keep2 = abs(next_sweep) - trim2
    if trim1 < -0.05 or keep1 < _junction_keep_floor(r1):
        return None
    if trim2 < -0.05 or keep2 < _junction_keep_floor(r2):
        return None
    if trim1 > min(ARC_CONNECT_TRIM_MAX_DEG, 0.6 * abs(prev_sweep)):
        return None
    if trim2 > min(ARC_CONNECT_TRIM_MAX_DEG, 0.6 * abs(next_sweep)):
        return None
    if r1 > ARC_CONNECT_MAX_RADIUS_MM and math.radians(trim1) * r1 > ARC_CONNECT_FAR_TRIM_MM:
        return None
    if r2 > ARC_CONNECT_MAX_RADIUS_MM and math.radians(trim2) * r2 > ARC_CONNECT_FAR_TRIM_MM:
        return None
    phi_a = math.degrees(math.atan2(a_pt[1] - centre[1], a_pt[0] - centre[0]))
    phi_b = math.degrees(math.atan2(b_pt[1] - centre[1], b_pt[0] - centre[0]))
    conn_sweep = g.deg_norm180(phi_b - phi_a)
    # the connector must be at least ~2 mm long: the firmware rejects an arc
    # shorter than 1 mm with RANGE (a real abort on 2026-10-07), and the client
    # rounds the sweep to 0.1 deg
    min_sweep = max(0.5, math.degrees(ARC_WIRE_MIN_LENGTH_MM / max(1.0, radius)))
    if (conn_sweep > 0.0) != (sc > 0.0) or abs(conn_sweep) < min_sweep:
        return None
    if abs(conn_sweep) > 170.0 or abs(conn_sweep) > abs(delta) + 35.0:
        return None
    # corner cut: how far the connector strays from the two arcs
    samples = _arc_samples(centre, radius, math.radians(phi_a), math.radians(conn_sweep))
    deviation = max(
        min(_point_to_polyline(sample, prev_pts), _point_to_polyline(sample, next_pts))
        for sample in samples
    )
    if deviation > ARC_CONNECT_MAX_DEV_MM or not stays_legal(samples):
        return None
    target = min(r1, r2)
    score = deviation / 10.0 + 1.2 * max(0.0, (target - radius) / target) + abs(conn_sweep) / 200.0
    return {
        "score": score,
        "radius": radius,
        "oc": centre,
        "a": a_pt,
        "b": b_pt,
        "trim1": trim1,
        "trim2": trim2,
        "sweep": conn_sweep,
        "samples": samples,
    }


def _anchored_junction_connectors(
    prev: Dict,
    nxt: Dict,
    delta: float,
    o1: Point,
    r1: float,
    s1: float,
    o2: Point,
    r2: float,
    s2: float,
    sc: float,
    prev_pts: Sequence[Point],
    next_pts: Sequence[Point],
    stays_legal,
) -> List[Dict]:
    """Connectors for a junction next to a NEAR-STRAIGHT ARC.

    A near-straight ARC drives its chord with the radius and the sweep both on
    the wire resolution: 0.1 deg of sweep is ~50 mm at R = 28646 mm, so a free
    trim moves its exit away from the declared one (real 6 mm jump on
    2026-10-07). Such a neighbour is therefore never trimmed freely - the
    connector touches it at its own end point or a whole quantum away from it
    and the OTHER neighbour carries the trim.

    The connector is the unique circle tangent at that touch point with the
    neighbour's travel heading and tangent to the other circle: the closed form
    below is the same equation family the launch weave solves
    (``|centre - o| = r +- rho`` with the centre on the touch point's normal).
    Returns the candidates that pass the shared gates, best first by score.
    """
    candidates: List[Dict] = []
    for keeper_is_next, keeper_flat in ((True, r2 > ARC_WIRE_FLAT_MM), (False, r1 > ARC_WIRE_FLAT_MM)):
        if not keeper_flat:
            continue
        keeper = nxt if keeper_is_next else prev
        o_other, r_other, s_other = (o1, r1, s1) if keeper_is_next else (o2, r2, s2)
        junction = (float(prev["x1"]), float(prev["y1"]))
        # the quantum the board can represent on this neighbour (mm of travel)
        quantum_mm = math.radians(LINE_ARC_SWEEP_QUANT_DEG) * abs(float(keeper["radius_mm"]))
        for quanta in range(0, 3):
            distance = quanta * quantum_mm
            if distance > ARC_CONNECT_FAR_TRIM_MM:
                break
            touch, heading = _arc_point_at(keeper, distance, from_start=keeper_is_next)
            normal = (-math.sin(math.radians(heading)), math.cos(math.radians(heading)))
            wx = touch[0] - o_other[0]
            wy = touch[1] - o_other[1]
            n_dot_w = normal[0] * wx + normal[1] * wy
            for internal in (False, True):
                # centre = touch + sc*rho*normal; |centre - o| = r_other +- rho
                denominator = 2.0 * (n_dot_w + (r_other * sc if internal else -r_other * sc))
                if abs(denominator) < 1e-9:
                    continue
                lam = (r_other * r_other - (wx * wx + wy * wy)) / denominator
                radius = abs(lam)
                if radius < ARC_CONNECT_MIN_RADIUS_MM or radius > ARC_CONNECT_MAX_RADIUS_MM:
                    continue
                if math.copysign(1.0, lam) != sc:
                    continue
                centre = (touch[0] + lam * normal[0], touch[1] + lam * normal[1])
                span = math.dist(centre, o_other)
                if span < 1e-9:
                    continue
                ux = (o_other[0] - centre[0]) / span
                uy = (o_other[1] - centre[1]) / span
                # the touch point sits on the line of centres: between the two
                # centres for an external touch; for an internal one it is on
                # the side that puts it back ON the other circle
                side = -1.0 if (internal and radius < r_other) else 1.0
                other_touch = (
                    centre[0] + side * radius * ux,
                    centre[1] + side * radius * uy,
                )
                other_angle = math.degrees(
                    math.atan2(other_touch[1] - o_other[1], other_touch[0] - o_other[0])
                )
                junction_other = math.degrees(
                    math.atan2(junction[1] - o_other[1], junction[0] - o_other[0])
                )
                trim_keep = math.degrees(distance / max(1.0, abs(float(keeper["radius_mm"]))))
                if keeper_is_next:
                    # prev keeps the trim: its exit moves back to the touch point
                    a_pt, b_pt = other_touch, touch
                    trim1 = s_other * g.deg_norm180(junction_other - other_angle)
                    trim2 = trim_keep
                else:
                    a_pt, b_pt = touch, other_touch
                    trim1 = trim_keep
                    trim2 = s_other * g.deg_norm180(other_angle - junction_other)
                candidate = _junction_candidate(
                    radius, centre, a_pt, b_pt, trim1, trim2,
                    float(prev["sweep_deg"]), float(nxt["sweep_deg"]), r1, r2, delta, sc,
                    prev_pts, next_pts, stays_legal,
                )
                if candidate is not None:
                    candidates.append(candidate)
    candidates.sort(key=lambda candidate: candidate["score"])
    return candidates


def _circle_intersections(c1: Point, r1: float, c2: Point, r2: float) -> List[Point]:
    """Intersection points of two circles (0/1/2); no tangency specials."""
    dx = c2[0] - c1[0]
    dy = c2[1] - c1[1]
    distance = math.hypot(dx, dy)
    if distance < 1e-9 or distance > r1 + r2 + 1e-6 or distance < abs(r1 - r2) - 1e-6:
        return []
    a = (r1 * r1 - r2 * r2 + distance * distance) / (2.0 * distance)
    h = math.sqrt(max(0.0, r1 * r1 - a * a))
    xm = c1[0] + a * dx / distance
    ym = c1[1] + a * dy / distance
    if h < 1e-6:
        return [(xm, ym)]
    rx = -dy * (h / distance)
    ry = dx * (h / distance)
    return [(xm + rx, ym + ry), (xm - rx, ym - ry)]


def _touch_points(center: Point, radius: float, ccw: bool, oc: Point, rc: float, conn_ccw: bool) -> List[Point]:
    """Tangent touch points of a connector circle on a neighbour circle.

    Both possible radial placements are tested numerically (``Oi +/- u*ri``)
    and kept only when the point lies on the connector circle AND the travel
    direction is continuous (the neighbour tangent equals the connector
    tangent there). Robust to fitted arcs whose end does not sit exactly on
    their circle. Returns 0..2 points.
    """
    ux = oc[0] - center[0]
    uy = oc[1] - center[1]
    distance = math.hypot(ux, uy)
    if distance < 1e-9:
        return []
    ux /= distance
    uy /= distance
    result: List[Point] = []
    for sign in (1.0, -1.0):
        point = (center[0] + sign * ux * radius, center[1] + sign * uy * radius)
        if abs(math.dist(point, oc) - rc) > 0.25:
            continue
        tangent = _circle_tangent_deg(point, center, radius, ccw=ccw)
        conn_tangent = _circle_tangent_deg(point, oc, rc, ccw=conn_ccw)
        if abs(g.deg_norm180(tangent - conn_tangent)) <= 3.0:
            result.append(point)
    return result


def _arc_span(segment: Dict, start: Point, end: Point) -> Dict:
    """Copy an ARC restricted to ``start`` .. ``end`` on its OWN circle.

    Only the declared ends, the entry tangent and the sweep change: the circle
    (centre and radius) is kept, so the geometry driven between the two points
    is the one already fitted and audited - a junction may slide along it
    without re-fitting anything.
    """
    radius = float(segment["radius_mm"])
    centre = _arc_center(
        (float(segment["x0"]), float(segment["y0"])), float(segment["th0_deg"]), radius
    )
    sense = 1.0 if float(segment["sweep_deg"]) > 0.0 else -1.0
    phi0 = math.atan2(start[1] - centre[1], start[0] - centre[0])
    phi1 = math.atan2(end[1] - centre[1], end[0] - centre[0])
    raw = math.degrees(phi1 - phi0)
    sweep = (raw % 360.0) if sense > 0.0 else -((-raw) % 360.0)
    moved = dict(segment)
    moved.update(
        {
            "x0": round(start[0], 1),
            "y0": round(start[1], 1),
            "x1": round(end[0], 1),
            "y1": round(end[1], 1),
            "th0_deg": round(g.deg_norm180(math.degrees(phi0) + 90.0 * sense), 1),
            "sweep_deg": round(sweep, 1),
            "points": [
                [round(x, 1), round(y, 1)]
                for x, y in _arc_samples(centre, abs(radius), phi0, math.radians(sweep))
            ],
        }
    )
    return moved


def _circle_meeting_points(o1: Point, r1: float, o2: Point, r2: float) -> List[Point]:
    """Points the two neighbour circles have in common (0..3 of them).

    Two arcs fitted to ONE drawn curve are frequently (nearly) tangent where
    the curvature steps from a tight to a wide section, or cross by a couple
    of degrees: the fit puts the step exactly at the junction, which is why no
    connector circle tangent to both can exist there (2026-10-09).
    """
    points = list(_circle_intersections(o1, r1, o2, r2))
    distance = math.dist(o1, o2)
    if distance > 1e-9 and abs(distance - abs(r2 - r1)) <= 2.0:
        # (nearly) internally tangent: the touch point of the two circles
        ux = (o1[0] - o2[0]) / distance
        uy = (o1[1] - o2[1]) / distance
        if r1 <= r2:
            points.append((o1[0] + r1 * ux, o1[1] + r1 * uy))
        else:
            points.append((o2[0] - r2 * ux, o2[1] - r2 * uy))
    return points


def _relocated_junction(
    prev: Dict,
    nxt: Dict,
    o1: Point,
    r1: float,
    s1: float,
    o2: Point,
    r2: float,
    s2: float,
    options: PlanOptions,
    stays_legal,
) -> Optional[List[Dict]]:
    """Slide the junction onto a meeting point of the two circles (or None).

    A tight-to-wide transition of a drawn curve comes out of the fit as two
    arcs whose circles are (nearly) tangent or slightly crossing, and the
    joint between them cannot be bridged: any connector tangent to both would
    have to touch the second circle BEHIND the junction. Where the two circles
    actually meet, however, they are within millimetres of each other, so the
    last ``move`` millimetres of the incoming arc can simply be driven as part
    of the outgoing one: the junction slides along both circles, the
    downstream geometry does not move at all, and the joint becomes tangent
    (touching pair) or small enough for a connector (crossing pair).

    Refused when the slide would be longer than ``JUNCTION_RELOCATE_MAX_MM``,
    would eat too much of a neighbour's sweep, would stray further than
    ``ARC_CONNECT_MAX_DEV_MM`` from the geometry it replaces, or leaves the
    legal area. Returns the replacement segments - ``[prev', nxt']`` when the
    joint is tangent, ``[prev', connector, nxt']`` when a connector is still
    needed - or None to keep the in-place TURN.
    """
    junction = (float(prev["x1"]), float(prev["y1"]))
    prev_pts = [(float(px), float(py)) for px, py in prev["points"]]
    next_pts = [(float(px), float(py)) for px, py in nxt["points"]]

    def break_at(point: Point) -> float:
        """Heading mismatch of the two circles at ``point`` (their meeting point)."""
        heading1 = _circle_tangent_deg(point, o1, r1, ccw=s1 > 0.0)
        heading2 = _circle_tangent_deg(point, o2, r2, ccw=s2 > 0.0)
        return abs(g.deg_norm180(heading2 - heading1))

    candidates = sorted(
        _circle_meeting_points(o1, r1, o2, r2),
        key=lambda candidate_point: (break_at(candidate_point), math.dist(candidate_point, junction)),
    )
    for point in candidates:
        move = math.dist(point, junction)
        if move < 1.0 or move > JUNCTION_RELOCATE_MAX_MM:
            continue
        moved_prev = _arc_span(prev, (float(prev["x0"]), float(prev["y0"])), point)
        moved_next = _arc_span(nxt, point, (float(nxt["x1"]), float(nxt["y1"])))
        if abs(float(moved_prev["sweep_deg"])) < _junction_keep_floor(r1):
            continue
        if abs(float(moved_next["sweep_deg"])) <= abs(float(nxt["sweep_deg"])):
            continue  # the meeting point must lie BEHIND the junction
        deviation = max(
            min(
                _point_to_polyline((float(px), float(py)), prev_pts),
                _point_to_polyline((float(px), float(py)), next_pts),
            )
            for px, py in moved_next["points"]
        )
        if deviation > ARC_CONNECT_MAX_DEV_MM or not stays_legal(moved_next["points"]):
            continue
        break_after = g.deg_norm180(
            float(moved_next["th0_deg"]) - (float(moved_prev["th0_deg"]) + float(moved_prev["sweep_deg"]))
        )
        if abs(break_after) <= ARC_JOINT_TOL_DEG:
            return [moved_prev, moved_next]
        rounded = _round_arc_junction(moved_prev, moved_next, options, stays_legal, allow_relocate=False)
        if rounded is not None:
            return rounded
    return None


def _round_arc_junction(
    prev: Dict,
    nxt: Dict,
    options: PlanOptions,
    stays_legal,
    allow_relocate: bool = True,
) -> Optional[List[Dict]]:
    """Round one tangent break between two fitted ARCs (corner rounding).

    Returns the segments that replace the ``(prev, turn, nxt)`` triple - either
    ``[prev', connector, nxt']`` or ``[prev', nxt']`` when sliding the junction
    onto a meeting point of the two circles already made the joint tangent - or
    None when the caller must keep the in-place TURN.

    Three constructions are tried, most faithful first: sliding the junction
    onto a point the two circles share, a connector that keeps a NEAR-STRAIGHT
    neighbour whole (touching it at a whole 0.1 deg quantum), and the tangent
    search over both neighbours (which may trim them freely - the sanitizer
    derives a near-straight ARC's radius from its chord, so any trim stays
    representable).
    """
    point = (float(prev["x1"]), float(prev["y1"]))
    prev_sweep = float(prev["sweep_deg"])
    next_sweep = float(nxt["sweep_deg"])
    delta = g.deg_norm180(float(nxt["th0_deg"]) - (float(prev["th0_deg"]) + prev_sweep))
    if abs(delta) < ARC_JOINT_TOL_DEG or abs(delta) > ARC_CONNECT_MAX_DEG:
        return None

    r1 = abs(float(prev["radius_mm"]))
    r2 = abs(float(nxt["radius_mm"]))
    flat_prev = r1 > ARC_WIRE_FLAT_MM
    flat_next = r2 > ARC_WIRE_FLAT_MM
    if flat_prev and flat_next:
        # two near-straight ARCs meeting at a break: the connector would have to
        # touch both at their own end point, and their circles are (almost)
        # tangent there - nothing to round with, the in-place TURN stays
        return None
    s1 = 1.0 if prev_sweep > 0.0 else -1.0
    s2 = 1.0 if next_sweep > 0.0 else -1.0
    sc = 1.0 if delta > 0.0 else -1.0

    o1 = _arc_center((float(prev["x0"]), float(prev["y0"])), float(prev["th0_deg"]), float(prev["radius_mm"]))
    o2 = _arc_center((float(nxt["x0"]), float(nxt["y0"])), float(nxt["th0_deg"]), float(nxt["radius_mm"]))
    if allow_relocate:
        relocated = _relocated_junction(
            prev, nxt, o1, r1, s1, o2, r2, s2, options, stays_legal
        )
        if relocated is not None:
            return relocated
    ang_p1 = math.degrees(math.atan2(point[1] - o1[1], point[0] - o1[0]))
    ang_p2 = math.degrees(math.atan2(point[1] - o2[1], point[0] - o2[0]))
    prev_pts = [(float(px), float(py)) for px, py in prev["points"]]
    next_pts = [(float(px), float(py)) for px, py in nxt["points"]]

    steps = 40
    ratio = ARC_CONNECT_MAX_RADIUS_MM / ARC_CONNECT_MIN_RADIUS_MM
    best: Optional[Dict] = None
    # a connector that keeps a near-straight neighbour WHOLE is preferred: the
    # flat ARC then still drives exactly the straight it was built for
    for candidate in _anchored_junction_connectors(
        prev, nxt, delta, o1, r1, s1, o2, r2, s2, sc, prev_pts, next_pts, stays_legal
    ):
        if best is None or candidate["score"] < best["score"]:
            best = candidate
    if best is None:
        # no anchored connector exists: fall back to the tangent search, which
        # may trim a near-straight neighbour too (its radius comes from its
        # chord, so any trim stays representable on the wire)
        for step in range(steps + 1):
            radius = ARC_CONNECT_MIN_RADIUS_MM * (ratio ** (step / steps))
            if abs(r1 - radius) < 1.0 or abs(r2 - radius) < 1.0:
                continue  # degenerate (concentric) solution
            d1 = abs(r1 - radius) if s1 == sc else r1 + radius
            d2 = abs(r2 - radius) if s2 == sc else r2 + radius
            for oc in _circle_intersections(o1, d1, o2, d2):
                for a_pt in _touch_points(o1, r1, s1 > 0.0, oc, radius, sc > 0.0):
                    for b_pt in _touch_points(o2, r2, s2 > 0.0, oc, radius, sc > 0.0):
                        # A lies upstream of the junction on its arc, B downstream
                        ang_a = math.degrees(math.atan2(a_pt[1] - o1[1], a_pt[0] - o1[0]))
                        trim1 = s1 * g.deg_norm180(ang_p1 - ang_a)
                        ang_b = math.degrees(math.atan2(b_pt[1] - o2[1], b_pt[0] - o2[0]))
                        trim2 = s2 * g.deg_norm180(ang_b - ang_p2)
                        candidate = _junction_candidate(
                            radius, oc, a_pt, b_pt, trim1, trim2,
                            prev_sweep, next_sweep, r1, r2, delta, sc,
                            prev_pts, next_pts, stays_legal,
                        )
                        if candidate is not None and (
                            best is None or candidate["score"] < best["score"]
                        ):
                            best = candidate
    if best is None:
        return None

    radius = best["radius"]
    oc = best["oc"]
    a_pt = best["a"]
    b_pt = best["b"]
    prev_keep = prev_sweep - s1 * best["trim1"]
    next_keep = next_sweep - s2 * best["trim2"]

    prev_new = dict(prev)
    phi0_1 = math.atan2(float(prev["y0"]) - o1[1], float(prev["x0"]) - o1[0])
    prev_new.update(
        {
            "x1": round(a_pt[0], 1),
            "y1": round(a_pt[1], 1),
            "sweep_deg": round(prev_keep, 1),
            "points": [
                [round(x, 1), round(y, 1)]
                for x, y in _arc_samples(o1, r1, phi0_1, math.radians(prev_keep))
            ],
        }
    )

    next_new = dict(nxt)
    phi_b = math.atan2(b_pt[1] - o2[1], b_pt[0] - o2[0])
    next_new.update(
        {
            "x0": round(b_pt[0], 1),
            "y0": round(b_pt[1], 1),
            "th0_deg": round(g.deg_norm180(_circle_tangent_deg(b_pt, o2, r2, ccw=s2 > 0.0)), 1),
            "sweep_deg": round(next_keep, 1),
            "points": [
                [round(x, 1), round(y, 1)]
                for x, y in _arc_samples(o2, r2, phi_b, math.radians(next_keep))
            ],
        }
    )

    cruise = min(
        float(prev.get("v", options.speed_mm_s)),
        float(nxt.get("v", options.speed_mm_s)),
        math.sqrt(max(1.0, options.lat_accel_mm_s2) * radius),
        float(PROTO_MAX_SPEED),
    )
    connector_length = abs(best["sweep"]) * math.pi / 180.0 * radius
    cruise = min(cruise, max(1.0, connector_length) / CHAIN_MIN_DURATION_S)
    cruise = max(float(options.min_speed_mm_s), cruise)
    connector = {
        "kind": "arc",
        "x0": round(a_pt[0], 1),
        "y0": round(a_pt[1], 1),
        "x1": round(b_pt[0], 1),
        "y1": round(b_pt[1], 1),
        "radius_mm": round(math.copysign(radius, best["sweep"]), 1),
        "sweep_deg": round(best["sweep"], 1),
        "th0_deg": round(g.deg_norm180(_circle_tangent_deg(a_pt, oc, radius, ccw=best["sweep"] > 0.0)), 1),
        "v": int(cruise),
        "points": [[round(x, 1), round(y, 1)] for x, y in best["samples"]],
    }
    return [prev_new, connector, next_new]


def _smooth_arc_junctions(segments: List[Dict], options: PlanOptions, stays_legal) -> None:
    """Round small tangent breaks between consecutive fitted ARCs, in place.

    The fitter cannot make every junction tangent-continuous, so the emission
    inserts an in-place TURN at each break. This pass replaces such a turn by
    a connector arc tangent to BOTH neighbours (corner rounding): each
    neighbour is trimmed a little, the connector covers the heading change,
    and the chain becomes one tangent-continuous ARC run that the console
    executes without stopping (QUEUE_ARC chaining). Explicit rotations (user
    steps, waypoint arrival angles) are never touched, big breaks keep the
    turn, and a connector that would leave the table or the body clearance
    (or cut the corner by more than ``ARC_CONNECT_MAX_DEV_MM``) is rejected -
    the in-place turn stays as the fallback.
    """
    index = 0
    while index + 2 < len(segments):
        prev, turn, nxt = segments[index], segments[index + 1], segments[index + 2]
        if (
            prev.get("kind") != "arc"
            or turn.get("kind") != "turn"
            or nxt.get("kind") != "arc"
            or turn.get("explicit")
        ):
            index += 1
            continue
        rounded = _round_arc_junction(prev, nxt, options, stays_legal)
        if rounded is None:
            index += 1
            continue
        segments[index : index + 3] = rounded
        index += 1


def _reanchor_turns(segments: List[Dict]) -> bool:
    """Mid-path in-place TURNs become absolute GOTO nodes.

    The board drives every ARC relative (radius + sweep from the CURRENT
    pose): the per-arc completion tolerance and the wheel slip add up along a
    chain and the run ends tens of mm off the drawing (real mission
    2026-10-07: the robot finished 70 mm short of the plan's own end node).
    A GOTO is ABSOLUTE: the board turns, drives the residual distance to the
    coordinate IT measured and turns again - the same single stop, but the
    position is anchored to the plan again. The opening alignment keeps its
    TURN (the robot is standing on the plan's start anyway) and explicit
    rotations are never touched.
    """
    changed = False
    for index in range(1, len(segments) - 1):
        segment = segments[index]
        if segment.get("kind") != "turn" or segment.get("explicit"):
            continue
        before = segments[index - 1]
        after = segments[index + 1]
        if before.get("kind") == "arc":
            point = (float(before["x1"]), float(before["y1"]))
        elif before.get("kind") == "goto":
            point = (float(before["x"]), float(before["y"]))
        else:
            continue
        if after.get("kind") == "arc":
            heading = float(after["th0_deg"])
        elif after.get("kind") == "goto":
            heading = math.degrees(math.atan2(after["y"] - point[1], after["x"] - point[0]))
        else:
            continue
        segments[index] = {
            "kind": "goto",
            "x": round(point[0], 1),
            "y": round(point[1], 1),
            "th": round(g.deg_norm180(heading), 1),
            "v": int(segment.get("v", 150)),
            "junction_reanchor": True,
        }
        changed = True
    return changed


def _blend_tangent_reanchors(segments: List[Dict]) -> bool:
    """Drop a junction GOTO when sanitization proves both adjacent ARCs tangent.

    Mid-path turns are first converted to absolute GOTOs to correct heading
    and position drift. If the final wire arcs already meet at that point and
    their headings agree, the GOTO adds a full stop without correcting either
    value. Long-chain re-anchors are not tagged and remain as drift guards.
    """
    changed = False
    for index in range(len(segments) - 2, 0, -1):
        anchor = segments[index]
        if anchor.get("kind") != "goto" or not anchor.get("junction_reanchor"):
            continue
        prev, nxt = segments[index - 1], segments[index + 1]
        if prev.get("kind") != "arc" or nxt.get("kind") != "arc":
            continue
        point = (float(anchor["x"]), float(anchor["y"]))
        if (
            math.dist(point, (float(prev["x1"]), float(prev["y1"]))) > 0.5
            or math.dist(point, (float(nxt["x0"]), float(nxt["y0"]))) > 0.5
        ):
            continue
        exit_heading = float(prev["th0_deg"]) + float(prev["sweep_deg"])
        entry_heading = float(nxt["th0_deg"])
        if (
            abs(g.deg_norm180(float(anchor["th"]) - exit_heading)) > ARC_JOINT_TOL_DEG
            or abs(g.deg_norm180(entry_heading - exit_heading)) > ARC_JOINT_TOL_DEG
        ):
            continue
        del segments[index]
        changed = True

    for segment in segments:
        segment.pop("junction_reanchor", None)
    return changed


def _cap_chain_length(segments: List[Dict]) -> bool:
    """Re-anchor a long relative chain with ONE absolute GOTO node.

    ARCs blend without stopping, but each one ends on its own tolerance and
    the wheel slip is never fed back, so a long all-arc run drifts. Past
    ``CHAIN_REANCHOR_MM`` of continuous chaining the plan inserts an absolute
    GOTO to the junction it is about to pass (one stop) and continues from
    there - the position is taken from the plan again.
    """
    changed = False
    chained = 0.0
    index = 0
    while index < len(segments):
        segment = segments[index]
        kind = segment.get("kind")
        if kind in ("goto", "turn", "pause"):
            chained = 0.0
            index += 1
            continue
        if kind != "arc":
            index += 1
            continue
        chained += (
            abs(float(segment["sweep_deg"]))
            * math.pi
            / 180.0
            * abs(float(segment["radius_mm"]))
        )
        after = segments[index + 1] if index + 1 < len(segments) else None
        if chained <= CHAIN_REANCHOR_MM or after is None or after.get("kind") not in ("arc", "goto"):
            index += 1
            continue
        if after.get("kind") == "arc":
            heading = float(after["th0_deg"])
        else:
            heading = math.degrees(
                math.atan2(
                    after["y"] - float(segment["y1"]),
                    after["x"] - float(segment["x1"]),
                )
            )
        segments.insert(
            index + 1,
            {
                "kind": "goto",
                "x": round(float(segment["x1"]), 1),
                "y": round(float(segment["y1"]), 1),
                "th": round(g.deg_norm180(heading), 1),
                "v": int(segment.get("v", 150)),
            },
        )
        chained = 0.0
        changed = True
        index += 2
    return changed


def _sanitize_arcs(segments: List[Dict], options: PlanOptions) -> bool:
    """Keep every ARC inside what the board will accept and actually drive.

    The board drives radius + sweep from the CURRENT pose and heading, so an
    ARC's declared exit is only a label: when the fitted parameters do not
    reach it the robot arrives millimetres off, the next absolute GOTO has to
    creep that residual away, and the arcs read as "not connected" - measured
    on the loaded waypoint plan (2026-10-09): four arcs declaring exits the
    board could not reach, up to 7.5 mm, in one 5 m mission.

    Every ARC is therefore re-derived so that the plan IS the geometry the
    board drives:

    * the arc is AIMED at the exit it declares - a circle through two points
      that is tangent to the entry heading is unique, it turns by twice the
      bearing error - so the board ends exactly where the plan says, which is
      what stops the residual (and the tiny creep that ate it) for good;
    * the sweep travels on the 0.1 deg wire, so the RADIUS absorbs the
      rounding (whole mm): that is the only way a near-straight span reaches
      its declared exit at all (at R = 31011 mm one quantum is 54 mm);
    * when the wire cannot hold the aimed curvature (a tight junction
      connector, or a turn under the 50 mm radius / 2 mm length floors) the
      FITTED curvature is kept and the exit is re-landed on the point the
      board really reaches;
    * the exit - and the next segment's start, and the heading labels of the
      next arc - are written from the board's own arithmetic, so nothing
      downstream sees a phantom step or a stale tangent.

    The window the board accepts: radius 50..32767 mm, sweep < 360 deg, arc
    length 2..2900 mm, sweep sign = radius sign. An arc that cannot be
    represented becomes a GOTO to its own exit (one stop, but drivable and in
    the plan's frame).
    """
    changed = False
    cursor: Optional[Point] = None
    cursor_is_arc_exit = False
    for index, segment in enumerate(segments):
        kind = segment.get("kind")
        if kind == "goto":
            node = (float(segment["x"]), float(segment["y"]))
            if (
                cursor_is_arc_exit
                and cursor is not None
                and math.dist(node, cursor) <= ARC_NODE_SNAP_MM
            ):
                # a re-anchor node parked a hair away from the arc's own exit
                # (the node came from the reference path, the arc from the fit):
                # put the node ON the exit - a 0.5 mm phantom back-step there is
                # a stop plus a creep for nothing
                segment["x"] = round(cursor[0], 1)
                segment["y"] = round(cursor[1], 1)
                node = cursor
                changed = True
            cursor = node
            cursor_is_arc_exit = False
            continue
        if kind != "arc":
            continue
        start = (float(segment["x0"]), float(segment["y0"]))
        if cursor is not None and math.dist(start, cursor) > 0.05:
            # the previous arc landed a hair away from where this one says it
            # starts: the BOARD only knows where it really is, so take that
            start = cursor
            segment["x0"] = round(start[0], 1)
            segment["y0"] = round(start[1], 1)
            changed = True
        entry = float(segment["th0_deg"])
        declared_sweep = float(segment["sweep_deg"])
        target = (float(segment["x1"]), float(segment["y1"]))
        chord = math.dist(start, target)
        if chord <= 0.05:
            # a sub-0.05 mm speck (the junction trims bottom out there): nothing
            # is drivable in it, keep the label and move on
            cursor = target
            cursor_is_arc_exit = True
            continue
        # A circle through two points that is tangent to the entry heading is
        # unique: it turns by twice the bearing error, so aim at the declared
        # exit first and only give up on it when the wire cannot hold that
        # curvature. Reaching the declared exit IS the fix: a board that lands
        # millimetres away from it leaves the next GOTO to creep the residual
        # away - the "tiny movements and turns" of 2026-10-09.
        bearing = math.degrees(math.atan2(target[1] - start[1], target[0] - start[0]))
        turn = 2.0 * g.deg_norm180(bearing - entry)
        units = max(1, min(int(round(abs(turn) * 10.0)), 3599))
        # a near-straight span needs a MINIMUM sweep to stay under the 32767 mm
        # radius cap (no circle through two points is flatter than that)
        while units < 3599 and chord / (
            2.0 * math.sin(math.radians(units) / 20.0)
        ) > ARC_WIRE_MAX_RADIUS_MM:
            units += 1
        magnitude = chord / (2.0 * math.sin(math.radians(units) / 20.0))
        # a fitted curvature the board cannot drive is a hand drawn corner, not
        # an arc to rescue: leave it to the GOTO fallback below
        fitted_radius = abs(float(segment["radius_mm"]))
        if not (
            ARC_WIRE_MIN_RADIUS_MM <= fitted_radius <= ARC_WIRE_MAX_RADIUS_MM
            and ARC_WIRE_MIN_RADIUS_MM <= magnitude <= ARC_WIRE_MAX_RADIUS_MM
            and ARC_WIRE_MIN_LENGTH_MM - 0.5
            <= magnitude * math.radians(units / 10.0)
            <= ARC_WIRE_MAX_LENGTH_MM + 0.5
        ):
            # the declared exit is out of reach (a tight junction connector, or
            # a turn the 50 mm radius / 2 mm length floors forbid): keep the
            # FITTED curvature and ride the length floor - the exit is re-landed
            # on the point the board really reaches, just below
            turn = declared_sweep
            units = max(1, min(int(round(abs(turn) * 10.0)), 3599))
            while units < 3599 and chord / (
                2.0 * math.sin(math.radians(units) / 20.0)
            ) > ARC_WIRE_MAX_RADIUS_MM:
                units += 1
            magnitude = chord / (2.0 * math.sin(math.radians(units) / 20.0))
            fitted_circle = False
            if not (
                ARC_WIRE_MIN_RADIUS_MM <= magnitude <= ARC_WIRE_MAX_RADIUS_MM
                and magnitude * math.radians(units / 10.0)
                >= ARC_WIRE_MIN_LENGTH_MM - 0.5
            ):
                magnitude = abs(float(segment["radius_mm"]))
                fitted_circle = True
                if ARC_WIRE_MIN_RADIUS_MM <= magnitude <= ARC_WIRE_MAX_RADIUS_MM:
                    units = min(
                        3599,
                        max(
                            units,
                            int(
                                math.ceil(
                                    ARC_WIRE_MIN_LENGTH_MM
                                    / (magnitude * math.pi / 180.0)
                                    * 10.0
                                )
                            ),
                        ),
                    )
                else:
                    units = 0
            while (
                units > 1
                and magnitude * math.radians(units / 10.0) > ARC_WIRE_MAX_LENGTH_MM
            ):
                units -= 1
                if not fitted_circle:
                    magnitude = chord / (2.0 * math.sin(math.radians(units) / 20.0))
        else:
            while (
                units > 1
                and magnitude * math.radians(units / 10.0) > ARC_WIRE_MAX_LENGTH_MM
            ):
                units -= 1
                magnitude = chord / (2.0 * math.sin(math.radians(units) / 20.0))
        sweep = math.copysign(units / 10.0, turn if turn else 1.0)
        length = magnitude * math.radians(units / 10.0)
        if units <= 0 or not (
            ARC_WIRE_MIN_RADIUS_MM <= magnitude <= ARC_WIRE_MAX_RADIUS_MM
            and ARC_WIRE_MIN_LENGTH_MM - 0.5 <= length <= ARC_WIRE_MAX_LENGTH_MM + 0.5
        ):
            segments[index] = {
                "kind": "goto",
                "x": round(target[0], 1),
                "y": round(target[1], 1),
                "th": round(g.deg_norm180(entry + declared_sweep), 1),
                "v": int(segment.get("v", options.min_speed_mm_s)),
            }
            changed = True
            cursor = (float(segments[index]["x"]), float(segments[index]["y"]))
            cursor_is_arc_exit = True
            continue
        signed_radius = math.copysign(round(magnitude), sweep)
        centre = _arc_center(start, entry, signed_radius)
        # the point the board really reaches: the very rotation the firmware
        # and the mock drive (centre + R * (sin, -cos) of the exit heading)
        landed_angle = math.radians(entry + sweep)
        landed = (
            centre[0] + signed_radius * math.sin(landed_angle),
            centre[1] - signed_radius * math.cos(landed_angle),
        )
        if (
            abs(signed_radius - float(segment["radius_mm"])) > 0.05
            or abs(sweep - float(segment["sweep_deg"])) > 0.05
            or math.dist(landed, target) > 0.05
        ):
            changed = True
        segment["radius_mm"] = round(signed_radius, 1)
        segment["sweep_deg"] = round(sweep, 1)
        segment["x1"] = round(landed[0], 1)
        segment["y1"] = round(landed[1], 1)
        segment["th1_deg"] = round(entry + sweep, 1)
        # the board ends an arc on entry + sweep, so the segments after it have
        # to expect exactly that heading: a stale label there would make the
        # next aim wrong by the difference, and a re-anchor node that was
        # already tangent would turn into a tiny in-place pivot (a "touch-up"
        # the user sees as a dance). A node that really turns a corner (a
        # planned pivot) is left alone.
        following = segments[index + 1] if index + 1 < len(segments) else None
        if following is not None:
            if following.get("kind") == "arc":
                # only where the fit already wanted a tangent junction: a
                # drawing corner (a real kink between two arcs) keeps its own
                # heading, or the aim would turn the next arc into a hairpin
                if abs(
                    g.deg_norm180(float(following["th0_deg"]) - (entry + sweep))
                ) <= ARC_CHAIN_TANGENT_DEG:
                    if abs(
                        g.deg_norm180(float(following["th0_deg"]) - (entry + sweep))
                    ) > 0.05:
                        following["th0_deg"] = round(entry + sweep, 1)
                        changed = True
            elif following.get("kind") == "goto" and not following.get("arrival_gate") and abs(
                g.deg_norm180(float(following["th"]) - (entry + sweep))
            ) <= ARC_CHAIN_TANGENT_DEG:
                if abs(
                    g.deg_norm180(float(following["th"]) - (entry + sweep))
                ) > 0.05:
                    following["th"] = round(entry + sweep, 1)
                    changed = True
                if index + 2 < len(segments) and segments[index + 2].get("kind") == "arc":
                    after = segments[index + 2]
                    if abs(
                        g.deg_norm180(float(after["th0_deg"]) - (entry + sweep))
                    ) > 0.05:
                        after["th0_deg"] = round(entry + sweep, 1)
                        changed = True
        # the samples must BE the arc the board drives (start on the real start
        # and end on the point it really reaches): a junction connector starts
        # on a neighbour's TRIMMED exit, so the sampled geometry otherwise
        # misses the declared endpoint and every audit shows a phantom step
        phi0 = math.atan2(start[1] - centre[1], start[0] - centre[0])
        points = _arc_samples(centre, abs(signed_radius), phi0, math.radians(sweep))
        if points:
            points[0] = start
            points[-1] = landed
        segment["points"] = [[round(x, 1), round(y, 1)] for x, y in points]
        cursor = (float(segment["x1"]), float(segment["y1"]))
        cursor_is_arc_exit = True
    # two re-anchors can land on the same junction (a re-anchor GOTO next to
    # the arc's own pre-align node): keep ONE - a zero-length move is a stop
    # for nothing
    for index in range(len(segments) - 1, 0, -1):
        before = segments[index - 1]
        segment = segments[index]
        if before.get("kind") != "goto" or segment.get("kind") != "goto":
            continue
        if math.dist((before["x"], before["y"]), (segment["x"], segment["y"])) <= 1.0:
            before["th"] = segment["th"]
            before["v"] = max(int(before.get("v", 0)), int(segment.get("v", 0)))
            del segments[index]
            changed = True
    for segment in segments:
        segment.pop("arrival_gate", None)
    return changed


def _collapse_cusps(segments: List[Dict], options: PlanOptions, stays_legal) -> bool:
    """Replace a hand drawn switchback with ONE pivot node.

    The sampler follows the drawn geometry point by point, so a hand drawn
    reversal - real case 2026-10-07: 14 GOTO nodes 2-4 mm apart turning
    10-13 deg each - drove as a stop-go stutter (the board brakes to a FULL
    stop at every GOTO node), for an excursion that fits inside the robot's
    own footprint. The note is 300 mm wide; no wheel geometry can make a few
    millimetre cusp smooth.

    A run of GOTO nodes with an equivalent radius (travelled length over
    turned angle) below the drivable ARC minimum never fits an arc, so the
    run is replaced by its LAST node - the exit corner, which is also the
    entry node of the following ARC when there is one. The board drives
    straight there and turns in place ONCE (the GOTO final turn points the
    robot at the next step, the "faces the next node" pass sets the angle).
    Out-and-back jabs are left alone (they make little net progress and may
    be a target the operator drew), and the straight cut is rejected when it
    leaves the legal area.

    Returns True when something was replaced.
    """
    edited = False
    index = 0
    while index < len(segments):
        if segments[index].get("kind") != "goto":
            index += 1
            continue
        start = index
        while index + 1 < len(segments) and segments[index + 1].get("kind") == "goto":
            index += 1
        end = index
        index = end + 1
        if end - start + 1 < CUSP_MIN_NODES:
            continue
        run = segments[start : end + 1]
        span = 0.0
        for prev, nxt in zip(run, run[1:]):
            span += math.dist((prev["x"], prev["y"]), (nxt["x"], nxt["y"]))
        if span < 1.0 or span > CUSP_MAX_SPAN_MM:
            continue
        net = math.dist(
            (run[0]["x"], run[0]["y"]),
            (run[-1]["x"], run[-1]["y"]),
        )
        if net < CUSP_MIN_PROGRESS * span:
            continue  # out-and-back jab: keep the tip
        entry: Optional[float] = None
        if start > 0:
            before = segments[start - 1]
            if before.get("kind") == "goto":
                entry = math.degrees(
                    math.atan2(run[0]["y"] - before["y"], run[0]["x"] - before["x"])
                )
            elif before.get("kind") == "arc":
                entry = float(before["th0_deg"]) + float(before["sweep_deg"])
            elif before.get("kind") == "drive":
                # a departure drive (the close-waypoint backup) has no x1:
                # its sampled points give the travel direction into the run
                pts = before.get("points") or []
                if len(pts) >= 2:
                    entry = math.degrees(
                        math.atan2(
                            float(pts[-1][1]) - float(pts[-2][1]),
                            float(pts[-1][0]) - float(pts[-2][0]),
                        )
                    )
        if entry is None:
            entry = float(run[0].get("th", 0.0))
        exit_deg: Optional[float] = None
        if end + 1 < len(segments):
            after = segments[end + 1]
            if after.get("kind") == "arc":
                exit_deg = float(after["th0_deg"])
            elif after.get("kind") == "goto":
                exit_deg = math.degrees(
                    math.atan2(after["y"] - run[-1]["y"], after["x"] - run[-1]["x"])
                )
        if exit_deg is None:
            exit_deg = float(run[-1].get("th", 0.0))
        turn = abs(g.deg_norm180(exit_deg - entry))
        if turn < CUSP_MIN_TURN_DEG:
            continue
        radius_est = span / max(1e-6, math.radians(turn))
        if radius_est >= CUSP_MAX_RADIUS_MM:
            continue  # drivable as an arc: not a cusp
        keeper = run[-1]
        if start > 0:
            before = segments[start - 1]
            if before.get("kind") == "goto":
                from_pt = (float(before["x"]), float(before["y"]))
            elif before.get("kind") == "arc":
                from_pt = (float(before["x1"]), float(before["y1"]))
            elif before.get("kind") == "drive":
                pts = before.get("points") or []
                if len(pts) < 2:
                    continue
                from_pt = (float(pts[-1][0]), float(pts[-1][1]))
            else:
                continue
            to_pt = (float(keeper["x"]), float(keeper["y"]))
            samples = [
                (
                    from_pt[0] + (to_pt[0] - from_pt[0]) * step / 8.0,
                    from_pt[1] + (to_pt[1] - from_pt[1]) * step / 8.0,
                )
                for step in range(9)
            ]
            if not stays_legal(samples):
                continue
        segments[start : end + 1] = [keeper]
        edited = True
        index = start + 1
    return edited


def _launch_weave(
    segments: List[Dict],
    samples_all: List[Point],
    start: Point,
    start_heading_deg: float,
    stays_legal,
) -> bool:
    """Weave the opening into ONE connector arc tangent to the first curve.

    A stop-and-turn launch (turn to the travel heading, crawl the last few
    mm, turn again to the arc tangent - the "annoying first turn") is what a
    robot standing at the start of the drawing performs when the first fitted
    ARC cannot be entered on its tangent. This pass replaces the whole short
    opening lead (leading pause kept, user rotations never touched) with a
    connector ARC that starts AT the robot pose with the CURRENT heading and
    touches the first drawn circle tangentially: the mission launches as one
    continuous move onto the drawn curve.

    Applied only when the lead is short (the robot really stands at the
    start), the connector is gentle and within the radius limits, legal on
    the table with the body clearance, the first drawn arc keeps
    ``LAUNCH_WEAVE_MIN_REST_DEG`` of its sweep, and the connector deviates
    from the drawn opening by at most ``LAUNCH_WEAVE_MAX_DEV_MM``; anything
    else keeps the previous behaviour. Returns True when it changed the plan.
    """

    def arc_pts(center, signed_r, th0, sweep, max_step_deg=10.0):
        steps = max(1, int(abs(sweep) / max_step_deg) + 1)
        return [
            (
                center[0] + signed_r * math.sin(math.radians(th0 + sweep * k / steps)),
                center[1] - signed_r * math.cos(math.radians(th0 + sweep * k / steps)),
            )
            for k in range(steps + 1)
        ]

    index = 0
    while index < len(segments) and segments[index]["kind"] == "pause":
        index += 1
    first_motion = index
    lead_len = 0.0
    prev_point = start
    lead_points: List[Point] = [start]
    while index < len(segments):
        seg = segments[index]
        if seg["kind"] == "goto":
            point = (float(seg["x"]), float(seg["y"]))
            lead_len += math.dist(prev_point, point)
            prev_point = point
            lead_points.append(point)
            index += 1
        elif seg["kind"] == "turn":
            if seg.get("explicit"):
                return False  # a user rotation opens the mission: never eat it
            index += 1
        else:
            break
    if lead_len > LAUNCH_WEAVE_MAX_LEAD_MM or index >= len(segments):
        return False
    if segments[index]["kind"] != "arc":
        return False
    arc = segments[index]
    r2 = float(arc["radius_mm"])
    th0 = float(arc["th0_deg"])
    c2 = _arc_center((float(arc["x0"]), float(arc["y0"])), th0, r2)
    heading = math.radians(start_heading_deg)
    normal = (-math.sin(heading), math.cos(heading))
    dx, dy = start[0] - c2[0], start[1] - c2[1]
    # connector signed radius: the unique circle through the pose tangent to
    # the heading that touches the drawn circle (|c1-c2| = |s1-s2|)
    denominator = 2.0 * (normal[0] * dx + normal[1] * dy + r2)
    if abs(denominator) < 1e-6:
        return False
    s1 = (r2 * r2 - (dx * dx + dy * dy)) / denominator
    if not (ARC_CONNECT_MIN_RADIUS_MM <= abs(s1) <= ARC_CONNECT_MAX_RADIUS_MM):
        return False
    c1 = (start[0] + s1 * normal[0], start[1] + s1 * normal[1])
    vx, vy = c1[0] - c2[0], c1[1] - c2[1]
    distance = math.hypot(vx, vy)
    if distance < 1e-6 or abs(distance - abs(r2 - s1)) > 1.0:
        return False
    sin_j, neg_cos_j = vx / (r2 - s1), vy / (r2 - s1)
    theta_j = math.degrees(math.atan2(sin_j, -neg_cos_j))
    sweep1 = g.deg_norm180(theta_j - start_heading_deg)
    rest = g.deg_norm180(th0 + float(arc["sweep_deg"]) - theta_j)
    sweep_arc = abs(float(arc["sweep_deg"]))
    if abs(sweep1) < 2.0 or abs(sweep1) > LAUNCH_WEAVE_MAX_SWEEP_DEG or sweep1 * s1 <= 0.0:
        return False
    if rest * r2 <= 0.0 or abs(rest) < LAUNCH_WEAVE_MIN_REST_DEG:
        return False
    if abs(rest) > sweep_arc + 5.0:
        return False
    # natural tangency point: (rest may run a hair past the drawn sweep -
    # the exact tangent continuity is worth more than the last fraction of a
    # degree; the truncated arc still ends on the drawn endpoint)
    junction = (c2[0] + r2 * sin_j, c2[1] + r2 * neg_cos_j)
    connector_pts = arc_pts(c1, s1, start_heading_deg, sweep1)
    old_head = arc_pts(
        c2, r2, th0, g.deg_norm180(theta_j - th0) + math.copysign(10.0, r2)
    )
    old_polyline = lead_points + old_head
    # the robot may stand inside the border strip (match start): the first
    # millimetres leaving it are exempt from the explicit legality check,
    # exactly like the lead GOTO they replace
    checked_pts = [
        point for point in connector_pts
        if math.dist(point, start) > LAUNCH_WEAVE_LEGAL_SKIP_MM
    ]
    if checked_pts and not stays_legal(checked_pts):
        return False
    if max(_point_to_polyline(point, old_polyline) for point in connector_pts) > LAUNCH_WEAVE_MAX_DEV_MM:
        return False
    connector = {
        "kind": "arc",
        "x0": round(start[0], 1),
        "y0": round(start[1], 1),
        "x1": round(junction[0], 1),
        "y1": round(junction[1], 1),
        "radius_mm": round(s1, 1),
        "sweep_deg": round(sweep1, 1),
        "th0_deg": round(g.deg_norm180(start_heading_deg), 1),
        "v": int(arc["v"]),
        "points": [[round(x, 1), round(y, 1)] for x, y in connector_pts],
    }
    rest_pts = arc_pts(c2, r2, theta_j, rest)
    rest_pts[-1] = (float(arc["x1"]), float(arc["y1"]))
    new_arc = {
        **arc,
        "x0": round(junction[0], 1),
        "y0": round(junction[1], 1),
        "sweep_deg": round(rest, 1),
        "th0_deg": round(g.deg_norm180(theta_j), 1),
        "points": [[round(x, 1), round(y, 1)] for x, y in rest_pts],
    }
    segments[first_motion : index + 1] = [connector, new_arc]
    # the preview path: swap everything before the junction for the connector
    join = None
    for k in range(len(samples_all)):
        if abs(math.dist(samples_all[k], c2) - abs(r2)) <= 1.5:
            join = k
            break
    if join is not None:
        window = range(join, min(join + 80, len(samples_all)))
        nearest = min(window, key=lambda k: math.dist(samples_all[k], junction))
        samples_all[:] = [tuple(point) for point in connector_pts] + samples_all[nearest + 1 :]
    return True


#: a run flatter than this (huge radius, tiny sweep) is a straight line the
#: board would otherwise drive as an alternating S of 8+ arcs - one GOTO is
#: smoother (one acceleration, one deceleration) and exactly on the chord
STRAIGHT_MERGE_MIN_RADIUS_MM = 4000.0
STRAIGHT_MERGE_MAX_SWEEP_DEG = 4.0
STRAIGHT_MERGE_MAX_DEV_MM = 6.0
STRAIGHT_MERGE_MAX_TURN_DEG = 4.0


def _segment_polyline(segment: Dict) -> List[Point]:
    """Points a flat ARC/GOTO contributed to the run (for the merge)."""
    kind = segment.get("kind")
    if kind == "goto":
        return [(float(segment["x"]), float(segment["y"]))]
    if kind == "arc":
        stored = segment.get("points")
        if stored and len(stored) >= 2:
            return [(float(point[0]), float(point[1])) for point in stored]
        return [
            (float(segment["x0"]), float(segment["y0"])),
            (float(segment["x1"]), float(segment["y1"])),
        ]
    return []


def _is_flat_segment(segment: Dict) -> bool:
    kind = segment.get("kind")
    if kind == "goto":
        return True
    if kind != "arc":
        return False
    return (
        abs(float(segment.get("radius_mm", 0.0))) >= STRAIGHT_MERGE_MIN_RADIUS_MM
        and abs(float(segment.get("sweep_deg", 0.0))) <= STRAIGHT_MERGE_MAX_SWEEP_DEG
    )


def _chord_deviation_points(points: Sequence[Point]) -> float:
    if len(points) < 3:
        return 0.0
    ax, ay = points[0]
    bx, by = points[-1]
    length = math.hypot(bx - ax, by - ay)
    if length <= 1e-9:
        return math.inf
    worst = 0.0
    for px, py in points[1:-1]:
        worst = max(worst, abs((bx - ax) * (ay - py) - (ax - px) * (by - ay)) / length)
    return worst


def _merge_straight_runs(segments: List[Dict], start: Point) -> bool:
    """Collapse a near-straight ARC/GOTO chain into ONE GOTO.

    ``_near_straight_arcs`` approximates a straight run with an alternating
    S of very flat ARCs so the board never stops at a node. On the real
    robot those tiny curvature sign changes read as "a lot of unnecessary
    turns / acceleration and deceleration" (2026-10-18). The same geometry
    as one GOTO is exactly straight: the board turns to the travel heading,
    drives with one ramp, then turns to the arrival tangent. The polyline is
    kept on the segment (``points``) so the map preview still draws the run.
    """
    changed = False
    index = 0
    while index < len(segments):
        if not _is_flat_segment(segments[index]):
            index += 1
            continue
        end = index
        close_leg = bool(segments[index].get("close_waypoint"))
        while (
            end < len(segments)
            and _is_flat_segment(segments[end])
            and bool(segments[end].get("close_waypoint")) == close_leg
        ):
            end += 1
        if end - index < 2:
            index = end
            continue
        run = segments[index:end]
        if len({int(segment.get("v", 0)) for segment in run}) > 1:
            # a per-stroke speed override must keep its own move: merging it
            # into the neighbouring cruise would silently lose the cap
            index = end
            continue
        points: List[Point] = []
        for segment in run:
            piece = _segment_polyline(segment)
            if not piece:
                continue
            if not points or math.dist(points[-1], piece[0]) > 0.5:
                points.append(piece[0])
            points.extend(piece[1:])
        if index == 0 and points and math.dist(points[0], start) > 0.5:
            points.insert(0, start)
        if len(points) < 3 or _chord_deviation_points(points) > STRAIGHT_MERGE_MAX_DEV_MM:
            index = end
            continue
        legs = [
            math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            for a, b in zip(points, points[1:])
            if math.dist(a, b) > 0.5
        ]
        if len(legs) < 2:
            index = end
            continue
        turn = abs(g.deg_norm180(legs[-1] - legs[0]))
        if turn > STRAIGHT_MERGE_MAX_TURN_DEG:
            index = end
            continue
        following = segments[end] if end < len(segments) else None
        if following is not None and following.get("kind") == "arc":
            arrival_heading = float(following.get("th0_deg", legs[-1]))
        elif following is not None and following.get("kind") == "goto":
            arrival_heading = math.degrees(
                math.atan2(
                    float(following["y"]) - points[-1][1],
                    float(following["x"]) - points[-1][0],
                )
            )
        else:
            last = run[-1]
            if last.get("kind") == "arc":
                arrival_heading = float(last.get("th0_deg", 0.0)) + float(last.get("sweep_deg", 0.0))
            else:
                arrival_heading = float(last.get("th", legs[-1]))
        speed = max(int(segment.get("v", 0)) for segment in run)
        merged: Dict = {
            "kind": "goto",
            "x": round(points[-1][0], 1),
            "y": round(points[-1][1], 1),
            "th": round(g.deg_norm180(arrival_heading), 1),
            "v": max(1, speed),
            # the executor's per-segment watchdog derives its budget from this
            # length; without it a 2 m straight would get the 300 mm default
            "distance_mm": round(
                sum(math.dist(a, b) for a, b in zip(points, points[1:])), 1
            ),
            "points": [[round(x, 1), round(y, 1)] for x, y in points],
        }
        if run[-1].get("arrival_gate"):
            merged["arrival_gate"] = True
        if any(segment.get("close_waypoint") for segment in run):
            merged["close_waypoint"] = True
        if any(segment.get("close_connector") for segment in run):
            merged["close_connector"] = True
        segments[index:end] = [merged]
        changed = True
        index += 1
    return changed


def _preview_path(segments: Sequence[Dict]) -> List[Point]:
    """Rebuild the preview polyline from the FINAL emitted segments.

    The arrival weave (``_merge_arrival_turn``) rebuilds the tail AFTER the
    run samples were collected, so the map overlay kept drawing the old
    straight-through approach while the robot drove the woven arc - "combine
    with the approach" looked like a no-op on the map (2026-10-01). GOTO
    nodes contribute their points; an ARC contributes its sampled chords -
    rebuilt from the arc's own (x0, y0, th0, radius, sweep) when the stored
    slice is stale or too sparse, so the overlay shows what the board drives.
    """
    path: List[Point] = []
    for segment in segments:
        kind = segment.get("kind")
        if kind == "goto":
            stored = segment.get("points")
            if stored:
                candidates = [(float(point[0]), float(point[1])) for point in stored]
                end = (float(segment["x"]), float(segment["y"]))
                if not candidates or math.dist(candidates[-1], end) > 0.5:
                    candidates.append(end)
            else:
                candidates = [(float(segment["x"]), float(segment["y"]))]
        elif kind in ("drive", "strafe"):
            stored = segment.get("points")
            if stored:
                candidates = [(float(point[0]), float(point[1])) for point in stored]
            elif "x0" in segment and "x1" in segment:
                candidates = [
                    (float(segment["x0"]), float(segment["y0"])),
                    (float(segment["x1"]), float(segment["y1"])),
                ]
            else:
                continue
        elif kind == "arc":
            start = (float(segment["x0"]), float(segment["y0"]))
            end = (float(segment["x1"]), float(segment["y1"]))
            radius = float(segment["radius_mm"])
            centre = _arc_center(start, float(segment["th0_deg"]), radius)
            # always render the arc's OWN geometry: a junction trim rewrites
            # x0/x1/sweep and can leave a stale or 2-point ``points`` slice,
            # which made the overlay (and its turn audit) show a jump that the
            # board never drives
            candidates = _arc_samples(
                centre,
                abs(radius),
                math.atan2(start[1] - centre[1], start[0] - centre[0]),
                math.radians(float(segment["sweep_deg"])),
            )
            # ...and land exactly on the declared junction: connector arcs
            # start/end on the NEIGHBOURS' trimmed exits, so the sampled
            # geometry misses the declared endpoint by up to ~1 mm, which the
            # overlay reads as a back-step
            if len(candidates) >= 2:
                candidates[0] = start
                if math.dist(candidates[-1], end) > ARC_NODE_SNAP_MM:
                    candidates.append(end)
                else:
                    candidates[-1] = end
        else:
            continue
        for point in candidates:
            if not path or math.dist(path[-1], point) > 0.5:
                path.append(point)
    return path


def _split_turn(angle_deg: float, limit: float = TURN_CHUNK_LIMIT_DEG) -> List[float]:
    """Split a rotation into firmware-sized chunks (|angle| <= 180)."""
    chunks: List[float] = []
    remaining = angle_deg
    while abs(remaining) > limit:
        step = math.copysign(limit, remaining)
        chunks.append(step)
        remaining -= step
    if abs(remaining) > 0.05:
        chunks.append(remaining)
    return chunks


def _route(
    grid: OccupancyGrid,
    points: List[Point],
    warnings: List[str],
    relaxed: Optional[OccupancyGrid] = None,
    body: Optional[OccupancyGrid] = None,
) -> List[Point]:
    """Re-route every leg that enters a clearance zone, safely and shortly.

    The drawn geometry is kept wherever it is fully clear; a leg that enters
    the configured obstacle margin (or the field border) is replaced by ONE
    Theta* detour. The tiers are tried in order of SAFETY, not length:

    1. ``grid`` - the full robot + obstacle margin (the configured one);
    2. ``relaxed`` - half the margin (only when the full one has no room);
    3. ``body`` - the physical half-diagonal + a small buffer (last resort,
       e.g. a waypoint or a re-planned tail that already stands inside a
       freshly inflated zone).

    Theta* already returns the shortest route on the grid it runs on, so the
    first feasible tier is both the safest and the shortest legal route. The
    tier that actually ran is always named in a warning - the console must
    never silently drive closer than the configured margin.
    """
    check = grid
    routed: List[Point] = [points[0]]
    cum: List[float] = [0.0]  # routed length up to each routed point

    def append_point(point: Point) -> None:
        """Append one route point, keeping the cumulative length in sync."""
        cum.append(cum[-1] + math.dist(routed[-1], point))
        routed.append(point)

    if not check.is_free_mm(points[0][0], points[0][1]):
        # the run STARTS inside a clearance zone (a drawn start hugging the
        # wall, or a re-planned tail whose first node lies on a fresh
        # obstacle): start from the closest allowed cell. Keeping the blocked
        # point made the route dive into the zone and snap back out - the
        # connector from the anchor ends on the same snapped cell, so no gap
        # and no loop. The move is warned, never silent.
        try:
            snapped_start = check.nearest_free(points[0][0], points[0][1])
        except NoPathError:
            snapped_start = None
        if snapped_start is not None and math.dist(snapped_start, points[0]) > 0.5:
            warnings.append(
                "path start inside the robot clearance zone (field border or obstacle) - "
                f"moved to the closest allowed point ({snapped_start[0]:.0f}, {snapped_start[1]:.0f}) mm"
            )
            routed[0] = snapped_start
    total = len(points)
    i = 1
    while i < total:
        a = routed[-1]
        if check.los(a, points[i]):
            append_point(points[i])
            i += 1
            continue
        # extend over the whole chain of blocked drawn legs: one detour for
        # the run, resuming at b (the free end of the chain)
        j = i + 1
        while j < total and not check.los(points[j - 1], points[j]):
            j += 1
        b = points[j - 1]

        def plan_escape(anchor: Point) -> Optional[Tuple[List[Point], str]]:
            """Theta* detour from ``anchor`` to b: the SAFEST feasible tier wins."""
            for route_grid, tier, allow_blocked_ends in (
                (grid, "full", False),
                (relaxed, "squeezed", False),
                (body, "body", True),
            ):
                if route_grid is None:
                    continue
                if not allow_blocked_ends and not (
                    route_grid.is_free_mm(anchor[0], anchor[1])
                    and route_grid.is_free_mm(b[0], b[1])
                ):
                    continue  # a run end inside that inflation: tier impossible
                try:
                    # theta_star snaps a blocked endpoint to the nearest free
                    # cell, so the body tier also covers stops inside a fresh
                    # obstacle
                    route = theta_star(route_grid, anchor, b)
                except NoPathError:
                    continue
                return (route, tier)
            return None

        detour: Optional[List[Point]] = None
        tier = ""
        cut_steps = 0
        cut_mm = 0.0
        escape = plan_escape(a)
        if escape is not None:
            detour, tier = escape
            # corner cut: the detour may also START EARLIER on the drawn
            # approach. A drawn route that runs into the obstacle clearance
            # and would have to reverse out (a dead-end approach) wastes the
            # walk-in AND the reversal; leaving the drawing further back skips
            # both when that is genuinely shorter. (A robot re-planned onto
            # such an approach used to drive up, U-turn and come back - the
            # user's "the robot did a return".)
            best_score = cum[-1] + g.polyline_length(detour)
            floor_score = best_score - CORNER_CUT_MIN_SAVE_MM
            steps = 1
            while (
                steps <= CORNER_CUT_MAX_STEPS
                and i - 1 - steps >= 0
                and len(routed) > steps
            ):
                anchor = points[i - 1 - steps]
                if math.dist(anchor, routed[-1 - steps]) > 0.6:
                    break  # the routed tail is not the drawn approach any more
                prefix = cum[-1 - steps]
                if cum[-1] - prefix > CORNER_CUT_MAX_MM:
                    break
                # theta* is never shorter than the direct line: prune early
                if prefix + math.dist(anchor, b) <= floor_score:
                    earlier = plan_escape(anchor)
                    if earlier is not None:
                        score = prefix + g.polyline_length(earlier[0])
                        if score < best_score and score <= floor_score:
                            best_score = score
                            cut_steps = steps
                            cut_mm = cum[-1] - prefix
                            detour, tier = earlier
                steps += 1
        if detour is not None:
            if cut_steps:
                del routed[-cut_steps:]
                del cum[-cut_steps:]
            if tier == "squeezed":
                warnings.append(
                    f"leg {i} squeezed through the obstacle margin - no room for the full margin there"
                )
            elif tier == "body":
                warnings.append(
                    f"leg {i} re-routed at body clearance (the drawn line is blocked)"
                )
            else:
                warnings.append(f"leg {i} re-routed to stay clear (obstacle or field border)")
            if cut_steps:
                warnings.append(
                    f"leg {i} detour starts {cut_mm:.0f} mm earlier (the drawn approach runs "
                    "into the obstacle) - corner cut, no reversal"
                )
            # simplify the ROUTE only (cell-scale jitter): the drawn geometry
            # must keep its shape (a coarse RDP would flatten arcs)
            for point in g.rdp(detour, grid.res * 0.8)[1:]:
                append_point(point)
        else:
            for point in points[i:j]:  # keep the drawn run as-is
                append_point(point)
            blocked_inside = not check.is_free_mm(a[0], a[1]) or any(
                not check.is_free_mm(x, y) for x, y in points[i:j]
            )
            if blocked_inside:
                warnings.append(
                    f"leg {i} lies inside the robot clearance zone (field border or obstacle) - kept as drawn"
                )
            else:
                warnings.append(f"leg {i} blocked but no route found - kept direct")
        i = j
    # NO blanket simplification here: detours are already simplified where
    # they are planned, and a coarse RDP over the whole run flattened the
    # 5 deg-sampled fitted arcs back into a handful of long chords (the exact
    # geometry the fit promises to keep).
    return routed


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

    Not a planning policy any more: routes prefer the full margin, and this
    tier is only the last resort when no route fits the configured clearance
    (a stop point standing inside a freshly inflated zone, a corridor that
    simply has no room). Callers warn loudly when it is used.
    """
    border, _ = clearances(options)
    return border + max(30.0, options.grid_mm * 0.6)


def build_grid(
    width: float,
    height: float,
    obstacles: Sequence[Dict],
    options: PlanOptions,
    inflation_mm: Optional[float] = None,
) -> OccupancyGrid:
    """Occupancy grid for this planning run (see clearance.build_grid)."""
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


def _fit_stroke(pts: List[Point], options: PlanOptions, primitives: List[Dict]) -> List[Point]:
    """Fit a drawn stroke to lines/arcs (arcs kept and regenerated exactly)."""
    if options.fit_enabled and len(pts) >= 3:
        fitted = g.segment_line_arc(pts, options.fit_tol_mm)
        primitives.extend(fitted)
        stroke_points: List[Point] = []
        for primitive in fitted:
            part = g.primitive_to_points(primitive, max_step_deg=5.0)
            if stroke_points and part and math.dist(stroke_points[-1], part[0]) < 0.5:
                part = part[1:]
            stroke_points.extend(part)
        return stroke_points
    return g.rdp(pts, max(2.0, options.fit_tol_mm * 0.75))


def _smooth_run(
    points: List[Point],
    grid: Optional[OccupancyGrid],
    options: PlanOptions,
    warnings: List[str],
    veto_grid: Optional[OccupancyGrid] = None,
    waypoints: Optional[List[Point]] = None,
) -> List[Point]:
    """Simplify micro-vertices, then round corners with tangent fillets.

    The pre-simplification matters: fitted primitives can leave tiny legs at
    a sharp vertex, and a fillet whose legs are 50 mm long is useless - after
    simplifying, the fillet sees the real (long) legs and can swing wide.
    Rounding is **per corner**: a corner whose fillet would enter the robot
    clearance zone keeps its vertex, the others are rounded. ``veto_grid``
    defaults to ``grid`` (the FULL obstacle margin), so a fillet can never
    widen a drawn leg back into a clearance zone.
    """
    radius = float(options.corner_radius_mm)
    if not options.smooth or radius <= 0.0 or len(points) < 3:
        return list(points)
    simplified = g.rdp(points, max(6.0, options.fit_tol_mm))
    if len(simplified) < 3:
        return simplified
    if grid is None:
        return g.round_corners(simplified, radius)
    check = veto_grid if veto_grid is not None else grid
    cut = float(options.waypoint_cut_mm)
    limit_for = None
    if waypoints:
        def limit_for(vertex: Point, half: float) -> Optional[float]:
            """Speed cap for one node from the lateral acceleration limit."""
            for wp in waypoints:
                if math.dist(vertex, wp) <= 2.0:
                    if cut <= 0.0:
                        return 0.0  # exact crossing: keep the vertex sharp
                    # radius whose deepest point stays `cut` mm from the point
                    denom = max(1e-6, (1.0 / math.cos(half)) - 1.0)
                    return min(cut / denom, 800.0)
            return None
    # waypoints forced inside a clearance zone (the path is appended to them
    # anyway, with a warning) must still be swept smoothly: exempt their
    # fillets from the grid veto - refusing them would knife the corner
    forced = []
    if waypoints:
        forced = [wp for wp in waypoints if not check.is_free_mm(wp[0], wp[1])]
    forced_radius = cut + 70.0 if cut > 0.0 else 70.0
    kept: List[int] = []

    def fillet_veto(candidate: Sequence[Point]) -> bool:
        """True when a fillet at this corner would break a constraint."""
        if forced:
            mid = ((candidate[0][0] + candidate[-1][0]) * 0.5, (candidate[0][1] + candidate[-1][1]) * 0.5)
            for wp in forced:
                if math.dist(mid, wp) <= forced_radius:
                    return False
        return not all(check.is_free_mm(x, y) for x, y in candidate)

    rounded = g.round_corners(
        simplified,
        radius,
        veto=fillet_veto,
        radius_scales=(1.0, 0.6, 0.3),
        skipped=kept,
        radius_limit_for=limit_for,
    )
    if kept:
        warnings.append(f"{len(kept)} corner(s) kept sharp where rounding would leave the safety zone")
    return rounded


def _json_points(points: Sequence[Point]) -> List[List[float]]:
    """JSON-friendly point list."""
    return [[round(x, 1), round(y, 1)] for x, y in points]


def _json_primitives(primitives: Sequence[Dict]) -> List[Dict]:
    """JSON-friendly primitive list."""
    out: List[Dict] = []
    for primitive in primitives:
        if primitive["type"] == "line":
            out.append(
                {
                    "type": "line",
                    "a": [round(primitive["a"][0], 1), round(primitive["a"][1], 1)],
                    "b": [round(primitive["b"][0], 1), round(primitive["b"][1], 1)],
                }
            )
        else:
            out.append(
                {
                    "type": "arc",
                    "center": [round(primitive["center"][0], 1), round(primitive["center"][1], 1)],
                    "r": round(primitive["r"], 1),
                    "a_start_deg": round(primitive["a_start_deg"], 1),
                    "sweep_deg": round(primitive["sweep_deg"], 1),
                }
            )
    return out
