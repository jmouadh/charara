"""Planner tests: occupancy grid, Theta* routing and the full pipeline.

File:    tests/test_planner.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

import asyncio
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from software.core.mission.executor import EXEC_RUNNING  # noqa: E402
from software.core.mission.orchestrator import (  # noqa: E402
    _apply_resolved_waypoints,
    _assess_remaining_route,
    _clips_obstacle,
    _waypoint_blocked,
    active_obstacles,
    clip_gap_mm,
    elements_signature,
    execute_plan,
    resume_from_wait,
)
from software.core.navigation import geometry as g  # noqa: E402
from software.core.navigation.clearance import body_clear_at, waypoint_clearance  # noqa: E402
from software.core.navigation.trajectory import _collapse_cusps  # noqa: E402
from software.core.navigation.grid import NoPathError, OccupancyGrid, theta_star  # noqa: E402
from software.core.navigation.options import PlanOptions  # noqa: E402
from software.core.navigation.replan import (  # noqa: E402
    advance_cursor,
    continue_route_elements,
    drop_skipped_waypoints,
    executed_path,
    path_fraction,
    replan_route_elements,
    route_blocked,
    route_remainder_elements,
    slice_leading_stroke_at_robot,
    slice_remaining_elements,
)
from software.core.world.vision import VisionStore  # noqa: E402
from software.core.navigation.trajectory import (  # noqa: E402
    LINE_ARC_SWEEP_QUANT_DEG,
    _align_arc_after_arrival,
    _arc_center,
    _arrival_anchor,
    _cap_chain_length,
    _clamp_obstacles,
    _blend_tangent_reanchors,
    _point_to_polyline,
    _reanchor_turns,
    _sanitize_arcs,
    _set_arrival_heading,
    _smooth_arc_junctions,
    plan,
)

OBSTACLE = {"kind": "rect", "x": 1400.0, "y": 800.0, "w": 200.0, "h": 400.0}
INFLATE = 150.0
# Inflated blocked area: x 1250..1750, y 650..1350


def inside_inflated(x: float, y: float) -> bool:
    """Inside inflated."""
    return 1250.0 <= x <= 1750.0 and 650.0 <= y <= 1350.0


class GoalOnlyObstacleReplanTests(unittest.TestCase):
    def test_goal_only_replan_ignores_stale_executor_route(self):
        """An empty remainder routes directly to the goal, around static obstacles."""
        position = (600.0, 1000.0)
        goal = {"x": 2600.0, "y": 1000.0}
        obstacle = {"kind": "rect", "x": 1400.0, "y": 900.0, "w": 300.0, "h": 200.0}
        context = SimpleNamespace(
            pose=lambda: (*position, 0.0),
            last_plan={"path": [[600.0, 1000.0], [2600.0, 1000.0]]},
            field={"width_mm": 3000.0, "height_mm": 2000.0},
        )

        class StaleExecutor:
            def remaining_elements(self):
                return [
                    stroke([[600.0, 1000.0], [600.0, 1800.0], [2600.0, 1800.0], [2600.0, 1000.0]])
                ]

        affected, candidate, _ = asyncio.run(
            _assess_remaining_route(
                context,
                StaleExecutor(),
                [],
                goal,
                [obstacle],
                {"robot_width_mm": 300.0, "robot_height_mm": 300.0},
                "route",
                planned_elements=[],
            )
        )
        self.assertTrue(affected)
        self.assertIsNotNone(candidate)
        path = candidate["path"]
        self.assertLess(math.dist(path[-1], (goal["x"], goal["y"])), 5.0)
        self.assertLess(max(point[1] for point in path), 1500.0, "the stale route climbed to y=1800")
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        self.assertFalse(
            _clips_obstacle(path, [obstacle], options, 3000.0, 2000.0),
            "the optimized goal route must clear the static obstacle",
        )
        self.assertLess(candidate["stats"]["length_mm"], 2600.0)

    def test_wait_resume_discards_stale_executor_route_for_goal_only_mission(self):
        """A wait retry routes from live pose to goal, not along old segments."""
        position = (2639.5, 971.7)
        goal = {"x": 2775.9, "y": 802.5}
        obstacle = {"kind": "rect", "x": 2540.0, "y": 1650.0, "w": 300.0, "h": 300.0}
        vision = VisionStore()
        vision.update([obstacle], ttl_ms=60000, source="camera")

        class FakeExecutor:
            state = "aborted"

            def is_running(self):
                return False

            def set_speed_scale(self, scale):
                self.speed_scale = scale

            async def start(self, segments, deadman_ms=None, paused=False):
                self.segments = list(segments)
                self.state = "running"

        executor = FakeExecutor()
        stale_route = [stroke([[2600.0, 900.0], [2400.0, 1500.0], [2700.0, 1700.0]])]
        context = SimpleNamespace(
            plan_lock=asyncio.Lock(),
            executor=executor,
            vision=vision,
            opponent_pose=lambda: None,
            pose=lambda: (*position, -50.6),
            field={"width_mm": 3000.0, "height_mm": 2000.0},
            last_plan_request={"elements": [], "goal": goal, "options": {}, "obstacles": []},
            last_plan={"path": [[311.8, 1775.9], [2775.9, 802.5]]},
            vision_waiting={
                "elements": stale_route,
                "goal": goal,
                "obstacles": [],
                "options": {"robot_width_mm": 300.0, "robot_height_mm": 300.0},
                "deadman_ms": None,
                "paused_by_user": False,
                "direct_mm": 2000.0,
                "started_at": 0.0,
            },
            skipped_waypoints=set(),
        )

        async def ignore_event(event):
            return None

        async def ignore_log(level, message):
            return None

        context.emit = ignore_event
        context.log = ignore_log
        resumed = asyncio.run(resume_from_wait(context))
        self.assertTrue(resumed)
        self.assertIsNone(context.vision_waiting)
        self.assertLess(context.last_plan["stats"]["length_mm"], 300.0)
        self.assertTrue(executor.segments)


def stroke(points):
    """Stroke."""
    return {"kind": "stroke", "points": points}


def goto_segments(result):
    """Goto segments."""
    return [segment for segment in result["segments"] if segment["kind"] == "goto"]


def program_points(result):
    """Executed polyline: GOTO nodes plus the samples carried inside ARC segments.

    An arc is executed as one smooth sweep, so its interior samples belong to
    the driven path even though they are not GOTO nodes.
    """
    points = []
    for segment in result["segments"]:
        if segment["kind"] == "goto":
            candidates = [(segment["x"], segment["y"])]
        elif segment["kind"] == "arc":
            candidates = [(point[0], point[1]) for point in segment.get("points", [])]
        else:
            continue
        for point in candidates:
            if not points or math.dist(points[-1], point) > 1.0:
                points.append(point)
    return points


class BorderStripStrokeTests(unittest.TestCase):
    """Short strokes inside the border clearance strip must not vanish.

    A stroke lying fully inside the top strip (y=1800, legal for the centre
    is y <= ~1776) is "moved to the closest allowed point" by the router -
    for a 25 mm 2-point stroke that left ONE point, the stroke was dropped
    and a mission of (stroke + rotate) died as "no usable stroke" (real
    console flow, 2026-10-01). The clamped drawing is the drivable fallback.
    """

    def test_border_strip_stroke_survives_the_router(self):
        """The short blocked stroke falls back to the clamped drawing."""
        elements = [
            stroke([[675.0, 1800.0], [700.0, 1800.0]]),
            {"kind": "rotate", "angle_deg": 45.0},
        ]
        result = plan(elements, (675.0, 1775.0, 0.0), None, [], PlanOptions(), 3000.0, 2000.0)
        kinds = [segment["kind"] for segment in result["segments"]]
        self.assertIn("goto", kinds)  # the 25 mm move survives
        self.assertIn("turn", kinds)  # ... together with the rotation
        gotos = [segment for segment in result["segments"] if segment["kind"] == "goto"]
        self.assertAlmostEqual(gotos[-1]["y"], 1775.9, delta=1.0)  # clamped inside

    def test_actions_only_mission_plans(self):
        """A rotation-only mission plans as a single turn."""
        result = plan(
            [{"kind": "rotate", "angle_deg": 45.0}], (675.0, 1775.0, 0.0), None, [],
            PlanOptions(), 3000.0, 2000.0,
        )
        self.assertEqual([segment["kind"] for segment in result["segments"]], ["turn"])
        self.assertAlmostEqual(result["segments"][0]["angle_deg"], 45.0, delta=0.1)


class LaunchWeaveTests(unittest.TestCase):
    """A robot standing at the start launches ONTO the first curve.

    Real-robot rehearsal (2026-10-01): the mission opened with turn-to-travel,
    a ~25 mm crawl and a turn to the arc tangent - "the first turn is
    annoying, it should be more smooth". A short lead in front of the first
    fitted ARC is now replaced by ONE connector arc tangent to the robot's
    CURRENT heading and to the drawn circle (exact C1 continuity).
    """

    @staticmethod
    def _circle_stroke(center, radius, th0_deg, sweep_deg):
        """Stroke samples of an arc in the robot convention (c + r(sin, -cos))."""
        import math as _math

        points = []
        for step in range(0, int(abs(sweep_deg) / 5.0) + 1):
            angle = _math.radians(th0_deg + sweep_deg * step / (abs(sweep_deg) / 5.0))
            points.append([
                round(center[0] + radius * _math.sin(angle), 1),
                round(center[1] - radius * _math.cos(angle), 1),
            ])
        return points

    def test_short_lead_is_woven_into_one_connector_arc(self):
        """The opening becomes ONE tangent connector arc onto the first curve."""
        import math as _math

        # circle of the real mission's first drawn arc (fit of 2026-10-01)
        c2 = (646.3, 1696.5)
        stroke = self._circle_stroke(c2, 336.3, -94.8, 92.4)
        result = plan(
            [{"kind": "stroke", "points": stroke}],
            (315.0, 1800.0, -90.0), None, [], PlanOptions(), 3000.0, 2000.0,
        )
        segments = result["segments"]
        self.assertTrue(
            any("smooth connector arc" in warning for warning in result["warnings"])
        )
        first, second = segments[0], segments[1]
        self.assertEqual(first["kind"], "arc")           # no turn, no crawl
        self.assertAlmostEqual(first["x0"], 315.0, delta=0.2)
        self.assertAlmostEqual(first["y0"], 1800.0, delta=0.2)
        self.assertAlmostEqual(first["th0_deg"], -90.0, delta=0.2)  # launch on the current heading
        self.assertLess(first["sweep_deg"], 0.0)         # gentle one-way curve
        self.assertGreater(abs(first["sweep_deg"]), 2.0)
        self.assertLess(abs(first["sweep_deg"]), 12.0)
        self.assertGreater(abs(first["radius_mm"]), 400.0)
        # exact C1 junction: the drawn arc keeps its geometry and its end
        self.assertEqual(second["kind"], "arc")
        exit_deg = first["th0_deg"] + first["sweep_deg"]
        self.assertAlmostEqual(second["th0_deg"], exit_deg, delta=1.0)
        self.assertAlmostEqual(
            _math.dist((first["x1"], first["y1"]), (second["x0"], second["y0"])), 0.0, delta=1.0
        )
        self.assertGreaterEqual(abs(second["sweep_deg"]), 12.0)
        # ... and the truncated drawn arc still ends on the DRAWN endpoint
        self.assertAlmostEqual(second["x1"], stroke[-1][0], delta=2.0)
        self.assertAlmostEqual(second["y1"], stroke[-1][1], delta=2.0)

    def test_far_start_keeps_the_drive_up_connector(self):
        """A robot far from the drawing still DRIVES to the start first."""
        stroke = self._circle_stroke((646.3, 1696.5), 336.3, -94.8, 92.4)
        result = plan(
            [{"kind": "stroke", "points": stroke}],
            (1500.0, 800.0, 20.0), None, [], PlanOptions(), 3000.0, 2000.0,
        )
        self.assertFalse(
            any("smooth connector arc" in warning for warning in result["warnings"])
        )
        self.assertEqual(result["segments"][0]["kind"], "goto")

    def test_explicit_rotation_at_the_start_is_never_woven(self):
        """A user rotation opening the mission stays exactly as authored."""
        stroke = self._circle_stroke((646.3, 1696.5), 336.3, -94.8, 92.4)
        result = plan(
            [{"kind": "rotate", "angle_deg": 45.0}, {"kind": "stroke", "points": stroke}],
            (315.0, 1800.0, -90.0), None, [], PlanOptions(), 3000.0, 2000.0,
        )
        self.assertFalse(
            any("smooth connector arc" in warning for warning in result["warnings"])
        )
        self.assertEqual(result["segments"][0]["kind"], "turn")
        self.assertAlmostEqual(result["segments"][0]["angle_deg"], 45.0, delta=0.1)


class StepSpeedTests(unittest.TestCase):
    """Per-step speed overrides: one slow step, the rest keeps the system speed.

    The sequence accepts ``speed_mm_s`` on rotate / drive / stroke elements
    ("creep up to the point at 60 mm/s, then run normally"). Only the step
    carrying the override changes pace; everything else uses the system speed.
    """

    def test_rotate_speed_overrides_just_that_turn(self):
        elements = [
            {"kind": "rotate", "angle_deg": 45.0},
            {"kind": "rotate", "angle_deg": 45.0, "speed_mm_s": 120.0},
        ]
        result = plan(elements, (500.0, 500.0, 0.0), None, [], PlanOptions(), 3000.0, 2000.0)
        turns = [segment for segment in result["segments"] if segment["kind"] == "turn"]
        self.assertEqual(len(turns), 2)
        self.assertGreater(turns[0]["v"], 120)  # system turn speed
        self.assertEqual(turns[1]["v"], 120)    # the overridden one

    def test_drive_speed_caps_that_advance_only(self):
        elements = [
            {"kind": "drive", "distance_mm": 200.0, "speed_mm_s": 120.0},
            {"kind": "rotate", "angle_deg": 90.0},
        ]
        result = plan(elements, (500.0, 500.0, 0.0), None, [], PlanOptions(), 3000.0, 2000.0)
        gotos = [segment for segment in result["segments"] if segment["kind"] == "goto"]
        self.assertTrue(gotos)
        self.assertLessEqual(max(segment["v"] for segment in gotos), 120)
        turn = next(segment for segment in result["segments"] if segment["kind"] == "turn")
        self.assertGreater(turn["v"], 120)     # the rest is not slowed down

    def test_backward_drive_keeps_its_own_speed(self):
        elements = [
            {"kind": "drive", "distance_mm": -120.0, "speed_mm_s": 120.0},
            {"kind": "stroke", "points": [[500.0, 880.0], [500.0, 800.0]]},
        ]
        result = plan(elements, (500.0, 1000.0, 0.0), None, [], PlanOptions(), 3000.0, 2000.0)
        drive = next(segment for segment in result["segments"] if segment["kind"] == "drive")
        self.assertEqual(drive["v"], 120)

    def test_waypoint_speed_caps_the_approach(self):
        elements = [{"kind": "waypoint", "x": 900.0, "y": 500.0, "speed_mm_s": 130.0}]
        result = plan(
            elements, (500.0, 500.0, 0.0), None, [],
            PlanOptions(goto_step_mm=200.0), 3000.0, 2000.0,
        )
        motion = [s for s in result["segments"] if s["kind"] in ("goto", "arc")]
        self.assertTrue(motion)
        self.assertLessEqual(max(s.get("v", 0) for s in motion), 130)

    def test_stroke_speed_caps_only_that_stroke(self):
        elements = [
            {"kind": "stroke", "points": [[500.0, 500.0], [900.0, 500.0]], "speed_mm_s": 120.0},
            {"kind": "pause", "seconds": 0.5},   # keeps the two runs separate
            {"kind": "stroke", "points": [[900.0, 500.0], [1300.0, 500.0]]},
        ]
        options = PlanOptions(speed_mm_s=300.0, goto_step_mm=200.0, avoid_obstacles=False)
        result = plan(elements, (500.0, 500.0, 0.0), None, [], options, 3000.0, 2000.0)
        # a stroke may drive as flat ARCs now: check the motion of each run by
        # where its segments END, not by the GOTO nodes alone
        speeds = []
        for segment in result["segments"]:
            if segment["kind"] == "arc":
                centre = (segment["x0"] + segment["x1"]) / 2.0
            elif segment["kind"] == "goto":
                centre = segment["x"]
            else:
                continue
            speeds.append((centre, segment["v"]))
        first = [speed for centre, speed in speeds if centre <= 900.0]
        second = [speed for centre, speed in speeds if centre > 900.0]
        self.assertTrue(first and second)
        self.assertLessEqual(max(first), 120)
        self.assertGreater(max(second), 120)

    def test_speed_is_clamped_to_the_protocol_range(self):
        elements = [{"kind": "rotate", "angle_deg": 90.0, "speed_mm_s": 5.0}]
        result = plan(elements, (500.0, 500.0, 0.0), None, [], PlanOptions(), 3000.0, 2000.0)
        turn = next(segment for segment in result["segments"] if segment["kind"] == "turn")
        self.assertGreaterEqual(turn["v"], PlanOptions().min_speed_mm_s)


class ReleaseActionTests(unittest.TestCase):
    """Motor release is a geometry-free action inside a strategy sequence."""

    def test_release_is_kept_between_move_and_pause(self):
        elements = [
            {"kind": "stroke", "points": [[500.0, 500.0], [800.0, 500.0]]},
            {"kind": "release"},
            {"kind": "pause", "seconds": 1.0},
            {"kind": "stroke", "points": [[800.0, 500.0], [1100.0, 500.0]]},
        ]
        result = plan(elements, (500.0, 500.0, 0.0), None, [], PlanOptions(), 3000.0, 2000.0)
        kinds = [segment["kind"] for segment in result["segments"]]
        self.assertEqual(kinds.count("release"), 1)
        release = kinds.index("release")
        self.assertIn("pause", kinds[release + 1:])
        self.assertTrue(any(kind in ("goto", "arc") for kind in kinds[release + 1:]))

    def test_release_only_mission_is_valid(self):
        result = plan(
            [{"kind": "release"}], (500.0, 500.0, 0.0), None, [],
            PlanOptions(), 3000.0, 2000.0,
        )
        self.assertEqual([segment["kind"] for segment in result["segments"]], ["release"])


class LeadingStrokeSliceTests(unittest.TestCase):
    """A fresh plan continues on the route when the robot already stands on it.

    Real-robot UX bug (2026-10-01): a strategy starting with a straight line
    was "run" while the robot stood near the line's end - the plan drove it
    back to the line's start (two 180 deg pivots) and re-drove the whole
    line. The leading stroke must be cut at the robot instead.
    """

    def test_robot_on_the_route_continues_from_its_position(self):
        """The leading stroke is cut at the robot's projection."""
        elements = [
            {"kind": "stroke", "points": [[315.0, 1800.0], [700.0, 1800.0]]},
            {"kind": "rotate", "angle_deg": 45.0},
        ]
        sliced = slice_leading_stroke_at_robot(elements, (675.0, 1775.0))
        self.assertIsNot(sliced, elements)
        self.assertEqual(sliced[0]["kind"], "stroke")
        self.assertEqual(sliced[0]["points"][0], [675.0, 1800.0])  # projection, not the start
        self.assertEqual(sliced[0]["points"][-1], [700.0, 1800.0])
        self.assertEqual(sliced[1], elements[1])  # later elements verbatim
        self.assertEqual(elements[0]["points"][0], [315.0, 1800.0])  # source untouched

    def test_robot_at_the_start_keeps_the_drawing(self):
        """At (or within a few cm of) the start nothing changes."""
        elements = [{"kind": "stroke", "points": [[315.0, 1800.0], [700.0, 1800.0]]}]
        self.assertIs(slice_leading_stroke_at_robot(elements, (315.0, 1800.0)), elements)
        self.assertIs(slice_leading_stroke_at_robot(elements, (330.0, 1795.0)), elements)

    def test_robot_off_the_route_keeps_the_drawing(self):
        """Far from the line: the normal connect-to-the-start flow stays."""
        elements = [{"kind": "stroke", "points": [[315.0, 1800.0], [700.0, 1800.0]]}]
        self.assertIs(slice_leading_stroke_at_robot(elements, (500.0, 1200.0)), elements)

    def test_non_stroke_leading_element_keeps_the_drawing(self):
        """Only a leading STROKE is considered for slicing."""
        elements = [
            {"kind": "rotate", "angle_deg": 90.0},
            {"kind": "stroke", "points": [[315.0, 1800.0], [700.0, 1800.0]]},
        ]
        self.assertIs(slice_leading_stroke_at_robot(elements, (675.0, 1775.0)), elements)

    def test_robot_at_the_far_end_leaves_only_the_rest(self):
        """Standing on the line's end: the line is done, later steps remain."""
        elements = [
            {"kind": "stroke", "points": [[315.0, 1800.0], [700.0, 1800.0]]},
            {"kind": "rotate", "angle_deg": 45.0},
        ]
        sliced = slice_leading_stroke_at_robot(elements, (700.0, 1800.0))
        self.assertEqual(sliced, [elements[1]])


class GridTests(unittest.TestCase):
    def test_grid_blocks_inflated_rect(self):
        """Grid blocks inflated rect."""
        grid = OccupancyGrid(3000.0, 2000.0, 50.0, INFLATE)
        grid.add_rect(OBSTACLE["x"], OBSTACLE["y"], OBSTACLE["w"], OBSTACLE["h"])
        self.assertFalse(grid.is_free_mm(1500.0, 1000.0))   # inside
        self.assertFalse(grid.is_free_mm(1300.0, 1000.0))   # inside inflation
        self.assertTrue(grid.is_free_mm(1200.0, 1000.0))    # outside inflation

    def test_nearest_free_returns_the_closest_cell(self):
        # ring order used to decide: a point near the border snapped to the
        # first free cell of a ring, up to a ring-diagonal away, which bent
        # the approach legs sideways (46,811) used to snap 175 mm too low
        """Nearest free returns the closest cell."""
        grid = OccupancyGrid(3000.0, 2000.0, 50.0, 212.1, border_mm=212.1)
        self.assertEqual(grid.nearest_free(46.0, 811.0), (225.0, 825.0))
        self.assertEqual(grid.nearest_free(1506.0, 41.0), (1525.0, 225.0))

    def test_los_through_obstacle_is_blocked(self):
        """Los through obstacle is blocked."""
        grid = OccupancyGrid(3000.0, 2000.0, 50.0, INFLATE)
        grid.add_rect(OBSTACLE["x"], OBSTACLE["y"], OBSTACLE["w"], OBSTACLE["h"])
        self.assertFalse(grid.los((1000.0, 1000.0), (2000.0, 1000.0)))
        self.assertTrue(grid.los((1000.0, 400.0), (2000.0, 400.0)))

    def test_theta_star_avoids_obstacle(self):
        """Theta star avoids obstacle."""
        grid = OccupancyGrid(3000.0, 2000.0, 50.0, INFLATE)
        grid.add_rect(OBSTACLE["x"], OBSTACLE["y"], OBSTACLE["w"], OBSTACLE["h"])
        start, goal = (500.0, 1000.0), (2500.0, 1000.0)
        path = theta_star(grid, start, goal)
        self.assertGreaterEqual(len(path), 2)
        self.assertAlmostEqual(math.dist(path[0], start), 0.0, delta=1.0)
        self.assertAlmostEqual(math.dist(path[-1], goal), 0.0, delta=1.0)
        for x, y in path:
            self.assertFalse(inside_inflated(x, y), f"path point inside obstacle: {(x, y)}")
        length = sum(math.dist(path[i], path[i + 1]) for i in range(len(path) - 1))
        self.assertGreater(length, 2000.0)  # must be longer than the direct line

    def test_theta_star_straight_when_free(self):
        """Theta star straight when free."""
        grid = OccupancyGrid(3000.0, 2000.0, 50.0, INFLATE)
        start, goal = (500.0, 1000.0), (2500.0, 1000.0)
        path = theta_star(grid, start, goal)
        length = sum(math.dist(path[i], path[i + 1]) for i in range(len(path) - 1))
        self.assertLess(length, 2100.0)  # near-straight

    def test_theta_star_unreachable(self):
        """Theta star unreachable."""
        grid = OccupancyGrid(1000.0, 1000.0, 50.0, 50.0)
        grid.add_rect(0.0, 400.0, 1000.0, 200.0)  # full wall
        with self.assertRaises(NoPathError):
            theta_star(grid, (500.0, 100.0), (500.0, 900.0))


class PipelineTests(unittest.TestCase):
    def test_plan_without_obstacles(self):
        """Plan without obstacles."""
        elements = [stroke([[500.0, 1000.0], [1500.0, 1000.0], [2500.0, 1000.0]])]
        options = PlanOptions(avoid_obstacles=True, goto_step_mm=200.0, grid_mm=50.0)
        result = plan(elements, (500.0, 1000.0, 0.0), None, [], options, 3000.0, 2000.0)
        self.assertGreaterEqual(len(result["segments"]), 1)
        for segment in goto_segments(result):
            self.assertGreater(segment["v"], 0)
        self.assertEqual(result["warnings"], [])
        # a straight run may drive as ONE near-straight ARC now (no stop at a
        # GOTO node every goto_step_mm): check where the plan ENDS instead
        end = program_points(result)[-1]
        self.assertAlmostEqual(end[0], 2500.0, delta=60.0)
        self.assertAlmostEqual(end[1], 1000.0, delta=60.0)

    def test_plan_reroutes_around_obstacle(self):
        """Plan reroutes around obstacle."""
        elements = [stroke([[200.0, 1000.0], [2800.0, 1000.0]])]
        options = PlanOptions(avoid_obstacles=True, goto_step_mm=200.0, grid_mm=50.0, inflate_mm=INFLATE)
        result = plan(elements, (200.0, 1000.0, 0.0), None, [OBSTACLE], options, 3000.0, 2000.0)
        self.assertTrue(any("re-routed" in w for w in result["warnings"]))
        for segment in goto_segments(result):
            self.assertFalse(
                inside_inflated(segment["x"], segment["y"]),
                f"node inside obstacle: {segment}",
            )

    def test_plan_with_goal_angle(self):
        """Plan with goal angle."""
        elements = [stroke([[500.0, 500.0], [1000.0, 500.0]])]
        options = PlanOptions(goto_step_mm=250.0)
        result = plan(
            elements,
            (500.0, 500.0, 0.0),
            {"x": 1500.0, "y": 800.0, "theta_deg": 45.0},
            [],
            options,
            3000.0,
            2000.0,
        )
        last = result["segments"][-1]
        self.assertEqual(last["kind"], "goto")
        self.assertAlmostEqual(last["th"], 45.0, delta=0.2)
        self.assertAlmostEqual(last["x"], 1500.0, delta=60.0)

    def test_plan_goal_only(self):
        """Plan goal only."""
        options = PlanOptions(goto_step_mm=250.0)
        result = plan([], (500.0, 500.0, 0.0), {"x": 1200.0, "y": 900.0}, [], options, 3000.0, 2000.0)
        gotos = goto_segments(result)
        self.assertGreaterEqual(len(gotos), 1)
        self.assertAlmostEqual(gotos[-1]["x"], 1200.0, delta=40.0)

    def test_plan_emits_native_arc_for_a_circle(self):
        # a drawn quarter circle R=300 to the left must come out as native
        # ARC segment(s), not a string of GOTO nodes (the robot heading at
        # the start matches the tangent, so the arc needs no entry node)
        """Plan emits native arc for a circle."""
        circle = [
            [500.0 + 300.0 * math.sin(math.radians(a)), 800.0 - 300.0 * math.cos(math.radians(a))]
            for a in range(0, 91, 3)
        ]
        options = PlanOptions(avoid_obstacles=False, goto_step_mm=120.0)
        result = plan([stroke(circle)], (500.0, 500.0, 0.0), None, [], options, 3000.0, 2000.0)
        arcs = [segment for segment in result["segments"] if segment["kind"] == "arc"]
        self.assertTrue(arcs, "expected a native arc segment")
        for arc in arcs:
            self.assertGreater(arc["radius_mm"], 0.0)         # left arc
            self.assertAlmostEqual(arc["radius_mm"], 300.0, delta=30.0)
        self.assertAlmostEqual(sum(arc["sweep_deg"] for arc in arcs), 90.0, delta=10.0)
        self.assertLess(math.dist((arcs[0]["x0"], arcs[0]["y0"]), (500.0, 500.0)), 80.0)
        self.assertEqual(result["stats"]["n_arcs"], len(arcs))

    def test_native_arcs_can_be_disabled(self):
        """Native arcs can be disabled."""
        circle = [
            [500.0 + 300.0 * math.sin(math.radians(a)), 800.0 - 300.0 * math.cos(math.radians(a))]
            for a in range(0, 91, 3)
        ]
        options = PlanOptions(avoid_obstacles=False, goto_step_mm=120.0, native_arcs=False)
        result = plan([stroke(circle)], (500.0, 500.0, 0.0), None, [], options, 3000.0, 2000.0)
        self.assertFalse([segment for segment in result["segments"] if segment["kind"] == "arc"])

    def test_goal_leg_is_routed_around_obstacles(self):
        # regression: the goal connector used to ignore the obstacle grid
        # completely - a goal behind a box drove straight through it
        """Goal leg is routed around obstacles."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        obstacles = [{"kind": "rect", "x": 1400.0, "y": 900.0, "w": 300.0, "h": 200.0}]
        result = plan(
            [], (600.0, 1000.0, 0.0),
            {"x": 2600.0, "y": 1000.0, "theta_deg": 45.0},
            obstacles, options, 3000.0, 2000.0,
        )
        body = math.hypot(300.0, 300.0) / 2.0 - 1.0
        for x, y in result["path"]:
            inside = 1400.0 - body <= x <= 1700.0 + body and 900.0 - body <= y <= 1100.0 + body
            self.assertFalse(inside, f"goal leg inside the body clearance: ({x:.0f},{y:.0f})")
        gotos = goto_segments(result)
        self.assertAlmostEqual(gotos[-1]["x"], 2600.0, delta=2.0)
        self.assertAlmostEqual(gotos[-1]["y"], 1000.0, delta=2.0)
        self.assertAlmostEqual(gotos[-1]["th"], 45.0, delta=0.5)

    def test_margin_gap_is_kept_short(self):
        # obstacle (1400,560,200,200): the direct leg would pass through the
        # full 362 mm clearance. The route must go around it with the FULL
        # margin (never merely at body clearance - the configured margin is a
        # safety buffer the robot really keeps), and it must stay a compact
        # detour, not loop across the field.
        """The full obstacle margin is kept, with a short detour."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        obstacles = [{"kind": "rect", "x": 1400.0, "y": 560.0, "w": 200.0, "h": 200.0}]
        result = plan(
            [], (500.0, 250.0, 0.0), {"x": 2500.0, "y": 250.0},
            obstacles, options, 3000.0, 2000.0,
        )
        self.assertTrue(
            any("re-routed to stay clear" in w for w in result["warnings"]),
            result["warnings"],
        )
        self.assertLess(
            result["stats"]["length_mm"], 3000.0,
            "the detour must stay local, not loop across the field",
        )
        full_margin = math.hypot(300.0, 300.0) / 2.0 + 150.0
        for x, y in result["path"]:
            dx = max(1400.0 - x, x - 1600.0, 0.0)
            dy = max(560.0 - y, y - 760.0, 0.0)
            self.assertGreaterEqual(math.hypot(dx, dy), full_margin - 2.0,
                                    f"({x:.0f}, {y:.0f}) entered the full margin")

    def test_drawn_line_is_followed_and_detoured_locally(self):
        # stroke crossing the obstacle box and grazing its margin afterwards:
        # the crossing gets a LOCAL body-clearance detour and the margin-using
        # tails stay as drawn - the plan looks like the drawing (the old
        # behaviour looped around the top of the whole field)
        """Drawn line is followed and detoured locally."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        obstacles = [{"kind": "rect", "x": 803.0, "y": 1058.0, "w": 212.0, "h": 184.0}]
        pts = [(350.0, 1850.0), (450.0, 1650.0), (550.0, 1470.0), (650.0, 1330.0),
               (750.0, 1240.0), (850.0, 1180.0), (950.0, 1130.0), (1050.0, 1100.0),
               (1150.0, 1075.0), (1250.0, 1050.0), (1328.0, 1030.0)]
        elements = [stroke(pts), {"kind": "waypoint", "x": 1328.0, "y": 1030.0}]
        result = plan(elements, (350.0, 1850.0, -90.0), None, obstacles, options, 3000.0, 2000.0)
        self.assertLess(
            result["stats"]["length_mm"], 2300.0,
            "the plan must follow the drawing, not loop around the field",
        )
        self.assertTrue(any("margin" in w for w in result["warnings"]), result["warnings"])
        body = math.hypot(300.0, 300.0) / 2.0 - 1.0
        for x, y in result["path"]:
            dx = max(803.0 - x, x - 1015.0, 0.0)
            dy = max(1058.0 - y, y - 1242.0, 0.0)
            self.assertGreaterEqual(math.hypot(dx, dy), body)
        gotos = goto_segments(result)
        # straights may drive as flat ARCs now: measure the executed geometry
        nearest = min(math.dist(p, (1328.0, 1030.0)) for p in program_points(result))
        self.assertLess(nearest, 70.0, "the waypoint must be swept closely")

    def test_blocked_run_gets_one_clean_detour(self):
        # regression: several consecutive blocked drawn legs used to be
        # re-routed ONE BY ONE - the chained detours zig-zagged wildly through
        # the clearance pockets. A whole blocked run must become ONE detour.
        """Blocked run gets one clean detour."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        obstacles = [{"kind": "rect", "x": 1400.0, "y": 900.0, "w": 400.0, "h": 200.0}]
        elements = [stroke([
            [600.0, 1000.0], [1200.0, 1000.0], [1300.0, 1050.0], [1500.0, 1100.0],
            [1700.0, 1050.0], [1800.0, 1000.0], [2400.0, 1000.0],
        ])]
        result = plan(elements, (600.0, 1000.0, 0.0), None, obstacles, options, 3000.0, 2000.0)
        reroutes = [w for w in result["warnings"] if "re-routed" in w or "squeezed" in w]
        self.assertEqual(len(reroutes), 1, result["warnings"])
        path = result["path"]
        reversals = 0
        direction = 0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            if abs(dx) < 1.0:
                continue
            new_dir = 1 if dx > 0 else -1
            if direction and new_dir != direction:
                reversals += 1
            direction = new_dir
        self.assertLessEqual(reversals, 1, f"the path zig-zags ({reversals} x-reversals)")
        max_turn = 0.0
        for i in range(1, len(path) - 1):
            a, b, c = path[i - 1], path[i], path[i + 1]
            h1 = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            h2 = math.degrees(math.atan2(c[1] - b[1], c[0] - b[0]))
            max_turn = max(max_turn, abs((h2 - h1 + 540.0) % 360.0 - 180.0))
        self.assertLess(max_turn, 25.0, f"sharp corner left (max turn {max_turn:.0f} deg)")

    def test_plan_requires_geometry(self):
        """Plan requires geometry."""
        with self.assertRaises(ValueError):
            plan([], (0.0, 0.0, 0.0), None, [], PlanOptions(), 3000.0, 2000.0)

    def test_sequence_rotate_and_pause_between_strokes(self):
        """Sequence rotate and pause between strokes."""
        elements = [
            stroke([[500.0, 500.0], [900.0, 500.0]]),
            {"kind": "rotate", "angle_deg": 90.0},
            {"kind": "pause", "seconds": 2.0},
            stroke([[900.0, 500.0], [900.0, 900.0]]),
        ]
        options = PlanOptions(goto_step_mm=250.0, avoid_obstacles=False, smooth=False)
        result = plan(elements, (500.0, 500.0, 0.0), None, [], options, 3000.0, 2000.0)
        kinds = [segment["kind"] for segment in result["segments"]]
        self.assertIn("turn", kinds)
        self.assertIn("pause", kinds)
        turn_index = kinds.index("turn")
        pause_index = kinds.index("pause")
        self.assertLess(turn_index, pause_index)
        # the stretch before the rotation is one motion run (flat ARCs now)
        self.assertTrue(all(k in ("goto", "arc") for k in kinds[:turn_index]))
        self.assertGreater(len(kinds) - pause_index, 1)
        self.assertAlmostEqual(result["segments"][turn_index]["angle_deg"], 90.0, delta=0.1)
        self.assertAlmostEqual(result["segments"][pause_index]["seconds"], 2.0, delta=0.01)
        self.assertGreaterEqual(result["stats"]["est_s"], 2.0)

    def test_turn_split_for_large_rotation(self):
        """Turn split for large rotation."""
        elements = [{"kind": "rotate", "angle_deg": 400.0}]
        options = PlanOptions(avoid_obstacles=False)
        result = plan(elements, (500.0, 500.0, 0.0), {"x": 700.0, "y": 500.0}, [], options, 3000.0, 2000.0)
        turns = [segment for segment in result["segments"] if segment["kind"] == "turn"]
        self.assertGreaterEqual(len(turns), 3)
        for turn in turns:
            self.assertLessEqual(abs(turn["angle_deg"]), 170.01)
        self.assertAlmostEqual(sum(turn["angle_deg"] for turn in turns), 400.0, delta=0.5)

    def test_far_strokes_are_connected(self):
        """Far strokes are connected."""
        elements = [
            stroke([[500.0, 500.0], [800.0, 500.0]]),
            stroke([[2000.0, 1500.0], [2300.0, 1500.0]]),
        ]
        options = PlanOptions(goto_step_mm=250.0, avoid_obstacles=False, smooth=False)
        result = plan(elements, (500.0, 500.0, 0.0), None, [], options, 3000.0, 2000.0)
        mids = [
            p
            for p in program_points(result)
            if 900.0 < p[0] < 1900.0 and 600.0 < p[1] < 1400.0
        ]
        self.assertGreaterEqual(len(mids), 1, f"no connector leg: {result['segments']}")

    def test_arc_stays_arc_with_intermediate_nodes(self):
        """Arc stays arc with intermediate nodes."""
        center = (2000.0, 1000.0)
        pts = [g.point_at_angle(center, 400.0, 10.0 + 90.0 * i / 40) for i in range(41)]
        options = PlanOptions(goto_step_mm=250.0, smooth=True, corner_radius_mm=0.0)
        result = plan(
            [stroke([list(p) for p in pts])],
            (pts[0][0], pts[0][1], 0.0),
            None,
            [],
            options,
            3000.0,
            2000.0,
        )
        arcs = [p for p in result["primitives"] if p["type"] == "arc"]
        self.assertEqual(len(arcs), 1, "the drawn arc must stay an arc")
        fitted_r = arcs[0]["r"]
        # the drawn arc may be driven as one native ARC (no GOTO chain): check
        # the executed polyline, where the arc's samples belong too
        driven = program_points(result)
        self.assertGreaterEqual(len(driven), 5)
        worst = max(abs(math.dist(point, center) - fitted_r) for point in driven)
        self.assertLess(worst, 10.0, "the executed path must stay on the fitted arc")

    def test_sharp_corner_starts_turning_before_the_vertex(self):
        """Sharp corner starts turning before the vertex."""
        corner = (1600.0, 600.0)
        options = PlanOptions(goto_step_mm=250.0, corner_radius_mm=150.0)
        result = plan(
            [stroke([[600.0, 600.0], [1600.0, 600.0], [1600.0, 1600.0]])],
            (600.0, 600.0, 0.0),
            None,
            [],
            options,
            3000.0,
            2000.0,
        )
        # the fillet may be driven as a native ARC: measure on the executed
        # polyline (nodes plus arc samples), not on the GOTO list alone
        points = program_points(result)
        nearest = min(math.dist(point, corner) for point in points)
        expected = 150.0 * (math.sqrt(2.0) - 1.0)  # fillet passes this close to the vertex
        self.assertGreater(nearest, expected * 0.5)
        self.assertLess(nearest, expected * 1.6)
        legs = [(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:])]
        max_turn = 0.0
        for (ax, ay), (bx, by) in zip(legs, legs[1:]):
            norm_a, norm_b = math.hypot(ax, ay), math.hypot(bx, by)
            if norm_a < 1e-6 or norm_b < 1e-6:
                continue
            cos = (ax * bx + ay * by) / (norm_a * norm_b)
            max_turn = max(max_turn, math.degrees(math.acos(max(-1.0, min(1.0, cos)))))
        self.assertLess(max_turn, 40.0, "the 90 deg corner must be split into small turns")

    def test_stroke_joint_is_rounded(self):
        """Stroke joint is rounded."""
        elements = [
            stroke([[600.0, 600.0], [1600.0, 600.0]]),
            stroke([[1600.0, 600.0], [1600.0, 1600.0]]),
        ]
        options = PlanOptions(goto_step_mm=250.0, corner_radius_mm=150.0)
        result = plan(elements, (600.0, 600.0, 0.0), None, [], options, 3000.0, 2000.0)
        # the joint must be ROUNDED: no step of the driven path comes near the
        # drawn corner (straight runs now drive as flat ARCs, so the check
        # looks at the executed geometry, not at GOTO nodes)
        nearest = min(math.dist(p, (1600.0, 600.0)) for p in program_points(result))
        self.assertGreater(nearest, 30.0, "the joint between two strokes must be rounded too")

    def test_small_fillet_keeps_its_nodes(self):
        # short legs at a junction leave a small fillet - it must be driven as
        # a SMOOTH piece (a small ARC or its dense GOTO nodes), never swallowed
        # into a single in-place pivot
        """Small fillet keeps its nodes."""
        elements = [
            stroke([[500.0, 600.0], [700.0, 600.0]]),
            stroke([[700.0, 600.0], [640.0, 700.0]]),
        ]
        options = PlanOptions(goto_step_mm=250.0, corner_radius_mm=200.0)
        result = plan(elements, (500.0, 600.0, 0.0), None, [], options, 3000.0, 2000.0)
        gotos = goto_segments(result)
        near = [s for s in gotos if math.hypot(s["x"] - 700.0, s["y"] - 600.0) < 60.0]
        arcs = [
            s
            for s in result["segments"]
            if s["kind"] == "arc"
            and min(math.hypot(px - 700.0, py - 600.0) for px, py in s.get("points", [])) < 60.0
        ]
        self.assertTrue(
            len(near) >= 3 or arcs,
            f"the small fillet was swallowed into one pivot: {result['segments']}",
        )
        legs = [(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(gotos, gotos[1:])]
        max_turn = 0.0
        for (ax, ay), (bx, by) in zip(legs, legs[1:]):
            norm_a, norm_b = math.hypot(ax, ay), math.hypot(bx, by)
            if norm_a < 1e-6 or norm_b < 1e-6:
                continue
            cos = (ax * bx + ay * by) / (norm_a * norm_b)
            max_turn = max(max_turn, math.degrees(math.acos(max(-1.0, min(1.0, cos)))))
        self.assertLessEqual(max_turn, 22.0, "the small fillet must be swept, not pivoted")

    def test_waypoint_is_swept_smoothly(self):
        # waypoints are swept like drawn corners: the fillet keeps the path
        # within ~waypoint_cut_mm of the point (no knife-point pivot) while
        # the run continues smoothly through it
        """Waypoint is swept smoothly."""
        elements = [
            stroke([[500.0, 500.0], [900.0, 500.0]]),
            {"kind": "waypoint", "x": 1200.0, "y": 300.0},
            stroke([[1500.0, 900.0], [1800.0, 900.0]]),
        ]
        options = PlanOptions(goto_step_mm=250.0, avoid_obstacles=False)
        result = plan(elements, (500.0, 500.0, 0.0), None, [], options, 3000.0, 2000.0)
        # the sweep may be driven as a native ARC: measure the executed
        # polyline (nodes plus arc samples)
        nearest = min(math.dist(point, (1200.0, 300.0)) for point in program_points(result))
        self.assertLess(nearest, 70.0, "the sweep must stay close to the waypoint")
        self.assertGreater(nearest, 1.0, "the corner must be swept, not knifed through")
        # and it continues to the stroke that follows the waypoint (the tail
        # may be driven as a native arc, so check the program end point)
        last = result["segments"][-1]
        end = (last["x"], last["y"]) if last["kind"] == "goto" else (last["x1"], last["y1"])
        self.assertAlmostEqual(end[0], 1800.0, delta=60.0)

    def test_waypoint_cut_zero_is_exact(self):
        """Waypoint cut zero is exact."""
        elements = [
            stroke([[500.0, 500.0], [900.0, 500.0]]),
            {"kind": "waypoint", "x": 1200.0, "y": 300.0},
            stroke([[1500.0, 900.0], [1800.0, 900.0]]),
        ]
        options = PlanOptions(goto_step_mm=250.0, avoid_obstacles=False, waypoint_cut_mm=0.0)
        result = plan(elements, (500.0, 500.0, 0.0), None, [], options, 3000.0, 2000.0)
        nearest = min(math.dist(p, (1200.0, 300.0)) for p in program_points(result))
        self.assertLess(nearest, 0.5, "cut 0 must cross the waypoint exactly")

    def test_waypoint_inside_the_border_clearance_is_clamped_inside(self):
        # a waypoint hugging the border (x=46, y=41) cannot be reached without
        # the body leaving the table: it is CLAMPED to the closest legal
        # centre point (half-diagonal inset) and the path sweeps through that
        # clamped point - never an out-and-back stub to an unreachable spot
        """Waypoint inside the border clearance is clamped inside."""
        elements = [
            {"kind": "waypoint", "x": 46.0, "y": 811.0},
            {"kind": "waypoint", "x": 1506.0, "y": 41.0},
        ]
        options = PlanOptions(goto_step_mm=250.0)
        result = plan(elements, (309.0, 1887.0, -91.0), None, [], options, 3000.0, 2000.0)
        gotos = goto_segments(result)
        driven = program_points(result)
        self.assertGreaterEqual(len(driven), 8)
        self.assertLessEqual(len(gotos), 60, "nodes must stay economical")
        # the whole run stays on points the border clearance accepts (a corner
        # may be driven as one smooth ARC, so check the executed polyline)
        for node in driven:
            self.assertGreaterEqual(node[0], 199.0, f"node {node} entered the border clearance")
            self.assertGreaterEqual(node[1], 199.0, f"node {node} entered the border clearance")
        # the waypoint was moved to the closest legal centre point and then
        # really swept through (within the pass-through cut): it is respected,
        # not silently skipped
        inset = math.hypot(300.0, 300.0) * 0.5  # robot half-diagonal
        nearest = min(math.dist(node, (inset, 811.0)) for node in driven)
        self.assertLess(nearest, 70.0, "the clamped waypoint must be swept through")
        self.assertTrue(any("clamped inside the table" in w for w in result["warnings"]))
        # no hairpin, no 3 mm crawl, no dramatic slowdown (measured on the
        # executed polyline: a corner may be driven as one smooth ARC and a
        # junction CONNECTOR is a few mm by nature - it is blended, so it
        # costs no stop)
        points = program_points(result)
        turns = []
        for prev, node, nxt in zip(points, points[1:], points[2:]):
            h1 = math.atan2(node[1] - prev[1], node[0] - prev[0])
            h2 = math.atan2(nxt[1] - node[1], nxt[0] - node[0])
            delta = math.degrees(abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1))))
            turns.append(delta)
        self.assertLessEqual(max(turns), 25.0, "a hairpin survived")
        steps = [math.dist(a, b) for a, b in zip(points, points[1:])]
        self.assertGreaterEqual(min(steps), 4.0, "micro-step crawl")
        # no dramatic slowdown: the slowest move is a junction CONNECTOR or a
        # tight corner (the lateral-acceleration cap), not a crawl
        speeds = [
            segment["v"]
            for segment in result["segments"]
            if segment["kind"] in ("goto", "arc")
        ]
        self.assertGreaterEqual(min(speeds), 100.0, "speed collapsed")

    def test_manual_steps_follow_the_heading_chain(self):
        # advance 300, turn +45.6, advance 200 - the second advance goes along
        # the ROTATED heading, exactly like two Drive-pad moves back to back
        """Manual advance/turn steps chain by heading."""
        elements = [
            {"kind": "drive", "distance_mm": 300.0},
            {"kind": "rotate", "angle_deg": 45.6},
            {"kind": "drive", "distance_mm": 200.0},
        ]
        options = PlanOptions(goto_step_mm=250.0)
        result = plan(elements, (315.0, 1800.0, -90.0), None, [], options, 3000.0, 2000.0)
        self.assertIn("turn", [segment["kind"] for segment in result["segments"]])
        last = result["segments"][-1]
        self.assertEqual(last["kind"], "goto")
        expected_x = 315.0 + math.cos(math.radians(-44.4)) * 200.0
        expected_y = 1500.0 + math.sin(math.radians(-44.4)) * 200.0
        self.assertAlmostEqual(last["x"], expected_x, delta=2.0)
        self.assertAlmostEqual(last["y"], expected_y, delta=2.0)

    def test_drawn_path_is_clamped_inside_the_table(self):
        # a path poking out of the table must be pulled back so the body
        # (half-diagonal 212 mm, ANY heading) never leaves it - also without
        # the obstacle router (avoid_obstacles off)
        """Drawn geometry outside the table is clamped inside."""
        options = PlanOptions(goto_step_mm=250.0, avoid_obstacles=False)
        result = plan(
            [{"kind": "stroke", "points": [[100.0, 80.0], [2900.0, 1920.0]]}],
            (315.0, 1800.0, -90.0), None, [], options, 3000.0, 2000.0,
        )
        inset = math.hypot(300.0, 300.0) * 0.5
        for x, y in program_points(result):
            self.assertGreaterEqual(x, inset - 1.0, f"({x}, {y}) left the table")
            self.assertGreaterEqual(y, inset - 1.0, f"({x}, {y}) left the table")
            self.assertLessEqual(x, 3000.0 - inset + 1.0, f"({x}, {y}) left the table")
            self.assertLessEqual(y, 2000.0 - inset + 1.0, f"({x}, {y}) left the table")
        self.assertTrue(any("clamped inside the table" in w for w in result["warnings"]))

    def test_combine_updates_the_preview_path_too(self):
        """The map preview follows the woven approach, not the old straight line.

        Ticking "combine with the approach" changed the executed segments but
        the plan's preview path kept the straight-through approach - on the map
        the option looked like a no-op (2026-10-01). The path must now be
        rebuilt from the final segments.
        """
        base = {"kind": "stroke", "points": [[315.0, 1800.0], [1200.0, 1200.0]]}
        waypoint = {"kind": "waypoint", "x": 1300.0, "y": 1100.0, "theta_deg": 0.0, "combine": True}
        options = PlanOptions(goto_step_mm=250.0)
        result = plan([base, waypoint], (315.0, 1800.0, -90.0), None, [], options, 3000.0, 2000.0)
        # geometry of the FINAL segments (goto nodes + arc chords)
        geometry = []
        for segment in result["segments"]:
            if segment["kind"] == "goto":
                geometry.append((segment["x"], segment["y"]))
            elif segment["kind"] == "arc":
                geometry.extend((p[0], p[1]) for p in segment.get("points") or [])
        # walk back ~60 mm from the path end: that point must hug the woven
        # ARC (the stale straight-line preview sat ~15-25 mm off it)
        walk = [result["path"][-1]]
        for point in reversed(result["path"][:-1]):
            if math.dist(walk[-1], point) >= 60.0:
                walk.append(point)
                break
            walk.append(point)
        probe = walk[-1]
        best = min(math.dist(probe, node) for node in geometry)
        self.assertLess(best, 4.0, f"preview point {probe} is off the woven approach ({best:.1f} mm)")

    def test_final_waypoint_stops_exactly_with_its_angle(self):
        # the LAST waypoint carrying theta: stop exactly on it facing theta;
        # with combine the arrival turn is woven into the approach as an ARC
        """Final waypoint stops exactly, with or without a woven turn."""
        base = {"kind": "stroke", "points": [[315.0, 1800.0], [1200.0, 1200.0]]}
        waypoint = {"kind": "waypoint", "x": 1300.0, "y": 1100.0, "theta_deg": 0.0, "combine": True}
        options = PlanOptions(goto_step_mm=250.0)
        result = plan([base, waypoint], (315.0, 1800.0, -90.0), None, [], options, 3000.0, 2000.0)
        last = result["segments"][-1]
        self.assertEqual(last["kind"], "arc")
        self.assertAlmostEqual(last["x1"], 1300.0, delta=1.0)
        self.assertAlmostEqual(last["y1"], 1100.0, delta=1.0)
        exit_deg = (last["th0_deg"] + last["sweep_deg"] + 180.0) % 360.0 - 180.0
        self.assertAlmostEqual(exit_deg, 0.0, delta=1.0)
        # combine off: a plain GOTO lands on the point carrying the heading -
        # the board turns in place there (its GOTO final turn)
        result = plan(
            [dict(base), {**waypoint, "combine": False}],
            (315.0, 1800.0, -90.0), None, [], options, 3000.0, 2000.0,
        )
        last = result["segments"][-1]
        self.assertEqual(last["kind"], "goto")
        self.assertAlmostEqual(last["x"], 1300.0, delta=1.0)
        self.assertAlmostEqual(last["y"], 1100.0, delta=1.0)
        self.assertAlmostEqual(last["th"], 0.0, delta=1.0)

    def test_large_arrival_turn_is_still_woven(self):
        # a real mission: the path arrives heading ~-169 deg and the waypoint
        # asks for 90 deg (delta ~ -100 deg) - combine must still produce the
        # joining ARC ending ON the waypoint, facing the wished angle
        """Large arrival turns are woven as one arc."""
        elements = [
            {"kind": "stroke", "points": [[2000.0, 500.0], [900.0, 1050.0]]},
            {"kind": "waypoint", "x": 600.0, "y": 989.0, "theta_deg": 90.0, "combine": True},
        ]
        options = PlanOptions(goto_step_mm=250.0)
        result = plan(elements, (315.0, 1800.0, -90.0), None, [], options, 3000.0, 2000.0)
        last = result["segments"][-1]
        self.assertEqual(last["kind"], "arc", result["warnings"])
        self.assertAlmostEqual(last["x1"], 600.0, delta=1.0)
        self.assertAlmostEqual(last["y1"], 989.0, delta=1.0)
        exit_deg = (last["th0_deg"] + last["sweep_deg"] + 180.0) % 360.0 - 180.0
        self.assertAlmostEqual(exit_deg, 90.0, delta=1.0)
        # geometry pin: the arc's circle must pass through BOTH its start and
        # the waypoint - the board drives exactly this arc, so a mock/real run
        # then lands on the waypoint (a wrong chord made it miss before)
        arc_radius = abs(last["radius_mm"])
        centre = (
            last["x0"] + last["radius_mm"] * -math.sin(math.radians(last["th0_deg"])),
            last["y0"] + last["radius_mm"] * math.cos(math.radians(last["th0_deg"])),
        )
        self.assertAlmostEqual(math.dist(centre, (last["x0"], last["y0"])), arc_radius, delta=1.0)
        self.assertAlmostEqual(math.dist(centre, (last["x1"], last["y1"])), arc_radius, delta=1.0)

    def test_border_waypoint_never_leaves_the_table(self):
        # a final waypoint near the border with a big arrival angle: the arc
        # swing is CHECKED point-wise - when it would leave the table the
        # weave is refused and the robot turns in place (always inside, the
        # point is clamped), and every driven point keeps the pad
        """Near-border arrival stays on the table, woven or not."""
        elements = [
            {"kind": "stroke", "points": [[1500.0, 1200.0], [500.0, 420.0]]},
            {"kind": "waypoint", "x": 250.0, "y": 260.0, "theta_deg": 135.0, "combine": True},
        ]
        options = PlanOptions(goto_step_mm=250.0)
        result = plan(elements, (315.0, 1800.0, -90.0), None, [], options, 3000.0, 2000.0)
        margin = math.hypot(300.0, 300.0) * 0.5 + 12.0
        for x, y in program_points(result):
            self.assertGreaterEqual(x, margin - 1.0, f"({x}, {y}) left the table")
            self.assertGreaterEqual(y, margin - 1.0, f"({x}, {y}) left the table")
        last = result["segments"][-1]
        if last["kind"] == "goto":
            self.assertAlmostEqual(last["th"], 135.0, delta=1.0)
        end_x = last.get("x1", last.get("x"))
        end_y = last.get("y1", last.get("y"))
        self.assertAlmostEqual(end_x, 250.0, delta=1.0)
        self.assertAlmostEqual(end_y, 260.0, delta=1.0)

    def test_arc_approach_arrival_never_leaves_a_gap(self):
        # the approach into the waypoint is a fitted ARC chain (a curved
        # stroke): the arrival weave used to rebuild the tail around an ARC
        # base and left a ~300 mm HOLE - the last arc started where the robot
        # had never been, so the board (it builds every ARC from the CURRENT
        # pose) drove it from the wrong circle and the mission "completed"
        # 300-450 mm short of the waypoint (real-robot bug, 2026-10-01).
        # The chain must stay continuous and still land on the waypoint.
        """Arc-tail arrivals stay continuous and land on the waypoint."""
        # a curved stroke: quarter circle around (1500, 1000), radius 400,
        # ending at (1500, 600) - the waypoint sits on its end
        points = []
        for step in range(25):
            phi = math.radians(180.0 + 90.0 * step / 24.0)
            points.append([1500.0 + 400.0 * math.cos(phi), 1000.0 + 400.0 * math.sin(phi)])
        elements = [
            stroke(points),
            {"kind": "waypoint", "x": 1500.0, "y": 600.0, "theta_deg": -25.0, "combine": True},
        ]
        options = PlanOptions(goto_step_mm=250.0)
        result = plan(elements, (315.0, 1800.0, -90.0), None, [], options, 3000.0, 2000.0)
        cursor = None
        for index, segment in enumerate(result["segments"]):
            if segment["kind"] == "goto":
                cursor = (segment["x"], segment["y"])
            elif segment["kind"] == "arc":
                if cursor is not None:
                    gap = math.dist(cursor, (segment["x0"], segment["y0"]))
                    self.assertLess(gap, 3.0, f"arc {index} starts {gap:.0f} mm from the previous exit")
                cursor = (segment["x1"], segment["y1"])
        last = [s for s in result["segments"] if s["kind"] in ("goto", "arc")][-1]
        if last["kind"] == "arc":
            end = (last["x1"], last["y1"])
            exit_deg = (last["th0_deg"] + last["sweep_deg"] + 180.0) % 360.0 - 180.0
        else:
            end = (last["x"], last["y"])
            exit_deg = last["th"]
        self.assertLess(math.dist(end, (1500.0, 600.0)), 2.0, "the plan must land on the waypoint")
        self.assertAlmostEqual(exit_deg, -25.0, delta=1.0)
        self.assertFalse(
            [w for w in result["warnings"] if "continuity" in w], result["warnings"]
        )

    def test_robot_geometry_sets_obstacle_clearance(self):
        # obstacle 200x400 at (1400,800); robot 300x300 + 150 margin ->
        # inflation = half-diagonal (212.1) + 150 = 362.1 mm, auto (inflate 0).
        # The body clearance (212.1) is inviolable; corner fillets may use up
        # to half of the margin (soft boundary = 212.1 + 75 mm)
        """Robot geometry sets obstacle clearance."""
        options = PlanOptions(
            goto_step_mm=200.0, grid_mm=50.0, inflate_mm=0.0,
            robot_width_mm=300.0, robot_height_mm=300.0, obstacle_margin_mm=150.0,
        )
        result = plan(
            [stroke([[200.0, 1000.0], [2800.0, 1000.0]])],
            (400.0, 1000.0, 0.0), None, [OBSTACLE], options, 3000.0, 2000.0,
        )
        body = 212.1
        for segment in goto_segments(result):
            inside = (
                1400.0 - body <= segment["x"] <= 1600.0 + body
                and 800.0 - body <= segment["y"] <= 1200.0 + body
            )
            self.assertFalse(inside, f"node inside the robot body clearance: {segment}")

    def test_detour_corners_stay_rounded(self):
        # regression: a corner the fillet cannot fully take (it would clip the
        # safety zone) used to cancel rounding for the WHOLE run - the start
        # junction and the obstacle detour stayed visibly sharp. Rounding is
        # now per corner (smaller radius fallback, then keep only that vertex)
        """Detour corners stay rounded."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        obstacles = [{"kind": "rect", "x": 1400.0, "y": 950.0, "w": 200.0, "h": 100.0}]
        result = plan(
            [stroke([[800.0, 1000.0], [2200.0, 1000.0]])],
            (600.0, 1000.0, 0.0), None, obstacles, options, 3000.0, 2000.0,
        )
        path = result["path"]
        max_turn = 0.0
        for i in range(1, len(path) - 1):
            a, b, c = path[i - 1], path[i], path[i + 1]
            h1 = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            h2 = math.degrees(math.atan2(c[1] - b[1], c[0] - b[0]))
            max_turn = max(max_turn, abs((h2 - h1 + 540.0) % 360.0 - 180.0))
        self.assertLess(max_turn, 25.0, f"sharp corner left in the detour (max turn {max_turn:.0f} deg)")
        # ...while the body clearance is still respected
        half_diagonal = math.hypot(300.0, 300.0) / 2.0 - 1.0
        for x, y in path:
            dx = max(1400.0 - x, x - 1600.0, 0.0)
            dy = max(950.0 - y, y - 1050.0, 0.0)
            self.assertGreaterEqual(math.hypot(dx, dy), half_diagonal)

    def test_start_junction_is_rounded(self):
        # the connector joins the stroke at an angle (robot placed inside the
        # border zone): that junction is the first "sharp thing" - it must be
        # swept by a fillet, not nicked
        """Start junction is rounded."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        result = plan(
            [stroke([[1000.0, 900.0], [1900.0, 900.0]])],
            (500.0, 100.0, 0.0), None, [], options, 3000.0, 2000.0,
        )
        path = result["path"]
        max_turn = 0.0
        for i in range(1, len(path) - 1):
            a, b, c = path[i - 1], path[i], path[i + 1]
            h1 = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            h2 = math.degrees(math.atan2(c[1] - b[1], c[0] - b[0]))
            max_turn = max(max_turn, abs((h2 - h1 + 540.0) % 360.0 - 180.0))
        self.assertLess(max_turn, 25.0, f"start junction left sharp (max turn {max_turn:.0f} deg)")

    def test_border_keeps_robot_inside(self):
        # a stroke drawn in the border zone (the robot body would leave the
        # field): the planner cannot route there, keeps the drawing and says
        # clearly why - and it still produces a usable plan
        """Border keeps robot inside."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        result = plan(
            [stroke([[100.0, 1000.0], [300.0, 1000.0]]), stroke([[300.0, 1000.0], [1200.0, 1000.0]])],
            (500.0, 1000.0, 0.0), None, [], options, 3000.0, 2000.0,
        )
        self.assertTrue(
            any("clearance zone" in w or "re-routed" in w for w in result["warnings"]),
            result["warnings"],
        )
        self.assertTrue(goto_segments(result))

    def test_rough_v_angle_gets_a_wide_smooth_turn(self):
        # two hand-drawn *point sets* roughly forming two long straight lines
        # that meet at a sharp angle (as drawn on the field): the plan must
        # sweep through a wide fillet instead of nicking the vertex.
        """Rough v angle gets a wide smooth turn."""
        def rough_line(a, b, n=18, wobble=4.0):
            """Straight polyline with small jitter, as a hand-drawn stroke."""
            return [
                (
                    a[0] + (b[0] - a[0]) * i / (n - 1) + math.sin(i * 2.1) * wobble,
                    a[1] + (b[1] - a[1]) * i / (n - 1) + math.cos(i * 1.7) * wobble,
                )
                for i in range(n)
            ]

        vertex = (672.0, 207.0)
        leg1 = rough_line((852.0, 734.0), vertex)
        leg2 = rough_line(vertex, (2451.0, 832.0))
        elements = [stroke([list(p) for p in leg1]), stroke([list(p) for p in leg2])]
        options = PlanOptions(goto_step_mm=250.0, corner_radius_mm=200.0)
        result = plan(elements, leg1[0], None, [], options, 3000.0, 2000.0)
        # the wide sweep may be driven as a native ARC: measure on the
        # executed polyline (nodes plus arc samples)
        points = program_points(result)
        nearest = min(math.dist(point, vertex) for point in points)
        self.assertGreater(nearest, 60.0, "the turn must swing wide of the vertex")
        self.assertLess(nearest, 400.0)
        legs = [(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:])]
        max_turn = 0.0
        for (ax, ay), (bx, by) in zip(legs, legs[1:]):
            norm_a, norm_b = math.hypot(ax, ay), math.hypot(bx, by)
            if norm_a < 15.0 or norm_b < 15.0:
                continue
            cos = (ax * bx + ay * by) / (norm_a * norm_b)
            max_turn = max(max_turn, math.degrees(math.acos(max(-1.0, min(1.0, cos)))))
        self.assertLess(max_turn, 40.0, "no leg pair may turn abruptly at the V")


    def test_blocked_run_takes_the_shortest_detour(self):
        # an obstacle sitting ON the drawn line: the detour must keep the FULL
        # obstacle margin (the configured safety buffer) and still be the
        # shortest legal route around it - not a loop across the field
        """Blocked run takes the shortest detour at the full margin."""
        options = PlanOptions(goto_step_mm=250.0, smooth=False)
        result = plan(
            [stroke([[500.0, 500.0], [2500.0, 500.0]])],
            (500.0, 500.0, 0.0), None,
            [{"kind": "rect", "x": 1400.0, "y": 400.0, "w": 200.0, "h": 200.0}],
            options, 3000.0, 2000.0,
        )
        length = result["stats"]["length_mm"]
        self.assertGreater(length, 2000.0)
        self.assertLess(length, 2350.0, "the detour must be the shortest legal one")
        x0, y0, x1, y1 = 1400.0, 400.0, 1600.0, 600.0
        gap = min(
            math.hypot(max(x0 - px, 0.0, px - x1), max(y0 - py, 0.0, py - y1))
            for px, py in result["path"]
        )
        full_margin = math.hypot(300.0, 300.0) / 2.0 + 150.0
        self.assertGreaterEqual(gap, full_margin - 2.0, "the detour cut into the obstacle margin")
        self.assertLess(gap, full_margin + 60.0, "it must hug the margin, not loop wide")
        self.assertTrue(any("re-routed to stay clear" in w for w in result["warnings"]), result["warnings"])

    def test_fresh_opponent_pose_keeps_the_route_clear(self):
        """Fresh opponent pose is inflated by both robot footprints."""
        ctx = SimpleNamespace(
            vision=VisionStore(),
            opponent_pose=lambda: {"x_mm": 1500.0, "y_mm": 500.0, "theta_deg": 0.0},
        )
        options = PlanOptions.from_dict({"smooth": False})
        obstacles = active_obstacles(ctx, [], {"smooth": False})
        result = plan(
            [stroke([[500.0, 500.0], [2500.0, 500.0]])],
            (500.0, 500.0, 0.0), None, obstacles, options, 3000.0, 2000.0,
        )
        nearest = min(math.dist((x, y), (1500.0, 500.0)) for x, y in result["path"])
        self.assertGreater(nearest, 430.0, "both robot footprints must remain separated")

    def test_far_obstacle_leaves_the_drawn_line_untouched(self):
        """Far obstacle leaves the drawn line untouched."""
        options = PlanOptions(goto_step_mm=250.0, smooth=False)
        result = plan(
            [stroke([[500.0, 500.0], [2500.0, 500.0]])],
            (500.0, 500.0, 0.0), None,
            [{"kind": "rect", "x": 2200.0, "y": 1400.0, "w": 200.0, "h": 200.0}],
            options, 3000.0, 2000.0,
        )
        self.assertAlmostEqual(result["stats"]["length_mm"], 2000.0, delta=5.0)
        self.assertFalse(any("re-routed" in w for w in result["warnings"]), result["warnings"])

    def test_dead_end_approach_gets_corner_cut(self):
        # regression (dynamic obstacle session): the drawn route climbed a
        # wall corridor whose top turn was blocked - the re-planned route
        # drove UP the corridor, U-turned and came back ("the robot did a
        # return"). The detour must START EARLIER instead: skip the dead-end
        # walk-in entirely when that is genuinely shorter.
        """Dead end approach gets corner cut."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        drawn = [[350.0, 1500.0], [300.0, 1300.0], [240.0, 1000.0], [225.0, 840.0],
                 [235.0, 700.0], [265.0, 555.0], [340.0, 520.0], [600.0, 505.0],
                 [1000.0, 420.0], [1200.0, 280.0], [1600.0, 240.0]]
        obstacle = {"kind": "rect", "x": 470.0, "y": 370.0, "w": 300.0, "h": 300.0}
        result = plan([stroke(drawn)], (350.0, 1500.0, -90.0), None, [obstacle], options, 3000.0, 2000.0)
        self.assertTrue(any("corner cut" in w for w in result["warnings"]), result["warnings"])
        # the dead-end pocket (west of the obstacle, north of y=700) is gone
        pocket = [(x, y) for x, y in result["path"] if x < 430.0 and y < 700.0]
        self.assertEqual(pocket, [], f"the plan still drives into the dead end: {pocket}")
        # and no reversal either: the route leaves the corridor smoothly
        max_turn = 0.0
        path = result["path"]
        for i in range(2, len(path)):
            a, b, c = path[i - 2], path[i - 1], path[i]
            h1 = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            h2 = math.degrees(math.atan2(c[1] - b[1], c[0] - b[0]))
            max_turn = max(max_turn, abs((h2 - h1 + 540.0) % 360.0 - 180.0))
        self.assertLess(max_turn, 40.0, f"the path U-turns (max turn {max_turn:.0f} deg)")
        # the corner cut really saves the walk-in: shorter than the free plan
        # that drives the whole drawn route, and it still ends on the drawing
        free = plan([stroke(drawn)], (350.0, 1500.0, -90.0), None, [], options, 3000.0, 2000.0)
        self.assertLess(result["stats"]["length_mm"], free["stats"]["length_mm"] - 200.0)
        self.assertAlmostEqual(result["path"][-1][0], 1600.0, delta=5.0)
        self.assertAlmostEqual(result["path"][-1][1], 240.0, delta=5.0)

    def test_waypoint_arrival_angle_keeps_the_continuation_aligned(self):
        # regression (the "robot went completely to the right" session): the
        # board builds an ARC's circle from the CURRENT heading, so after a
        # waypoint arrival angle the continuation arc used to start on a
        # wrong circle and the robot left the path entirely. The program must
        # hold the chain "perform the angle, turn toward the next point,
        # continue on the arc's tangent".
        """Waypoint arrival angle keeps the continuation aligned."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        elements = [
            stroke([[600.0, 900.0], [1000.0, 900.0], [1400.0, 900.0]]),
            {"kind": "waypoint", "x": 1400.0, "y": 900.0, "theta_deg": -90.0},
            stroke([[1400.0, 900.0], [1340.0, 800.0], [1230.0, 740.0], [1120.0, 720.0]]),
        ]
        result = plan(elements, (600.0, 900.0, 0.0), None, [], options, 3000.0, 2000.0)
        # walk the program like the board drives it: every ARC must be
        # entered on its tangent - from a turn, a goto's final heading or
        # the previous arc's exit
        heading = 0.0
        for index, segment in enumerate(result["segments"]):
            if segment["kind"] == "goto":
                heading = segment["th"]
            elif segment["kind"] == "turn":
                heading = g.deg_norm180(heading + segment["angle_deg"])
            elif segment["kind"] == "arc":
                gap = g.deg_norm180(segment["th0_deg"] - heading)
                self.assertLess(
                    abs(gap), 1.05,
                    f"segment {index}: the arc starts {gap:.1f} deg off its tangent",
                )
                heading = g.deg_norm180(segment["th0_deg"] + segment["sweep_deg"])
        # the arrival node faces -90 and the alignment turn right after it
        # hands the robot over to the arc's entry tangent
        arrival = next(
            index for index, segment in enumerate(result["segments"])
            if segment["kind"] == "goto"
            and math.dist((segment["x"], segment["y"]), (1400.0, 900.0)) < 2.0
        )
        self.assertAlmostEqual(result["segments"][arrival]["th"], -90.0, delta=0.1)
        self.assertEqual(result["segments"][arrival + 1]["kind"], "turn")
        needed = g.deg_norm180(result["segments"][arrival + 2]["th0_deg"] + 90.0)
        self.assertAlmostEqual(result["segments"][arrival + 1]["angle_deg"], needed, delta=0.2)
        self.assertLess(math.dist(result["path"][-1], (1120.0, 720.0)), 5.0)

    def test_arrival_angle_is_performed_before_the_following_pause(self):
        # the mission order the user asked for: reach the waypoint, perform
        # the commanded angle, THEN wait, and only then turn away toward the
        # next stroke. The alignment turn used to be inserted right at the
        # waypoint, so the robot waited already facing the NEXT arc instead of
        # the commanded angle (2026-10-09).
        """Arrive -> face the commanded angle -> wait -> continue."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        elements = [
            stroke([[600.0, 900.0], [1400.0, 900.0]]),
            {"kind": "waypoint", "x": 1400.0, "y": 900.0, "theta_deg": -90.0},
            {"kind": "pause", "seconds": 2.0},
            stroke([[1400.0, 900.0], [1340.0, 800.0], [1120.0, 720.0]]),
        ]
        result = plan(elements, (600.0, 900.0, 0.0), None, [], options, 3000.0, 2000.0)
        segments = result["segments"]
        anchor = _arrival_anchor(segments, (1400.0, 900.0))
        self.assertIsNotNone(anchor)
        index, kind = anchor
        self.assertEqual(kind, "goto")
        self.assertAlmostEqual(segments[index]["th"], -90.0, delta=0.1)
        self.assertEqual(segments[index + 1]["kind"], "pause")
        # the alignment turn comes AFTER the wait and hands over to the arc
        self.assertEqual(segments[index + 2]["kind"], "turn")
        needed = g.deg_norm180(segments[index + 3]["th0_deg"] + 90.0)
        self.assertAlmostEqual(segments[index + 2]["angle_deg"], needed, delta=0.2)
        self.assertEqual(segments[index + 3]["kind"], "arc")

    def test_arrival_angle_is_performed_when_the_point_is_an_arc_exit(self):
        # the real 2026-10-09 plan: the fit landed the drawn path ON the
        # waypoint as the EXIT of a near-straight ARC, so no GOTO node carried
        # the angle and the command was dropped silently. A pivot must be
        # inserted right there - before the pause, so the wait happens facing
        # the commanded angle - and the continuation still needs its tangent
        # alignment afterwards.
        """Arc-exit arrivals perform the commanded angle."""
        segments = [
            {
                "kind": "arc", "x0": 750.0, "y0": 500.0, "x1": 1200.0, "y1": 500.0,
                "radius_mm": 28648.0, "sweep_deg": 0.9, "th0_deg": 0.0, "v": 150,
                "points": [[750.0, 500.0], [1200.0, 500.0]],
            },
            {"kind": "pause", "seconds": 2.0},
            {
                "kind": "arc", "x0": 1200.0, "y0": 500.0, "x1": 1600.0, "y1": 900.0,
                "radius_mm": 900.0, "sweep_deg": 45.0, "th0_deg": 45.0, "v": 150,
                "points": [[1200.0, 500.0], [1600.0, 900.0]],
            },
        ]
        point = (1200.0, 500.0)
        self.assertEqual(_arrival_anchor(segments, point), (0, "arc"))
        self.assertTrue(_set_arrival_heading(segments, point, -25.0, 120))
        self.assertEqual([s["kind"] for s in segments], ["arc", "turn", "pause", "arc"])
        self.assertAlmostEqual(segments[1]["angle_deg"], -25.9, delta=0.1)
        self.assertTrue(segments[1].get("explicit"))
        _align_arc_after_arrival(segments, point, -25.0, 120)
        self.assertEqual([s["kind"] for s in segments], ["arc", "turn", "pause", "turn", "arc"])
        self.assertAlmostEqual(segments[3]["angle_deg"], 70.0, delta=0.1)

    def test_combine_weaves_the_angle_through_a_straight_approach(self):
        # "combine with the approach": a straight approach is emitted as flat
        # ARCs and the weave used to refuse EVERY arc approach, so the tick
        # did nothing on the most common shape at all (2026-10-09). The
        # approach arc is re-aimed to end on the weave base and the arrival
        # ARC lands ON the waypoint facing the commanded angle.
        """combine weaves the arrival angle into a flat-ARC approach."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        elements = [
            stroke([[600.0, 900.0], [1400.0, 900.0]]),
            {"kind": "waypoint", "x": 1400.0, "y": 900.0, "theta_deg": -30.0, "combine": True},
            {"kind": "pause", "seconds": 2.0},
            stroke([[1400.0, 900.0], [1300.0, 700.0], [1000.0, 600.0]]),
        ]
        result = plan(elements, (600.0, 900.0, 0.0), None, [], options, 3000.0, 2000.0)
        self.assertFalse(
            [w for w in result["warnings"] if "woven" in w], result["warnings"]
        )
        segments = result["segments"]
        anchor = _arrival_anchor(segments, (1400.0, 900.0))
        self.assertIsNotNone(anchor)
        index, kind = anchor
        self.assertEqual(kind, "arc")
        landing = segments[index]
        self.assertAlmostEqual(landing["x1"], 1400.0, delta=0.5)
        self.assertAlmostEqual(landing["y1"], 900.0, delta=0.5)
        exit_deg = g.deg_norm180(landing["th0_deg"] + landing["sweep_deg"])
        self.assertAlmostEqual(exit_deg, -30.0, delta=1.0)
        # the wait happens facing the commanded angle: no pivot before it
        self.assertEqual(segments[index + 1]["kind"], "pause")

    def test_the_commanded_angle_is_always_performed_at_the_waypoint(self):
        # whatever the approach shape (straight / fitted curve) and whatever
        # "combine with the approach" says, the robot must HOLD the commanded
        # angle on the waypoint - before the first pause after it. The curved
        # approach cannot be re-aimed (that would cut the drawn curve): the
        # weave then refuses with a warning and the in-place turn is used.
        """Every approach shape holds the commanded angle on the waypoint."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        curved = []
        for step in range(25):
            phi = math.radians(180.0 + 90.0 * step / 24.0)
            curved.append([1100.0 + 400.0 * math.cos(phi), 1000.0 + 400.0 * math.sin(phi)])
        shapes = [
            (
                [
                    stroke([[600.0, 900.0], [1400.0, 900.0]]),
                    {"kind": "pause", "seconds": 2.0},
                    stroke([[1400.0, 900.0], [1300.0, 700.0], [1000.0, 600.0]]),
                ],
                1400.0,
                900.0,
            ),
            (
                [
                    stroke(curved),
                    {"kind": "pause", "seconds": 1.0},
                ],
                1100.0,
                600.0,
            ),
        ]
        for combine in (False, True):
            for tail, wx, wy in shapes:
                elements = [tail[0], None, *tail[1:]]
                elements[1] = {
                    "kind": "waypoint", "x": wx, "y": wy, "theta_deg": -25.0, "combine": combine,
                }
                result = plan(elements, (600.0, 900.0, 0.0), None, [], options, 3000.0, 2000.0)
                segments = result["segments"]
                anchor = _arrival_anchor(segments, (wx, wy))
                self.assertIsNotNone(anchor, (combine, wx, wy, result["warnings"]))
                index, kind = anchor
                if kind == "goto":
                    held = float(segments[index]["th"])
                else:
                    held = g.deg_norm180(segments[index]["th0_deg"] + segments[index]["sweep_deg"])
                if index + 1 < len(segments) and segments[index + 1].get("arrival"):
                    held = g.deg_norm180(held + segments[index + 1]["angle_deg"])
                self.assertAlmostEqual(
                    held, -25.0, delta=1.2,
                    msg=f"combine={combine} at ({wx}, {wy}): {result['warnings']}",
                )


class ArrivalSafetyTests(unittest.TestCase):
    """Combined arrivals respect the game area, the obstacle margin and the
    shortest-path rule (2026-10-18 field session)."""

    def test_wall_waypoint_is_moved_out_and_the_angle_is_performed(self):
        # the reported session: robot mid-field, waypoint 213 mm above a
        # 502 mm tall wall facing the wall (-90 deg), "combine" ticked. The
        # old plan stopped at BODY clearance (775 mm, 61 mm of air) and the
        # commanded angle was never performed because the point was not a
        # stop node. Now the point is moved to the FULL clearance boundary,
        # the arrival arc is legal and the robot really faces -90 there.
        """Wall waypoint: safe stop point + performed arrival angle."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        wall = {"kind": "rect", "x": 0.0, "y": 0.0, "w": 3000.0, "h": 502.0}
        elements = [{"kind": "waypoint", "x": 2394.0, "y": 613.0,
                     "theta_deg": -90.0, "combine": True}]
        result = plan(elements, (1521.0, 1110.0, 0.0), None, [wall], options, 3000.0, 2000.0)
        self.assertTrue(
            any("moved to the closest allowed point" in warning for warning in result["warnings"]),
            result["warnings"],
        )
        full_margin = math.hypot(300.0, 300.0) / 2.0 + 150.0
        legal = math.hypot(300.0, 300.0) / 2.0 + 12.0
        for x, y in program_points(result):
            self.assertGreaterEqual(y, legal - 1.0, f"({x:.0f}, {y:.0f}) left the table")
            self.assertGreaterEqual(y, 502.0 + full_margin - 2.0,
                                    f"({x:.0f}, {y:.0f}) entered the wall clearance")
            self.assertGreaterEqual(x, legal - 1.0)
            self.assertLessEqual(x, 3000.0 - legal + 1.0)
        last = result["segments"][-1]
        if last["kind"] == "arc":
            held = g.deg_norm180(last["th0_deg"] + last["sweep_deg"])
        else:
            held = float(last["th"])
        self.assertAlmostEqual(held, -90.0, delta=1.5, msg=result["warnings"])
        self.assertAlmostEqual(last.get("x1", last.get("x")), 2394.0, delta=1.5)
        self.assertGreaterEqual(last.get("y1", last.get("y")), 502.0 + full_margin - 2.0)

    def test_combined_arrival_never_leaves_the_field(self):
        # a waypoint hugging a corner with a heading that swings the arrival
        # arc outwards: every candidate arc is checked, so the plan either
        # weaves a legal one or turns in place - never a sweep outside
        """Corner arrival: never outside the legal region."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        elements = [
            {"kind": "waypoint", "x": 350.0, "y": 350.0, "theta_deg": 135.0, "combine": True},
        ]
        result = plan(elements, (1500.0, 1200.0, 180.0), None, [], options, 3000.0, 2000.0)
        legal = math.hypot(300.0, 300.0) / 2.0 + 12.0
        for x, y in program_points(result):
            self.assertGreaterEqual(x, legal - 1.0, f"({x:.0f}, {y:.0f}) left the table")
            self.assertGreaterEqual(y, legal - 1.0, f"({x:.0f}, {y:.0f}) left the table")
            self.assertLessEqual(x, 3000.0 - legal + 1.0)
            self.assertLessEqual(y, 2000.0 - legal + 1.0)
        last = result["segments"][-1]
        held = (g.deg_norm180(last["th0_deg"] + last["sweep_deg"])
                if last["kind"] == "arc" else float(last["th"]))
        self.assertAlmostEqual(held, 135.0, delta=1.5, msg=result["warnings"])

    def test_combined_arrival_takes_the_short_way(self):
        # "combine" must not turn the trip into a loop: a straight 1 m run
        # plus a 90 deg arrival arc stays close to the direct distance
        """A combined arrival stays near the direct route."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        elements = [
            {"kind": "waypoint", "x": 1500.0, "y": 1000.0, "theta_deg": 90.0, "combine": True},
        ]
        result = plan(elements, (500.0, 1000.0, 0.0), None, [], options, 3000.0, 2000.0)
        self.assertLess(
            result["stats"]["length_mm"], 1250.0,
            f"the combined arrival detoured ({result['stats']['length_mm']:.0f} mm)",
        )
        last = result["segments"][-1]
        held = (g.deg_norm180(last["th0_deg"] + last["sweep_deg"])
                if last["kind"] == "arc" else float(last["th"]))
        self.assertAlmostEqual(held, 90.0, delta=1.5, msg=result["warnings"])
        self.assertFalse(
            [warning for warning in result["warnings"] if "outside" in warning],
            result["warnings"],
        )

    def test_waypoint_inside_a_box_is_moved_out(self):
        # the drawn box is an obstacle, not a suggestion: a waypoint dropped
        # inside its clearance is moved to the closest fully-clear point
        """A waypoint inside an obstacle clearance is moved out."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        box = {"kind": "rect", "x": 1100.0, "y": 600.0, "w": 200.0, "h": 200.0}
        result = plan(
            [{"kind": "waypoint", "x": 1200.0, "y": 700.0, "theta_deg": 0.0, "combine": True}],
            (500.0, 500.0, 0.0), None, [box], options, 3000.0, 2000.0,
        )
        self.assertTrue(
            any("moved to the closest allowed point" in warning for warning in result["warnings"]),
            result["warnings"],
        )
        full_margin = math.hypot(300.0, 300.0) / 2.0 + 150.0
        for x, y in program_points(result):
            dx = max(1100.0 - x, x - 1300.0, 0.0)
            dy = max(600.0 - y, y - 800.0, 0.0)
            self.assertGreaterEqual(math.hypot(dx, dy), full_margin - 2.0,
                                    f"({x:.0f}, {y:.0f}) entered the box clearance")
        # the console/cursor needs the effective point, not the original one
        self.assertEqual(len(result["resolved_waypoints"]), 1)
        resolved = result["resolved_waypoints"][0]
        self.assertLess(math.dist(resolved, result["path"][-1]), 2.0)

    def test_resolved_waypoints_rewrite_only_the_waypoint_elements(self):
        """The stored re-plan request follows the effective waypoint points."""
        """Rewrite waypoints, leave the rest untouched."""
        elements = [
            {"kind": "stroke", "points": [[0.0, 0.0], [10.0, 10.0]]},
            {"kind": "waypoint", "x": 1.0, "y": 2.0, "theta_deg": 0.0},
            {"kind": "pause", "seconds": 1.0},
            {"kind": "waypoint", "x": 3.0, "y": 4.0},
        ]
        out = _apply_resolved_waypoints(elements, [[10.0, 20.0], [30.0, 40.0]])
        self.assertIs(out[0], elements[0])
        self.assertEqual((out[1]["x"], out[1]["y"]), (10.0, 20.0))
        self.assertEqual(out[1]["theta_deg"], 0.0)
        self.assertIs(out[2], elements[2])
        self.assertEqual((out[3]["x"], out[3]["y"]), (30.0, 40.0))
        # no resolved list / fewer entries: the elements are kept as they are
        self.assertEqual(_apply_resolved_waypoints(elements, []), elements)
        self.assertEqual(len(_apply_resolved_waypoints(elements, [[1.0, 2.0]])), 4)


class CloseWaypointTests(unittest.TestCase):
    """Configured waypoint stop clearance, straight final legs and rotation.

    A waypoint may be placed much closer to an obstacle/border than the
    routing margin. Such a point is reached by a straight final leg; an
    arrival angle is performed at a safe stand-off BEFORE that leg, because
    turning in place on the close point would sweep a body corner into the
    obstacle (reported field case, 2026-10-19).
    """

    WALL = {"kind": "rect", "x": 0.0, "y": 0.0, "w": 3000.0, "h": 502.0}
    HALF_BODY = 150.0
    TURN_CLEARANCE = math.hypot(300.0, 300.0) / 2.0 + 6.0

    @staticmethod
    def options(margin):
        """Planner options with a configured waypoint stop clearance."""
        return PlanOptions(
            robot_width_mm=300.0, robot_height_mm=300.0,
            obstacle_margin_mm=150.0, waypoint_margin_mm=margin,
            goto_step_mm=250.0,
        )

    def test_clearance_scales_from_half_the_body(self):
        """0 mm means half the body side; the configured value adds to it."""
        self.assertAlmostEqual(
            waypoint_clearance(self.options(0.0)), self.HALF_BODY, delta=0.1,
        )
        self.assertAlmostEqual(
            waypoint_clearance(self.options(50.0)), self.HALF_BODY + 50.0, delta=0.1,
        )
        # omitting the option keeps the historical full routing clearance
        legacy = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0,
                             obstacle_margin_mm=150.0)
        self.assertAlmostEqual(
            waypoint_clearance(legacy),
            math.hypot(300.0, 300.0) / 2.0 + 150.0, delta=0.1,
        )

    def test_close_waypoint_is_reached_by_a_straight_final_leg(self):
        """A waypoint 150 mm from the wall is not pushed to the routing margin."""
        waypoint = {"kind": "waypoint", "x": 2394.0, "y": 652.0,
                    "theta_deg": -90.0, "combine": True}
        result = plan(
            [waypoint], (1521.0, 1110.0, 0.0), None, [self.WALL],
            self.options(0.0), 3000.0, 2000.0,
        )
        resolved = result["resolved_waypoints"][0]
        self.assertLessEqual(abs(resolved[1] - 652.0), 3.0,
                             f"the point was pushed to the routing margin: {resolved}")
        self.assertFalse(
            [w for w in result["warnings"]
             if "sits inside the robot clearance zone" in w],
            result["warnings"],
        )
        self.assertTrue(
            any("approached straight from a safe stand-off" in w for w in result["warnings"]),
            result["warnings"],
        )
        motion = [s for s in result["segments"] if s["kind"] in ("goto", "arc")]
        last = motion[-1]
        self.assertEqual(last["kind"], "goto")
        self.assertAlmostEqual(last["x"], 2394.0, delta=1.0)
        self.assertAlmostEqual(last["y"], resolved[1], delta=1.0)
        self.assertAlmostEqual(last["th"], -90.0, delta=0.5)
        # the body really clears the wall on the final leg
        self.assertTrue(
            body_clear_at(
                (last["x"], last["y"]), last["th"], [self.WALL],
                300.0, 300.0, 3000.0, 2000.0,
            ),
            "the final stop touches the wall",
        )
        # the rotation happened at a stand-off that is turn-safe
        before = motion[-2]
        self.assertAlmostEqual(before["x"], 2394.0, delta=2.0)
        self.assertGreaterEqual(
            before["y"] - 502.0, self.TURN_CLEARANCE - 2.0,
            f"the pre-rotation point is too close to the wall: {before}",
        )
        self.assertAlmostEqual(before["th"], -90.0, delta=1.0)

    def test_margin_keeps_the_waypoint_proportionally_farther(self):
        """A configured margin is added to the half-body floor."""
        waypoint = {"kind": "waypoint", "x": 2394.0, "y": 652.0,
                    "theta_deg": -90.0, "combine": True}
        close = plan([dict(waypoint)], (1521.0, 1110.0, 0.0), None, [self.WALL],
                     self.options(0.0), 3000.0, 2000.0)
        far = plan([dict(waypoint)], (1521.0, 1110.0, 0.0), None, [self.WALL],
                   self.options(50.0), 3000.0, 2000.0)
        close_y = close["resolved_waypoints"][0][1]
        far_y = far["resolved_waypoints"][0][1]
        self.assertGreater(far_y, close_y + 40.0)
        self.assertLess(abs(far_y - 702.0), 3.0)

    def test_parallel_angle_moves_the_point_to_a_turn_safe_spot(self):
        """A final heading parallel to the wall cannot be pre-rotated there.

        The straight leg would keep the same wall distance; safety wins: the
        stop is moved to the closest turning-safe point so the angle can be
        performed (and the mission can continue) without touching.
        """
        waypoint = {"kind": "waypoint", "x": 2394.0, "y": 652.0,
                    "theta_deg": 0.0, "combine": True}
        result = plan(
            [waypoint], (1000.0, 1000.0, 0.0), None, [self.WALL],
            self.options(0.0), 3000.0, 2000.0,
        )
        resolved = result["resolved_waypoints"][0]
        self.assertGreaterEqual(
            resolved[1] - 502.0, self.TURN_CLEARANCE - 2.0,
            f"the stop stayed too close to rotate: {resolved}",
        )
        self.assertTrue(
            any("turning-safe point" in w for w in result["warnings"]),
            result["warnings"],
        )
        last = [s for s in result["segments"] if s["kind"] in ("goto", "arc")][-1]
        end = (last["x"], last["y"]) if last["kind"] == "goto" else (last["x1"], last["y1"])
        self.assertLess(math.dist(end, resolved), 2.0, "the moved stop was not reached")
        self.assertAlmostEqual(
            last.get("th", last.get("th0_deg", 0.0) + last.get("sweep_deg", 0.0)),
            0.0, delta=1.0,
        )

    def test_close_waypoint_near_the_border_is_allowed_and_safe(self):
        """The same rule applies to the game-area border."""
        waypoint = {"kind": "waypoint", "x": 150.0, "y": 1000.0,
                    "theta_deg": 180.0, "combine": True}
        result = plan(
            [waypoint], (1000.0, 1000.0, 0.0), None, [],
            self.options(0.0), 3000.0, 2000.0,
        )
        resolved = result["resolved_waypoints"][0]
        self.assertLessEqual(abs(resolved[0] - 150.0), 3.0,
                             f"the border waypoint was pushed inward: {resolved}")
        last = [s for s in result["segments"] if s["kind"] in ("goto", "arc")][-1]
        end = (last["x"], last["y"]) if last["kind"] == "goto" else (last["x1"], last["y1"])
        self.assertLess(math.dist(end, resolved), 2.0)
        self.assertTrue(
            body_clear_at(
                end, last.get("th", last.get("th0_deg", 0.0) + last.get("sweep_deg", 0.0)),
                [], 300.0, 300.0, 3000.0, 2000.0,
            ),
            "the final stop leaves the body over the field border",
        )

    def test_close_waypoint_with_following_motion_backs_up_first(self):
        """A later stroke keeps the exact stop, then backs out before turning."""
        elements = [
            {"kind": "waypoint", "x": 2394.0, "y": 652.0,
             "theta_deg": -90.0, "combine": True},
            stroke([[2394.0, 900.0], [2000.0, 1200.0]]),
        ]
        result = plan(
            elements, (1521.0, 1110.0, 0.0), None, [self.WALL],
            self.options(0.0), 3000.0, 2000.0,
        )
        resolved = result["resolved_waypoints"][0]
        self.assertLessEqual(
            abs(resolved[1] - 652.0), 3.0,
            f"the stop must stay exact, not move to a turn-safe point: {resolved}",
        )
        # the stop node is followed by a REVERSE drive before any turn
        stop_index = next(
            index for index, segment in enumerate(result["segments"])
            if segment["kind"] == "goto"
            and math.dist((segment["x"], segment["y"]), resolved) <= 2.0
        )
        departure = result["segments"][stop_index + 1]
        self.assertEqual(departure["kind"], "drive", result["segments"])
        self.assertTrue(departure.get("departure"))
        self.assertLess(departure["distance_mm"], -100.0)
        start, end = departure["points"]
        self.assertLess(math.dist(start, resolved), 2.0)
        self.assertGreaterEqual(
            end[1] - 502.0, self.TURN_CLEARANCE - 2.0,
            "the reverse must end at a point where the body can rotate",
        )
        self.assertTrue(
            any("reached exactly" in w and "backs up" in w for w in result["warnings"]),
            result["warnings"],
        )
        # the continuation geometry really starts after the reverse
        self.assertTrue(
            any(
                segment.get("kind") in ("goto", "arc")
                and math.dist(
                    (segment.get("x", segment.get("x0", 0.0)),
                     segment.get("y", segment.get("y0", 0.0))),
                    end,
                ) <= 60.0
                for segment in result["segments"][stop_index + 2:]
            ),
            result["segments"],
        )

    def test_border_waypoint_is_reached_exactly_then_backed_up(self):
        """The reported map-limit case: exact stop, reverse, then continue."""
        band = {"kind": "rect", "x": 13.8, "y": 0.0,
                "w": 2986.2, "h": 499.9}
        elements = [
            {"kind": "waypoint", "x": 2850.0, "y": 1052.0,
             "theta_deg": 0.0, "combine": True},
            stroke([[2850.0, 1052.0], [2682.1, 1145.8],
                    [2672.7, 1155.2], [2660.1, 1167.7],
                    [2647.6, 1180.3], [2635.0, 1189.7]]),
        ]
        result = plan(
            elements, (2400.0, 1400.0, 0.0), None, [band],
            self.options(0.0), 3000.0, 2000.0,
        )
        resolved = result["resolved_waypoints"][0]
        self.assertAlmostEqual(resolved[0], 2850.0, delta=1.5)
        stop_index = next(
            index for index, segment in enumerate(result["segments"])
            if segment["kind"] == "goto"
            and math.dist((segment["x"], segment["y"]), resolved) <= 2.0
        )
        self.assertAlmostEqual(result["segments"][stop_index]["th"], 0.0, delta=0.5)
        departure = result["segments"][stop_index + 1]
        self.assertEqual(departure["kind"], "drive")
        self.assertLess(departure["distance_mm"], 0.0)
        _, end = departure["points"]
        # the reverse ends where the body can turn without crossing the border
        self.assertLessEqual(end[0], 3000.0 - self.TURN_CLEARANCE + 2.0)
        self.assertTrue(
            any("reached exactly" in w and "backs up" in w for w in result["warnings"]),
            result["warnings"],
        )
        # the continuation is joined where it is nearest the reverse exit,
        # not back at the tight waypoint
        following = result["segments"][stop_index + 2]
        first = (
            (following["x"], following["y"])
            if following["kind"] == "goto"
            else (following["x0"], following["y0"])
        )
        self.assertLess(first[0], resolved[0] - 10.0, result["segments"])
        # the reverse-direction vertex must not be mistaken for an in-place
        # rotation: the mid-run reaction checks must keep this plan running
        options = self.options(0.0)
        self.assertFalse(
            _clips_obstacle(result["path"], [band], options, 3000.0, 2000.0),
            "the legal exact-stop / reverse departure was flagged as a clip",
        )
        gap, clip = clip_gap_mm(
            result["path"], [band],
            {
                "robot_width_mm": 300.0, "robot_height_mm": 300.0,
                "obstacle_margin_mm": 150.0, "waypoint_margin_mm": 0.0,
                "goto_step_mm": 250.0,
            },
            3000.0, 2000.0,
        )
        self.assertEqual(gap, math.inf, f"false clip in the reaction ladder at {clip}")

    def test_shallow_border_curve_is_not_an_in_place_rotation(self):
        """A sampled curve inside the 12 mm pad still fits the body."""
        band = {"kind": "rect", "x": 0.0, "y": 0.0, "w": 3000.0, "h": 631.0}
        path = [
            (1489.8, 1774.3),
            (1426.3, 1786.2),
            (1361.7, 1788.2),
            (1297.6, 1780.2),
            (1235.5, 1762.4),
        ]
        self.assertFalse(
            _clips_obstacle(path, [band], self.options(0.0), 3000.0, 2000.0),
            "a moving turn near the pad was treated as an in-place rotation",
        )

    def test_departure_turn_happens_after_the_reverse(self):
        """An explicit rotation after a close waypoint runs after the back-up."""
        elements = [
            {"kind": "waypoint", "x": 2394.0, "y": 652.0,
             "theta_deg": -90.0, "combine": True},
            {"kind": "rotate", "angle_deg": 90.0},
        ]
        result = plan(
            elements, (1521.0, 1110.0, 0.0), None, [self.WALL],
            self.options(0.0), 3000.0, 2000.0,
        )
        kinds = [segment["kind"] for segment in result["segments"]]
        drive_index = next(
            index for index, segment in enumerate(result["segments"])
            if segment["kind"] == "drive"
        )
        turn_index = next(
            index for index, segment in enumerate(result["segments"])
            if segment["kind"] == "turn" and index > drive_index
        )
        self.assertEqual(kinds[drive_index], "drive")
        self.assertGreater(turn_index, drive_index)


class CloseWaypointReactionTests(unittest.TestCase):
    """Mid-run obstacle checks must not mistake a close approach for a clip.

    Live regression (2026-10-19): a waypoint 151 mm above a static wall was
    reached by a straight final leg. The camera re-plan ladder measured the
    centre against the half-diagonal circle, thought an obstacle was 400 mm
    ahead and stopped the robot; the replan then rejected its own legal path.
    """

    WALL = {"kind": "rect", "x": 0.0, "y": 0.0, "w": 2921.9, "h": 436.0}
    OPTIONS = PlanOptions(
        robot_width_mm=300.0, robot_height_mm=300.0,
        obstacle_margin_mm=150.0, waypoint_margin_mm=0.0,
    )

    def test_close_final_leg_is_not_a_clip(self):
        """A body aligned with the final leg clears the wall."""
        path = [(933.0, 802.0), (933.0, 587.0)]
        self.assertFalse(
            _clips_obstacle(path, [self.WALL], self.OPTIONS, 3000.0, 2000.0),
            "the legal close final leg was flagged as an obstacle clip",
        )
        gap, point = clip_gap_mm(
            path, [self.WALL], {
                "robot_width_mm": 300.0, "robot_height_mm": 300.0,
                "obstacle_margin_mm": 150.0, "waypoint_margin_mm": 0.0,
            }, 3000.0, 2000.0,
        )
        self.assertEqual(gap, math.inf, f"the reaction ladder saw a false clip at {point}")
        self.assertFalse(
            route_blocked(path, [self.WALL], self.OPTIONS, 3000.0, 2000.0),
            "the re-plan gate rejected the legal close approach",
        )
        self.assertFalse(
            _waypoint_blocked(
                (933.0, 587.0), [self.WALL],
                {"robot_width_mm": 300.0, "robot_height_mm": 300.0,
                 "obstacle_margin_mm": 150.0, "waypoint_margin_mm": 0.0},
            ),
            "the configured stop clearance itself was treated as blocked",
        )

    def test_exact_clearance_with_a_rounded_path_is_not_a_clip(self):
        """Sub-millimetre path rounding at an exact stop is not a crash.

        Live regression (2026-10-19): a top wall 510.4276 mm deep and a
        waypoint 150 mm below it. The path is serialised to 0.1 mm, so the
        waypoint reads back at 660.4 (0.028 mm inside the allowed stop) and
        the camera re-plan rejected its own legal route.
        """
        wall = {"kind": "rect", "x": 0.0, "y": 0.0,
                "w": 3000.0, "h": 510.4276320672195}
        path = [(798.0, 875.4), (798.0, 660.4)]
        self.assertFalse(
            _clips_obstacle(path, [wall], self.OPTIONS, 3000.0, 2000.0),
            "sub-millimetre path rounding was treated as a body clip",
        )

    def test_cusp_collapse_handles_a_departure_drive(self):
        """A DRIVE before a cusp run must not raise KeyError 'x1'.

        Live regression (2026-10-19): a vision re-plan near the close-waypoint
        departure built a GOTO cusp run right after the signed DRIVE and the
        cusp pass assumed every predecessor had an x1 field.
        """
        segments = [
            {"kind": "drive", "distance_mm": -100.0,
             "points": [[-10.0, 0.0], [0.0, 0.0]]},
            {"kind": "goto", "x": 10.0, "y": 0.0, "th": 0.0},
            {"kind": "goto", "x": 20.0, "y": 0.0, "th": 0.0},
            {"kind": "goto", "x": 20.0, "y": 10.0, "th": 90.0},
            {"kind": "goto", "x": 20.0, "y": 20.0, "th": 90.0},
        ]
        changed = _collapse_cusps(
            segments, PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0),
            lambda points: True,
        )
        self.assertTrue(changed)
        self.assertEqual(segments[0]["kind"], "drive")
        self.assertEqual((segments[1]["x"], segments[1]["y"]), (20.0, 20.0))

    def test_real_body_clip_is_still_caught(self):
        """The same check still sees the body entering the wall."""
        path = [(933.0, 802.0), (933.0, 500.0)]
        self.assertTrue(
            _clips_obstacle(path, [self.WALL], self.OPTIONS, 3000.0, 2000.0),
        )
        gap, _ = clip_gap_mm(
            path, [self.WALL], {
                "robot_width_mm": 300.0, "robot_height_mm": 300.0,
                "obstacle_margin_mm": 150.0, "waypoint_margin_mm": 0.0,
            }, 3000.0, 2000.0,
        )
        self.assertLess(gap, math.inf)
        self.assertTrue(
            route_blocked(path, [self.WALL], self.OPTIONS, 3000.0, 2000.0),
        )
        self.assertTrue(
            _waypoint_blocked(
                (933.0, 500.0), [self.WALL],
                {"robot_width_mm": 300.0, "robot_height_mm": 300.0,
                 "obstacle_margin_mm": 150.0, "waypoint_margin_mm": 0.0},
            ),
        )

    def test_mid_run_assessment_keeps_the_close_route(self):
        """A camera update must leave the legal close approach unchanged."""
        elements = [{"kind": "waypoint", "x": 933.0, "y": 587.0,
                     "theta_deg": -90.0, "combine": True}]
        options = {
            "robot_width_mm": 300.0, "robot_height_mm": 300.0,
            "obstacle_margin_mm": 150.0, "waypoint_margin_mm": 0.0,
            "goto_step_mm": 250.0,
        }
        first = plan(
            elements, (315.0, 1800.0, -90.0), None, [self.WALL],
            PlanOptions.from_dict(options), 3000.0, 2000.0,
        )

        class RemainingExecutor:
            """Only what the mission still has to drive."""

            def remaining_elements(self):
                """The remainder is the same waypoint."""
                return list(elements)

        context = SimpleNamespace(
            pose=lambda: (700.0, 1000.0, -60.0),
            last_plan=first,
            last_plan_request={"elements": elements, "options": options, "cursor": 0},
            field={"width_mm": 3000.0, "height_mm": 2000.0},
            skipped_waypoints=set(),
        )
        affected, candidate, _ = asyncio.run(
            _assess_remaining_route(
                context, RemainingExecutor(), elements, None, [self.WALL],
                options, "route", cursor=0, planned_elements=elements,
            )
        )
        self.assertFalse(
            affected,
            "a camera update must not stop/re-plan a legal close-waypoint approach",
        )
        self.assertIsNotNone(candidate)


class DynamicObstacleWaitTests(unittest.IsolatedAsyncioTestCase):
    """A moving obstacle on the route must never leave a permanent stop."""

    async def test_clipped_replan_waits_for_the_moving_obstacle(self):
        """A failed re-plan stores a wait so the next camera change retries."""
        options = {
            "speed_mm_s": 200.0, "goto_step_mm": 250.0,
            "robot_width_mm": 300.0, "robot_height_mm": 300.0,
            "obstacle_margin_mm": 150.0, "waypoint_margin_mm": 0.0,
        }
        elements = [{"kind": "stroke", "points": [[650.0, 500.0], [1500.0, 500.0]]}]
        static = [{"kind": "rect", "x": 0.0, "y": 0.0, "w": 3000.0, "h": 300.0}]
        dynamic = [{"kind": "rect", "x": 600.0, "y": 400.0, "w": 300.0, "h": 300.0}]
        initial = {
            "path": [[650.0, 500.0], [1500.0, 500.0]],
            "segments": [{"kind": "goto", "x": 1500.0, "y": 500.0}],
            "stats": {"length_mm": 850.0, "n_segments": 1, "n_gotos": 1,
                      "n_arcs": 0, "n_actions": 0, "est_s": 1.0},
            "warnings": [], "resolved_waypoints": [],
        }

        class Executor:
            state = EXEC_RUNNING
            speed_scale = 1.0
            last_stream = False
            started = False

            def is_running(self):
                """Still running when the camera report arrives."""
                return True

            def remaining_departure_mm(self, position):
                """No departure move in this mission."""
                return 0.0

            def remaining_elements(self, position=None):
                """The remainder of the drawn stroke."""
                return [{"kind": "stroke", "points": [[650.0, 500.0], [1500.0, 500.0]]}]

            async def stop(self, reason=""):
                """Stop the simulated run."""

            async def wait_idle(self):
                """Already idle."""

            async def start(self, segments, deadman_ms=None, paused=False, **kwargs):
                """Remember that a restart was attempted."""
                self.started = True

            @property
            def progress(self):
                """Executor progress snapshot."""
                return {"message": ""}

        executor = Executor()
        logs = []

        async def log(level, message):
            """Capture launch logs."""
            logs.append((level, message))

        async def emit(message):
            """Discard plan broadcasts."""

        context = SimpleNamespace(
            executor=executor,
            pose=lambda: (650.0, 500.0, 0.0),
            last_plan=initial,
            last_plan_request={
                "elements": elements, "options": options, "obstacles": static,
                "goal": None, "cursor": 0, "signature": elements_signature(elements),
            },
            field={"width_mm": 3000.0, "height_mm": 2000.0},
            skipped_waypoints=set(),
            vision=SimpleNamespace(active=lambda: list(dynamic)),
            opponent_pose=lambda: None,
            log=log, emit=emit, plan_lock=asyncio.Lock(),
            vision_waiting=None, wait_deadline_task=None,
        )

        async def force_affected(*args, **kwargs):
            """The moving obstacle changed the route."""
            return (True, initial, 1000.0)

        async def clipping_candidate(**kwargs):
            """No legal detour: every candidate still touches the opponent."""
            return {
                "path": [[650.0, 500.0], [900.0, 500.0]],
                "segments": [{"kind": "goto", "x": 900.0, "y": 500.0}],
                "stats": {"length_mm": 250.0, "n_segments": 1, "n_gotos": 1,
                          "n_arcs": 0, "n_actions": 0, "est_s": 1.0},
                "warnings": [], "resolved_waypoints": [],
            }

        with patch(
            "software.core.mission.orchestrator._assess_remaining_route",
            force_affected,
        ), patch(
            "software.core.mission.orchestrator._plan_off_loop",
            clipping_candidate,
        ):
            result = await execute_plan(context, elements, None, static, options, 0)

        self.assertTrue(result.get("waiting"))
        self.assertIsNotNone(context.vision_waiting)
        self.assertIn("moving obstacle", context.vision_waiting["reason"])
        self.assertFalse(executor.started, "a clipping route must not restart the robot")
        if context.wait_deadline_task is not None:
            context.wait_deadline_task.cancel()


class StartWaypointTests(unittest.TestCase):
    """The first waypoint is usually the robot's own start pose."""

    def test_waypoint_at_the_start_is_not_a_zero_length_stop(self):
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        elements = [
            {"kind": "waypoint", "x": 2775.867965644036, "y": 1012.0,
             "theta_deg": 0.0, "combine": True},
            {"kind": "waypoint", "x": 1500.0, "y": 1000.0,
             "theta_deg": -90.0, "combine": True},
        ]
        result = plan(elements, (2775.9, 1012.0, 0.0), None, [], options, 3000.0, 2000.0)
        self.assertFalse(
            [w for w in result["warnings"] if "not a stop node" in w or "could not be woven" in w],
            result["warnings"],
        )
        first = result["segments"][0]
        self.assertEqual(first["kind"], "goto")
        self.assertGreater(
            math.hypot(first["x"] - 2775.9, first["y"] - 1012.0), 50.0,
            "the start waypoint must not emit a zero-length GOTO at the robot pose",
        )
        self.assertEqual(result["stats"]["n_gotos"], 1, result["segments"])
        self.assertEqual(len(result["resolved_waypoints"]), 2)

    def test_obstacle_geometry_is_clamped_to_the_field(self):
        clamped = _clamp_obstacles(
            [
                {"kind": "rect", "x": -100.0, "y": -50.0, "w": 500.0, "h": 200.0},
                {"kind": "circle", "x": 50.0, "y": 1950.0, "r": 400.0},
            ],
            3000.0,
            2000.0,
        )
        rect, circle = clamped
        self.assertEqual((rect["x"], rect["y"], rect["w"], rect["h"]), (0.0, 0.0, 400.0, 150.0))
        self.assertEqual((circle["x"], circle["y"]), (400.0, 1600.0))
        self.assertEqual(circle["r"], 400.0)


class SliceRemainingTests(unittest.TestCase):
    """Mid-run re-plan slicing: the mission (drawing) is the source of truth."""

    def test_mid_stroke_slice_keeps_remainder_and_later_elements(self):
        """Mid stroke slice keeps remainder and later elements."""
        elements = [
            stroke([[500.0, 500.0], [1500.0, 500.0]]),
            {"kind": "rotate", "angle_deg": 90.0},
            {"kind": "pause", "seconds": 2.0},
            stroke([[1500.0, 500.0], [1500.0, 1500.0]]),
        ]
        sliced = slice_remaining_elements(elements, (900.0, 505.0), progress=0.25)
        self.assertEqual([entry["kind"] for entry in sliced], ["stroke", "rotate", "pause", "stroke"])
        first = sliced[0]["points"]
        self.assertAlmostEqual(first[0][0], 900.0, delta=6.0)  # cut at the robot
        self.assertAlmostEqual(first[0][1], 500.0, delta=6.0)
        self.assertGreaterEqual(len(first), 2, "the remainder of the current stroke")
        for point in first:  # the already-driven part is gone
            self.assertGreaterEqual(point[0], 894.0)
        self.assertEqual(sliced[3]["points"][-1], [1500.0, 1500.0])  # later stroke verbatim

    def test_later_stroke_drops_completed_ones_and_their_actions(self):
        """Later stroke drops completed ones and their actions."""
        elements = [
            stroke([[500.0, 500.0], [1500.0, 500.0]]),
            {"kind": "rotate", "angle_deg": -45.0},
            stroke([[1500.0, 500.0], [1500.0, 1500.0]]),
        ]
        sliced = slice_remaining_elements(elements, (1500.0, 900.0), progress=0.75)
        self.assertEqual([entry["kind"] for entry in sliced], ["stroke"])
        self.assertAlmostEqual(sliced[0]["points"][0][1], 900.0, delta=6.0)
        self.assertEqual(sliced[0]["points"][-1], [1500.0, 1500.0])

    def test_no_strokes_slices_to_nothing(self):
        """No strokes slices to nothing."""
        self.assertEqual(slice_remaining_elements([{"kind": "pause", "seconds": 1.0}], (100.0, 100.0)), [])

    def test_path_fraction(self):
        """Path fraction."""
        path = [[0.0, 0.0], [100.0, 0.0], [100.0, 100.0]]
        self.assertAlmostEqual(path_fraction(path, (50.0, 5.0)), 50.0 / 200.0, delta=0.02)
        self.assertAlmostEqual(path_fraction(path, (105.0, 150.0)), 1.0, delta=0.02)
        self.assertAlmostEqual(path_fraction(path, (-20.0, 0.0)), 0.0, delta=0.02)


class ContinueRouteTests(unittest.TestCase):
    """Mid-run route continuation: one clean stroke, never a backwards leg."""

    def test_on_drawing_continue_keeps_the_mission_structure(self):
        """On drawing continue keeps the mission structure."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        elements = [
            stroke([[500.0, 500.0], [1500.0, 500.0]]),
            {"kind": "rotate", "angle_deg": 90.0},
            stroke([[1500.0, 500.0], [1500.0, 1500.0]]),
        ]
        base = plan(elements, (500.0, 500.0, 0.0), None, [], options, 3000.0, 2000.0)
        continued = continue_route_elements(elements, base["path"], (900.0, 503.0), 0.45)
        self.assertEqual([entry["kind"] for entry in continued], ["stroke", "rotate", "stroke"])
        first = continued[0]["points"][0]
        self.assertLess(math.hypot(first[0] - 900.0, first[1] - 503.0), 10.0,
                        "the continuation must start at the robot")
        self.assertEqual(continued[2]["points"][-1], [1500.0, 1500.0])  # later stroke verbatim

    def test_mid_detour_continue_starts_at_the_robot_and_rejoins(self):
        """Mid detour continue starts at the robot and rejoins."""
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        elements = [stroke([[500.0, 500.0], [2500.0, 500.0]])]
        obstacle = {"kind": "rect", "x": 1400.0, "y": 400.0, "w": 200.0, "h": 200.0}
        detour = plan(elements, (500.0, 500.0, 0.0), None, [obstacle], options, 3000.0, 2000.0)
        path = detour["path"]
        apex = max(path, key=lambda point: point[1])  # deepest point of the detour
        continued = continue_route_elements(elements, path, (apex[0], apex[1]), 0.5)
        self.assertTrue(continued and continued[0].get("points"), "nothing to continue")
        first = continued[0]["points"][0]
        self.assertLess(math.hypot(first[0] - apex[0], first[1] - apex[1]), 6.0,
                        "the route must start at the robot")
        xs = [point[0] for point in continued[0]["points"]]
        self.assertGreater(min(xs), apex[0] - 6.0, "the continuation must never go backwards")
        replanned = plan(continued, (apex[0], apex[1], 0.0), None, [obstacle], options, 3000.0, 2000.0)
        end = replanned["path"][-1]
        self.assertLess(math.hypot(end[0] - 2500.0, end[1] - 500.0), 5.0,
                        "the continuation still ends on the mission end")
        for px, py in replanned["path"]:
            gap = math.hypot(max(1400.0 - px, 0.0, px - 1600.0), max(400.0 - py, 0.0, py - 600.0))
            self.assertGreater(gap, 200.0, f"path point ({px:.0f}, {py:.0f}) too close to the obstacle")

    def test_obstacle_replan_slices_drawing_without_old_detour(self):
        """Replanning starts at the current stroke and keeps later mission steps."""
        elements = [
            stroke([[500.0, 1000.0], [1500.0, 1000.0]]),
            {"kind": "waypoint", "x": 1600.0, "y": 1000.0},
            stroke([[1600.0, 1000.0], [2500.0, 1000.0]]),
        ]
        continued = replan_route_elements(elements, 0, (900.0, 1005.0), 0.4)
        self.assertEqual([element["kind"] for element in continued], ["stroke", "waypoint", "stroke"])
        self.assertLess(math.dist(continued[0]["points"][0], (900.0, 1005.0)), 10.0)
        self.assertEqual(continued[0]["points"][-1], [1500.0, 1000.0])
        self.assertEqual(continued[1], elements[1])
        self.assertEqual(continued[2], elements[2])

    def test_waypoint_only_mission_has_no_drawn_rejoin(self):
        """Waypoint only mission has no drawn rejoin."""
        elements = [{"kind": "waypoint", "x": 1000.0, "y": 500.0}]
        self.assertEqual(continue_route_elements(elements, [[0.0, 0.0], [100.0, 0.0]], (50.0, 0.0)), [])


class MissionCursorTests(unittest.TestCase):
    """The mission cursor: which elements the driven route has REALLY completed.

    Regression for the "waypoints ignored one by one" session: a detour that
    bypassed a waypoint (or a route crossing a LATER stroke) used to drop the
    rest of the mission from every following re-plan.
    """

    def test_waypoint_visited_only_when_really_swept(self):
        """Waypoint visited only when really swept."""
        elements = [
            {"kind": "waypoint", "x": 700.0, "y": 224.0},
            {"kind": "waypoint", "x": 1497.0, "y": 550.0},
        ]
        # the route swept the first waypoint (within 80 mm), not the second
        swept = [(500.0, 500.0), (640.0, 260.0), (900.0, 700.0)]
        self.assertEqual(advance_cursor(elements, 0, swept), 1)
        # a compromise approach (obstacle on the waypoint) does NOT visit it
        short = [(500.0, 500.0), (700.0, 350.0)]
        self.assertEqual(advance_cursor(elements, 0, short), 0)
        # ...and the cursor never moves backwards
        self.assertEqual(advance_cursor(elements, 2, []), 2)

    def test_stroke_done_only_when_its_end_is_reached(self):
        """Stroke done only when its end is reached."""
        elements = [
            {"kind": "stroke", "points": [[500.0, 500.0], [1500.0, 500.0]]},
            {"kind": "pause", "seconds": 2.0},
            {"kind": "waypoint", "x": 1600.0, "y": 500.0},
        ]
        mid = [(500.0, 500.0), (1000.0, 500.0)]
        self.assertEqual(advance_cursor(elements, 0, mid), 0)
        # end reached: the stroke is done, but the pause is only done once
        # the waypoint after it is really visited
        end = [(500.0, 500.0), (1500.0, 500.0)]
        self.assertEqual(advance_cursor(elements, 0, end), 1)
        reached = [(500.0, 500.0), (1500.0, 500.0), (1560.0, 500.0)]
        self.assertEqual(advance_cursor(elements, 0, reached), 3)

    def test_executed_path_prefers_the_earlier_projection(self):
        """Executed path prefers the earlier projection."""
        path = [[0.0, 0.0], [1000.0, 0.0], [1000.0, 500.0], [500.0, 500.0], [500.0, 0.0]]
        relevant = executed_path(path, (500.0, 0.0))  # on the first leg AND its tail
        self.assertEqual(len(relevant), 2)
        self.assertAlmostEqual(relevant[-1][0], 500.0, delta=1.0)
        self.assertAlmostEqual(relevant[-1][1], 0.0, delta=1.0)

    def test_route_remainder_targets_the_next_waypoint_not_the_later_stroke(self):
        """Route remainder targets the next waypoint, never a later stroke."""
        elements = [
            stroke([[500.0, 500.0], [700.0, 500.0]]),
            {"kind": "waypoint", "x": 720.0, "y": 500.0, "theta_deg": 0.0},
            stroke([[2500.0, 560.0], [2400.0, 560.0], [2300.0, 560.0]]),
        ]
        # the route wanders near the LATER stroke - the old slice jumped there
        # and dropped the waypoint; the cursor keeps element 0 (the stroke)
        path = [[520.0, 500.0], [600.0, 900.0], [1200.0, 700.0], [2380.0, 560.0]]
        remainder = route_remainder_elements(elements, 0, path, (520.0, 500.0), 0.4)
        self.assertEqual([entry["kind"] for entry in remainder], ["stroke", "waypoint", "stroke"])
        self.assertAlmostEqual(remainder[0]["points"][0][0], 520.0, delta=8.0)
        self.assertEqual(remainder[2]["points"][0], [2500.0, 560.0])  # later stroke verbatim
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        result = plan(remainder, (520.0, 500.0, 0.0), None, [], options, 3000.0, 2000.0)
        gotos = goto_segments(result)
        nearest = min(math.hypot(s["x"] - 720.0, s["y"] - 500.0) for s in gotos)
        self.assertLess(nearest, 40.0, "the unvisited waypoint must stay the target")

    def test_next_waypoint_is_returned_verbatim(self):
        """Next waypoint is returned verbatim."""
        elements = [
            stroke([[500.0, 500.0], [700.0, 500.0]]),
            {"kind": "pause", "seconds": 2.0},
            {"kind": "waypoint", "x": 700.0, "y": 224.0, "theta_deg": -90.0},
            stroke([[700.0, 224.0], [1400.0, 224.0]]),
        ]
        remainder = route_remainder_elements(elements, 1, [], (0.0, 0.0))
        self.assertEqual([entry["kind"] for entry in remainder], ["pause", "waypoint", "stroke"])
        self.assertEqual(remainder[1]["x"], 700.0)
        self.assertEqual(remainder[1]["theta_deg"], -90.0)

    def test_skipped_waypoint_counts_as_done_and_is_dropped(self):
        """A checkpoint given up after a bounded wait never becomes a target again."""
        elements = [
            {"kind": "waypoint", "x": 800.0, "y": 500.0},
            {"kind": "waypoint", "x": 1800.0, "y": 500.0},
            {"kind": "waypoint", "x": 2500.0, "y": 500.0},
        ]
        skipped = {(1800.0, 500.0)}
        # the driven route swept only the FIRST waypoint; the skipped one must
        # still count as passed so the cursor moves on to the third
        driven = [(500.0, 500.0), (820.0, 500.0)]
        self.assertEqual(advance_cursor(elements, 0, driven, skipped), 2)
        # ...and it must vanish from any remainder built from those elements
        remainder = drop_skipped_waypoints(elements, skipped)
        self.assertEqual([entry["x"] for entry in remainder], [800.0, 2500.0])
        # without skips nothing changes
        self.assertEqual(advance_cursor(elements, 0, driven), 1)
        self.assertEqual(len(drop_skipped_waypoints(elements, [])), 3)


def _planner_arc(start, th0_deg, radius, sweep_deg, speed=200):
    """Build one planner-style ARC segment (sampled like ``_native_arcs``)."""
    heading = math.radians(th0_deg)
    center = (start[0] - radius * math.sin(heading), start[1] + radius * math.cos(heading))
    sweep = math.radians(sweep_deg)
    phi0 = math.atan2(start[1] - center[1], start[0] - center[0])
    steps = max(2, int(abs(sweep_deg) / 10.0) + 1)
    points = [
        (
            center[0] + abs(radius) * math.cos(phi0 + sweep * (index / steps)),
            center[1] + abs(radius) * math.sin(phi0 + sweep * (index / steps)),
        )
        for index in range(steps + 1)
    ]
    return {
        "kind": "arc",
        "x0": round(start[0], 1),
        "y0": round(start[1], 1),
        "x1": round(points[-1][0], 1),
        "y1": round(points[-1][1], 1),
        "radius_mm": radius,
        "sweep_deg": sweep_deg,
        "th0_deg": th0_deg,
        "v": speed,
        "points": [[round(x, 1), round(y, 1)] for x, y in points],
    }


class ArcJunctionRoundingTests(unittest.TestCase):
    """Small tangent breaks between fitted arcs are rounded, not pivoted."""

    def _pair(self, break_deg, radius2=400.0, sweep2=55.0):
        """Two arcs meeting with a deliberate tangent break."""
        first = _planner_arc((1500.0, 500.0), 180.0, -500.0, -40.0)
        second = _planner_arc(
            (first["x1"], first["y1"]),
            first["th0_deg"] + first["sweep_deg"] + break_deg,
            radius2,
            sweep2,
        )
        return first, second

    def _smooth(self, segments):
        """Run the rounding pass with no legality constraints."""
        _smooth_arc_junctions(segments, PlanOptions(), lambda points: True)

    def test_small_tangent_break_becomes_one_tangent_arc_chain(self):
        """Small tangent break becomes one tangent arc chain."""
        first, second = self._pair(20.0)
        segments = [first, {"kind": "turn", "angle_deg": 20.0, "v": 150}, second]
        self._smooth(segments)
        self.assertEqual([s["kind"] for s in segments], ["arc", "arc", "arc"])
        for prev, nxt in zip(segments, segments[1:]):
            gap = abs(g.deg_norm180(nxt["th0_deg"] - (prev["th0_deg"] + prev["sweep_deg"])))
            self.assertLessEqual(gap, 0.2, "rounded junctions must stay tangent")
            self.assertEqual((prev["x1"], prev["y1"]), (nxt["x0"], nxt["y0"]))
            self.assertEqual(prev["radius_mm"] > 0, prev["sweep_deg"] > 0)
        # the chain keeps the original end point
        self.assertEqual((segments[-1]["x1"], segments[-1]["y1"]), (second["x1"], second["y1"]))
        # ...and the corner cut stays local (ARC_CONNECT_MAX_DEV_MM cap). The
        # cut is measured against the neighbour POLYLINES, exactly like the
        # planner does: these test arcs carry one sample per 10 deg, so the
        # distance to the nearest VERTEX over-states it by up to half a chord
        connector = segments[1]
        first_polyline = [(px, py) for px, py in first["points"]]
        second_polyline = [(px, py) for px, py in second["points"]]
        deviation = max(
            min(
                _point_to_polyline((x, y), first_polyline),
                _point_to_polyline((x, y), second_polyline),
            )
            for x, y in connector["points"]
        )
        self.assertLessEqual(deviation, 24.5)

    def test_junction_beside_a_near_straight_arc_is_rounded(self):
        """A junction beside a flat ARC is rounded without moving that ARC's exit.

        A near-straight ARC drives its chord with the radius and the sweep both
        on the wire resolution (0.1 deg of sweep is ~50 mm at R = 28646 mm), so
        a free trim would move its exit away from the declared one (real 6 mm
        jump on 2026-10-07). The connector touches it at its own end point or a
        whole quantum away instead, and the other neighbour carries the trim.
        """
        first = _planner_arc((700.0, 1000.0), 0.0, 500.0, 40.0)
        flat = _planner_arc(
            (first["x1"], first["y1"]),
            first["th0_deg"] + first["sweep_deg"] + 9.0,
            28646.0,
            0.5,
        )
        segments = [first, {"kind": "turn", "angle_deg": 9.0, "v": 150}, flat]
        self._smooth(segments)
        self.assertEqual([s["kind"] for s in segments], ["arc", "arc", "arc"])
        rounded_flat = segments[2]
        self.assertLessEqual(
            math.dist((rounded_flat["x1"], rounded_flat["y1"]), (flat["x1"], flat["y1"])), 0.1
        )
        trimmed_deg = abs(flat["sweep_deg"]) - abs(rounded_flat["sweep_deg"])
        quanta = trimmed_deg / LINE_ARC_SWEEP_QUANT_DEG
        self.assertAlmostEqual(quanta, round(quanta), places=6, msg="whole wire quanta only")
        for prev, nxt in zip(segments, segments[1:]):
            gap = abs(g.deg_norm180(nxt["th0_deg"] - (prev["th0_deg"] + prev["sweep_deg"])))
            self.assertLessEqual(gap, 0.2, "the rounded chain must stay tangent")

    def test_junction_beside_a_flat_arc_is_rounded_by_trimming_it(self):
        """No whole-arc connector exists -> the flat ARC is trimmed instead.

        Real junction of the loaded strategy (2026-10-09): a 162 mm straight run
        (a very flat ARC, R = 31011 mm) meets an R = 142 mm curve with a 15.8 deg
        break. Anchoring on the flat ARC cannot reach that circle (the connector
        would have to touch it behind the junction), so the tangent search trims
        the straight by ~20 mm - representable because the sanitizer derives a
        flat ARC's radius from its chord.
        """
        flat = _planner_arc((315.1, 1773.6), -92.0, 31011.0, 0.3)
        curve = _planner_arc((flat["x1"], flat["y1"]), -75.9, 142.0, 81.4)
        segments = [flat, {"kind": "turn", "angle_deg": 15.8, "v": 150}, curve]
        self._smooth(segments)
        self.assertEqual([s["kind"] for s in segments], ["arc", "arc", "arc"])
        trimmed, connector = segments[0], segments[1]
        self.assertAlmostEqual(trimmed["x1"], connector["x0"])
        self.assertAlmostEqual(trimmed["y1"], connector["y0"])
        chord = math.dist((trimmed["x0"], trimmed["y0"]), (trimmed["x1"], trimmed["y1"]))
        self.assertLess(chord, 162.4 - 10.0, "the straight carried the trim")
        for prev, nxt in zip(segments, segments[1:]):
            gap = abs(g.deg_norm180(nxt["th0_deg"] - (prev["th0_deg"] + prev["sweep_deg"])))
            self.assertLessEqual(gap, 0.2, "the rounded chain must stay tangent")
        _sanitize_arcs(segments, PlanOptions())
        driven = abs(trimmed["sweep_deg"]) * math.pi / 180.0 * abs(trimmed["radius_mm"])
        chord = math.dist((trimmed["x0"], trimmed["y0"]), (trimmed["x1"], trimmed["y1"]))
        self.assertAlmostEqual(driven, chord, delta=1.0, msg="the trimmed straight still lands on its exit")

    def test_nearly_tangent_circles_slide_the_junction_instead_of_pivoting(self):
        """A curvature step of a drawn curve becomes a tangent joint, not a pivot.

        Real junction of the loaded strategy (aa11, 2026-10-09): the fit split a
        tight-to-wide transition into R = 226 mm and R = 1712 mm arcs whose
        circles are 0.5 mm from internal tangency, so NO connector tangent to
        both exists (every candidate touched the second circle behind the
        junction). The joint is slid ~46 mm onto the circles' meeting point
        instead: the last millimetres of the tight arc are driven on the wide
        one, and the joint is tangent - no stop, no pivot.
        """
        tight = _planner_arc((1209.3, 600.4), -83.6, 226.0, 79.5)
        wide = _planner_arc((tight["x1"], tight["y1"]), -14.9, 1712.0, 20.7)
        break_deg = g.deg_norm180(wide["th0_deg"] - (tight["th0_deg"] + tight["sweep_deg"]))
        self.assertGreater(abs(break_deg), 5.0, "the fit really did leave a break here")
        segments = [tight, {"kind": "turn", "angle_deg": break_deg, "v": 150}, wide]
        self._smooth(segments)
        kinds = [segment["kind"] for segment in segments]
        self.assertNotIn("turn", kinds, f"the pivot must be gone: {kinds}")
        for prev, nxt in zip(segments, segments[1:]):
            gap = abs(g.deg_norm180(nxt["th0_deg"] - (prev["th0_deg"] + prev["sweep_deg"])))
            self.assertLessEqual(gap, 0.2, "the relocated joint must be tangent")
        junction = (segments[0]["x1"], segments[0]["y1"])
        moved = math.dist(junction, (tight["x1"], tight["y1"]))
        self.assertGreater(moved, 10.0, "the junction slid onto a meeting point of the two circles")
        self.assertLess(moved, 250.0, "the slide must stay local")

    def test_coarsely_sampled_neighbour_does_not_block_the_rounding(self):
        """The corner cut is capped against the neighbour GEOMETRY, not its samples.

        A fitted ARC keeps the sampler's node spacing (up to ~250 mm on a large
        radius), so distance to the nearest sample over-states a local cut by up
        to half a chord - it used to reject a connector that really cut 1.6 mm of
        a 24 mm cap (measurement of the loaded strategy, 2026-10-09).
        """
        coarse = _planner_arc((700.0, 1000.0), 0.0, 1200.0, 20.0)
        coarse["points"] = [coarse["points"][0], coarse["points"][-1]]
        dense = _planner_arc(
            (coarse["x1"], coarse["y1"]),
            coarse["th0_deg"] + coarse["sweep_deg"] - 13.5,
            -380.0,
            -60.0,
        )
        segments = [coarse, {"kind": "turn", "angle_deg": -13.5, "v": 150}, dense]
        self._smooth(segments)
        self.assertEqual([s["kind"] for s in segments], ["arc", "arc", "arc"])
        connector = segments[1]
        coarse_polyline = [(px, py) for px, py in coarse["points"]]
        dense_polyline = [(px, py) for px, py in dense["points"]]
        deviation = max(
            min(
                _point_to_polyline((x, y), coarse_polyline),
                _point_to_polyline((x, y), dense_polyline),
            )
            for x, y in connector["points"]
        )
        self.assertLessEqual(deviation, 24.5)
        # the same connector reports a far larger cut against the sample VERTICES
        vertex_distance = max(
            min(math.dist((x, y), vertex) for vertex in coarse_polyline + dense_polyline)
            for x, y in connector["points"]
        )
        self.assertGreater(vertex_distance, deviation)

    def test_explicit_rotation_is_never_rounded(self):
        """Explicit rotation is never rounded."""
        first, second = self._pair(20.0)
        turn = {"kind": "turn", "angle_deg": 20.0, "v": 150, "explicit": True}
        segments = [first, turn, second]
        self._smooth(segments)
        self.assertEqual([s["kind"] for s in segments], ["arc", "turn", "arc"])

    def test_big_break_keeps_the_in_place_turn(self):
        """Big break keeps the in place turn."""
        first, second = self._pair(70.0)
        segments = [first, {"kind": "turn", "angle_deg": 70.0, "v": 150}, second]
        self._smooth(segments)
        self.assertEqual([s["kind"] for s in segments], ["arc", "turn", "arc"])

    def test_planned_stroke_has_no_pivot_at_small_breaks(self):
        # a wave with a deliberate ~20 deg break must plan as arcs only
        """Planned stroke has no pivot at small breaks."""
        first, second = self._pair(20.0)
        points = [[x, y] for x, y in first["points"]]
        points += [list(point) for point in second["points"]][1:]
        options = PlanOptions(goto_step_mm=250.0)
        start_pose = (points[0][0], points[0][1], first["th0_deg"])
        result = plan([stroke(points)], start_pose, None, [], options, 3000.0, 2000.0)
        kinds = [s["kind"] for s in result["segments"]]
        # a leading alignment turn (entry of the run) is fine; a pivot BETWEEN
        # two fitted arcs is what the rounding pass must have removed
        for index, kind in enumerate(kinds):
            if kind != "turn":
                continue
            before = kinds[:index]
            after = kinds[index + 1 :]
            self.assertFalse(
                any(entry == "arc" for entry in before) and any(entry == "arc" for entry in after),
                "no in-place pivot between fitted arcs",
            )
        arcs = [s for s in result["segments"] if s["kind"] == "arc"]
        for prev, nxt in zip(arcs, arcs[1:]):
            gap = abs(g.deg_norm180(nxt["th0_deg"] - (prev["th0_deg"] + prev["sweep_deg"])))
            self.assertLessEqual(gap, 0.2, "every arc must be entered on its tangent")


class NearStraightArcTests(unittest.TestCase):
    """Straight runs drive as very flat ARCs, not a GOTO node every step.

    The board brakes to a FULL stop at every GOTO node (TURN -> DRIVE ->
    TURN, ramp reset per segment), so the old 250 mm node cadence turned a
    long straight into stop-and-go. A straight run becomes near-straight
    ARCs (radius up to the 32767 mm protocol cap, sweep an exact multiple of
    the 0.1 deg wire quantum) which the console chains through QUEUE_ARC
    without stopping.
    """

    def _straight(self, length_mm=1400.0, y=1000.0, steps=56):
        return [[400.0 + length_mm * index / steps, y] for index in range(steps + 1)]

    def test_straight_run_drives_as_one_goto(self):
        """A long straight is ONE move, not an alternating S of flat ARCs.

        The real robot read the tiny curvature sign alternation as
        acceleration/deceleration and unnecessary turns; a single GOTO has
        one acceleration, one straight drive and one deceleration.
        """
        points = self._straight()
        result = plan(
            [stroke(points)], (points[0][0], points[0][1], 0.0), None, [], PlanOptions(), 3000.0, 2000.0
        )
        kinds = [segment["kind"] for segment in result["segments"]]
        gotos = goto_segments(result)
        self.assertEqual(len(gotos), 1, f"a straight must be one move: {kinds}")
        self.assertAlmostEqual(gotos[0]["x"], 1800.0, delta=5.0)
        self.assertAlmostEqual(gotos[0]["y"], 1000.0, delta=5.0)
        self.assertGreaterEqual(len(gotos[0].get("points", [])), 2)
        arcs = [segment for segment in result["segments"] if segment["kind"] == "arc"]
        self.assertEqual(arcs, [])
        driven = result["stats"]["length_mm"]
        self.assertAlmostEqual(driven, 1400.0, delta=5.0, msg="the straight must keep its length")
        # ...and the geometry stays on the drawn line
        for x, y in program_points(result):
            self.assertAlmostEqual(y, 1000.0, delta=3.5, msg=f"({x:.0f}, {y:.0f}) left the line")

    def test_option_restores_the_goto_cadence(self):
        """``native_lines=False`` keeps the plain GOTO node chain."""
        points = self._straight()
        options = PlanOptions(native_lines=False)
        result = plan([stroke(points)], (points[0][0], points[0][1], 0.0), None, [], options, 3000.0, 2000.0)
        self.assertTrue(goto_segments(result), "the fallback must still emit GOTO nodes")


class CuspCollapseTests(unittest.TestCase):
    """A switchback too tight for the wheels becomes ONE pivot, not a stutter.

    The sampler follows the drawn geometry point by point; a hand drawn
    reversal came out as a run of tiny GOTO nodes (real mission 2026-10-07:
    14 nodes 2-4 mm apart turning 10-13 deg each - "a lot of turning
    unnecessary and not smooth") and the board stopped at every one.
    """

    def _switchback(self):
        """A 1.2 m run out, a tight reversal, and a 0.6 m run back."""
        points = [[700.0 + index * 25.0, 1000.0] for index in range(49)]
        points += [[1900.0 - index * 20.0, 986.0 + index * 20.0] for index in range(31)]
        return points

    def test_tight_switchback_has_no_tiny_node_stutter(self):
        """No two consecutive GOTO nodes sit a few mm apart turning a few deg."""
        points = self._switchback()
        result = plan(
            [stroke(points)], (points[0][0], points[0][1], 0.0), None, [], PlanOptions(), 3000.0, 2000.0
        )
        gotos = goto_segments(result)
        self.assertLessEqual(len(gotos), 4, f"the switchback still stutters: {len(gotos)} GOTO nodes")
        for prev, nxt in zip(gotos, gotos[1:]):
            step = math.hypot(nxt["x"] - prev["x"], nxt["y"] - prev["y"])
            self.assertGreaterEqual(step, 25.0, "a tiny node hop means the cusp was not collapsed")
        # every motion segment is a drivable primitive
        for segment in result["segments"]:
            self.assertIn(segment["kind"], ("arc", "goto", "turn"), segment)

    def test_straight_line_mission_stays_one_pivot_or_less(self):
        """A single 90 deg corner plans as arcs plus at most one pivot."""
        points = [[600.0 + index * 30.0, 800.0] for index in range(41)]
        points += [[1800.0, 800.0 + index * 30.0] for index in range(1, 21)]
        result = plan(
            [stroke(points)], (600.0, 800.0, 0.0), None, [], PlanOptions(), 3000.0, 2000.0
        )
        turns = [segment for segment in result["segments"] if segment["kind"] == "turn"]
        self.assertLessEqual(len(turns), 1, f"a right angle needs at most one pivot: {turns}")
        for segment in result["segments"]:
            self.assertIn(segment["kind"], ("arc", "goto", "turn"), segment)


class ChainAnchorWireTests(unittest.TestCase):
    """Absolute re-anchors and the ARC wire window, keeping the plan drivable.

    The board rejected a real plan with RANGE (BAD_PARAM aux 22/23, mission
    2026-10-07): sub-millimetre connector arcs and near-straight sweeps that
    disagreed with the chord. The same mission finished 70 mm short because
    every ARC is driven RELATIVE (radius + sweep from the CURRENT pose) and
    one aborted segment left the rest of the plan in the wrong frame.
    """

    @staticmethod
    def _arc(x0, y0, x1, y1, radius, sweep, th0, speed=200):
        return {
            "kind": "arc",
            "x0": x0,
            "y0": y0,
            "x1": x1,
            "y1": y1,
            "radius_mm": radius,
            "sweep_deg": sweep,
            "th0_deg": th0,
            "v": speed,
        }

    def test_mid_path_turn_becomes_an_absolute_goto(self):
        """A junction TURN becomes a GOTO at the junction, facing the next arc."""
        segments = [
            self._arc(0.0, 0.0, 100.0, 0.0, 5000.0, 1.1, -0.6),
            {"kind": "turn", "angle_deg": 30.0, "v": 100},
            self._arc(100.0, 0.0, 200.0, 40.0, 500.0, 30.0, 29.4),
        ]
        self.assertTrue(_reanchor_turns(segments))
        anchor = segments[1]
        self.assertEqual(anchor["kind"], "goto")
        self.assertAlmostEqual(anchor["x"], 100.0)
        self.assertAlmostEqual(anchor["y"], 0.0)
        self.assertAlmostEqual(anchor["th"], 29.4, msg="the anchor faces the next arc")

    def test_tangent_junction_reanchor_is_removed_after_smoothing(self):
        """A redundant stop between position- and heading-contiguous ARCs is removed."""
        segments = [
            self._arc(0.0, 0.0, 100.0, 0.0, 5000.0, 1.1, -0.55),
            {"kind": "turn", "angle_deg": 1.0, "v": 100},
            self._arc(100.0, 0.0, 200.0, 0.0, 5000.0, 1.1, 0.55),
        ]
        self.assertTrue(_reanchor_turns(segments))
        self.assertTrue(_blend_tangent_reanchors(segments))
        self.assertEqual([segment["kind"] for segment in segments], ["arc", "arc"])

    def test_non_tangent_and_chain_reanchors_are_preserved(self):
        """Only proven-redundant junction anchors may be blended away."""
        corner = [
            self._arc(0.0, 0.0, 100.0, 0.0, 5000.0, 1.1, -0.55),
            {
                "kind": "goto", "x": 100.0, "y": 0.0, "th": 15.0, "v": 100,
                "junction_reanchor": True,
            },
            self._arc(100.0, 0.0, 200.0, 20.0, 500.0, 30.0, 15.0),
        ]
        self.assertFalse(_blend_tangent_reanchors(corner))
        self.assertEqual(corner[1]["kind"], "goto")

        chain_anchor = [
            self._arc(0.0, 0.0, 100.0, 0.0, 5000.0, 1.1, -0.55),
            {"kind": "goto", "x": 100.0, "y": 0.0, "th": 0.55, "v": 100},
            self._arc(100.0, 0.0, 200.0, 0.0, 5000.0, 1.1, 0.55),
        ]
        self.assertFalse(_blend_tangent_reanchors(chain_anchor))
        self.assertEqual(chain_anchor[1]["kind"], "goto")

    def test_opening_and_explicit_turns_are_kept(self):
        """The launch alignment and explicit rotations stay TURNs."""
        opener = {"kind": "turn", "angle_deg": 12.0, "v": 100}
        explicit = {"kind": "turn", "angle_deg": 90.0, "v": 100, "explicit": True}
        segments = [
            opener,
            self._arc(0.0, 0.0, 100.0, 0.0, 5000.0, 1.1, -0.6),
            explicit,
            self._arc(100.0, 0.0, 200.0, 40.0, 500.0, 30.0, 29.4),
        ]
        self.assertFalse(_reanchor_turns(segments))
        self.assertIs(segments[0], opener)
        self.assertIs(segments[2], explicit)

    def test_long_chain_gets_one_absolute_reanchor(self):
        """Long relative-arc chains still get an absolute re-anchor."""
        segments = [
            self._arc(400.0 * i, 0.0, 400.0 * (i + 1), 0.0, 30000.0, 0.76, 0.0, 500)
            for i in range(6)
        ]
        self.assertTrue(_cap_chain_length(segments))
        anchors = [segment for segment in segments if segment["kind"] == "goto"]
        self.assertEqual(len(anchors), 1, f"one anchor per ~1800 mm: {segments}")
        self.assertAlmostEqual(anchors[0]["x"], 2000.0, delta=0.5)
        self.assertAlmostEqual(anchors[0]["th"], 0.0, delta=0.5)
        kinds = [segment["kind"] for segment in segments]
        self.assertEqual(kinds.count("arc"), 6, "the arcs themselves are untouched")

    def test_short_chain_is_untouched(self):
        """A run shorter than the cap keeps its full blend."""
        segments = [
            self._arc(0.0, 0.0, 300.0, 0.0, 30000.0, 0.57, 0.0, 500),
            self._arc(300.0, 0.0, 600.0, 0.0, 30000.0, 0.57, 0.0, 500),
        ]
        self.assertFalse(_cap_chain_length(segments))
        self.assertEqual(len(segments), 2)

    def test_sanitize_quantises_radius_and_sweep(self):
        """Radius snaps to whole mm, the sweep to the 0.1 deg wire quantum.

        The arc here is geometrically consistent (R 500.4 mm, 3.44 deg of
        sweep, entry due +X): the sanitiser re-derives the radius from the
        chord and the 0.1 deg sweep so the board really reaches the exit.
        """
        segments = [self._arc(0.0, 0.0, 30.03, 0.9, 500.4, 3.44, 0.0, 150)]
        self.assertTrue(_sanitize_arcs(segments, PlanOptions()))
        segment = segments[0]
        self.assertEqual(segment["kind"], "arc")
        self.assertAlmostEqual(segment["radius_mm"], round(segment["radius_mm"]))
        self.assertAlmostEqual(segment["sweep_deg"], 3.4)
        self.assertLess(math.dist((segment["x1"], segment["y1"]), (30.03, 0.9)), 0.1)

    def test_sanitize_turns_an_undrivable_arc_into_a_goto(self):
        """Below the firmware's 50 mm radius floor the arc becomes a GOTO."""
        segments = [self._arc(0.0, 0.0, 10.0, 0.0, 40.0, 14.3, 0.0, 150)]
        self.assertTrue(_sanitize_arcs(segments, PlanOptions()))
        segment = segments[0]
        self.assertEqual(segment["kind"], "goto", f"R=40 mm cannot be driven: {segment}")
        self.assertAlmostEqual(segment["x"], 10.0)
        self.assertAlmostEqual(segment["th"], 14.3)

    def test_sanitize_stretches_a_sub_millimetre_arc(self):
        """A connector shorter than the 1 mm firmware floor is stretched."""
        segments = [self._arc(0.0, 0.0, 0.5, 0.0, 60.0, 0.44, 0.0, 150)]
        _sanitize_arcs(segments, PlanOptions())
        self.assertEqual(segments[0]["kind"], "arc")
        length = abs(segments[0]["sweep_deg"]) * math.pi / 180.0 * abs(segments[0]["radius_mm"])
        self.assertGreaterEqual(length, 1.9, "the board rejects arcs under 1 mm with RANGE")

    def test_sanitize_keeps_a_flat_arc_on_its_declared_chord(self):
        """A near-straight ARC drives exactly the chord it declares.

        The sweep travels on the 0.1 deg wire quantum (54 mm at R = 31011 mm),
        so the RADIUS has to absorb the rounding: keeping the radius and moving
        the sweep left the board up to ~27 mm away from the declared exit (a
        junction connector then could not trim such an ARC at all - 2026-10-09).
        """
        segments = [self._arc(0.0, 0.0, 141.6, 0.0, 31011.0, 0.3, 0.0, 500)]
        _sanitize_arcs(segments, PlanOptions())
        segment = segments[0]
        self.assertEqual(segment["kind"], "arc")
        driven = abs(segment["sweep_deg"]) * math.pi / 180.0 * abs(segment["radius_mm"])
        self.assertAlmostEqual(driven, 141.6, delta=1.0, msg=f"driven {driven} vs declared chord")
        self.assertAlmostEqual(segment["sweep_deg"], 0.3)

    def test_sanitize_merges_coincident_reanchors(self):
        """Two GOTOs on the same junction collapse into one (no stop for nothing)."""
        segments = [
            {"kind": "goto", "x": 100.0, "y": 100.0, "th": 10.0, "v": 150},
            {"kind": "goto", "x": 100.4, "y": 100.0, "th": 25.0, "v": 200},
        ]
        self.assertTrue(_sanitize_arcs(segments, PlanOptions()))
        self.assertEqual(len(segments), 1)
        self.assertAlmostEqual(segments[0]["th"], 25.0)
        self.assertEqual(segments[0]["v"], 200)

    def test_sanitize_lands_samples_on_the_declared_end(self):
        """The segment's own samples end on ITS x1/y1 (no phantom back-step).

        The plan must declare the point the board will really reach: a fitted
        ARC whose radius/sweep cannot get to the labelled exit left a residual
        the next GOTO crept away - the visible dance of 2026-10-09.
        """
        segments = [self._arc(0.0, 0.0, 397.9, 2.64, 30000.0, 0.76, 0.0, 500)]
        segments[0]["points"] = [[0.0, 0.0], [200.0, 1.3], [397.0, 2.6]]
        _sanitize_arcs(segments, PlanOptions())
        segment = segments[0]
        self.assertEqual(segment["kind"], "arc")
        self.assertAlmostEqual(segment["points"][-1][0], segment["x1"], places=1)
        self.assertAlmostEqual(segment["points"][-1][1], segment["y1"], places=1)
        # and x1/y1 is the very point the board's own rotation reaches
        centre = _arc_center(
            (segment["x0"], segment["y0"]), segment["th0_deg"], segment["radius_mm"]
        )
        angle = math.radians(segment["th0_deg"] + segment["sweep_deg"])
        self.assertAlmostEqual(
            segment["x1"], centre[0] + segment["radius_mm"] * math.sin(angle), places=1
        )
        self.assertAlmostEqual(
            segment["y1"], centre[1] - segment["radius_mm"] * math.cos(angle), places=1
        )

    def test_sanitize_keeps_a_drawing_corner_between_two_flat_arcs(self):
        """A real corner between two ARCs keeps its own entry heading.

        Propagating the previous arc's exit heading across a 110 deg kink sent
        the next aim chasing its exit from a heading that is 110 deg off, and
        a flat 250 mm span came back as a 220 deg hairpin (2026-10-09).
        """
        segments = [
            self._arc(0.0, 0.0, 250.0, 0.0, 28646.0, 0.5, 0.0, 200),
            # the second ARC is a CONSISTENT flat arc (its exit sits on its own
            # circle) that starts 110 deg away from where the first one ended:
            # a drawing corner, exactly what the pipeline feeds the sanitiser
            self._arc(250.0, 0.0, 163.5, 234.5, 28646.0, 0.5, 110.0, 200),
        ]
        _sanitize_arcs(segments, PlanOptions())
        self.assertAlmostEqual(segments[1]["th0_deg"], 110.0)
        self.assertLessEqual(abs(segments[1]["sweep_deg"]), 5.0, "no hairpin")
        self.assertGreaterEqual(abs(segments[1]["radius_mm"]), 20000.0)

    def test_sanitize_lands_a_reanchor_node_on_the_arc_exit(self):
        """A GOTO node a hair from the arc's exit is moved onto it.

        The node comes from the reference path, the arc from the fit: left
        0.5-5 mm apart the plan shows a phantom back-step and the board stops,
        creeps the difference away and turns - a tiny movement plus a turn at a
        junction that was meant to be one stop with nothing to do.
        """
        segments = [
            self._arc(0.0, 0.0, 200.0, 0.0, 28646.0, 0.4, 0.0, 200),
            {"kind": "goto", "x": 202.4, "y": 0.4, "th": 0.0, "v": 200},
        ]
        self.assertTrue(_sanitize_arcs(segments, PlanOptions()))
        segment, node = segments[0], segments[1]
        self.assertLess(math.dist((node["x"], node["y"]), (segment["x1"], segment["y1"])), 0.06)

    def test_sanitize_makes_every_arc_reach_its_declared_exit(self):
        """Each ARC lands on its own exit and hands that point to the next one.

        The board drives radius + sweep from its own pose, so a declared exit
        it cannot reach is a gap the next segment has to creep away - one tiny
        movement and one touch-up turn per junction, 26 arcs in a mission.
        """
        segments = [
            self._arc(0.0, 0.0, 30.03, 0.9, 500.4, 3.44, 0.0, 150),
            self._arc(30.03, 0.9, 120.0, 30.0, 200.0, 26.0, 3.44, 150),
        ]
        _sanitize_arcs(segments, PlanOptions())
        for segment in segments:
            self.assertEqual(segment["kind"], "arc")
            centre = _arc_center(
                (segment["x0"], segment["y0"]), segment["th0_deg"], segment["radius_mm"]
            )
            angle = math.radians(segment["th0_deg"] + segment["sweep_deg"])
            reached = (
                centre[0] + segment["radius_mm"] * math.sin(angle),
                centre[1] - segment["radius_mm"] * math.cos(angle),
            )
            self.assertLess(
                math.dist(reached, (segment["x1"], segment["y1"])),
                0.06,
                f"the board lands off the declared exit: {segment}",
            )
        self.assertLess(
            math.dist(
                (segments[0]["x1"], segments[0]["y1"]),
                (segments[1]["x0"], segments[1]["y0"]),
            ),
            0.06,
            "the next ARC must start where the previous one lands",
        )


if __name__ == "__main__":
    unittest.main()
