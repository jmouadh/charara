"""Custom polygon obstacles: geometry, planner integration and mirroring.

The console's "Custom" tool builds an obstacle from clicked points (the last
vertex joins the first automatically), so the service has to treat a vertex
list exactly like a box or a zone: distance for the stop clearances, a rotated
body test, occupancy-grid blocking, mirroring for the other side and the
mid-run obstacle reactions.

File:    tests/test_polygon_obstacles.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-23
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from software.core.mission.orchestrator import (  # noqa: E402
    _clips_obstacle,
    obstacle_distance_mm,
)
from software.core.mission.teams import mirror_obstacle  # noqa: E402
from software.core.navigation.clearance import (  # noqa: E402
    body_clear_at,
    build_grid,
    obstacle_clearance,
)
from software.core.navigation.grid import OccupancyGrid  # noqa: E402
from software.core.navigation.options import PlanOptions  # noqa: E402
from software.core.navigation.polygon import (  # noqa: E402
    area_mm2,
    clean_vertices,
    distance_to_polygon,
    is_usable,
    point_in_polygon,
    polygon_rect_distance,
)
from software.core.navigation.trajectory import (  # noqa: E402
    _clamp_obstacles,
    plan,
)

#: a right triangle with its legs on the axes of its own bounding box
TRIANGLE = [[1000.0, 1000.0], [1400.0, 1000.0], [1000.0, 1400.0]]
#: a concave "L": the notch must stay free
L_SHAPE = [
    [800.0, 800.0], [1200.0, 800.0], [1200.0, 1000.0],
    [1000.0, 1000.0], [1000.0, 1200.0], [800.0, 1200.0],
]


def polygon(points):
    """A polygon obstacle dict as the console sends it."""
    return {"kind": "polygon", "points": [list(point) for point in points]}


class PolygonGeometryTests(unittest.TestCase):
    """Inside test, distances and the rectangle overlap used by the body check."""

    def test_inside_test_handles_a_concave_shape(self):
        self.assertTrue(point_in_polygon(1050.0, 1050.0, [tuple(p) for p in TRIANGLE]))
        self.assertFalse(point_in_polygon(1350.0, 1350.0, [tuple(p) for p in TRIANGLE]))
        # inside the L, and inside its notch
        self.assertTrue(point_in_polygon(850.0, 850.0, [tuple(p) for p in L_SHAPE]))
        self.assertFalse(point_in_polygon(1150.0, 1150.0, [tuple(p) for p in L_SHAPE]))

    def test_distance_is_zero_inside_and_measured_outside(self):
        points = [tuple(p) for p in TRIANGLE]
        self.assertEqual(distance_to_polygon(1050.0, 1050.0, points), 0.0)
        # 100 mm right of the vertical leg x = 1000 at a height the leg covers
        self.assertAlmostEqual(
            distance_to_polygon(900.0, 1050.0, points), 100.0, delta=0.5
        )
        # the hypotenuse from (1400, 1000) to (1000, 1400): x + y = 2400
        self.assertAlmostEqual(
            distance_to_polygon(1300.0, 1300.0, points), 200.0 / math.sqrt(2.0), delta=0.5
        )

    def test_rectangle_distance_detects_overlap_and_clearance(self):
        points = [tuple(p) for p in L_SHAPE]
        inside = [(900.0, 900.0), (1100.0, 900.0), (1100.0, 1100.0), (900.0, 1100.0)]
        self.assertEqual(polygon_rect_distance(inside, points), 0.0)
        # a rectangle parked in the notch does NOT overlap the L
        notch = [(1050.0, 1050.0), (1150.0, 1050.0), (1150.0, 1150.0), (1050.0, 1150.0)]
        self.assertGreater(polygon_rect_distance(notch, points), 0.0)
        # parked 100 mm right of the L's vertical edge (x = 1200, y 800..1000)
        clear = [(1300.0, 850.0), (1350.0, 850.0), (1350.0, 950.0), (1300.0, 950.0)]
        self.assertAlmostEqual(
            polygon_rect_distance(clear, points), 100.0, delta=1.0
        )

    def test_vertex_cleanup_and_area(self):
        self.assertEqual(clean_vertices([[0, 0], [0, 0], [10, 0], [10, 10], [0, 0]]),
                         [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)])
        self.assertAlmostEqual(area_mm2([(0.0, 0.0), (10.0, 0.0), (0.0, 10.0)]), 50.0)
        self.assertFalse(is_usable([(0.0, 0.0), (10.0, 0.0)]))
        self.assertFalse(is_usable([(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)]))
        self.assertTrue(is_usable([(0.0, 0.0), (10.0, 0.0), (0.0, 10.0)]))


class PolygonClearanceTests(unittest.TestCase):
    """The stop clearances and the rotated-body check honour the real outline."""

    def test_obstacle_clearance_measures_the_shape(self):
        obstacle = polygon(TRIANGLE)
        self.assertEqual(obstacle_clearance((1050.0, 1050.0), [obstacle]), 0.0)
        self.assertAlmostEqual(obstacle_clearance((800.0, 1050.0), [obstacle]), 200.0, delta=0.5)

    def test_rotated_body_inside_or_touching_the_shape_is_not_clear(self):
        obstacle = polygon(TRIANGLE)
        self.assertFalse(
            body_clear_at((1100.0, 1100.0), 0.0, [obstacle], 300.0, 300.0, 3000.0, 2000.0)
        )
        self.assertTrue(
            body_clear_at((600.0, 600.0), 0.0, [obstacle], 300.0, 300.0, 3000.0, 2000.0)
        )

    def test_grid_blocks_the_shape_and_leaves_the_notch_free(self):
        grid = OccupancyGrid(2000.0, 2000.0, 50.0, 0.0, border_mm=0.0)
        grid.add_polygon([tuple(p) for p in L_SHAPE])
        self.assertFalse(grid.is_free_mm(850.0, 850.0))
        self.assertTrue(grid.is_free_mm(1150.0, 1150.0), "the notch must stay free")
        self.assertTrue(grid.is_free_mm(1500.0, 1500.0))

    def test_build_grid_blocks_a_polygon_with_inflation(self):
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0, grid_mm=40.0)
        obstacle = polygon(TRIANGLE)
        grid = build_grid(3000.0, 2000.0, [obstacle], options)
        self.assertFalse(grid.is_free_mm(1050.0, 1050.0))
        self.assertTrue(grid.is_free_mm(2400.0, 400.0))

    def test_midrun_obstacle_distance_and_clipping_know_polygons(self):
        obstacle = polygon(TRIANGLE)
        self.assertEqual(obstacle_distance_mm(obstacle, (1100.0, 1100.0)), 0.0)
        self.assertAlmostEqual(
            obstacle_distance_mm(obstacle, (800.0, 1050.0)), 200.0, delta=0.5
        )
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0)
        through = [[500.0, 1050.0], [1500.0, 1050.0]]
        around = [[500.0, 1700.0], [2500.0, 1700.0]]
        self.assertTrue(_clips_obstacle(through, [obstacle], options, 3000.0, 2000.0))
        self.assertFalse(_clips_obstacle(around, [obstacle], options, 3000.0, 2000.0))


class PolygonPersistenceTests(unittest.TestCase):
    """Clamping to the table and mirroring for the other side."""

    def test_points_are_clamped_to_the_field(self):
        obstacle = polygon([[-100.0, 500.0], [5000.0, 500.0], [1000.0, 4000.0]])
        clamped = _clamp_obstacles([obstacle], 3000.0, 2000.0)[0]
        self.assertEqual(
            clamped["points"], [[0.0, 500.0], [3000.0, 500.0], [1000.0, 2000.0]]
        )

    def test_a_degenerate_shape_is_dropped(self):
        broken = {"kind": "polygon", "points": [[10.0, 10.0], [20.0, 20.0]]}
        self.assertEqual(_clamp_obstacles([broken], 3000.0, 2000.0), [])

    def test_mirroring_reflects_every_vertex(self):
        mirrored = mirror_obstacle(polygon(TRIANGLE), 3000.0)
        self.assertEqual(
            mirrored["points"],
            [[2000.0, 1000.0], [1600.0, 1000.0], [2000.0, 1400.0]],
        )
        # the reflected shape is the same size
        self.assertAlmostEqual(
            area_mm2([tuple(p) for p in mirrored["points"]]),
            area_mm2([tuple(p) for p in TRIANGLE]),
        )


class PolygonPlanTests(unittest.TestCase):
    """A mission routed past a custom shape never cuts through it."""

    def test_planner_routes_around_a_triangle(self):
        options = PlanOptions(
            robot_width_mm=300.0,
            robot_height_mm=300.0,
            avoid_obstacles=True,
            smooth=False,
        )
        obstacle = polygon(TRIANGLE)
        elements = [{"kind": "stroke", "points": [[600.0, 1000.0], [1600.0, 1000.0]]}]
        result = plan(
            elements, (600.0, 1000.0, 0.0), None, [obstacle], options, 3000.0, 2000.0
        )
        points = [tuple(point) for point in result["path"]]
        shape = [(p[0], p[1]) for p in TRIANGLE]
        self.assertFalse(
            any(point_in_polygon(x, y, shape) for x, y in points),
            "the route must not cross the drawn shape",
        )
        self.assertIn("path", result)
        self.assertTrue(result["segments"], "a route must be produced")

    def test_a_shape_nobody_can_pass_still_warns(self):
        options = PlanOptions(robot_width_mm=300.0, robot_height_mm=300.0, avoid_obstacles=True)
        wall = polygon([[1400.0, 0.0], [1500.0, 0.0], [1500.0, 2000.0], [1400.0, 2000.0]])
        elements = [{"kind": "stroke", "points": [[600.0, 1000.0], [2400.0, 1000.0]]}]
        result = plan(
            elements, (600.0, 1000.0, 0.0), None, [wall], options, 3000.0, 2000.0
        )
        self.assertIn("warnings", result)
        self.assertTrue(result["warnings"], "a blocked mission must say so")


if __name__ == "__main__":
    unittest.main()
