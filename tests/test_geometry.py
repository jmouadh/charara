"""Geometry tests: RDP, circle fit, line/arc segmentation, sampling.

File:    tests/test_geometry.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from software.core.navigation import geometry as g  # noqa: E402


def arc_points(cx, cy, r, start_deg, end_deg, n=40):
    """Arc points."""
    return [
        g.point_at_angle((cx, cy), r, start_deg + (end_deg - start_deg) * i / (n - 1))
        for i in range(n)
    ]


# Field-drawn stroke that used to come back right-to-left: it splits into
# [arc(first half), line(second half)] and the stack used to emit the line
# first, doubling the rebuilt length (regression: primitives out of order).
STROKE_SPLIT_REGRESSION = [
    (1208.1, 1376.8), (1231.9, 1341.1), (1231.9, 1325.2), (1231.9, 1309.3),
    (1231.9, 1293.4), (1231.9, 1269.6), (1231.9, 1253.7), (1235.9, 1237.8),
    (1239.8, 1221.9), (1243.8, 1206.0), (1243.8, 1190.2), (1243.8, 1186.2),
]


def primitive_start(primitive):
    """Primitive start."""
    if primitive["type"] == "line":
        return tuple(primitive["a"])
    return g.point_at_angle(primitive["center"], primitive["r"], primitive["a_start_deg"])


def primitive_end(primitive):
    """Primitive end."""
    if primitive["type"] == "line":
        return tuple(primitive["b"])
    return g.point_at_angle(primitive["center"], primitive["r"], primitive["a_end_deg"])


def rebuild(points, primitives):
    """Rebuild."""
    out = []
    for primitive in primitives:
        part = g.primitive_to_points(primitive, max_step_deg=10.0)
        if out and part and part[0] == out[-1]:
            part = part[1:]
        out.extend(part)
    return out


def dist_to_polyline(point, polyline):
    """Dist to polyline."""
    best = float("inf")
    for a, b in zip(polyline, polyline[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        t = 0.0 if length2 < 1e-9 else max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2))
        best = min(best, math.dist(point, (a[0] + t * dx, a[1] + t * dy)))
    return best


STROKE_LONG_REGRESSION = [
    (369, 1931), (353, 1905), (356, 1892), (356, 1879), (360, 1866), (360, 1853),
    (363, 1818), (366, 1805), (369, 1792), (372, 1773), (376, 1757), (379, 1737),
    (385, 1715), (389, 1692), (395, 1666), (402, 1647), (405, 1624), (408, 1611),
    (411, 1592), (418, 1569), (424, 1553), (434, 1531), (440, 1511), (447, 1489),
    (456, 1476), (466, 1447), (485, 1411), (492, 1395), (505, 1372), (515, 1356),
    (531, 1337), (544, 1321), (557, 1305), (566, 1292), (586, 1266), (599, 1253),
    (621, 1233), (640, 1217), (660, 1204), (676, 1191), (689, 1179), (715, 1166),
    (728, 1156), (750, 1146), (766, 1146), (779, 1140), (795, 1130), (815, 1127),
    (831, 1124), (867, 1117), (883, 1117), (908, 1117), (931, 1114), (957, 1114),
    (983, 1114), (1018, 1114), (1047, 1114), (1080, 1114), (1112, 1114), (1135, 1114),
    (1164, 1114), (1189, 1114), (1215, 1114), (1238, 1114), (1264, 1114), (1283, 1114),
    (1309, 1114), (1325, 1114), (1348, 1114), (1361, 1114), (1373, 1114), (1396, 1117),
    (1419, 1120), (1435, 1124), (1454, 1130), (1470, 1133), (1486, 1137), (1503, 1140),
    (1516, 1140), (1522, 1143),
]


class GeometryTests(unittest.TestCase):
    def test_rdp_collapses_noisy_line(self):
        """Rdp collapses noisy line."""
        points = [(i * 10.0, math.sin(i * 0.3) * 0.4) for i in range(50)]
        simplified = g.rdp(points, 1.0)
        self.assertLess(len(simplified), 5)
        self.assertEqual(simplified[0], points[0])
        self.assertEqual(simplified[-1], points[-1])

    def test_circle_fit_recovers_center_and_radius(self):
        """Circle fit recovers center and radius."""
        points = arc_points(1000.0, 500.0, 300.0, 10.0, 80.0)
        (cx, cy), r = g.fit_circle(points)
        self.assertAlmostEqual(cx, 1000.0, delta=2.0)
        self.assertAlmostEqual(cy, 500.0, delta=2.0)
        self.assertAlmostEqual(r, 300.0, delta=2.0)

    def test_segment_line_arc_finds_arc(self):
        """Segment line arc finds arc."""
        points = arc_points(1000.0, 500.0, 300.0, 0.0, 90.0)
        primitives = g.segment_line_arc(points, tol=5.0)
        self.assertEqual(len(primitives), 1)
        self.assertEqual(primitives[0]["type"], "arc")
        self.assertAlmostEqual(abs(primitives[0]["sweep_deg"]), 90.0, delta=2.0)

    def test_segment_line_arc_finds_line(self):
        """Segment line arc finds line."""
        points = [(i * 20.0, 3.0 * math.sin(i)) for i in range(30)]  # noisy straight
        primitives = g.segment_line_arc(points, tol=5.0)
        self.assertTrue(all(p["type"] == "line" for p in primitives))

    def test_segment_mixed_stroke(self):
        """Segment mixed stroke."""
        straight = [(i * 25.0, 0.0) for i in range(21)]        # (0,0)..(500,0)
        quarter = arc_points(500.0, 200.0, 200.0, -90.0, 0.0, 25)  # (500,0)..(700,200)
        points = straight + quarter[1:]
        primitives = g.segment_line_arc(points, tol=6.0)
        kinds = [p["type"] for p in primitives]
        self.assertIn("arc", kinds)
        # the quarter circle must be recovered as a real arc of r=200
        radii = [p["r"] for p in primitives if p["type"] == "arc"]
        self.assertTrue(any(abs(radius - 200.0) < 25.0 for radius in radii), radii)
        # and the whole rebuilt shape must hug the drawn polyline (<= tol + slack)
        rebuilt = rebuild(points, primitives)
        worst = max(dist_to_polyline(p, points) for p in rebuilt)
        self.assertLess(worst, 8.0)

    def test_primitives_follow_drawing_order(self):
        """Primitives follow drawing order."""
        primitives = g.segment_line_arc(STROKE_SPLIT_REGRESSION, tol=8.0)
        self.assertGreaterEqual(len(primitives), 2)
        starts = [primitive_start(p) for p in primitives]
        ends = [primitive_end(p) for p in primitives]
        # endpoints and joins may deviate from the raw span by the fit
        # tolerance at most - the order/continuity is what matters here
        self.assertAlmostEqual(math.dist(starts[0], STROKE_SPLIT_REGRESSION[0]), 0.0, delta=8.0)
        self.assertAlmostEqual(math.dist(ends[-1], STROKE_SPLIT_REGRESSION[-1]), 0.0, delta=8.0)
        for previous_end, next_start in zip(ends, starts[1:]):
            self.assertLess(math.dist(previous_end, next_start), 8.0)

    def test_rebuild_does_not_double_length(self):
        """Rebuild does not double length."""
        raw = g.polyline_length(STROKE_SPLIT_REGRESSION)
        primitives = g.segment_line_arc(STROKE_SPLIT_REGRESSION, tol=8.0)
        rebuilt = rebuild(STROKE_SPLIT_REGRESSION, primitives)
        self.assertLess(g.polyline_length(rebuilt), raw * 1.2)
        self.assertLess(math.dist(rebuilt[0], STROKE_SPLIT_REGRESSION[0]), 1.0)
        self.assertLess(math.dist(rebuilt[-1], STROKE_SPLIT_REGRESSION[-1]), 1.0)

    def test_line_then_arc_keeps_order(self):
        """Line then arc keeps order."""
        straight = [(i * 20.0, 0.0) for i in range(11)]  # (0,0)..(200,0)
        turn = arc_points(200.0, 150.0, 150.0, -90.0, -30.0, 20)  # continues up-right
        points = straight + turn[1:]
        primitives = g.segment_line_arc(points, tol=5.0)
        self.assertEqual(primitives[0]["type"], "line")
        self.assertEqual(primitives[-1]["type"], "arc")
        self.assertAlmostEqual(math.dist(primitive_start(primitives[0]), (0.0, 0.0)), 0.0, delta=1.0)
        self.assertAlmostEqual(math.dist(primitive_end(primitives[-1]), points[-1]), 0.0, delta=2.0)
        rebuilt = rebuild(points, primitives)
        self.assertLess(g.polyline_length(rebuilt), g.polyline_length(points) * 1.2)

    def test_sparse_corner_is_not_fitted_as_one_arc(self):
        # 3 points on a right angle lie exactly on a circle (semicircle) -
        # the fit must reject it: the arc bulges far from the drawn legs.
        """Sparse corner is not fitted as one arc."""
        points = [(600.0, 600.0), (1600.0, 600.0), (1600.0, 1600.0)]
        primitives = g.segment_line_arc(points, tol=8.0)
        self.assertTrue(all(p["type"] == "line" for p in primitives), primitives)

    def test_jittery_arc_is_corrected_and_keeps_its_endpoint(self):
        # a hand-drawn arc: +-5 mm of jitter plus a badly jittered last point.
        # The interior jitter is corrected (the primitives hug the circle),
        # but the stroke must still FINISH on the drawn endpoint - the old
        # fit silently trimmed the stray tail off the drawing.
        """Jittery arc is corrected and keeps its endpoint."""
        center, radius = (2000.0, 900.0), 600.0
        base = arc_points(center[0], center[1], radius, 20.0, 110.0, 60)
        points = []
        for i, (x, y) in enumerate(base):
            jitter = math.sin(i * 2.7) * 5.0
            direction = math.atan2(y - center[1], x - center[0]) + math.pi / 2.0
            points.append((x + math.cos(direction) * jitter, y + math.sin(direction) * jitter))
        points[-1] = (points[-1][0] + 30.0, points[-1][1] + 30.0)  # stray endpoint
        primitives = g.segment_line_arc(points, tol=8.0)
        arcs = [p for p in primitives if p["type"] == "arc"]
        self.assertGreaterEqual(len(arcs), 1, primitives)
        for arc in arcs:
            self.assertAlmostEqual(arc["r"], radius, delta=20.0)
        sweep = sum(arc["sweep_deg"] for arc in arcs)
        self.assertAlmostEqual(abs(sweep), 90.0, delta=8.0)
        # honest ends: the chain terminates on the drawn endpoint
        self.assertAlmostEqual(math.dist(primitive_end(primitives[-1]), points[-1]), 0.0, delta=1.0)

    def test_long_real_stroke_fits_cleanly(self):
        # a drawn arc + long straight + gentle end: must become a handful of
        # primitives in drawing order (used to be dozens of tiny lines, and
        # the splitter used to cycle on it)
        """Long real stroke fits cleanly."""
        points = STROKE_LONG_REGRESSION
        primitives = g.segment_line_arc(points, tol=8.0)
        self.assertGreaterEqual(len(primitives), 3)
        self.assertLessEqual(len(primitives), 8)
        self.assertTrue(any(p["type"] == "arc" for p in primitives))
        starts = [primitive_start(p) for p in primitives]
        ends = [primitive_end(p) for p in primitives]
        self.assertLess(math.dist(ends[-1], points[-1]), 8.0)
        for previous_end, next_start in zip(ends, starts[1:]):
            self.assertLess(math.dist(previous_end, next_start), 8.0)
        rebuilt = rebuild(points, primitives)
        self.assertLess(g.polyline_length(rebuilt), g.polyline_length(points) * 1.1)

    def test_straight_stroke_stays_a_line(self):
        """Straight stroke stays a line."""
        points = [(100.0 + i * 25.0, 500.0 + math.sin(i * 1.3) * 4.0) for i in range(40)]
        primitives = g.segment_line_arc(points, tol=8.0)
        self.assertTrue(all(p["type"] == "line" for p in primitives), primitives)

    def test_arc_to_points_endpoints(self):
        """Arc to points endpoints."""
        primitives = g.segment_line_arc(arc_points(0.0, 0.0, 100.0, 0.0, 90.0), tol=4.0)
        arc = primitives[0]
        points = g.arc_to_points(arc, max_step_deg=15.0)
        start = g.point_at_angle(arc["center"], arc["r"], arc["a_start_deg"])
        end = g.point_at_angle(arc["center"], arc["r"], arc["a_end_deg"])
        self.assertAlmostEqual(math.dist(points[0], start), 0.0, delta=0.5)
        self.assertAlmostEqual(math.dist(points[-1], end), 0.0, delta=0.5)

    def test_resample_spacing(self):
        """Resample spacing."""
        points = [(i * 100.0, 0.0) for i in range(11)]  # 1000 mm straight
        samples = g.resample(points, 100.0)
        self.assertEqual(len(samples), 11)
        for a, b in zip(samples, samples[1:]):
            self.assertAlmostEqual(math.dist(a, b), 100.0, delta=1.0)

    def test_catmull_rom_keeps_endpoints(self):
        """Catmull rom keeps endpoints."""
        points = [(0.0, 0.0), (100.0, 50.0), (200.0, 0.0)]
        dense = g.catmull_rom(points, 8)
        self.assertEqual(dense[0], points[0])
        self.assertEqual(dense[-1], points[-1])
        self.assertGreater(len(dense), len(points))

    def test_headings(self):
        """Headings."""
        headings = g.point_headings([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)])
        self.assertAlmostEqual(headings[0], 0.0)
        self.assertAlmostEqual(headings[1], 90.0)
        self.assertAlmostEqual(headings[2], 90.0)


class PathResampleTests(unittest.TestCase):
    """Node economy: limits must be local to the geometry they measure."""

    @staticmethod
    def _bend_with_straight():
        # 1265 mm straight, then a smooth 90 deg bend of r = 95 mm
        """Bend with straight."""
        straight = [(0.0, 0.0), (1265.0, 0.0)]
        arc = [g.point_at_angle((1265.0, 95.0), 95.0, -90.0 + 90.0 * i / 24) for i in range(25)]
        return straight + arc[1:]

    def test_straight_before_a_bend_keeps_the_base_step(self):
        # a window-based radius estimate bled the bend's small radius backwards
        # and filled the whole straight with 3-80 mm nodes; every limit is now
        # measured on the chord itself, so the straight keeps 250 mm steps
        """Straight before a bend keeps the base step."""
        samples = g.path_resample(self._bend_with_straight(), 250.0, 6.0, 15.0, 40.0)
        straight_xs = sorted(round(x) for x, y in samples if abs(y) < 1.0 and x < 1265.0)
        self.assertEqual(straight_xs, [0, 250, 500, 750, 1000, 1250])
        steps = [math.dist(a, b) for a, b in zip(samples, samples[1:])]
        self.assertGreaterEqual(min(steps), 3.0, "micro-step crawl")
        self.assertLessEqual(len(samples), 16, "the bend must not bloat the node count")

    def test_nodes_on_a_bend_turn_within_the_cap(self):
        """Nodes on a bend turn within the cap."""
        samples = g.path_resample(self._bend_with_straight(), 250.0, 6.0, 15.0, 40.0)
        turns = []
        for a, b, c in zip(samples, samples[1:], samples[2:]):
            h1 = math.atan2(b[1] - a[1], b[0] - a[0])
            h2 = math.atan2(c[1] - b[1], c[0] - b[0])
            turns.append(abs(math.degrees(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1)))))
        self.assertLessEqual(max(turns), 16.0, "the 90 deg bend must be swept, not pivoted")
        # the bend is sampled densely (a chain of small turns), the straight is not
        on_bend = [p for p in samples if p[0] > 1265.0 or p[1] > 1.0]
        self.assertGreaterEqual(len(on_bend), 5)

    def test_arc_nodes_stay_on_the_arc(self):
        """Arc nodes stay on the arc."""
        center = (2000.0, 1000.0)
        pts = [g.point_at_angle(center, 400.0, 10.0 + 90.0 * i / 40) for i in range(41)]
        samples = g.path_resample(pts, 250.0, 6.0, 15.0, 40.0)
        self.assertGreaterEqual(len(samples), 5)
        worst = max(abs(math.dist(p, center) - 400.0) for p in samples)
        self.assertLess(worst, 3.0, "every node must stay on the arc (chord tolerance)")


if __name__ == "__main__":
    unittest.main()
