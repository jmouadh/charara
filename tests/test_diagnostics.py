"""Diagnostics tests: self-test geometry and statistics (pure math).

Covered: the square waypoints (corners, snap back to the start, headings),
the arc end pose in both directions (the STM_docs/06 section 2.6 quarter-arc
examples plus a full circle), pose error wrap-around, flat-arc sweeps on the
0.1 deg wire quantum, drift statistics (static, creep, heading wrap) and the
series statistics used by the rest-drift audit.

File:    tests/test_diagnostics.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-09
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from software.core import diagnostics  # noqa: E402


class SquareWaypointTests(unittest.TestCase):
    """The four GOTO targets of the bench square test."""

    def test_square_from_the_origin(self):
        """Corners follow the bench square and return to the exact start."""
        waypoints = diagnostics.square_waypoints(0.0, 0.0, 0.0, 500.0)
        self.assertEqual(len(waypoints), 4)
        self.assertAlmostEqual(waypoints[0]["x"], 500.0, places=6)
        self.assertAlmostEqual(waypoints[0]["y"], 0.0, places=6)
        self.assertAlmostEqual(waypoints[0]["th"], 0.0, places=6)
        self.assertAlmostEqual(waypoints[1]["x"], 500.0, places=6)
        self.assertAlmostEqual(waypoints[1]["y"], 500.0, places=6)
        self.assertAlmostEqual(waypoints[1]["th"], 90.0, places=6)
        self.assertAlmostEqual(waypoints[2]["x"], 0.0, places=6)
        self.assertAlmostEqual(waypoints[2]["y"], 500.0, places=6)
        self.assertAlmostEqual(waypoints[2]["th"], -180.0, places=6)  # 180 wraps to -180
        self.assertAlmostEqual(waypoints[3]["x"], 0.0, places=6)
        self.assertAlmostEqual(waypoints[3]["y"], 0.0, places=6)
        self.assertAlmostEqual(waypoints[3]["th"], -90.0, places=6)  # 270 wrapped

    def test_square_rotates_with_the_start_heading(self):
        """A square drawn from a rotated pose stays in the robot's frame."""
        waypoints = diagnostics.square_waypoints(1000.0, 1000.0, 90.0, 200.0)
        self.assertAlmostEqual(waypoints[0]["x"], 1000.0, places=6)
        self.assertAlmostEqual(waypoints[0]["y"], 1200.0, places=6)
        self.assertAlmostEqual(waypoints[1]["x"], 800.0, places=6)
        self.assertAlmostEqual(waypoints[2]["x"], 800.0, places=6)
        self.assertAlmostEqual(waypoints[2]["y"], 1000.0, places=6)
        self.assertAlmostEqual(waypoints[3]["x"], 1000.0, places=6)
        self.assertAlmostEqual(waypoints[3]["y"], 1000.0, places=6)
        self.assertAlmostEqual(waypoints[3]["th"], 0.0, places=6)  # 360 wrapped


class ArcEndPoseTests(unittest.TestCase):
    """Exact end poses of one firmware ARC."""

    def test_bench_quarter_arcs(self):
        """The STM_docs/06 examples: +300/+90 and -300/-90 from (0,0,0)."""
        left = diagnostics.arc_end_pose(0.0, 0.0, 0.0, 300.0, 90.0)
        self.assertAlmostEqual(left[0], 300.0, places=6)
        self.assertAlmostEqual(left[1], 300.0, places=6)
        self.assertAlmostEqual(left[2], 90.0, places=6)
        right = diagnostics.arc_end_pose(0.0, 0.0, 0.0, -300.0, -90.0)
        self.assertAlmostEqual(right[0], 300.0, places=6)
        self.assertAlmostEqual(right[1], -300.0, places=6)
        self.assertAlmostEqual(right[2], -90.0, places=6)

    def test_full_circle_returns_to_start(self):
        """A full sweep closes the circle (heading wraps back)."""
        end = diagnostics.arc_end_pose(500.0, 400.0, 30.0, 250.0, 360.0)
        self.assertAlmostEqual(end[0], 500.0, places=6)
        self.assertAlmostEqual(end[1], 400.0, places=6)
        self.assertAlmostEqual(end[2], 30.0, places=6)


class PoseErrorTests(unittest.TestCase):
    """Pose error helper."""

    def test_heading_error_wraps(self):
        """A -179 -> +179 flip is a 2 deg error, not 358."""
        error = diagnostics.pose_error((10.0, 10.0, -179.0), (10.0, 10.0, 179.0))
        self.assertAlmostEqual(error["pos_mm"], 0.0, places=6)
        self.assertAlmostEqual(error["heading_deg"], 2.0, places=6)


class FlatSweepTests(unittest.TestCase):
    """Near-straight ARC sweeps for the chain audit."""

    def test_wire_quantum_and_floor(self):
        """Flat sweeps snap to the 0.1 deg quantum with a 0.1 deg floor."""
        self.assertAlmostEqual(diagnostics.flat_sweep_deg(300.0, 20000.0), 0.9, places=6)
        self.assertAlmostEqual(diagnostics.flat_sweep_deg(5.0, 20000.0), 0.1, places=6)


class DriftStatsTests(unittest.TestCase):
    """Drift statistics used by the rest-drift audit."""

    def test_static_series_reports_no_drift(self):
        """A perfectly still series has zero drift (and a span)."""
        samples = [(index * 0.1, 100.0, 200.0, 45.0) for index in range(11)]
        stats = diagnostics.drift_stats(samples)
        self.assertEqual(stats["samples"], 11)
        self.assertAlmostEqual(stats["span_s"], 1.0, places=6)
        self.assertAlmostEqual(stats["max_pos_mm"], 0.0, places=6)
        self.assertAlmostEqual(stats["max_heading_deg"], 0.0, places=6)

    def test_creep_and_heading_wrap_are_measured(self):
        """Drift is measured against the first sample, headings wrapped."""
        samples = [
            (0.0, 0.0, 0.0, 179.0),
            (1.0, 3.0, 4.0, -179.0),
            (2.0, 0.5, 0.0, 179.5),
        ]
        stats = diagnostics.drift_stats(samples)
        self.assertAlmostEqual(stats["max_pos_mm"], 5.0, places=6)
        self.assertAlmostEqual(stats["max_heading_deg"], 2.0, places=6)


class SeriesStatsTests(unittest.TestCase):
    """Mean/sigma helper used for the gyro noise readout."""

    def test_mean_and_sigma(self):
        """Mean and sample sigma of a short series."""
        stats = diagnostics.series_stats([1.0, 2.0, 3.0])
        self.assertEqual(stats["count"], 3)
        self.assertAlmostEqual(stats["mean"], 2.0, places=6)
        self.assertAlmostEqual(stats["sigma"], 1.0, places=6)

    def test_empty_series_is_zero(self):
        """An empty series reports zero without dividing by zero."""
        stats = diagnostics.series_stats([])
        self.assertEqual(stats["count"], 0)
        self.assertAlmostEqual(stats["sigma"], 0.0, places=6)
