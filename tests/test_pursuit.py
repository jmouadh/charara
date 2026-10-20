"""Tests for the pure-pursuit path follower and its simulator (Track B).

Covers the geometry helpers, the corner cases (path overrun, arrival) and
the Track B acceptance numbers of Docs/10 section 6.3: tracking error at
cruise speed on a circle, a filleted course and an S-curve - all measured
by the deterministic kinematic simulator (no robot needed).

File:    tests/test_pursuit.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-09
"""

from __future__ import annotations

import math
import unittest

from software.core.navigation.pursuit import (
    DEFAULT_PARAMS,
    PursuitParams,
    distance_to_path,
    lookahead_distance,
    lookahead_target,
    path_length,
    pursuit_curvature,
    pursuit_step,
    remaining_distance,
    resample,
    speed_for_curvature,
)
from software.core.navigation.sim import KinematicSim, polyline_distance, simulate_path
from software.core.navigation.tracking import DEFAULT_GAINS, ramsete_command


def circle_path(cx: float, cy: float, radius: float, samples: int = 720) -> list:
    """CCW circle sampled every ``samples`` points (closing loop)."""
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / samples),
            cy + radius * math.sin(2.0 * math.pi * i / samples),
        )
        for i in range(samples + 1)
    ]


class GeometryTests(unittest.TestCase):
    """Lookahead, projection and remaining-distance helpers."""

    def test_path_length_of_a_straight(self):
        """A straight polyline measures the sum of its segments."""
        points = [(0.0, 0.0), (300.0, 0.0), (300.0, 400.0)]
        self.assertAlmostEqual(path_length(points), 700.0, delta=1e-9)

    def test_lookahead_walks_a_straight_line(self):
        """From (200, 0) a 300 mm lookahead lands exactly on (500, 0)."""
        points = [(0.0, 0.0), (1000.0, 0.0)]
        target = lookahead_target(points, (200.0, 0.0, 0.0), 300.0)
        self.assertAlmostEqual(target[0], 500.0, delta=1e-6)
        self.assertAlmostEqual(target[1], 0.0, delta=1e-6)

    def test_lookahead_clamps_to_the_path_end(self):
        """A lookahead longer than the path clamps to the last point."""
        points = [(0.0, 0.0), (300.0, 0.0)]
        target = lookahead_target(points, (0.0, 0.0, 0.0), 10000.0)
        self.assertEqual(target, points[-1])

    def test_lookahead_distance_follows_the_speed_with_bounds(self):
        """L = clamp(k_l * v, L_min, L_max)."""
        self.assertAlmostEqual(lookahead_distance(0.0), DEFAULT_PARAMS.l_min, delta=1e-9)
        self.assertAlmostEqual(lookahead_distance(100.0), 100.0, delta=1e-9)
        self.assertAlmostEqual(lookahead_distance(2000.0), DEFAULT_PARAMS.l_max, delta=1e-9)

    def test_remaining_distance_shrinks_along_the_path(self):
        """Remaining distance counts the path length behind the robot."""
        points = circle_path(0.0, 0.0, 300.0, 360)
        full = path_length(points)
        start = (300.0, 0.0)
        self.assertAlmostEqual(remaining_distance(points, start), full, delta=2.0)
        half = (-300.0, 0.0)  # opposite side of the CCW circle = half walked
        self.assertAlmostEqual(remaining_distance(points, half), full / 2.0, delta=4.0)

    def test_curvature_of_an_on_axis_target_is_zero(self):
        """A target straight ahead needs no turning."""
        kappa, dist = pursuit_curvature((0.0, 0.0, 0.0), (100.0, 0.0))
        self.assertAlmostEqual(kappa, 0.0, delta=1e-12)
        self.assertAlmostEqual(dist, 100.0, delta=1e-9)

    def test_curvature_of_a_diagonal_target(self):
        """Target (100, 100) at 45 deg gives kappa = 1/100 (radius 100 mm)."""
        kappa, _ = pursuit_curvature((0.0, 0.0, 0.0), (100.0, 100.0))
        self.assertAlmostEqual(kappa, 0.01, delta=1e-9)

    def test_speed_for_curvature_respects_the_budget(self):
        """v = sqrt(a_lat / |kappa|) capped by the cruise speed and floor."""
        self.assertAlmostEqual(speed_for_curvature(0.0, 500.0), 500.0, delta=1e-9)
        # kappa = 1/300 -> v = sqrt(300 * 300) = 300 mm/s
        self.assertAlmostEqual(speed_for_curvature(1.0 / 300.0, 800.0), 300.0, delta=1e-6)
        # a hairpin falls back to the minimum speed, never below
        self.assertAlmostEqual(
            speed_for_curvature(1.0, 800.0), DEFAULT_PARAMS.v_min, delta=1e-9
        )

    def test_resample_keeps_geometry_and_spacing(self):
        """Resampling a straight keeps it straight and evenly spaced."""
        points = resample([(0.0, 0.0), (1000.0, 0.0)], 100.0)
        self.assertEqual(len(points), 11)
        self.assertAlmostEqual(points[-1][0], 1000.0, delta=1e-6)
        for i in range(len(points) - 1):
            self.assertAlmostEqual(math.dist(points[i], points[i + 1]), 100.0, delta=1e-6)


class PursuitStepTests(unittest.TestCase):
    """One-step controller behaviour (arrival, braking, heading)."""

    def test_step_returns_none_at_the_path_end(self):
        """Closer than stop_tol_mm to the last point = done, no command."""
        points = [(0.0, 0.0), (500.0, 0.0)]
        self.assertIsNone(pursuit_step(points, (490.0, 0.0, 0.0), 500.0))

    def test_speed_brakes_near_the_end(self):
        """The braking profile slows the command before the final stop."""
        points = [(0.0, 0.0), (1000.0, 0.0)]
        near = pursuit_step(points, (900.0, 0.0, 0.0), 500.0)
        far = pursuit_step(points, (100.0, 0.0, 0.0), 500.0)
        self.assertIsNotNone(near)
        self.assertIsNotNone(far)
        # remaining 100 mm: sqrt(2 * 700 * 100) = 374 mm/s < 500
        self.assertLess(near.v_mm_s, far.v_mm_s)
        self.assertAlmostEqual(near.v_mm_s, math.sqrt(2.0 * 700.0 * 100.0), delta=1e-6)

    def test_step_turns_towards_an_offset_lookahead(self):
        """A target left of the heading commands a positive yaw rate."""
        points = [(0.0, 0.0), (0.0, 1000.0)]  # path goes +y, robot faces +x
        cmd = pursuit_step(points, (0.0, 100.0, 0.0), 300.0)
        self.assertIsNotNone(cmd)
        self.assertGreater(cmd.omega_rad_s, 0.0)
        self.assertGreater(cmd.kappa, 0.0)

    def test_target_behind_commands_a_turn_in_place(self):
        """A lookahead behind the robot must rotate, never drive forward.

        Real-robot bug (2026-10-18): the plan started behind the robot; pure
        pursuit computed a tiny curvature with full speed and drove straight
        out of the field. The step must become v=0 and a real yaw rate.
        """
        points = [(0.0, 0.0), (0.0, -1000.0)]  # path -y, robot faces +x
        cmd = pursuit_step(points, (0.0, -100.0, 0.0), 300.0)
        self.assertIsNotNone(cmd)
        self.assertTrue(cmd.turn_only)
        self.assertEqual(cmd.v_mm_s, 0.0)
        self.assertGreater(abs(cmd.omega_rad_s), 0.3)

    def test_distance_to_path_is_the_nearest_segment(self):
        """A point 50 mm off a polyline is 50 mm from it."""
        points = [(0.0, 0.0), (1000.0, 0.0), (1000.0, 500.0)]
        self.assertAlmostEqual(distance_to_path(points, (500.0, 50.0, 0.0)), 50.0, delta=1e-9)
        self.assertAlmostEqual(distance_to_path(points, (1040.0, 250.0, 0.0)), 40.0, delta=1e-9)


class RamseteTests(unittest.TestCase):
    """The optional RAMSETE tracking law (pure math)."""

    def test_zero_error_follows_the_reference_exactly(self):
        """On the reference, the command IS the reference twist."""
        pose = (100.0, 200.0, 30.0)
        v, omega = ramsete_command(pose, pose, 400.0, 0.5, DEFAULT_GAINS)
        self.assertAlmostEqual(v, 400.0, delta=1e-9)
        self.assertAlmostEqual(omega, 0.5, delta=1e-9)

    def test_lateral_error_steers_back(self):
        """A robot right of the reference (facing +x) must turn left."""
        v, omega = ramsete_command(
            (0.0, -50.0, 0.0), (0.0, 0.0, 0.0), 300.0, 0.0, DEFAULT_GAINS
        )
        self.assertGreater(omega, 0.0)  # +y error needs a CCW correction

    def test_heading_error_slows_the_robot(self):
        """Facing 90 deg off the reference, the forward speed drops."""
        v, _ = ramsete_command(
            (0.0, 0.0, 90.0), (0.0, 0.0, 0.0), 400.0, 0.0, DEFAULT_GAINS
        )
        self.assertAlmostEqual(v, 0.0, delta=1e-9)  # cos(90 deg) = 0

    def test_converges_onto_a_circle_reference(self):
        """Starting 100 mm off a circular reference, RAMSETE converges."""
        radius = 400.0
        v_ref = 300.0
        omega_ref = v_ref / radius
        sim = KinematicSim(radius - 100.0, 0.0, 90.0)
        t = 0.0
        dt = 0.02
        while t < 25.0:
            ang = omega_ref * t
            ref = (
                radius * math.cos(ang),
                radius * math.sin(ang),
                math.degrees(ang) + 90.0,
            )
            v, omega = ramsete_command(sim.pose(), ref, v_ref, omega_ref, DEFAULT_GAINS)
            sim.step(v, omega, dt)
            t += dt
        ang = omega_ref * t
        ref_x = radius * math.cos(ang)
        ref_y = radius * math.sin(ang)
        error = math.hypot(sim.x_mm - ref_x, sim.y_mm - ref_y)
        self.assertLess(error, 30.0)


class SimulatorAcceptanceTests(unittest.TestCase):
    """Track B acceptance numbers (Docs/10 section 6.3)."""

    def test_straight_path_is_tracked_tightly(self):
        """A straight run stays within a millimetre and arrives stopped."""
        result = simulate_path([(0.0, 0.0), (1500.0, 0.0)], 500.0)
        self.assertTrue(result["done"])
        self.assertLess(result["max_dev_mm"], 1.0)
        self.assertLess(result["final_error_mm"], DEFAULT_PARAMS.stop_tol_mm)

    def test_circle_tracking_within_15_mm(self):
        """A 300 mm radius circle at 500 mm/s cruise stays within 15 mm."""
        points = circle_path(0.0, 0.0, 300.0)
        result = simulate_path(points, 500.0, start=(300.0, 0.0, 90.0))
        self.assertTrue(result["done"])
        self.assertLessEqual(result["max_dev_mm"], 15.0)

    def test_filleted_course_within_15_mm(self):
        """Straight -> R=300 left fillet -> straight, tracked within 15 mm."""
        points = [(0.0, 0.0), (400.0, 0.0)]
        # fillet: centre (400, 300), from angle -90 deg sweeping +90 (CCW)
        for i in range(1, 91):
            ang = -math.pi / 2.0 + (math.pi / 2.0) * i / 90.0
            points.append((400.0 + 300.0 * math.cos(ang), 300.0 + 300.0 * math.sin(ang)))
        points.append((700.0, 700.0))
        result = simulate_path(points, 500.0)
        self.assertTrue(result["done"])
        self.assertLessEqual(result["max_dev_mm"], 15.0)

    def test_s_curve_within_15_mm(self):
        """A sine S-curve is tracked within 15 mm at cruise speed."""
        points = [
            (x, 150.0 * math.sin(2.0 * math.pi * x / 1200.0))
            for x in range(0, 1201, 5)
        ]
        result = simulate_path(points, 500.0)
        self.assertTrue(result["done"])
        self.assertLessEqual(result["max_dev_mm"], 15.0)

    def test_polyline_distance_of_a_point_off_the_line(self):
        """A point 50 mm above a straight measures 50 mm to the polyline."""
        self.assertAlmostEqual(
            polyline_distance([(0.0, 0.0), (1000.0, 0.0)], 500.0, 50.0), 50.0, delta=1e-9
        )

    def test_params_are_frozen(self):
        """PursuitParams is a frozen dataclass (safe to share as default)."""
        params = PursuitParams()
        with self.assertRaises(Exception):
            params.k_l = 1.0  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main(verbosity=2)
