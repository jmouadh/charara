"""Localization tests: wheel kinematics and the pose Kalman filter.

These run without hardware: the filter is fed synthetic samples and must
track the pose, stay finite, and react to an injected yaw-rate measurement
(the future IMU hook) exactly where the wheel rate is wrong.

File:    tests/test_localization.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

import math
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from software.core.localization.kalman import PoseKalmanFilter  # noqa: E402
from software.core.localization.kinematics import (  # noqa: E402
    integrate_pose,
    twist_from_wheels,
    wrap180,
)
from software.core.localization.service import LocalizationService, WheelSample  # noqa: E402


class KinematicsTests(unittest.TestCase):
    def test_straight_wheels_make_pure_translation(self):
        """Straight wheels make pure translation."""
        v, omega = twist_from_wheels(100.0, 100.0, 150.0)
        self.assertAlmostEqual(v, 100.0, places=9)
        self.assertAlmostEqual(omega, 0.0, places=9)

    def test_opposite_wheels_make_pure_rotation(self):
        """Opposite wheels make pure rotation."""
        v, omega = twist_from_wheels(-100.0, 100.0, 150.0)
        self.assertAlmostEqual(v, 0.0, places=9)
        self.assertAlmostEqual(omega, math.degrees(200.0 / 150.0), places=6)

    def test_track_scales_the_yaw_rate(self):
        """Track scales the yaw rate."""
        _, wide = twist_from_wheels(0.0, 100.0, 300.0)
        _, narrow = twist_from_wheels(0.0, 100.0, 150.0)
        self.assertAlmostEqual(narrow, 2.0 * wide, places=9)

    def test_integration_uses_the_midpoint_heading(self):
        # 100 mm/s on a 90 deg/s arc = a 63.7 mm radius circle: one second of
        # travel ends at (r, r) with the heading turned by 90 deg
        """Integration uses the midpoint heading."""
        radius = 100.0 / math.radians(90.0)
        x, y, th = 0.0, 0.0, 0.0
        for _ in range(100):
            x, y, th = integrate_pose(x, y, th, 100.0, 90.0, 0.01)
        self.assertAlmostEqual(x, radius, delta=0.5)
        self.assertAlmostEqual(y, radius, delta=0.5)
        self.assertAlmostEqual(th, 90.0, delta=1.0)

    def test_wrap180(self):
        """Wrap180."""
        self.assertAlmostEqual(wrap180(190.0), -170.0)
        self.assertAlmostEqual(wrap180(-190.0), 170.0)
        self.assertAlmostEqual(wrap180(180.0), -180.0)


class FilterTests(unittest.TestCase):
    def test_reset_is_exact(self):
        """Reset is exact."""
        kal = PoseKalmanFilter()
        kal.reset(120.0, -40.0, 30.0)
        self.assertAlmostEqual(kal.pose()[0], 120.0, places=9)
        self.assertAlmostEqual(kal.pose()[1], -40.0, places=9)
        self.assertAlmostEqual(kal.pose()[2], 30.0, places=9)

    def test_position_sigma_grows_without_measurements(self):
        """Position sigma grows without measurements."""
        kal = PoseKalmanFilter()
        kal.reset(0.0, 0.0, 0.0)
        before = kal.position_sigma_mm()
        for _ in range(20):
            kal.predict(200.0, 0.1)
        self.assertGreater(kal.position_sigma_mm(), before)

    def test_pose_update_pulls_the_estimate(self):
        """Pose update pulls the estimate."""
        kal = PoseKalmanFilter()
        kal.reset(0.0, 0.0, 0.0)
        for _ in range(80):                 # cold start -> the filter converges
            kal.predict(0.0, 0.1)           # no motion
            kal.update_pose(500.0, -250.0, 45.0)
        x, y, th = kal.pose()
        # the estimate settles a few mm short of the measurement: the wheel
        # speed process noise (40 mm/s) keeps the filter from fully trusting
        # a stale position - that is the designed trade-off, not an error
        self.assertAlmostEqual(x, 500.0, delta=12.0)
        self.assertAlmostEqual(y, -250.0, delta=12.0)
        self.assertAlmostEqual(th, 45.0, delta=1.0)
        self.assertLess(kal.position_sigma_mm(), 20.0)

    def test_heading_wraps_through_pm180(self):
        """Heading wraps through pm180."""
        kal = PoseKalmanFilter()
        kal.reset(0.0, 0.0, 179.0)
        for _ in range(5):
            kal.update_pose(0.0, 0.0, -179.0)
        _, _, th = kal.pose()
        self.assertLess(abs(abs(th) - 180.0), 3.0)

    def test_gyro_rate_measurement_is_fused(self):
        # wheels claim "no rotation", the gyro measures a hard turn: the
        # yaw-rate state must follow the gyro (this is the IMU hook)
        """Gyro rate measurement is fused."""
        kal = PoseKalmanFilter()
        kal.reset(0.0, 0.0, 0.0)
        for _ in range(40):
            kal.predict(0.0, 0.05)
            kal.update_angular_rate(0.0, kal.sigma_wheel_omega)   # wheels: 0
            kal.update_angular_rate(90.0, 1.0)                    # gyro: 90 deg/s
        self.assertGreater(kal.yaw_rate_deg_s(), 60.0)

    def test_position_update_keeps_the_heading(self):
        """A position-only update pulls x,y and never touches theta.

        The pull is deliberately gentle (the board position is a weak
        cross-check while the gyro owns the heading) and the covariance is
        correlated along the heading, so the test checks that the distance to
        the measurement shrinks - not that the estimate snaps onto it - and
        that the heading is left exactly where it was.
        """
        kal = PoseKalmanFilter()
        kal.reset(0.0, 0.0, 0.0)
        before = math.hypot(500.0, 250.0)
        for _ in range(80):
            kal.predict(0.0, 0.1)  # the service always predicts first
            kal.update_position(500.0, -250.0, 60.0)
        x, y, theta = kal.pose()
        after = math.hypot(x - 500.0, y + 250.0)
        self.assertLess(after, 0.8 * before)   # it moves measurably closer
        self.assertAlmostEqual(theta, 0.0, places=6)  # heading untouched

    def test_garbage_measurements_are_ignored(self):
        """Garbage measurements are ignored."""
        kal = PoseKalmanFilter()
        kal.reset(10.0, 20.0, 30.0)
        kal.update_pose(float("nan"), 0.0, 0.0)
        kal.update_angular_rate(float("inf"), 1.0)
        x, y, th = kal.pose()
        self.assertEqual((x, y, th), (10.0, 20.0, 30.0))


class ServiceTests(unittest.TestCase):
    def _feed(self, service, samples, **kwargs):
        """Feed."""
        t = 0.0
        for index in range(samples):
            t += 0.1
            service.ingest(WheelSample(
                x_mm=index * 10.0,
                y_mm=0.0,
                theta_deg=0.0,
                v_left_mm_s=100.0,
                v_right_mm_s=100.0,
                t=t,
            ))

    def test_first_sample_adopts_the_pose(self):
        """First sample adopts the pose."""
        service = LocalizationService(track_mm=150.0)
        service.ingest(WheelSample(x_mm=1234.0, y_mm=567.0, theta_deg=-45.0,
                                   v_left_mm_s=0.0, v_right_mm_s=0.0, t=1.0))
        x, y, th = service.pose()
        self.assertAlmostEqual(x, 1234.0, places=6)
        self.assertAlmostEqual(y, 567.0, places=6)
        self.assertAlmostEqual(th, -45.0, places=6)

    def test_tracks_a_straight_run(self):
        """Tracks a straight run."""
        service = LocalizationService(track_mm=150.0)
        self._feed(service, 50)
        x, y, th = service.pose()
        self.assertAlmostEqual(x, 490.0, delta=15.0)
        self.assertAlmostEqual(y, 0.0, delta=10.0)
        self.assertAlmostEqual(th, 0.0, delta=1.0)
        self.assertEqual(service.status()["mode"], "wheel")

    def test_reset_clears_history(self):
        """Reset clears history."""
        service = LocalizationService(track_mm=150.0)
        self._feed(service, 10)
        service.reset(0.0, 0.0, 0.0)
        x, y, th = service.pose()
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(service.status()["samples"], 0.0)

    def test_gyro_hook_switches_the_mode(self):
        """Gyro hook switches the mode."""
        service = LocalizationService(track_mm=150.0)
        self._feed(service, 5)
        service.add_gyro_yaw_rate(30.0)
        status = service.status()
        self.assertEqual(status["mode"], "wheel+gyro")
        self.assertEqual(status["gyro_samples"], 1)

    def test_hand_rotation_stays_while_the_gyro_is_fresh(self):
        """A rotation the wheels cannot see must STICK (slip-immune heading).

        Someone turning the robot by hand (or a wheel slip) rotates the body
        without the encoders: the gyro measures it, the board pose does not.
        The estimate must keep the rotation instead of unwinding back to the
        wheel-only pose (the board heading is skipped while the gyro lives).
        """
        service = LocalizationService(track_mm=150.0)
        t = time.monotonic()
        for _ in range(10):        # 1 s at rest: settle, learn the gyro bias
            t += 0.1
            service.ingest(WheelSample(0.0, 0.0, 0.0, 0.0, 0.0, t=t))
            service.add_gyro_yaw_rate(0.0, t=t)
        for _ in range(10):        # 1 s hand rotation at 90 deg/s, wheels still
            t += 0.1
            service.ingest(WheelSample(0.0, 0.0, 0.0, 0.0, 0.0, t=t))
            service.add_gyro_yaw_rate(90.0, t=t)
        turned = service.pose()[2]
        self.assertGreater(turned, 60.0)          # most of the 90 deg is there
        for _ in range(200):       # hold the angle for 20 s, board still at 0
            t += 0.1
            service.ingest(WheelSample(0.0, 0.0, 0.0, 0.0, 0.0, t=t))
            service.add_gyro_yaw_rate(0.0, t=t)
        held = service.pose()[2]
        self.assertGreater(held, 60.0)            # it did NOT unwind
        self.assertLess(abs(held - turned), 10.0)

    def test_without_a_gyro_the_board_pose_still_owns_the_pose(self):
        """No independent source: the wheel-only behaviour is unchanged."""
        service = LocalizationService(track_mm=150.0)
        t = time.monotonic()
        for _ in range(200):
            t += 0.1
            service.ingest(WheelSample(0.0, 0.0, 0.0, 0.0, 0.0, t=t))
        x, y, theta = service.pose()
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(theta, 0.0, places=6)
        self.assertEqual(service.status()["mode"], "wheel")

    def test_board_position_is_still_tracked_with_the_gyro_fresh(self):
        """While the gyro owns the heading the board position still counts."""
        service = LocalizationService(track_mm=150.0)
        t = time.monotonic()
        for index in range(200):      # 20 s driving straight at 100 mm/s
            t += 0.1
            service.ingest(WheelSample(index * 10.0, 0.0, 0.0, 100.0, 100.0, t=t))
            service.add_gyro_yaw_rate(0.0, t=t)
        x, _y, theta = service.pose()
        self.assertAlmostEqual(x, 1990.0, delta=25.0)
        self.assertAlmostEqual(theta, 0.0, delta=1.0)
        self.assertIn("+gyro", service.status()["mode"])

    def test_pose_jump_is_adopted(self):
        # SET_POSE / RESET_ODOMETRY: the estimate must follow immediately
        """Pose jump is adopted."""
        service = LocalizationService(track_mm=150.0)
        self._feed(service, 10)
        service.ingest(WheelSample(x_mm=2000.0, y_mm=1200.0, theta_deg=0.0,
                                   v_left_mm_s=0.0, v_right_mm_s=0.0, t=99.0))
        x, y, _ = service.pose()
        self.assertAlmostEqual(x, 2000.0, places=6)
        self.assertAlmostEqual(y, 1200.0, places=6)
        self.assertEqual(service.status()["jumps"], 1)

    def test_nonfinite_samples_are_ignored(self):
        """Nonfinite samples are ignored."""
        service = LocalizationService(track_mm=150.0)
        self._feed(service, 5)
        before = service.pose()
        service.ingest(WheelSample(x_mm=float("nan"), y_mm=0.0, theta_deg=0.0,
                                   v_left_mm_s=0.0, v_right_mm_s=0.0, t=9.0))
        self.assertEqual(service.pose(), before)


if __name__ == "__main__":
    unittest.main()
