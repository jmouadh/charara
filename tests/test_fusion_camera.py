"""Sensor fusion tests: camera gating, latency, board-pose policy, gyro.

These tests exercise the pure localization/fusion layer and the session
ingestion path; no board, no camera process, no IMU hardware is needed.

File:    tests/test_fusion_camera.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-05
"""

import math
import os
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ.setdefault("ROBOT_TRANSPORT", "mock")

from software.config import settings  # noqa: E402
from software.core.localization import LocalizationService, WheelSample  # noqa: E402
from software.session import RobotSession  # noqa: E402


def wheel(x=0.0, y=0.0, theta=0.0, v_left=0.0, v_right=0.0, t=None):
    """Build one board odometry sample."""
    return WheelSample(
        x_mm=x, y_mm=y, theta_deg=theta, v_left_mm_s=v_left, v_right_mm_s=v_right, t=t
    )


def wrap180(angle):
    """Wrap an angle to (-180, 180] degrees."""
    return ((angle + 180.0) % 360.0) - 180.0


class DegradedModeTests(unittest.TestCase):
    def test_no_sensors_is_wheel_only(self):
        """Without gyro/camera the service behaves exactly as before."""
        service = LocalizationService()
        service.ingest(wheel(0.0, 0.0, 0.0))
        service.ingest(wheel(10.0, 0.0, 0.0))
        status = service.status()
        self.assertEqual(status["mode"], "wheel")
        self.assertEqual(status["camera_samples"], 0)
        self.assertEqual(status["camera_rejected"], 0)

    def test_gyro_mode_reported(self):
        """A fresh gyro sample flips the reported mode."""
        service = LocalizationService()
        service.ingest(wheel())
        service.add_gyro_yaw_rate(0.0)
        self.assertIn("+gyro", service.status()["mode"])


class CameraFusionTests(unittest.TestCase):
    def test_camera_converges(self):
        """A repeated camera fix pulls the estimate towards it."""
        service = LocalizationService()
        service.ingest(wheel())
        # first fix is far from the estimate: pending, then confirmed
        self.assertFalse(service.update_camera_pose(200.0, -100.0, 30.0))
        self.assertTrue(service.update_camera_pose(200.0, -100.0, 30.0))
        self.assertEqual(service.status()["jumps"], 1)
        x, y, theta = service.pose()
        self.assertLess(math.hypot(x - 200.0, y + 100.0), 1.0)
        self.assertLess(abs(wrap180(theta - 30.0)), 1.0)
        self.assertIn("+camera", service.status()["mode"])

    def test_single_outlier_is_rejected(self):
        """One implausible fix never moves the estimate."""
        service = LocalizationService()
        service.ingest(wheel())
        service.update_camera_pose(200.0, -100.0, 30.0)
        service.update_camera_pose(200.0, -100.0, 30.0)  # confirmed teleport
        x_before, y_before, _ = service.pose()
        accepted = service.update_camera_pose(1200.0, -100.0, 30.0)
        self.assertFalse(accepted)
        self.assertGreaterEqual(service.status()["camera_rejected"], 1)
        x, y, _ = service.pose()
        self.assertLess(math.hypot(x - x_before, y - y_before), 1.0)

    def test_teleport_is_adopted_only_when_confirmed(self):
        """A persistent (repeated) teleport is adopted, a single one is not."""
        service = LocalizationService()
        service.ingest(wheel())
        self.assertFalse(service.update_camera_pose(400.0, 0.0, 0.0))
        self.assertTrue(service.update_camera_pose(400.0, 0.0, 0.0))
        x, _, _ = service.pose()
        self.assertLess(abs(x - 400.0), 1.0)

    def test_sustained_correction_is_reacquired(self):
        """A persistent sub-teleport correction overrides the gate (N-of-M).

        A 100 mm discrepancy is below the teleport bound and, with a tightly
        converged estimate, gated out - but the camera is the absolute anchor:
        a consistent repeat must re-acquire instead of being rejected forever.
        """
        service = LocalizationService()
        service.ingest(wheel())
        service.update_camera_pose(200.0, -100.0, 30.0)
        service.update_camera_pose(200.0, -100.0, 30.0)  # tight baseline
        self.assertFalse(service.update_camera_pose(300.0, -100.0, 30.0))
        self.assertTrue(service.update_camera_pose(300.0, -100.0, 30.0))
        x, y, _ = service.pose()
        self.assertLess(math.hypot(x - 300.0, y + 100.0), 1.0)

    def test_inconsistent_outliers_are_never_adopted(self):
        """Two wild samples at different places do not confirm each other."""
        service = LocalizationService()
        service.ingest(wheel())
        service.update_camera_pose(200.0, -100.0, 30.0)
        service.update_camera_pose(200.0, -100.0, 30.0)  # tight baseline
        self.assertFalse(service.update_camera_pose(600.0, 0.0, 0.0))
        self.assertFalse(service.update_camera_pose(-600.0, 0.0, 0.0))
        x, y, _ = service.pose()
        self.assertLess(math.hypot(x - 200.0, y + 100.0), 1.0)

    def test_tag_flip_rejects_heading_but_keeps_position(self):
        """A heading-only outlier is a planar-tag flip: x,y fused, theta not."""
        service = LocalizationService()
        service.ingest(wheel())
        service.update_camera_pose(200.0, -100.0, 30.0)
        service.update_camera_pose(200.0, -100.0, 30.0)  # confirmed baseline
        accepted = service.update_camera_pose(203.0, -101.0, 150.0)  # +120 deg
        self.assertTrue(accepted)
        self.assertEqual(service.status()["camera_flips"], 1)
        _, _, theta = service.pose()
        self.assertLess(abs(wrap180(theta - 30.0)), 20.0)  # heading not adopted
        x, y, _ = service.pose()
        self.assertLess(math.hypot(x - 203.0, y + 101.0), 10.0)  # position is

    def test_latency_compensation_moves_the_fix_forward(self):
        """A delayed fix is extrapolated forward along the current motion."""
        service = LocalizationService()
        service.ingest(wheel(0.0, 0.0, 0.0, v_left=300.0, v_right=300.0))
        service.filter.p[0][0] = 100.0  # a normally-converged position prior
        service.filter.p[1][1] = 100.0
        accepted = service.update_camera_pose(0.0, 0.0, 0.0, latency_s=0.1)
        self.assertTrue(accepted)
        x, y, _ = service.pose()
        # 300 mm/s * 0.1 s = 30 mm of travel happened during the pipeline
        self.assertGreater(x, 5.0)
        self.assertLess(abs(y), 5.0)

    def test_board_pose_is_demoted_while_camera_is_fresh(self):
        """With a fresh camera fix, a divergent board pose stays a cross-check."""
        control = LocalizationService()
        control.ingest(wheel())
        control.filter.p[0][0] = 100.0
        control.filter.p[1][1] = 100.0
        control.ingest(wheel(120.0, 0.0, 0.0))

        fused = LocalizationService()
        fused.ingest(wheel())
        fused.filter.p[0][0] = 100.0
        fused.filter.p[1][1] = 100.0
        self.assertTrue(fused.update_camera_pose(1.0, 0.0, 0.0))
        fused.ingest(wheel(120.0, 0.0, 0.0))

        control_x, _, _ = control.pose()
        fused_x, _, _ = fused.pose()
        self.assertGreater(control_x, 50.0)  # today: board pose dominates
        self.assertLess(fused_x, 20.0)  # fused: camera is the anchor

    def test_gyro_bias_refines_at_rest(self):
        """Stationary gyro samples slowly learn the bias."""
        service = LocalizationService()
        for _ in range(5):
            service.ingest(wheel())  # at rest: speeds are zero
        # the refinement is deliberately slow (guarded, ~5 s time constant),
        # so a long stretch of samples is needed for a clear convergence
        for _ in range(800):
            service.add_gyro_yaw_rate(2.0)
        self.assertGreater(service.status()["gyro_bias_deg_s"], 1.0)
        # the bias-corrected rate is near zero, so the yaw rate estimate too
        self.assertLess(abs(service.filter.yaw_rate_deg_s()), 1.0)

    def test_hand_rotation_is_never_learned_as_bias(self):
        """Still wheels + a real rotation: the gyro rate is NOT bias."""
        service = LocalizationService()
        for _ in range(5):
            service.ingest(wheel())  # wheels at rest
        for _ in range(200):
            service.add_gyro_yaw_rate(90.0)  # hand rotation, wheels still
        self.assertLess(service.status()["gyro_bias_deg_s"], 0.5)


class SessionIngestionTests(unittest.IsolatedAsyncioTestCase):
    async def test_session_fuses_camera_messages(self):
        """The session stores the snapshot and feeds the fusion."""
        config = replace(settings, camera_poses_enabled=True)
        session = RobotSession(config)
        session.localization.ingest(wheel())
        changed = await session.on_camera_poses(
            {"x_mm": 1.0, "y_mm": 0.0, "theta_deg": 0.0},
            {"x_mm": 2000.0, "y_mm": 1000.0, "theta_deg": 90.0},
            ttl_ms=5000,
            source="vision:test",
            t_ms=1000.0,
            t_server_ms=1000.1,
        )
        self.assertTrue(changed)
        self.assertTrue(session.poses.fresh)
        self.assertEqual(session.status()["loc"]["camera_samples"], 1)
        self.assertIn("+camera", session.status()["loc"]["mode"])
        # a malformed entry is dropped by the store and never reaches the filter
        await session.on_camera_poses({"x_mm": "bad"}, None, ttl_ms=5000)
        self.assertEqual(session.status()["loc"]["camera_rejected"], 0)

    async def test_camera_can_be_disabled(self):
        """CAMERA_POSES=off stores the snapshot but never fuses it."""
        config = replace(settings, camera_poses_enabled=False)
        session = RobotSession(config)
        session.localization.ingest(wheel())
        await session.on_camera_poses(
            {"x_mm": 1.0, "y_mm": 0.0, "theta_deg": 0.0}, None, ttl_ms=5000
        )
        self.assertTrue(session.poses.fresh)
        self.assertEqual(session.status()["loc"]["camera_samples"], 0)
        self.assertEqual(session.status()["loc"]["mode"], "wheel")

    async def test_release_on_disconnect(self):
        """Releasing the source empties the pose snapshot."""
        session = RobotSession(replace(settings, camera_poses_enabled=True))
        await session.on_camera_poses(
            {"x_mm": 1.0, "y_mm": 0.0, "theta_deg": 0.0},
            None,
            ttl_ms=5000,
            source="vision:test",
        )
        self.assertTrue(session.release_camera_poses("vision:test"))
        self.assertFalse(session.poses.fresh)
