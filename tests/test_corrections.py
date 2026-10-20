"""Boundary re-anchor tests (Phase 4): idle-only, camera-gated corrections.

The policy must be conservative by construction: a correction happens only
with a fresh accepted camera, only while the board is idle, only outside the
minimum interval, and (after the first anchor) only above the discrepancy
thresholds. The first anchor doubles as match-start initialization.

File:    tests/test_corrections.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-05
"""

import os
import sys
import time
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ.setdefault("ROBOT_TRANSPORT", "mock")

from software.config import settings  # noqa: E402
from software.session import RobotSession  # noqa: E402


class FakeExecutor:
    """Executor stand-in with a settable running state."""

    def __init__(self, running=False):
        """Create the fake with the desired state."""
        self.running = running

    def is_running(self):
        """Report whether an execution is active."""
        return self.running


class FakeClient:
    """Robot-client stand-in recording SET_POSE calls."""

    def __init__(self):
        """Create the fake with an empty pose log."""
        self.poses = []

    async def set_pose(self, x_mm, y_mm, theta_deg):
        """Record one re-anchor."""
        self.poses.append((x_mm, y_mm, theta_deg))


class ReanchorTests(unittest.IsolatedAsyncioTestCase):
    """The re-anchor policy loop (session.maybe_reanchor)."""

    def _session(self, **over):
        """Build a session with corrections enabled and fast intervals."""
        values = {
            "reanchor_enabled": True,
            "reanchor_mm": 8.0,
            "reanchor_deg": 0.7,
            "reanchor_interval_s": 0.5,
        }
        values.update(over)
        session = RobotSession(replace(settings, **values))
        session.client = FakeClient()
        session.executor = FakeExecutor(running=False)
        session.odometry = SimpleNamespace(x_mm=0.0, y_mm=0.0, theta_deg=0.0, is_idle=True)
        return session

    def _fresh_camera(self, session):
        """Mark a fresh accepted camera pose (store + fusion freshness)."""
        session.poses.update(
            {"x_mm": 100.0, "y_mm": 0.0, "theta_deg": 0.0}, None, ttl_ms=5000, source="cam"
        )
        session.localization._last_camera_t = time.monotonic()

    async def test_first_anchor_is_match_start_initialization(self):
        """Even a zero discrepancy anchors once (the match-start case)."""
        session = self._session()
        self._fresh_camera(session)
        self.assertTrue(await session.maybe_reanchor())
        self.assertEqual(len(session.client.poses), 1)
        x, _y, _theta = session.client.poses[0]
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertEqual(session.reanchor_count, 1)

    async def test_threshold_gates_later_anchors(self):
        """After the first anchor, only a real discrepancy corrects."""
        session = self._session()
        self._fresh_camera(session)
        await session.maybe_reanchor()  # initial anchor
        session._last_reanchor_t = 0.0  # bypass the interval for the test
        session.localization.filter.x = [5.0, 0.0, 0.0, 0.0]
        self.assertFalse(await session.maybe_reanchor())  # below 8 mm
        self.assertEqual(session.reanchor_count, 1)
        session._last_reanchor_t = 0.0
        session.localization.filter.x = [50.0, 0.0, 0.0, 0.0]
        self.assertTrue(await session.maybe_reanchor())  # above 8 mm
        self.assertEqual(session.reanchor_count, 2)
        self.assertAlmostEqual(session.client.poses[-1][0], 50.0, places=3)

    async def test_never_while_executing(self):
        """A correction never fires during an execution."""
        session = self._session()
        session.executor = FakeExecutor(running=True)
        self._fresh_camera(session)
        self.assertFalse(await session.maybe_reanchor())
        self.assertEqual(session.client.poses, [])

    async def test_never_when_board_is_not_idle(self):
        """A correction never fires while the board reports motion."""
        session = self._session()
        session.odometry = SimpleNamespace(x_mm=0.0, y_mm=0.0, theta_deg=0.0, is_idle=False)
        self._fresh_camera(session)
        self.assertFalse(await session.maybe_reanchor())

    async def test_never_without_a_fresh_camera(self):
        """No camera or a stale camera: nothing happens at all."""
        session = self._session()
        self.assertFalse(await session.maybe_reanchor())  # no camera ever
        session.localization._last_camera_t = time.monotonic() - 5.0
        self.assertFalse(await session.maybe_reanchor())  # stale
        self.assertEqual(session.client.poses, [])

    async def test_minimum_interval(self):
        """Two anchors never happen back to back inside the interval."""
        session = self._session()
        self._fresh_camera(session)
        await session.maybe_reanchor()
        session.localization.filter.x = [99.0, 0.0, 0.0, 0.0]
        self.assertFalse(await session.maybe_reanchor())
        self.assertEqual(session.reanchor_count, 1)
