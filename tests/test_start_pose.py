"""Match-start pose bootstrap: the robot never starts at the field corner.

The session adopts the configured team start (``TEAM`` + ``TEAM_START_*``,
:data:`DEFAULT_START_YELLOW` when unset, Blue mirrored when it has no
configured pose) the first time it sees a board that still reports its boot
pose (0,0,0). A pose that was already set - by the operator, or by a previous
console run - is never overwritten; ``START_POSE=0`` disables the feature.

File:    tests/test_start_pose.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-07
"""

import os
import sys
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ.setdefault("ROBOT_TRANSPORT", "mock")

from software.config import settings  # noqa: E402
from software.core.mission.teams import DEFAULT_START_YELLOW  # noqa: E402
from software.session import RobotSession  # noqa: E402


class FakeClient:
    """Robot-client stand-in: a readable pose and a SET_POSE log."""

    def __init__(self, pose=(0.0, 0.0, 0.0), fail_read=False, fail_write=False):
        """Create the fake with a board pose and programmable failures."""
        self.pose = pose
        self.fail_read = fail_read
        self.fail_write = fail_write
        self.poses = []

    async def odometry(self):
        """Return the board pose."""
        if self.fail_read:
            raise RuntimeError("link down")
        x, y, theta = self.pose
        return SimpleNamespace(x_mm=x, y_mm=y, theta_deg=theta)

    async def set_pose(self, x_mm, y_mm, theta_deg):
        """Record the adopted pose (or refuse, when programmed to fail)."""
        if self.fail_write:
            raise RuntimeError("fault latched: SET_POSE refused")
        self.poses.append((x_mm, y_mm, theta_deg))
        self.pose = (x_mm, y_mm, theta_deg)


class StartPoseTests(unittest.IsolatedAsyncioTestCase):
    """The startup match-start adoption (session.maybe_apply_start_pose)."""

    def _session(self, client=None, **over):
        """Build a session with the link faked and the feature configured."""
        session = RobotSession(replace(settings, **over))
        session.client = client if client is not None else FakeClient()
        return session

    async def test_unset_board_adopts_the_yellow_start(self):
        """A fresh board (0,0,0) starts at the yellow match-start pose."""
        session = self._session()
        self.assertTrue(await session.maybe_apply_start_pose())
        self.assertEqual(len(session.client.poses), 1)
        x, y, theta = session.client.poses[0]
        self.assertAlmostEqual(x, DEFAULT_START_YELLOW[0], places=3)
        self.assertAlmostEqual(y, DEFAULT_START_YELLOW[1], places=3)
        self.assertAlmostEqual(theta, DEFAULT_START_YELLOW[2], places=3)

    async def test_blue_start_is_configured_or_mirrored(self):
        """Team Blue uses its configured start, or the mirrored yellow one."""
        session = self._session(team="blue")
        await session.maybe_apply_start_pose()
        x, _y, _theta = session.client.poses[0]
        self.assertAlmostEqual(x, settings.field_width_mm - DEFAULT_START_YELLOW[0], places=3)

        configured = self._session(team="blue", team_start_blue=(100.0, 200.0, 45.0))
        await configured.maybe_apply_start_pose()
        self.assertEqual(configured.client.poses[0], (100.0, 200.0, 45.0))

    async def test_a_placed_robot_is_never_moved(self):
        """A board pose that was already set stays untouched."""
        session = self._session(client=FakeClient(pose=(1200.0, 800.0, 30.0)))
        self.assertFalse(await session.maybe_apply_start_pose())
        self.assertEqual(session.client.poses, [])

    async def test_runs_once_per_process(self):
        """A second call (transport switch, reconnect) does nothing."""
        session = self._session()
        self.assertTrue(await session.maybe_apply_start_pose())
        self.assertFalse(await session.maybe_apply_start_pose())
        self.assertEqual(len(session.client.poses), 1)

    async def test_can_be_disabled(self):
        """START_POSE=0 leaves the board exactly as it is."""
        session = self._session(start_pose_enabled=False)
        self.assertFalse(await session.maybe_apply_start_pose())
        self.assertEqual(session.client.poses, [])

    async def test_unreadable_link_is_retried_later(self):
        """A link that cannot be read yet is not marked as done."""
        session = self._session(client=FakeClient(fail_read=True))
        self.assertFalse(await session.maybe_apply_start_pose())
        self.assertFalse(session._start_pose_done)
        session.client.fail_read = False
        self.assertTrue(await session.maybe_apply_start_pose())

    async def test_mock_start_is_kept(self):
        """The mock already starts at a start pose: nothing is written."""
        session = self._session(client=FakeClient(pose=(315.0, 1800.0, -90.0)))
        self.assertFalse(await session.maybe_apply_start_pose())

    async def test_a_refused_set_pose_never_breaks_the_connect(self):
        """A latched fault only warns; the session keeps working."""
        logs = []
        session = self._session(client=FakeClient(fail_write=True))

        async def logger(level, message):
            """Capture one log line."""
            logs.append((level, message))

        session.logger = logger
        self.assertFalse(await session.maybe_apply_start_pose())
        self.assertTrue(session._start_pose_done)
        self.assertTrue(any("match-start pose" in message for _level, message in logs))
