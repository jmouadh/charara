"""Session bootstrap tests: the host must survive a missing board link.

Covered (Docs/09 section 10): a link that cannot be opened at startup is a
degraded state, not a crash - the session logs once, retries in the
background and connects as soon as the link answers; a manual transport
switch (or stop) supersedes a pending retry so the two never fight.

File:    tests/test_session.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-05
"""

import asyncio
import os
import sys
import unittest
from dataclasses import replace
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ.setdefault("ROBOT_TRANSPORT", "mock")

from software.config import settings  # noqa: E402
from software.robot.link import LinkError, MockLink  # noqa: E402
from software.session import RobotSession  # noqa: E402
from software.session import session as session_module  # noqa: E402


class StartResilienceTests(unittest.IsolatedAsyncioTestCase):
    """start()/stop() around a board that is not there (yet)."""

    def _session(self) -> RobotSession:
        """Session on the i2c transport with a fast retry period (tests)."""
        session = RobotSession(replace(settings, transport="i2c", imu_enabled=False))
        session.CONNECT_RETRY_S = 0.01
        return session

    async def _wait_for(self, predicate, tries: int = 200, delay: float = 0.01) -> bool:
        """Await a condition with a bounded polling loop (test helper)."""
        for _ in range(tries):
            if predicate():
                return True
            await asyncio.sleep(delay)
        return predicate()

    async def test_start_survives_a_missing_link_and_retries(self):
        """No smbus2/bus at boot: start() returns, the retry connects later."""
        session = self._session()
        attempts = []

        def build_link(*_args, **_kwargs):
            """Fail the first build (no bus), answer with the mock after."""
            attempts.append(1)
            if len(attempts) == 1:
                raise LinkError("no i2c bus")
            return MockLink()

        with mock.patch.object(session_module, "I2CLink", side_effect=build_link):
            await session.start()  # must not raise
            self.assertIsNone(session.client)
            self.assertFalse(session.connected)
            self.assertIsNotNone(session._connect_task)
            connected = await self._wait_for(lambda: session.client is not None)
            self.assertTrue(connected)
        self.assertEqual(session.transport, "i2c")
        self.assertIsNotNone(session.executor)
        await session.stop()
        self.assertIsNone(session.client)
        self.assertIsNone(session._connect_task)
        self.assertFalse(session.connected)

    async def test_manual_switch_supersedes_a_pending_retry(self):
        """A /api/mode switch cancels the startup retry (no fight, no leak)."""
        session = self._session()

        def build_link(*_args, **_kwargs):
            """The i2c link never opens in this test."""
            raise LinkError("no i2c bus")

        with mock.patch.object(session_module, "I2CLink", side_effect=build_link):
            await session.start()
            retry = session._connect_task
            self.assertIsNotNone(retry)
            await asyncio.sleep(0.03)  # let the retry fail once or twice
            await session.switch_transport("mock")
            self.assertIsNone(session._connect_task)
            self.assertTrue(retry.cancelled() or retry.done())
            self.assertEqual(session.transport, "mock")
            self.assertIsNotNone(session.client)
        await session.stop()

    async def test_stop_is_safe_without_a_link(self):
        """stop() with no link ever opened is a no-op, not an error."""
        session = self._session()
        await session.stop()
        self.assertIsNone(session.client)
        self.assertFalse(session.connected)


class BoardGeometryTests(unittest.IsolatedAsyncioTestCase):
    """The estimator follows the board's calibrated odometry track."""

    async def test_track_adopted_from_the_board(self):
        """Register 7 wins over the environment default, once per change."""
        session = RobotSession(
            replace(settings, imu_enabled=False, odom_track_mm=150.0)
        )
        await session.switch_transport("mock")
        try:
            # mock registers start at the factory default (170 mm)
            self.assertAlmostEqual(session.localization.track_mm, 170.0, places=3)
            self.assertFalse(await session.sync_board_geometry())  # no change
            # a calibrated board value is adopted immediately
            assert session.client is not None
            await session.client.set_config(7, [1600])  # 160.0 mm
            self.assertTrue(await session.sync_board_geometry())
            self.assertAlmostEqual(session.localization.track_mm, 160.0, places=3)
        finally:
            await session.stop()

    async def test_out_of_range_track_is_rejected(self):
        """A nonsensical register value never reaches the estimator."""
        session = RobotSession(
            replace(settings, imu_enabled=False, odom_track_mm=150.0)
        )
        await session.switch_transport("mock")
        try:
            assert session.client is not None
            adopted = session.localization.track_mm  # the mock board's 170 mm
            raw = await session.client.get_all_config()
            raw[7] = 5  # below the register minimum (the board rejects such writes)
            session.client.get_all_config = mock.AsyncMock(return_value=raw)
            self.assertFalse(await session.sync_board_geometry())
            self.assertAlmostEqual(session.localization.track_mm, adopted, places=3)
        finally:
            await session.stop()
