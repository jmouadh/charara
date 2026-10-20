"""Silent gyro forward stream tests (SET_GYRO, Docs/09 section 7.3).

Covered: the write-only link path, silent mock bookkeeping (the response
state and the sequence window are never touched), the rate scale/clamp, the
firmware >= 3.3 gate, streaming only while motion is active and the IMU
fresh, and the mock/no-IMU transports never streaming at all.

File:    tests/test_stream.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-05
"""

import asyncio
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
from software.robot import protocol as p  # noqa: E402
from software.robot.client import RobotClient  # noqa: E402
from software.robot.link import I2CLink, MockLink  # noqa: E402
from software.session import RobotSession  # noqa: E402


class StreamFrameTests(unittest.IsolatedAsyncioTestCase):
    """The frame itself: encoding, write-only transport, silent semantics."""

    def test_protocol_parity(self):
        """The wire code and scale match the firmware spec (SET_GYRO = 0x18)."""
        self.assertEqual(p.CMD_SET_GYRO, 0x18)
        self.assertAlmostEqual(p.GYRO_RATE_SCALE, 0.1)
        self.assertEqual((p.GYRO_STREAM_FW_MAJOR, p.GYRO_STREAM_FW_MINOR), (3, 3))

    async def test_mock_link_stores_the_frame_silently(self):
        """The frame updates the gyro input only: response state untouched."""
        link = MockLink()
        client = RobotClient(link, retries=1)
        status_before = link._last_status
        seq_before = link._last_seq
        await client.stream_gyro(10.0, age_10ms=3)
        self.assertAlmostEqual(link.last_gyro_rate_dps, 10.0, places=1)
        self.assertEqual(link.last_gyro_age_10ms, 3)
        self.assertEqual(link.gyro_frames, 1)
        self.assertEqual(link._last_status, status_before)
        self.assertEqual(link._last_seq, seq_before)
        status = await client.status()
        self.assertEqual(status.gyro_fresh, 1)  # the board sees a fresh stream

    async def test_rate_scale_and_clamp(self):
        """0.1 deg/s units, clamped to the i16 range."""
        link = MockLink()
        client = RobotClient(link, retries=1)
        await client.stream_gyro(-12.3)
        self.assertAlmostEqual(link.last_gyro_rate_dps, -12.3, places=1)
        await client.stream_gyro(1e6)
        self.assertAlmostEqual(link.last_gyro_rate_dps, 3276.7, places=1)

    async def test_i2c_path_is_write_only(self):
        """I2CLink.send_stream writes one frame and reads nothing.

        smbus2 is Linux-only (no ``fcntl`` on Windows), so the module-level
        ``i2c_msg`` is stubbed: what matters is one write, no read.
        """
        sent = []

        class FakeBus:
            """Recording bus: only i2c_rdwr is exercised."""

            def i2c_rdwr(self, *messages):
                """Record the messages of one transaction."""
                sent.extend(messages)

            def close(self):
                """Close the fake bus (no-op)."""

        class FakeMsg:
            """Minimal smbus2.i2c_msg stand-in."""

            @staticmethod
            def write(address, values):
                """Build a recorded 'write' message."""
                return ("write", address, bytes(values))

        from software.robot import link as link_module

        original = link_module.i2c_msg
        link_module.i2c_msg = FakeMsg
        try:
            link = I2CLink.__new__(I2CLink)
            link._address = 0x0A
            link._bus = FakeBus()
            link._retries = 1
            link._retry_delay = 0.0
            frame = p.build_request(7, p.CMD_SET_GYRO, p.put_i16(100) + p.put_u8(1))
            link.send_stream(frame)
        finally:
            link_module.i2c_msg = original
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0], ("write", 0x0A, frame))


class SessionStreamTests(unittest.IsolatedAsyncioTestCase):
    """The session lifecycle: gate, freshness, motion gating, degradation."""

    class FakeClient:
        """Robot-client stand-in for the stream task."""

        def __init__(self, major=3, minor=3):
            """Create the fake with a firmware version and a frame log."""
            self.major = major
            self.minor = minor
            self.frames = []

        async def status(self):
            """Report the fake firmware version."""
            return SimpleNamespace(fw_major=self.major, fw_minor=self.minor)

        async def stream_gyro(self, rate_deg_s, age_10ms=0):
            """Record one forwarded frame."""
            self.frames.append((rate_deg_s, age_10ms))

    def _session(self, **over):
        """Build a session with the stream enabled and fast in tests."""
        values = {"imu_enabled": True, "gyro_stream_enabled": True, "gyro_stream_hz": 200.0}
        values.update(over)
        return RobotSession(replace(settings, **values))

    def _ready_imu(self, session, rate=5.0):
        """Pretend a healthy IMU just delivered a sample."""
        session.imu = object()
        session.imu_state = "ready"
        session.imu_rate_deg_s = rate
        session.imu_last_t = time.monotonic()

    async def test_unsupported_firmware_never_streams(self):
        """A 3.2 board is never sent SET_GYRO frames (no UNKNOWN_CMD flood)."""
        session = self._session()
        session.transport = "i2c"
        client = self.FakeClient(major=3, minor=2)
        session.client = client
        self._ready_imu(session)
        logs = []

        async def logger(level, message):
            """Capture one log line."""
            logs.append(message)

        session.logger = logger
        await session.start_gyro_stream()
        self.assertEqual(session.gyro_stream_state, "unsupported")
        self.assertIsNone(session._stream_task)
        self.assertTrue(any("3.3" in message for message in logs))
        await asyncio.sleep(0.03)
        self.assertEqual(client.frames, [])

    async def test_streams_while_moving(self):
        """A supported board receives frames while the robot moves."""
        session = self._session()
        session.transport = "i2c"
        client = self.FakeClient()
        session.client = client
        self._ready_imu(session, rate=5.0)
        session.odometry = SimpleNamespace(is_idle=False)
        await session.start_gyro_stream()
        self.assertEqual(session.gyro_stream_state, "active")
        for _ in range(50):
            if client.frames:
                break
            await asyncio.sleep(0.01)
        await session.stop_gyro_stream()
        self.assertGreaterEqual(len(client.frames), 1)
        rate, _age = client.frames[0]
        self.assertAlmostEqual(rate, 5.0, places=3)
        self.assertGreaterEqual(session.gyro_stream_frames, 1)

    async def test_no_frames_when_idle_or_stale(self):
        """No motion or a stale IMU: the board keeps its wheel heading."""
        session = self._session()
        session.transport = "i2c"
        client = self.FakeClient()
        session.client = client
        self._ready_imu(session)
        session.imu_last_t = time.monotonic() - 5.0  # stale sample
        session.odometry = SimpleNamespace(is_idle=False)
        await session.start_gyro_stream()
        await asyncio.sleep(0.05)
        await session.stop_gyro_stream()
        self.assertEqual(client.frames, [])
        self.assertEqual(session.gyro_stream_frames, 0)

    async def test_mock_transport_never_streams(self):
        """Mock mode has no board to forward to."""
        session = self._session()
        session.transport = "mock"
        client = self.FakeClient()
        session.client = client
        self._ready_imu(session)
        await session.start_gyro_stream()
        self.assertIsNone(session._stream_task)
        self.assertEqual(session.gyro_stream_state, "off")

    async def test_no_imu_never_streams(self):
        """No IMU mounted: nothing to forward."""
        session = self._session()
        session.transport = "i2c"
        client = self.FakeClient()
        session.client = client
        await session.start_gyro_stream()
        self.assertIsNone(session._stream_task)

    async def test_manual_stop_silences_and_start_resumes_the_stream(self):
        """Stop IMU ends the frames at once; a re-armed stream resumes them."""
        session = self._session()
        session.transport = "i2c"
        client = self.FakeClient()
        session.client = client
        self._ready_imu(session, rate=4.0)
        session.odometry = SimpleNamespace(is_idle=False)
        await session.start_gyro_stream()
        for _ in range(50):
            if client.frames:
                break
            await asyncio.sleep(0.01)
        self.assertGreaterEqual(session.gyro_stream_frames, 1)
        await session.stop_imu_sampling()
        self.assertEqual(session.gyro_stream_state, "off")
        self.assertEqual(session.imu_status()["state"], "stopped")
        count = len(client.frames)
        await asyncio.sleep(0.05)
        self.assertEqual(len(client.frames), count)  # silence: no new frames
        # the IMU comes back (Start IMU) and the stream is re-armed
        self._ready_imu(session, rate=4.0)
        session.odometry = SimpleNamespace(is_idle=False)
        await session.start_gyro_stream()
        for _ in range(50):
            if len(client.frames) > count:
                break
            await asyncio.sleep(0.01)
        self.assertGreater(len(client.frames), count)
        await session.stop_gyro_stream()
