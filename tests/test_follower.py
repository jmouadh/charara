"""Tests for the silent SET_VELOCITY stream, the mock board parity and the
streaming path follower (Track B, Docs/10 section 6).

Covered: wire codec parity (0x19, 0.1 deg/s omega scale, fw 3.4 gate), the
mock's stream integration/stale-stop/BUSY semantics, the simulated deadman
watchdog, and full follower runs (straight, arc) with the closing metrics.

File:    tests/test_follower.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-09
"""

import asyncio
import math
import os
import sys
import time
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ.setdefault("ROBOT_TRANSPORT", "mock")

from software.config import settings  # noqa: E402
from software.core.mission.executor import (  # noqa: E402
    follow_final_heading_deg,
    needs_in_place_actions,
)
from software.core.mission.follower import PathFollower  # noqa: E402
from software.robot import protocol as p  # noqa: E402
from software.robot.client import RobotClient  # noqa: E402
from software.robot.link import MockLink  # noqa: E402
from software.session import RobotSession  # noqa: E402


def _client():
    """A RobotClient over a fresh MockLink (the standard test harness)."""
    return RobotClient(MockLink(), retries=1)


class ProtocolParityTests(unittest.TestCase):
    """The codec must mirror the firmware constants exactly."""

    def test_set_velocity_wire_constants(self):
        """SET_VELOCITY = 0x19, omega in 0.1 deg/s, firmware gate 3.4."""
        self.assertEqual(p.CMD_SET_VELOCITY, 0x19)
        self.assertAlmostEqual(p.VELOCITY_OMEGA_SCALE, 0.1)
        self.assertEqual((p.VELOCITY_STREAM_FW_MAJOR, p.VELOCITY_STREAM_FW_MINOR), (3, 4))

    def test_stream_state_value(self):
        """MOTION_STATE_STREAM = 8 (motion.h parity) and it has a name."""
        self.assertEqual(p.STATE_STREAM, 8)
        self.assertEqual(p.STATE_NAMES[p.STATE_STREAM], "STREAM")


class MockStreamTests(unittest.IsolatedAsyncioTestCase):
    """MockLink parity for the SET_VELOCITY stream."""

    async def test_stream_moves_the_robot_along_its_heading(self):
        """Streaming v=200 on a -90 deg heading moves along -y."""
        client = _client()
        link = client.link
        start = await client.odometry()
        for _ in range(10):
            await client.stream_velocity(200.0, 0.0)
            await asyncio.sleep(0.02)
        mid = await client.odometry()
        self.assertEqual(mid.state, p.STATE_STREAM)
        self.assertGreater(start.y_mm - mid.y_mm, 10.0)  # moved along -y
        self.assertLess(abs(mid.x_mm - start.x_mm), 2.0)
        await client.stop()
        self.assertTrue((await client.odometry()).is_idle)
        self.assertIsNone(link._stream)

    async def test_stream_turns_with_omega(self):
        """A pure omega stream rotates in place (CCW positive)."""
        client = _client()
        start = await client.odometry()
        for _ in range(10):
            await client.stream_velocity(0.0, 30.0)  # 30 deg/s CCW
            await asyncio.sleep(0.02)
        mid = await client.odometry()
        self.assertGreater(mid.theta_deg, start.theta_deg + 2.0)
        await client.stop()

    async def test_program_command_is_rejected_while_streaming(self):
        """BUSY while the stream is active (firmware SEG_STREAM parity)."""
        client = _client()
        await client.stream_velocity(100.0, 0.0)
        with self.assertRaises(Exception) as ctx:
            await client.goto(400.0, 1700.0, -90.0, 200)
        self.assertEqual(getattr(ctx.exception, "status", None), p.ST_BUSY)
        await client.stop()

    async def test_stale_stream_stops_without_a_fault(self):
        """No frame for > 200 ms -> decay to rest, IDLE, faults stay 0.

        Even with the deadman ARMED, a starved stream must not latch: the
        stale-stop (200 ms) fires long before the watchdog (1500 ms), which
        is exactly the firmware's controlled-stop design.
        """
        client = _client()
        await client.set_deadman(1500)
        await client.stream_velocity(150.0, 0.0)
        await asyncio.sleep(0.3)  # stale beyond ROBOT_STREAM_STALE_MS
        await asyncio.sleep(0.8)  # let the decel ramp reach zero (mock)
        odometry = await client.odometry()
        self.assertTrue(odometry.is_idle)
        self.assertEqual(odometry.faults, 0)
        self.assertIsNone(client.link._stream)
        await client.set_deadman(0)

    async def test_stream_frame_is_silent(self):
        """Stream frames never touch the response state or the seq window."""
        client = _client()
        link = client.link
        seq_before = link._last_seq
        status_before = link._last_status
        await client.stream_velocity(120.0, -5.0)
        self.assertEqual(link._last_seq, seq_before)
        self.assertEqual(link._last_status, status_before)
        self.assertEqual(link.stream_frames, 1)
        await client.stop()

    async def test_omega_and_velocity_scale_and_clamp(self):
        """v in whole mm/s (i16 clamp), omega in 0.1 deg/s units."""
        client = _client()
        link = client.link
        await client.stream_velocity(123.4, -12.34)
        self.assertAlmostEqual(link._stream["v"], 123.0, delta=1.0)
        self.assertAlmostEqual(math.degrees(link._stream["w"]), -12.3, places=1)
        await client.stop()

    async def test_deadman_watchdog_trips_a_starved_drive(self):
        """An armed deadman + no frames while a DRIVE runs -> DEADMAN latch.

        A running program segment stays busy until its own ramp end, so a
        starved link lets the watchdog fire mid-move and freeze the robot
        exactly at the expiry instant (the drill's exact scenario). A
        starved STREAM would instead stale-stop at 200 ms and never reach
        the watchdog - that is the other test's territory.
        """
        client = _client()
        start = await client.odometry()
        await client.set_deadman(150)
        await client.drive(400.0, 100.0)  # 4 s of motion, starved after 150 ms
        await asyncio.sleep(0.4)  # no frames at all during this window
        odometry = await client.odometry()
        self.assertEqual(odometry.faults & p.FAULT_DEADMAN, p.FAULT_DEADMAN)
        self.assertFalse(odometry.is_idle)
        travelled = math.hypot(odometry.x_mm - start.x_mm, odometry.y_mm - start.y_mm)
        self.assertLess(travelled, 150.0)  # stopped far short of 400 mm
        await client.clear_faults()
        await client.set_deadman(0)
        self.assertTrue((await client.odometry()).is_idle)

    async def test_deadman_does_not_trip_while_frames_flow(self):
        """A fed stream never trips its own watchdog."""
        client = _client()
        await client.set_deadman(150)
        for _ in range(12):
            await client.stream_velocity(100.0, 0.0)
            await asyncio.sleep(0.03)
        odometry = await client.odometry()
        self.assertEqual(odometry.faults, 0)
        await client.stop()
        await client.set_deadman(0)


class FinalHeadingTests(unittest.TestCase):
    """The streaming backend performs the plan's final heading."""

    def test_last_motion_segment_heading(self):
        """GOTO th and ARC exit heading are read; actions are skipped."""
        self.assertEqual(follow_final_heading_deg([{"kind": "goto", "th": 45.0}]), 45.0)
        self.assertAlmostEqual(
            follow_final_heading_deg([
                {"kind": "goto", "th": 0.0},
                {"kind": "arc", "th0_deg": 10.0, "sweep_deg": 80.0},
            ]),
            90.0,
        )
        self.assertAlmostEqual(
            follow_final_heading_deg([
                {"kind": "arc", "th0_deg": 10.0, "sweep_deg": 80.0},
                {"kind": "pause", "seconds": 2.0},
            ]),
            90.0,
        )
        self.assertIsNone(follow_final_heading_deg([{"kind": "pause", "seconds": 1.0}]))

    def test_in_place_actions_force_the_segment_backend(self):
        """Rotations/pauses and mid-route arrival turns are not polyline data."""
        aligned = [
            {"kind": "goto", "x": 0.0, "y": 0.0, "th": 0.0},
            {"kind": "goto", "x": 200.0, "y": 0.0, "th": 0.0},
        ]
        self.assertFalse(needs_in_place_actions(aligned))
        # the final node's arrival heading IS performed by the follower
        self.assertFalse(needs_in_place_actions([
            {"kind": "goto", "x": 0.0, "y": 0.0, "th": 0.0},
            {"kind": "goto", "x": 200.0, "y": 0.0, "th": 90.0},
        ]))
        # ... but a mid-route arrival heading is not
        self.assertTrue(needs_in_place_actions([
            {"kind": "goto", "x": 0.0, "y": 0.0, "th": 0.0},
            {"kind": "goto", "x": 200.0, "y": 0.0, "th": 90.0},
            {"kind": "goto", "x": 400.0, "y": 0.0, "th": 0.0},
        ]))
        self.assertTrue(needs_in_place_actions([
            {"kind": "goto", "x": 0.0, "y": 0.0, "th": 0.0},
            {"kind": "turn", "angle_deg": 90.0},
            {"kind": "goto", "x": 200.0, "y": 0.0, "th": 0.0},
        ]))
        self.assertTrue(needs_in_place_actions([
            {"kind": "goto", "x": 0.0, "y": 0.0, "th": 0.0},
            {"kind": "pause", "seconds": 1.0},
        ]))
        # a woven arrival is geometry: the follower drives it
        self.assertFalse(needs_in_place_actions([
            {"kind": "goto", "x": 0.0, "y": 0.0, "th": 0.0},
            {"kind": "arc", "x0": 0.0, "y0": 0.0, "x1": 100.0, "y1": 100.0,
             "radius_mm": 100.0, "sweep_deg": 90.0, "th0_deg": 0.0},
        ]))
        # a close-waypoint final leg carries a safety-critical in-place turn
        # at its stand-off: Pure Pursuit would cut that corner
        self.assertTrue(needs_in_place_actions([
            {"kind": "goto", "x": 0.0, "y": 0.0, "th": 0.0},
            {"kind": "goto", "x": 200.0, "y": 0.0, "th": 0.0,
             "close_waypoint": True},
        ]))
        # a close-waypoint departure is a raw reverse: not polyline data
        self.assertTrue(needs_in_place_actions([
            {"kind": "drive", "distance_mm": -75.0, "departure": True},
        ]))


class StreamingGateTests(unittest.IsolatedAsyncioTestCase):
    """The STREAMING_EXEC + firmware capability gate of the session."""

    async def test_gate_requires_the_flag_and_a_link(self):
        """OFF by default; flag ON + mock link flips it to supported."""
        session = RobotSession(replace(settings, transport="mock"))
        session.client = RobotClient(MockLink(), retries=1)
        self.assertFalse(await session.supports_velocity_stream())  # flag off
        session.settings = replace(settings, transport="mock", streaming_exec_enabled=True)
        self.assertTrue(await session.supports_velocity_stream())
        session.client = None
        self.assertFalse(await session.supports_velocity_stream())  # no link


class FollowerRunTests(unittest.IsolatedAsyncioTestCase):
    """Full follower runs on the mock (no robot needed)."""

    async def test_straight_run_reaches_the_end(self):
        """A 300 mm straight ends within tolerance and leaves the board IDLE."""
        client = _client()
        follower = PathFollower(client)
        points = [(315.0, 1800.0), (315.0, 1500.0)]  # along the start heading
        result = await follower.run(points, 300.0, deadman_ms=1500)
        self.assertTrue(result["ok"], result["reason"])
        self.assertLessEqual(result["final_error_mm"], 45.0)
        self.assertGreater(result["frames"], 3)
        odometry = await client.odometry()
        self.assertTrue(odometry.is_idle)
        self.assertEqual(odometry.faults, 0)
        self.assertEqual(client.link._deadman_ms, 0)  # cleared after the run
        self.assertIsNone(client.link._stream)

    async def test_misaligned_start_turns_in_place_before_driving(self):
        """A plan starting BESIDE the robot turns first, it does not drive away.

        Real-robot incident 2026-10-18: the path pointed west, the robot
        faced east, so pure pursuit drove straight ahead - out of the field.
        The follower must rotate in place until the path is in front.
        """
        client = _client()
        follower = PathFollower(client)
        # the mock robot starts at (315, 1800) facing -90 (south); this path
        # goes EAST, i.e. 90 deg to the left of the heading
        result = await follower.run([(315.0, 1800.0), (715.0, 1800.0)], 250.0)
        self.assertTrue(result["ok"], result["reason"])
        odometry = await client.odometry()
        self.assertLess(abs(odometry.y_mm - 1800.0), 150.0,
                        "the robot drove along its old heading instead of turning")
        self.assertLess(math.hypot(odometry.x_mm - 715.0, odometry.y_mm - 1800.0), 80.0)
        self.assertTrue(odometry.is_idle)

    async def test_final_heading_is_performed_after_the_path(self):
        """The path end plus the plan's exit heading: rotate to it, then done."""
        client = _client()
        follower = PathFollower(client)
        # mock starts at (315, 1800, -90): drive south, then face EAST
        result = await follower.run(
            [(315.0, 1800.0), (315.0, 1600.0)], 250.0, final_heading_deg=0.0
        )
        self.assertTrue(result["ok"], result["reason"])
        odometry = await client.odometry()
        self.assertLess(abs((odometry.theta_deg + 180.0) % 360.0 - 180.0), 3.0,
                        f"final heading {odometry.theta_deg:.1f} deg, expected 0")
        self.assertTrue(odometry.is_idle)

    async def test_start_far_off_the_path_is_refused(self):
        """A pose hundreds of mm off the plan is not driven across the table."""
        client = _client()
        follower = PathFollower(client)
        result = await follower.run([(1500.0, 1500.0), (1500.0, 1200.0)], 250.0)
        self.assertFalse(result["ok"])
        self.assertIn("off the planned path", result["reason"])
        self.assertEqual(client.link.stream_frames, 0)
        self.assertTrue((await client.odometry()).is_idle)

    async def test_field_edge_geofence_stops_the_run(self):
        """The streaming geofence aborts before the robot can leave the area."""
        client = _client()
        follower = PathFollower(client)
        # the mock starts at x=315: an inset of 400 mm puts it outside
        result = await follower.run(
            [(315.0, 1800.0), (315.0, 1500.0)], 200.0,
            bounds=(3000.0, 2000.0, 400.0),
        )
        self.assertFalse(result["ok"])
        self.assertIn("field edge", result["reason"])
        self.assertEqual(client.link.stream_frames, 0)
        self.assertTrue((await client.odometry()).is_idle)

    async def test_quarter_arc_run_turns_the_robot(self):
        """A 300 mm quarter circle: ends near the arc end with heading turned."""
        client = _client()
        follower = PathFollower(client)
        centre = (615.0, 1800.0)
        points = []
        for i in range(0, 91):
            ang = math.radians(180.0 + i)
            points.append((centre[0] + 300.0 * math.cos(ang), centre[1] + 300.0 * math.sin(ang)))
        result = await follower.run(points, 300.0)
        self.assertTrue(result["ok"], result["reason"])
        self.assertLessEqual(result["final_error_mm"], 45.0)
        odometry = await client.odometry()
        # started at -90 deg, +90 deg CCW sweep -> about 0 deg at the end
        self.assertLess(abs((odometry.theta_deg + 180.0) % 360.0 - 180.0), 12.0)

    async def test_latched_fault_aborts_immediately(self):
        """A latched board fault aborts the run before the first frame."""
        client = _client()
        await client.estop()
        follower = PathFollower(client)
        result = await follower.run([(315.0, 1800.0), (315.0, 1400.0)], 300.0)
        self.assertFalse(result["ok"])
        self.assertIn("fault", result["reason"])
        await client.clear_faults()

    async def test_timeout_aborts_the_run(self):
        """A tiny timeout aborts a long path with a clear reason."""
        client = _client()
        follower = PathFollower(client)
        result = await follower.run(
            [(315.0, 1800.0), (315.0, 800.0)], 300.0, timeout_s=0.01
        )
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "path timeout")
        self.assertTrue((await client.odometry()).is_idle)

    async def test_empty_path_is_reported_not_raised(self):
        """A degenerate path returns ok=False instead of raising."""
        client = _client()
        follower = PathFollower(client)
        result = await follower.run([(0.0, 0.0)], 300.0)
        self.assertFalse(result["ok"])
        self.assertIn("two points", result["reason"])

    async def test_zero_cruise_speed_is_refused(self):
        """v_cruise <= 0 is refused BEFORE a single frame is sent."""
        client = _client()
        follower = PathFollower(client)
        result = await follower.run([(315.0, 1800.0), (315.0, 1500.0)], 0.0)
        self.assertFalse(result["ok"])
        self.assertIn("positive", result["reason"])
        self.assertEqual(client.link.stream_frames, 0)

    async def test_capability_gate_refuses_before_sending(self):
        """A false capability check returns without touching the board.

        The host passes the session's supports_velocity_stream; a board on
        firmware < 3.4 (or STREAMING_EXEC off) must never receive stream
        frames by accident.
        """
        client = _client()

        async def unsupported() -> bool:
            return False

        follower = PathFollower(client, capability=unsupported)
        result = await follower.run([(315.0, 1800.0), (315.0, 1500.0)], 300.0)
        self.assertFalse(result["ok"])
        self.assertIn("not supported", result["reason"])
        self.assertEqual(client.link.stream_frames, 0)
        self.assertEqual(result["frames"], 0)
        self.assertTrue((await client.odometry()).is_idle)

    async def test_capability_gate_allows_a_supported_board(self):
        """A true capability check lets the run proceed normally."""
        client = _client()

        async def supported() -> bool:
            return True

        follower = PathFollower(client, capability=supported)
        result = await follower.run([(315.0, 1800.0), (315.0, 1600.0)], 300.0)
        self.assertTrue(result["ok"], result["reason"])
        self.assertGreater(client.link.stream_frames, 0)


class FollowerHookTests(unittest.IsolatedAsyncioTestCase):
    """Executor hooks on the follower: caution scale, STOP abort, PAUSE hold."""

    async def test_speed_scale_slows_every_frame(self):
        """A 0.5 caution scale halves the commands; the run still completes."""
        client = _client()
        follower = PathFollower(client)
        result = await follower.run(
            [(315.0, 1800.0), (315.0, 1700.0)], 300.0, speed_scale=lambda: 0.5
        )
        self.assertTrue(result["ok"], result["reason"])
        self.assertLessEqual(result["max_speed_mm_s"], 160.0)

    async def test_should_abort_stops_the_robot(self):
        """An abort flag ends the run with a clean STOP, no stale frames."""
        client = _client()
        state = {"run": True}

        def abort() -> bool:
            return not state["run"]

        follower = PathFollower(client)
        task = asyncio.create_task(
            follower.run([(315.0, 1800.0), (315.0, 1000.0)], 300.0, should_abort=abort)
        )
        await asyncio.sleep(0.25)
        state["run"] = False
        result = await task
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "aborted")
        self.assertTrue((await client.odometry()).is_idle)
        self.assertIsNone(client.link._stream)

    async def test_pause_holds_zero_frames_and_resume_continues(self):
        """A pause sends NO frame (STOP then hold); resume drives to the end."""
        client = _client()
        paused = {"on": True}
        transitions = []

        async def on_pause(value: bool) -> None:
            transitions.append(value)

        follower = PathFollower(client)
        task = asyncio.create_task(
            follower.run(
                [(315.0, 1800.0), (315.0, 1650.0)],
                300.0,
                should_pause=lambda: paused["on"],
                on_pause=on_pause,
            )
        )
        await asyncio.sleep(0.2)
        self.assertTrue((await client.odometry()).is_idle)  # held, not creeping
        self.assertEqual(client.link.stream_frames, 0)  # no frame while paused
        paused["on"] = False
        result = await task
        self.assertTrue(result["ok"], result["reason"])
        self.assertGreater(client.link.stream_frames, 0)
        self.assertEqual(transitions, [True, False])


if __name__ == "__main__":
    unittest.main(verbosity=2)
