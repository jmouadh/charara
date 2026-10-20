"""MockLink tests: the virtual board must behave like the firmware's engine
(ACK/errors, BUSY rejection, STOP freeze, latched E-STOP, odometry units).

File:    tests/test_mock_link.py
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

from software.robot import protocol as p  # noqa: E402
from software.robot import robot_config  # noqa: E402
from software.robot.link import MockLink  # noqa: E402
from software.robot.models import Odometry, Status  # noqa: E402


def odometry(link) -> Odometry:
    """Odometry."""
    status, payload = p.parse_response(link.transact(p.build_request(1, p.CMD_QUERY_ODOMETRY)))
    assert status == p.ST_OK
    return Odometry.parse(payload)


def wait_idle(link, timeout_s: float = 3.0) -> Odometry:
    """Wait idle."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        odometry_now = odometry(link)
        if odometry_now.state == p.STATE_IDLE:
            return odometry_now
        time.sleep(0.05)
    return odometry(link)


class MockLinkTests(unittest.TestCase):
    def setUp(self):
        """Prepare the per-test fixtures.

        The mock boards boots at the match start pose (315, 1800, -90); most
        tests reason in body frame, so park the simulator at the origin.
        """
        self.link = MockLink()
        self._transact(p.CMD_SET_POSE, p.put_i16(0) + p.put_i16(0) + p.put_i16(0))

    def _transact(self, cmd, payload=b""):
        """Transact."""
        response = self.link.transact(p.build_request(1, cmd, payload))
        return p.parse_response(response)

    def test_ping_echo(self):
        """Ping echo."""
        status, payload = self._transact(p.CMD_PING, p.put_u32(0xDEADBEEF))
        self.assertEqual(status, p.ST_OK)
        self.assertEqual(p.get_u32(payload, 0), 0xDEADBEEF)

    def test_set_pose_and_readback(self):
        """Set pose and readback."""
        status, _ = self._transact(p.CMD_SET_POSE, p.put_i16(5000) + p.put_i16(-2000) + p.put_i16(900))
        self.assertEqual(status, p.ST_OK)
        odometry_now = odometry(self.link)
        self.assertAlmostEqual(odometry_now.x_mm, 500.0, delta=0.1)
        self.assertAlmostEqual(odometry_now.y_mm, -200.0, delta=0.1)
        self.assertAlmostEqual(odometry_now.theta_deg, 90.0, delta=0.1)

    def test_drive_completes_and_moves(self):
        """Drive completes and moves."""
        status, _ = self._transact(p.CMD_DRIVE, p.put_i16(100) + p.put_u16(255))
        self.assertEqual(status, p.ST_OK)
        snapshot = wait_idle(self.link)
        self.assertAlmostEqual(snapshot.x_mm, 100.0, delta=5.0)
        self.assertAlmostEqual(snapshot.y_mm, 0.0, delta=1.0)

    def test_backward_drive_reports_signed_wheel_speeds(self):
        # regression: an unsigned centre travel (hypot) made a reverse move
        # look like a forward one to the console estimator, so the displayed
        # pose first "advanced" a few mm before backing up
        """A reverse move reports negative wheel speeds and moves -x."""
        status, _ = self._transact(p.CMD_DRIVE, p.put_i16(-200) + p.put_u16(120))
        self.assertEqual(status, p.ST_OK)
        time.sleep(0.3)
        moving = odometry(self.link)
        self.assertEqual(moving.state, p.STATE_DRIVE)
        self.assertLess(moving.v_left_mm_s, 0.0)
        self.assertLess(moving.v_right_mm_s, 0.0)
        self.assertLess(moving.x_mm, 0.0)
        snapshot = wait_idle(self.link)
        self.assertAlmostEqual(snapshot.x_mm, -200.0, delta=10.0)

    def test_straight_drive_along_minus_x_does_not_spin(self):
        # regression: the drive phase interpolated its target heading RAW -
        # the robot held a wrapped -180 while atan2() reported +180 for the
        # -x direction, so the mock pirouetted a full 360 deg per straight
        # chunk (the "dancing" seen on a straight line)
        """A straight run along -x keeps its heading: no 360 deg pirouette."""
        self._transact(p.CMD_SET_POSE, p.put_i16(26000) + p.put_i16(12000) + p.put_i16(1500))
        status, _ = self._transact(
            p.CMD_GOTO, p.put_i16(23500) + p.put_i16(12000) + p.put_i16(-1800) + p.put_u16(500)
        )
        self.assertEqual(status, p.ST_OK)
        swept = 0.0
        previous = 150.0
        deadline = time.monotonic() + 5.0
        snapshot = None
        while time.monotonic() < deadline:
            snapshot = odometry(self.link)
            wrapped = math.fmod(snapshot.theta_deg - previous + 180.0, 360.0)
            if wrapped < 0.0:
                wrapped += 360.0
            swept += wrapped - 180.0
            previous = snapshot.theta_deg
            if snapshot.state == p.STATE_IDLE:
                break
            time.sleep(0.05)
        self.assertAlmostEqual(snapshot.x_mm, 2350.0, delta=5.0)
        self.assertAlmostEqual(snapshot.theta_deg, -180.0, delta=1.0)
        self.assertLess(abs(swept), 60.0, f"heading swept {swept:.0f} deg while driving straight")

    def test_goto_reaches_heading_the_short_way(self):
        # regression: every GOTO turn phase must normalize through the +-180
        # boundary (170 -> 0 -> -170 = 340 deg of turns), not sweep the long
        # way (two 190 deg turns = 380 deg: looked like a random full spin)
        """Goto reaches heading the short way."""
        self._transact(p.CMD_SET_POSE, p.put_i16(0) + p.put_i16(0) + p.put_i16(1700))
        status, _ = self._transact(
            p.CMD_GOTO, p.put_i16(1000) + p.put_i16(0) + p.put_i16(-1700) + p.put_u16(200)
        )
        self.assertEqual(status, p.ST_OK)
        snapshot = wait_idle(self.link, timeout_s=5.0)
        self.assertAlmostEqual(snapshot.theta_deg, -170.0, delta=1.0)
        # wheel odometry: dtheta = (right - left) / track ; translation cancels
        status, payload = self._transact(p.CMD_QUERY_TICKS)
        self.assertEqual(status, p.ST_OK)
        mm_per_tick = (
            robot_config.DEFAULTS["odom_wheel_circumference_mm"]
            / 10.0
            / robot_config.DEFAULTS["odom_ticks_per_rev"]
        )
        track_mm = robot_config.DEFAULTS["odom_track_mm"] / 10.0
        left = p.get_i32(payload, 0) * mm_per_tick
        right = p.get_i32(payload, 4) * mm_per_tick
        swept_deg = math.degrees((right - left) / track_mm)
        # firmware phase parity: 170 + 170 deg of turns; the long way would be 380
        self.assertLess(abs(swept_deg), 360.0, f"swept too far: {swept_deg:.0f} deg")
        self.assertGreater(abs(swept_deg), 300.0, f"did not turn enough: {swept_deg:.0f} deg")

    def test_goto_turns_in_place_before_driving(self):
        # firmware parity: GOTO is turn-to-heading, drive straight, turn to
        # the final heading - never translate while rotating (the blended
        # mock made the robot look like it was sliding sideways between an
        # arc and the next straight leg)
        """GOTO turns in place first, then drives straight."""
        status, _ = self._transact(
            p.CMD_GOTO, p.put_i16(1000) + p.put_i16(1000) + p.put_i16(0) + p.put_u16(120)
        )
        self.assertEqual(status, p.ST_OK)
        time.sleep(0.25)
        turning = odometry(self.link)
        self.assertEqual(turning.state, p.STATE_GOTO)
        self.assertLess(abs(turning.x_mm), 1.0)
        self.assertLess(abs(turning.y_mm), 1.0)
        self.assertGreater(abs(turning.theta_deg), 5.0)
        snapshot = wait_idle(self.link, timeout_s=5.0)
        self.assertAlmostEqual(snapshot.x_mm, 100.0, delta=2.0)
        self.assertAlmostEqual(snapshot.y_mm, 100.0, delta=2.0)
        self.assertAlmostEqual(snapshot.theta_deg, 0.0, delta=1.0)

    def test_busy_rejection(self):
        """Busy rejection."""
        self._transact(p.CMD_DRIVE, p.put_i16(300) + p.put_u16(100))
        status, _ = self._transact(p.CMD_TURN, p.put_i16(900) + p.put_u16(100))
        self.assertEqual(status, p.ST_BUSY)
        wait_idle(self.link)

    def test_stop_freezes(self):
        """Stop freezes."""
        self._transact(p.CMD_DRIVE, p.put_i16(1000) + p.put_u16(100))
        time.sleep(0.2)
        status, _ = self._transact(p.CMD_STOP)
        self.assertEqual(status, p.ST_OK)
        frozen = odometry(self.link)
        time.sleep(0.2)
        still = odometry(self.link)
        self.assertAlmostEqual(frozen.x_mm, still.x_mm, delta=0.5)
        self.assertEqual(still.state, p.STATE_IDLE)

    def test_estop_latches_until_clear(self):
        """Estop latches until clear."""
        status, _ = self._transact(p.CMD_ESTOP)
        self.assertEqual(status, p.ST_OK)
        snapshot = odometry(self.link)
        self.assertEqual(snapshot.state, p.STATE_ESTOP)
        self.assertTrue(snapshot.faults & p.FAULT_ESTOP)
        status, _ = self._transact(p.CMD_DRIVE, p.put_i16(100) + p.put_u16(100))
        self.assertEqual(status, p.ST_FAULT)  # latched fault blocks motion
        status, _ = self._transact(p.CMD_CLEAR_FAULTS)
        self.assertEqual(status, p.ST_OK)
        snapshot = odometry(self.link)
        self.assertEqual(snapshot.faults, 0)

    def test_status_payload(self):
        """Status payload."""
        status, payload = self._transact(p.CMD_QUERY_STATUS)
        self.assertEqual(status, p.ST_OK)
        decoded = Status.parse(payload)
        # mock parity with the firmware define: 3.5 (+ RELEASE_MOTORS)
        self.assertEqual((decoded.fw_major, decoded.fw_minor), (3, 5))
        self.assertEqual(decoded.state, p.STATE_IDLE)
        self.assertEqual(decoded.cfg_map_version, robot_config.MAP_VERSION)
        # the mock records its boot cause like the firmware does
        self.assertEqual(decoded.reset_cause, p.EVENT_RESET_POR)

    def test_ticks_move_with_odometry(self):
        """Ticks move with odometry."""
        self._transact(p.CMD_DRIVE, p.put_i16(100) + p.put_u16(255))
        wait_idle(self.link)
        status, payload = self._transact(p.CMD_QUERY_TICKS)
        self.assertEqual(status, p.ST_OK)
        left, right = p.get_i32(payload, 0), p.get_i32(payload, 4)
        self.assertAlmostEqual(left * 0.05, 100.0, delta=5.0)
        self.assertAlmostEqual(right * 0.05, 100.0, delta=5.0)

    def test_arc_moves_and_turns_together(self):
        # left quarter circle R=300 from (0,0) heading 0: end at (300,300), +90
        """Arc moves and turns together."""
        status, _ = self._transact(p.CMD_ARC, p.put_i16(300) + p.put_i16(900) + p.put_u16(255))
        self.assertEqual(status, p.ST_OK)
        self.assertEqual(odometry(self.link).state, p.STATE_ARC)
        snapshot = wait_idle(self.link)
        self.assertAlmostEqual(snapshot.x_mm, 300.0, delta=5.0)
        self.assertAlmostEqual(snapshot.y_mm, 300.0, delta=5.0)
        self.assertAlmostEqual(snapshot.theta_deg, 90.0, delta=1.0)

    def test_arc_rejects_inconsistent_signs(self):
        """Arc rejects inconsistent signs."""
        bad = p.put_i16(300) + p.put_i16(-900) + p.put_u16(120)  # signs disagree
        self.assertEqual(self._transact(p.CMD_ARC, bad)[0], p.ST_RANGE)
        zero_radius = p.put_i16(0) + p.put_i16(900) + p.put_u16(120)
        self.assertEqual(self._transact(p.CMD_ARC, zero_radius)[0], p.ST_RANGE)
        zero_sweep = p.put_i16(300) + p.put_i16(0) + p.put_u16(120)
        self.assertEqual(self._transact(p.CMD_ARC, zero_sweep)[0], p.ST_RANGE)

    # ---------------------------------------------------------- settings
    def _get_regs(self, offset=0, count=robot_config.REGISTER_COUNT):
        """Get regs."""
        values = []
        while len(values) < count:
            step = min(robot_config.GET_CHUNK_MAX, count - len(values))
            status, payload = self._transact(
                p.CMD_GET_CONFIG, p.put_u8(offset + len(values)) + p.put_u8(step)
            )
            self.assertEqual(status, p.ST_OK)
            number = p.get_u8(payload, 1)
            values.extend(p.get_u16(payload, 2 + 2 * slot) for slot in range(number))
        return offset, values

    def test_config_defaults_roundtrip(self):
        """Config defaults roundtrip."""
        base, values = self._get_regs()
        self.assertEqual(base, 0)
        self.assertEqual(values, robot_config.default_raw())

    def test_config_write_validation_and_roundtrip(self):
        """Config write validation and roundtrip."""
        index = robot_config.REGISTER_BY_KEY["wheel_diameter_mm"].index
        status, _ = self._transact(p.CMD_SET_CONFIG, p.put_u8(index) + p.put_u8(1) + p.put_u16(610))
        self.assertEqual(status, p.ST_OK)
        _, values = self._get_regs()
        self.assertEqual(values[index], 610)
        # read-only map version is rejected
        status, _ = self._transact(p.CMD_SET_CONFIG, p.put_u8(0) + p.put_u8(1) + p.put_u16(9))
        self.assertEqual(status, p.ST_RANGE)
        # out-of-range value is rejected, nothing applied
        status, _ = self._transact(p.CMD_SET_CONFIG, p.put_u8(index) + p.put_u8(1) + p.put_u16(50))
        self.assertEqual(status, p.ST_RANGE)
        _, values = self._get_regs()
        self.assertEqual(values[index], 610)

    def test_config_save_load_reset(self):
        """Config save load reset."""
        index = robot_config.REGISTER_BY_KEY["max_speed_mm_s"].index
        self._transact(p.CMD_SET_CONFIG, p.put_u8(index) + p.put_u8(1) + p.put_u16(500))
        self.assertEqual(self._transact(p.CMD_SAVE_CONFIG)[0], p.ST_OK)
        self.assertEqual(self._transact(p.CMD_RESET_CONFIG)[0], p.ST_OK)
        _, values = self._get_regs()
        self.assertEqual(values[index], robot_config.DEFAULTS["max_speed_mm_s"])
        self.assertEqual(self._transact(p.CMD_LOAD_CONFIG)[0], p.ST_OK)
        _, values = self._get_regs()
        self.assertEqual(values[index], 500)

    def test_config_refused_while_busy(self):
        """Config refused while busy."""
        self._transact(p.CMD_DRIVE, p.put_i16(2000) + p.put_u16(100))
        index = robot_config.REGISTER_BY_KEY["max_speed_mm_s"].index
        status, _ = self._transact(p.CMD_SET_CONFIG, p.put_u8(index) + p.put_u8(1) + p.put_u16(500))
        self.assertEqual(status, p.ST_BUSY)
        self.assertEqual(self._transact(p.CMD_SAVE_CONFIG)[0], p.ST_BUSY)
        self._transact(p.CMD_STOP)
        wait_idle(self.link)

    def test_events_are_recorded_and_readable(self):
        # estop + a rejected arc leave two events with context
        """Events are recorded and readable."""
        self._transact(p.CMD_ESTOP)
        self._transact(p.CMD_CLEAR_FAULTS)
        bad = p.put_i16(300) + p.put_i16(-900) + p.put_u16(120)
        self._transact(p.CMD_ARC, bad)
        status, payload = self._transact(p.CMD_QUERY_EVENTS, p.put_u16(0))
        self.assertEqual(status, p.ST_OK)
        latest = p.get_u16(payload, 0)
        count = p.get_u8(payload, 4)
        codes = [p.get_u8(payload, 5 + 4 * slot + 2) for slot in range(count)]
        self.assertIn(p.EVENT_ESTOP, codes)
        self.assertIn(p.EVENT_BAD_PARAM, codes)
        self.assertEqual(p.event_severity(p.EVENT_ESTOP), "error")
        # nothing new after the latest index
        status, payload = self._transact(p.CMD_QUERY_EVENTS, p.put_u16(latest))
        self.assertEqual(p.get_u8(payload, 4), 0)


class QueueArcTests(unittest.TestCase):
    """QUEUE_ARC: one pending slot, consumed as a blended continuation."""

    def setUp(self):
        """Park the simulator at the origin like the other mock tests."""
        self.link = MockLink()
        self._transact(p.CMD_SET_POSE, p.put_i16(0) + p.put_i16(0) + p.put_i16(0))

    def _transact(self, cmd, payload=b""):
        """Send one command through the raw frame API."""
        response = self.link.transact(p.build_request(1, cmd, payload))
        return p.parse_response(response)

    @staticmethod
    def _arc(radius, sweep_deg, speed=150):
        """ARC / QUEUE_ARC payload."""
        return p.put_i16(radius) + p.put_i16(round(sweep_deg * 10.0)) + p.put_u16(speed)

    def test_queue_while_idle_starts_the_arc_immediately(self):
        """Queue while idle starts the arc immediately."""
        status, _ = self._transact(p.CMD_QUEUE_ARC, self._arc(400, 40))
        self.assertEqual(status, p.ST_OK)
        self.assertEqual(odometry(self.link).state, p.STATE_ARC)

    def test_queued_arc_continues_without_stopping(self):
        # two arcs in a row: the second must start the instant the first ends
        """Queued arc continues without stopping."""
        self._transact(p.CMD_ARC, self._arc(300, 30))
        self.assertEqual(self.link.blend_count, 0)
        self.assertEqual(self._transact(p.CMD_QUEUE_ARC, self._arc(300, 30))[0], p.ST_OK)
        snapshot = wait_idle(self.link, timeout_s=6.0)
        self.assertEqual(self.link.blend_count, 1)   # blended, never idled between
        self.assertEqual(snapshot.segments_done, 2)  # both segments completed
        self.assertAlmostEqual(snapshot.theta_deg, 60.0, delta=1.5)

    def test_second_queued_arc_is_rejected_while_one_waits(self):
        """Second queued arc is rejected while one waits."""
        self._transact(p.CMD_ARC, self._arc(300, 30))
        self.assertEqual(self._transact(p.CMD_QUEUE_ARC, self._arc(300, 30))[0], p.ST_OK)
        self.assertEqual(self._transact(p.CMD_QUEUE_ARC, self._arc(300, 30))[0], p.ST_BUSY)
        self._transact(p.CMD_STOP)
        wait_idle(self.link)

    def test_stop_clears_the_pending_arc(self):
        """Stop clears the pending arc."""
        self._transact(p.CMD_ARC, self._arc(300, 30))
        self.assertEqual(self._transact(p.CMD_QUEUE_ARC, self._arc(300, 30))[0], p.ST_OK)
        self._transact(p.CMD_STOP)
        wait_idle(self.link)
        self.assertEqual(self.link.blend_count, 0)
        self.assertEqual(odometry(self.link).segments_done, 0)

    def test_faults_block_queue_arc(self):
        """Faults block queue arc."""
        self._transact(p.CMD_ESTOP)
        self.assertEqual(self._transact(p.CMD_QUEUE_ARC, self._arc(300, 30))[0], p.ST_FAULT)


if __name__ == "__main__":
    unittest.main()
