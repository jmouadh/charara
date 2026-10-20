"""Executor tail rebuild - dynamic-obstacle re-plan unit tests.

``remaining_elements`` turns the not-yet-executed part of a running program
back into planner elements: consecutive GOTO nodes become a stroke, a lone
node becomes a pass-through waypoint, rotations and pauses are preserved.

File:    tests/test_executor.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

import asyncio
import math
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from software.core.mission.executor import (  # noqa: E402
    EXEC_ABORTED,
    EXEC_DONE,
    EXEC_IDLE,
    EXEC_PAUSED,
    Executor,
    follow_cruise_mm_s,
    remaining_elements,
)
from software.robot import protocol as robot_protocol  # noqa: E402
from software.robot.client import RobotClient, RobotError  # noqa: E402
from software.robot.link import LinkError, MockLink  # noqa: E402


def _mock_arc(start, th0_deg, radius, sweep_deg, speed=200):
    """ARC segment dict with the geometry the board/MockLink will drive.

    The board builds the circle from the CURRENT pose and heading, so a CCW
    arc (+sweep) has its centre 90 deg left of ``th0`` and ends ``sweep``
    degrees further on that circle.
    """
    cx = start[0] + radius * math.cos(math.radians(th0_deg + 90.0))
    cy = start[1] + radius * math.sin(math.radians(th0_deg + 90.0))
    end_deg = th0_deg - 90.0 + sweep_deg  # start sits at th0 - 90 on the circle
    return {
        "kind": "arc",
        "x0": start[0],
        "y0": start[1],
        "x1": round(cx + radius * math.cos(math.radians(end_deg)), 1),
        "y1": round(cy + radius * math.sin(math.radians(end_deg)), 1),
        "radius_mm": float(radius),
        "sweep_deg": float(sweep_deg),
        "th0_deg": float(th0_deg),
        "v": speed,
    }


class ArcChainTests(unittest.IsolatedAsyncioTestCase):
    """Executor + MockLink: a tangent arc chain runs with no stop at all."""

    async def asyncSetUp(self):
        """Fresh mock board parked at the origin."""
        self.link = MockLink()
        self.client = RobotClient(self.link, retries=1)
        await self.client.set_pose(0.0, 0.0, 0.0)

    async def test_arc_chain_blends_every_junction(self):
        """Arc chain blends every junction."""
        executor = Executor(self.client, deadman_ms=0, timeout_slack_s=8.0, poll_s=0.02)
        segments = [
            {"kind": "arc", "radius_mm": 300.0, "sweep_deg": 20.0, "v": 200},
            {"kind": "arc", "radius_mm": 300.0, "sweep_deg": 20.0, "v": 200},
            {"kind": "arc", "radius_mm": 300.0, "sweep_deg": 20.0, "v": 200},
        ]
        await executor.start(segments)
        await executor.wait()
        self.assertEqual(executor.state, EXEC_DONE)
        self.assertEqual(self.link.blend_count, 2)  # both junctions blended
        final = await self.client.odometry()
        self.assertEqual(final.segments_done, 3)  # executed exactly once each
        self.assertAlmostEqual(final.theta_deg, 60.0, delta=1.5)

    async def test_chain_still_completes_after_pause_and_resume(self):
        """Chain still completes after pause and resume."""
        executor = Executor(self.client, deadman_ms=0, timeout_slack_s=8.0, poll_s=0.02)
        segments = [
            {"kind": "arc", "radius_mm": 300.0, "sweep_deg": 20.0, "v": 200},
            {"kind": "arc", "radius_mm": 300.0, "sweep_deg": 20.0, "v": 200},
            {"kind": "arc", "radius_mm": 300.0, "sweep_deg": 20.0, "v": 200},
        ]
        await executor.start(segments)
        executor.pause()
        for _ in range(250):
            if executor.state == EXEC_PAUSED:
                break
            await asyncio.sleep(0.02)
        self.assertEqual(executor.state, EXEC_PAUSED)
        executor.resume()
        await executor.wait()
        self.assertEqual(executor.state, EXEC_DONE)


class _ReleaseSpyLink(MockLink):
    """Mock board that records when RELEASE_MOTORS was executed."""

    def __init__(self):
        super().__init__()
        self.release_calls = 0
        self.release_while_busy = False

    def _execute(self, cmd, payload):
        if cmd == robot_protocol.CMD_RELEASE_MOTORS:
            self.release_calls += 1
            self.release_while_busy = self._busy() or bool(self._goto_phases)
        return super()._execute(cmd, payload)


class ReleaseMotorTests(unittest.IsolatedAsyncioTestCase):
    """Release runs after a move, and the next move re-energizes the drivers."""

    async def asyncSetUp(self):
        """Fresh mock board parked at the origin."""
        self.link = _ReleaseSpyLink()
        self.client = RobotClient(self.link, retries=1)
        await self.client.set_pose(0.0, 0.0, 0.0)

    async def test_release_step_runs_idle_and_next_move_reenergizes(self):
        """A release step is idle-only, then the following move re-energizes."""
        executor = Executor(self.client, deadman_ms=0, timeout_slack_s=8.0, poll_s=0.02)
        segments = [
            {"kind": "drive", "distance_mm": 100.0, "v": 300},
            {"kind": "release"},
            {"kind": "pause", "seconds": 0.1},
            {"kind": "drive", "distance_mm": 50.0, "v": 300},
        ]
        await executor.start(segments)
        await executor.wait()
        self.assertEqual(executor.state, EXEC_DONE)
        self.assertEqual(self.link.release_calls, 1)
        self.assertFalse(self.link.release_while_busy)
        self.assertFalse(self.link.motors_released)  # the next drive re-energized
        self.assertEqual((await self.client.odometry()).segments_done, 2)

    async def test_board_rejects_release_while_moving(self):
        """The board itself refuses a release while a segment is active."""
        await self.client.drive(1000.0, 100)
        with self.assertRaises(RobotError) as raised:
            await self.client.release_motors()
        self.assertEqual(raised.exception.status, robot_protocol.ST_BUSY)
        self.assertFalse(self.link.motors_released)
        await self.client.stop()


class _TinyBudgetExecutor(Executor):
    """Executor with tiny watchdogs so the chain window is easy to observe."""

    def _segment_timeout(self, segment):
        kind = segment.get("kind")
        if kind == "turn":
            return 1.0
        if kind == "arc":
            return 1.5
        return 10.0


class _StuckTurnLink(MockLink):
    """A board that accepts a TURN but never finishes it (watchdog test)."""

    def _execute(self, cmd, payload):
        if cmd == robot_protocol.CMD_TURN:
            now = time.monotonic()
            self._motion = {
                "x0": self._x, "y0": self._y, "th0": self._th,
                "x1": self._x, "y1": self._y, "th1": self._th,
                "t0": now, "t1": now + 3600.0,
            }
            self._state = robot_protocol.STATE_TURN
            return robot_protocol.ST_OK, b""
        return super()._execute(cmd, payload)


class ChainWatchdogTests(unittest.IsolatedAsyncioTestCase):
    """A chained run may outlive the FIRST segment's fixed budget.

    Real-robot rehearsal (2026-10-01): a plan [arc chain, turn -8.9 deg, long
    arc chain] made the pump queue the arcs AFTER the turn while the turn was
    awaited; the board blended through them and ran ~12 s, but the turn's
    watchdog window was a FIXED 10 s computed at entry -> "segment timeout"
    aborted a perfectly healthy run (the robot stopped mid-chain and the UI
    showed a bare "aborted"). The window must grow with every arc the pump
    queues as a continuation.
    """

    async def asyncSetUp(self):
        self.link = MockLink()
        self.client = RobotClient(self.link, retries=1)
        await self.client.set_pose(0.0, 0.0, 0.0)

    async def test_chain_beyond_the_entry_budget_finishes_normally(self):
        """Turn + 4 blended arcs: ~5.6 s of board time, still no abort."""
        segments = [
            {"kind": "turn", "angle_deg": 90.0, "v": 400},   # ~0.4 s pivot
            {"kind": "arc", "radius_mm": 300.0, "sweep_deg": 30.0, "v": 120},
            {"kind": "arc", "radius_mm": 300.0, "sweep_deg": 30.0, "v": 120},
            {"kind": "arc", "radius_mm": 300.0, "sweep_deg": 30.0, "v": 120},
            {"kind": "arc", "radius_mm": 300.0, "sweep_deg": 30.0, "v": 120},
        ]
        executor = _TinyBudgetExecutor(self.client, deadman_ms=0, poll_s=0.02)
        events = []

        async def sink(event):
            events.append(event)

        executor.set_event_sink(sink)
        await executor.start(segments)
        await executor.wait()
        self.assertEqual(executor.state, EXEC_DONE)
        self.assertEqual(self.link._segments_done, 5)  # every segment ran once
        aborts = [e for e in events if e.get("data", {}).get("status") == EXEC_ABORTED]
        self.assertEqual(aborts, [])


class AbortDiagnosticsTests(unittest.IsolatedAsyncioTestCase):
    """The abort message must keep the real cause, not a bare "aborted"."""

    async def test_segment_timeout_is_reported_not_masked(self):
        link = _StuckTurnLink()
        client = RobotClient(link, retries=1)
        await client.set_pose(0.0, 0.0, 0.0)
        executor = _TinyBudgetExecutor(client, deadman_ms=0, poll_s=0.02)
        events = []

        async def sink(event):
            events.append(event)

        executor.set_event_sink(sink)
        await executor.start([{"kind": "turn", "angle_deg": 90.0, "v": 150}])
        await executor.wait()
        self.assertEqual(executor.state, EXEC_ABORTED)
        self.assertEqual(executor._message, "segment timeout")  # not "aborted"
        self.assertEqual(events[-1]["data"]["message"], "segment timeout")

    async def test_plain_stop_still_reads_aborted(self):
        link = MockLink()
        client = RobotClient(link, retries=1)
        await client.set_pose(0.0, 0.0, 0.0)
        executor = Executor(client, deadman_ms=0, timeout_slack_s=8.0, poll_s=0.02)
        await executor.start([{"kind": "arc", "radius_mm": 300.0, "sweep_deg": 180.0, "v": 150}])
        await asyncio.sleep(0.3)
        await executor.stop()
        self.assertEqual(executor.state, EXEC_ABORTED)
        self.assertEqual(executor._message, "aborted")

    async def test_busy_send_is_waited_out_not_aborted(self):
        """A send that lands while a queued arc winds down waits, then runs.

        Real-robot rehearsal (2026-10-01): the pump queued a continuation ARC
        exactly as the board went idle; the next segment send answered BUSY
        and the old code aborted the whole mission with
        "segment 12: robot status BUSY". The send must wait for the board and
        retry instead.
        """
        link = MockLink()
        client = RobotClient(link, retries=1)
        await client.set_pose(0.0, 0.0, 0.0)
        await client.arc(300.0, 20.0, 100)  # ~1 s of board motion
        executor = Executor(client, deadman_ms=0, timeout_slack_s=8.0, poll_s=0.02)
        sent = await executor._send_segment({"kind": "turn", "angle_deg": 45.0, "v": 150})
        self.assertTrue(sent)
        for _ in range(200):
            if (await client.odometry()).is_idle:
                break
            await asyncio.sleep(0.02)
        final = await client.odometry()
        self.assertGreater(final.theta_deg, 50.0)  # the arc ran AND the turn ran


class _FlakyQueueLink(MockLink):
    """QUEUE_ARC answers that fail the given number of times first.

    ``status=None`` raises a TRANSPORT error - what the board's I2C recovery
    storms look like to the client; any other value is a board status.
    """

    def __init__(self, failures=1, status=None):
        super().__init__()
        self._failures = int(failures)
        self._status = status
        self.queue_attempts = 0

    def _execute(self, cmd, payload):
        if cmd == robot_protocol.CMD_QUEUE_ARC:
            self.queue_attempts += 1
            if self.queue_attempts <= self._failures:
                if self._status is None:
                    raise LinkError("I2C transaction failed after 5 tries: errno 121")
                return self._status, b""
        return super()._execute(cmd, payload)


class _FlakyOdometryLink(MockLink):
    """Odometry reads that fail at the TRANSPORT level for a few polls."""

    def __init__(self, failures=3, first=3):
        super().__init__()
        self._failures = int(failures)
        self._first = int(first)
        self.odometry_calls = 0

    def _execute(self, cmd, payload):
        if cmd == robot_protocol.CMD_QUERY_ODOMETRY:
            self.odometry_calls += 1
            if self._first <= self.odometry_calls < self._first + self._failures:
                raise LinkError("I2C transaction failed after 5 tries: errno 121")
        return super()._execute(cmd, payload)


class ChainResilienceTests(unittest.IsolatedAsyncioTestCase):
    """A chain must ride out the board's I2C stalls (real run 2026-10-07).

    The board was logging I2C_RECOVER every ~10 s and the old pump disabled
    chaining - or aborted - on ANY queueing failure: a 7 s plan of blended
    arcs crawled for 29 s, stopped at every junction, and (every ARC is
    driven RELATIVE from the CURRENT pose) the run ended ~70 mm off the
    drawing.
    """

    _CHAIN = [
        {"kind": "arc", "radius_mm": 300.0, "sweep_deg": 20.0, "v": 200},
        {"kind": "arc", "radius_mm": 300.0, "sweep_deg": 20.0, "v": 200},
        {"kind": "arc", "radius_mm": 300.0, "sweep_deg": 20.0, "v": 200},
    ]

    async def _run(self, link):
        client = RobotClient(link, retries=1)
        await client.set_pose(0.0, 0.0, 0.0)
        executor = Executor(client, deadman_ms=0, timeout_slack_s=8.0, poll_s=0.02)
        await executor.start(list(self._CHAIN))
        await executor.wait()
        return executor, client

    async def test_transport_error_retries_instead_of_killing_the_chain(self):
        """Two failed queue frames: the third attempt still blends it all."""
        link = _FlakyQueueLink(failures=2, status=None)
        executor, client = await self._run(link)
        self.assertEqual(executor.state, EXEC_DONE)
        self.assertGreaterEqual(link.queue_attempts, 3, "the queue frame must be retried")
        self.assertGreaterEqual(link.blend_count, 2, "the chain must still blend")
        self.assertEqual((await client.odometry()).segments_done, 3)

    async def test_old_board_disables_chaining_but_still_completes(self):
        """UNKNOWN_CMD means no QUEUE_ARC: fall back, never abort."""
        link = _FlakyQueueLink(failures=99, status=robot_protocol.ST_UNKNOWN_CMD)
        executor, client = await self._run(link)
        self.assertEqual(executor.state, EXEC_DONE)
        self.assertEqual(link.blend_count, 0)
        self.assertEqual((await client.odometry()).segments_done, 3)

    async def test_a_rejected_arc_is_skipped_not_fatal(self):
        """A RANGE on one arc: never queue THAT arc, keep chaining the rest."""
        link = _FlakyQueueLink(failures=1, status=robot_protocol.ST_RANGE)
        executor, client = await self._run(link)
        self.assertEqual(executor.state, EXEC_DONE)
        self.assertEqual(link.queue_attempts, 2, "the rejected arc must not be retried")
        self.assertEqual(link.blend_count, 1, "the following arc still blends")
        self.assertEqual((await client.odometry()).segments_done, 3)

    async def test_transient_telemetry_loss_does_not_abort(self):
        """A few unreadable odometry polls are waited out, not fatal."""
        link = _FlakyOdometryLink(failures=3, first=3)
        client = RobotClient(link, retries=1)
        await client.set_pose(0.0, 0.0, 0.0)
        executor = Executor(client, deadman_ms=0, timeout_slack_s=8.0, poll_s=0.02)
        await executor.start([{"kind": "arc", "radius_mm": 300.0, "sweep_deg": 20.0, "v": 200}])
        await executor.wait()
        self.assertEqual(executor.state, EXEC_DONE)
        self.assertEqual((await client.odometry()).segments_done, 1)


class ProgramRepairTests(unittest.IsolatedAsyncioTestCase):
    """Executor + MockLink: a discontinuous program is repaired, not run wrong.

    The board drives every ARC from the CURRENT pose and heading, so a
    program whose geometry jumps is executed from the wrong circle - the
    real-robot incident (2026-10-01) ended the mission ~450 mm away from the
    waypoint while reporting "completed". The executor must detect the break,
    repair it (GOTO to the arc's declared start, or an in-place TURN for a
    heading kink) and log a warning instead of completing somewhere else.
    """

    async def asyncSetUp(self):
        """Fresh mock board parked at the origin."""
        self.link = MockLink()
        self.client = RobotClient(self.link, retries=1)
        await self.client.set_pose(0.0, 0.0, 0.0)

    async def _run(self, segments):
        """Run ``segments`` on the executor; return it plus every sink event."""
        executor = Executor(self.client, deadman_ms=0, timeout_slack_s=8.0, poll_s=0.02)
        events = []

        async def sink(event):
            events.append(event)

        executor.set_event_sink(sink)
        await executor.start(segments)
        await executor.wait()
        return executor, events

    async def test_gap_before_arc_is_repaired_with_a_goto(self):
        """A jump before an ARC becomes a GOTO to its declared start."""
        # the arc declares its start at (500, 0) but the previous exit is at
        # (200, 0): without the repair the arc would be driven from (200, 0)
        # and the mission would end ~300 mm away from the planned exit
        arc = _mock_arc((500.0, 0.0), 0.0, 300.0, 20.0)
        executor, events = await self._run(
            [
                {"kind": "goto", "x": 200.0, "y": 0.0, "th": 0.0, "v": 200},
                arc,
            ]
        )
        self.assertEqual(executor.state, EXEC_DONE)
        warnings = [event for event in events if event.get("type") == "log"]
        self.assertTrue(
            any("plan discontinuity" in event["data"]["message"] for event in warnings), warnings
        )
        final = await self.client.odometry()
        self.assertAlmostEqual(final.x_mm, arc["x1"], delta=15.0)
        self.assertAlmostEqual(final.y_mm, arc["y1"], delta=15.0)
        self.assertAlmostEqual(final.theta_deg, 20.0, delta=2.0)

    async def test_kink_before_arc_is_repaired_with_a_turn(self):
        """An entry-heading mismatch becomes an in-place TURN first."""
        # the robot arrives at (300, 0) facing 0 deg but the arc must be
        # entered on 30 deg: turn 30 deg in place, then drive the arc
        arc = _mock_arc((300.0, 0.0), 30.0, 300.0, 20.0)
        executor, events = await self._run(
            [
                {"kind": "goto", "x": 300.0, "y": 0.0, "th": 0.0, "v": 200},
                arc,
            ]
        )
        self.assertEqual(executor.state, EXEC_DONE)
        warnings = [event for event in events if event.get("type") == "log"]
        self.assertTrue(
            any("plan kink" in event["data"]["message"] for event in warnings), warnings
        )
        final = await self.client.odometry()
        self.assertAlmostEqual(final.x_mm, arc["x1"], delta=20.0)
        self.assertAlmostEqual(final.y_mm, arc["y1"], delta=20.0)
        self.assertAlmostEqual(final.theta_deg, 50.0, delta=2.0)


class RemainingElementsTests(unittest.TestCase):
    def test_tail_becomes_stroke_with_actions(self):
        """Tail becomes stroke with actions."""
        segments = [
            {"kind": "goto", "x": 0.0, "y": 0.0, "th": 0.0, "v": 200},
            {"kind": "goto", "x": 100.0, "y": 0.0, "th": 0.0, "v": 200},
            {"kind": "goto", "x": 200.0, "y": 0.0, "th": 0.0, "v": 200},
            {"kind": "turn", "angle_deg": 90.0, "v": 150},
            {"kind": "pause", "seconds": 2.0},
            {"kind": "goto", "x": 200.0, "y": 100.0, "th": 90.0, "v": 200},
            {"kind": "goto", "x": 200.0, "y": 200.0, "th": 90.0, "v": 200},
        ]
        elements = remaining_elements(segments, 2)
        self.assertEqual([entry["kind"] for entry in elements], ["waypoint", "rotate", "pause", "stroke"])
        self.assertEqual(elements[0]["x"], 200.0)
        self.assertEqual(elements[1]["angle_deg"], 90.0)
        self.assertEqual(elements[2]["seconds"], 2.0)
        self.assertEqual(elements[3]["points"], [[200.0, 100.0], [200.0, 200.0]])

    def test_completed_tail_is_empty(self):
        """Completed tail is empty."""
        segments = [{"kind": "goto", "x": 0.0, "y": 0.0, "th": 0.0, "v": 200}]
        self.assertEqual(remaining_elements(segments, 1), [])

    def test_release_action_is_preserved_in_the_tail(self):
        """A dynamic re-plan must not silently drop a motor release step."""
        segments = [
            {"kind": "goto", "x": 0.0, "y": 0.0, "th": 0.0, "v": 200},
            {"kind": "release"},
            {"kind": "pause", "seconds": 1.0},
            {"kind": "goto", "x": 200.0, "y": 0.0, "th": 0.0, "v": 200},
        ]
        elements = remaining_elements(segments, 0)
        self.assertEqual(elements[0], {"kind": "waypoint", "x": 0.0, "y": 0.0})
        self.assertEqual(elements[1], {"kind": "release"})
        self.assertEqual(elements[2], {"kind": "pause", "seconds": 1.0})

    def test_single_trailing_node_becomes_waypoint(self):
        """Single trailing node becomes waypoint."""
        segments = [
            {"kind": "goto", "x": 0.0, "y": 0.0, "th": 0.0, "v": 200},
            {"kind": "turn", "angle_deg": -45.0, "v": 150},
            {"kind": "goto", "x": 500.0, "y": 500.0, "th": 0.0, "v": 200},
        ]
        elements = remaining_elements(segments, 0)
        self.assertEqual([entry["kind"] for entry in elements], ["waypoint", "rotate", "waypoint"])
        self.assertEqual((elements[0]["x"], elements[0]["y"]), (0.0, 0.0))
        self.assertEqual((elements[2]["x"], elements[2]["y"]), (500.0, 500.0))
        self.assertEqual(elements[1]["angle_deg"], -45.0)

    def test_arc_tail_keeps_its_geometry(self):
        # an unexecuted ARC contributes its preview points to the stroke so
        # the re-planned tail still bends where the arc bent
        """Arc tail keeps its geometry."""
        segments = [
            {"kind": "goto", "x": 0.0, "y": 0.0, "th": 0.0, "v": 150},
            {
                "kind": "arc",
                "radius_mm": -300.0,
                "sweep_deg": -90.0,
                "points": [[0.0, 0.0], [80.0, -40.0], [200.0, -80.0], [300.0, -100.0]],
                "v": 150,
            },
            {"kind": "goto", "x": 400.0, "y": -100.0, "th": 0.0, "v": 150},
        ]
        elements = remaining_elements(segments, 0)
        self.assertEqual([entry["kind"] for entry in elements], ["stroke"])
        points = elements[0]["points"]
        self.assertIn([80.0, -40.0], points)
        self.assertIn([300.0, -100.0], points)
        self.assertEqual(points[-1], [400.0, -100.0])

    def test_partial_departure_keeps_the_rest_of_the_reverse(self):
        """A re-plan during the backup must finish the reverse first.

        Live regression (2026-10-19): a vision re-plan interrupted a
        close-waypoint departure after ~34 mm. The rebuilt route started
        turning there and swept the body into the wall. The remaining
        signed DRIVE to the turn-safe point must stay in front of the tail.
        """
        segments = [
            {"kind": "goto", "x": 1131.0, "y": 872.0, "th": -90.0, "v": 200},
            {"kind": "goto", "x": 1131.0, "y": 657.0, "th": -90.0, "v": 200,
             "close_waypoint": True},
            {"kind": "drive", "distance_mm": -215.0, "v": 150,
             "departure": True, "points": [[1131.0, 657.0], [1131.0, 872.0]]},
            {"kind": "goto", "x": 1167.8, "y": 874.1, "th": -10.0, "v": 181},
            {"kind": "goto", "x": 1181.4, "y": 871.7, "th": -22.6, "v": 148},
        ]
        elements = remaining_elements(segments, 2, (1131.0, 691.0))
        self.assertEqual(elements[0], {"kind": "drive", "distance_mm": -181.0})
        self.assertEqual(elements[1]["kind"], "stroke")
        self.assertEqual(elements[1]["points"][0], [1167.8, 874.1])
        # without a pose the old behaviour (skip the in-flight drive) stays
        legacy = remaining_elements(segments, 2)
        self.assertNotEqual(legacy[0]["kind"], "drive")

    def test_final_close_leg_keeps_the_upcoming_departure(self):
        """A re-plan on the last straight leg must not skip the reverse.

        Live regression (2026-10-19): the proximity watchdog re-planned 4 mm
        before the close waypoint, dropped the departure, faced the diagonal
        route there and swept a body corner into the wall.
        """
        segments = [
            {"kind": "goto", "x": 1131.0, "y": 872.0, "th": -90.0, "v": 200},
            {"kind": "goto", "x": 1131.0, "y": 657.0, "th": -90.0, "v": 200,
             "close_waypoint": True},
            {"kind": "drive", "distance_mm": -215.0, "v": 150,
             "departure": True, "points": [[1131.0, 657.0], [1131.0, 872.0]]},
            {"kind": "goto", "x": 1167.8, "y": 874.1, "th": -10.0, "v": 181},
        ]
        elements = remaining_elements(segments, 1, (1131.0, 661.2))
        self.assertEqual(elements[0], {"kind": "waypoint", "x": 1131.0, "y": 657.0})
        self.assertEqual(elements[1], {"kind": "drive", "distance_mm": -215.0})
        self.assertEqual(
            elements[2], {"kind": "waypoint", "x": 1167.8, "y": 874.1}
        )


class CautionScaleTests(unittest.IsolatedAsyncioTestCase):
    """The dynamic-obstacle caution slows the run without re-planning it."""

    async def test_speed_scale_slows_the_commands_and_a_new_run_resets_it(self):
        """set_speed_scale scales the commands mid-run; start() restores 1.0."""
        link = MockLink()
        client = RobotClient(link, retries=1)
        await client.set_pose(0.0, 0.0, 0.0)
        sent = []
        original = client.goto

        async def spy(x, y, theta, speed_mm_s):
            """Record the speed the executor actually asks the board for."""
            sent.append(speed_mm_s)
            await original(x, y, theta, speed_mm_s)

        client.goto = spy
        executor = Executor(client, deadman_ms=0, poll_s=0.02)

        async def sink(event):
            """The dynamic obstacle appears while segment 1 is being sent."""
            data = event.get("data") or {}
            if event.get("type") == "exec" and int(data.get("index", -1)) == 1:
                executor.set_speed_scale(0.5)

        executor.set_event_sink(sink)
        segments = [
            {"kind": "goto", "x": 100.0, "y": 0.0, "th": 0.0, "v": 200},
            {"kind": "goto", "x": 200.0, "y": 0.0, "th": 0.0, "v": 200},
        ]
        await executor.start(segments)
        await executor.wait()
        self.assertEqual(executor.state, EXEC_DONE)
        self.assertEqual(sent, [200, 100], "the command after the caution must be at 50 %")
        sent.clear()
        executor.set_event_sink(None)  # the caution sink is gone (the obstacle did)
        executor.set_speed_scale(0.5)  # a fresh run must start at full speed anyway
        await executor.start(segments)
        await executor.wait()
        self.assertEqual(sent, [200, 200])
        self.assertAlmostEqual(executor.progress["speed_scale"], 1.0)


class StaleDeadmanRecoveryTests(unittest.IsolatedAsyncioTestCase):
    """A DEADMAN latch left by the previous stop must not kill the next run."""

    async def asyncSetUp(self):
        """Fresh mock board parked at (100, 0)."""
        self.link = MockLink()
        self.client = RobotClient(self.link, retries=1)
        await self.client.set_pose(100.0, 0.0, 0.0)

    async def test_start_clears_a_stale_deadman_latch(self):
        """The stop -> re-plan -> restart cycle must survive a latched deadman.

        A console-driven stop can leave the board's deadman latched when the
        link went quiet while the wheels were winding down (the mid-run re-plan
        used to block the poller for over a second on the Pi). The next run then
        aborted with "robot status FAULT" and every resume attempt failed the
        same way: the robot stood "waiting" for good with nothing driving it
        (2026-10-09). Starting a run clears that one bit - and says so.
        """
        await self.client.set_deadman(150)
        await self.client.drive(400.0, 100.0)
        await asyncio.sleep(0.4)  # starved: the watchdog latches mid-move
        odometry = await self.client.odometry()
        self.assertTrue(odometry.faults & robot_protocol.FAULT_DEADMAN)
        self.assertFalse(odometry.is_idle)
        await self.client.stop()  # STOP is not blocked by a latched fault
        await self.client.set_deadman(0)
        executor = Executor(self.client, deadman_ms=0, poll_s=0.02)
        await executor.start([{"kind": "goto", "x": -100.0, "y": 0.0, "th": 180.0, "v": 150}])
        await executor.wait()
        self.assertEqual(executor.state, EXEC_DONE, executor.progress)
        self.assertEqual((await self.client.odometry()).faults, 0)

    async def test_other_faults_still_block_the_run(self):
        """Only DEADMAN is auto-cleared: a latched E-STOP must still abort."""
        await self.client.estop()
        executor = Executor(self.client, deadman_ms=0, poll_s=0.02)
        await executor.start([{"kind": "goto", "x": -100.0, "y": 0.0, "th": 180.0, "v": 150}])
        await executor.wait()
        self.assertEqual(executor.state, EXEC_ABORTED)
        self.assertIn("fault", str(executor.progress.get("message", "")).lower())


class FollowCruiseTests(unittest.TestCase):
    """The cruise speed derived for a streaming run (median of the plan)."""

    def test_median_of_the_planned_speeds(self):
        """One slow docking segment does not drag the whole path down."""
        segments = [
            {"kind": "goto", "v": 500},
            {"kind": "arc", "v": 300},
            {"kind": "goto", "v": 500},
        ]
        self.assertEqual(follow_cruise_mm_s(segments), 500.0)

    def test_even_counts_average_the_middle_pair(self):
        """With an even count the middle pair is averaged."""
        segments = [{"kind": "goto", "v": 200}, {"kind": "goto", "v": 400}]
        self.assertEqual(follow_cruise_mm_s(segments), 300.0)

    def test_default_when_nothing_drives(self):
        """Actions-only programs fall back to the planner's default pace."""
        self.assertEqual(follow_cruise_mm_s([{"kind": "turn", "angle_deg": 90}]), 200.0)


class FollowModeTests(unittest.IsolatedAsyncioTestCase):
    """Executor FOLLOW mode (Track B): the streaming backend over the mock."""

    async def asyncSetUp(self):
        """Fresh mock board facing -y from (315, 1800), like the follower tests."""
        self.link = MockLink()
        self.client = RobotClient(self.link, retries=1)
        await self.client.set_pose(315.0, 1800.0, -90.0)

    @staticmethod
    def _capability(supported: bool):
        """Capability callable answering ``supported`` (session parity)."""
        async def check() -> bool:
            return supported
        return check

    async def test_follow_refused_without_capability(self):
        """No capability callable -> a requested stream refuses to start."""
        executor = Executor(self.client, deadman_ms=0, poll_s=0.02)
        with self.assertRaises(RuntimeError):
            await executor.start(
                [{"kind": "goto", "x": 315.0, "y": 1600.0, "th": -90.0, "v": 300}],
                follow_path=[(315.0, 1800.0), (315.0, 1600.0)],
            )
        self.assertEqual(executor.state, EXEC_IDLE)
        self.assertEqual(self.link.stream_frames, 0)

    async def test_follow_refused_when_board_cannot_stream(self):
        """Capability False (gate off / fw < 3.4) -> refused, nothing moved."""
        executor = Executor(
            self.client,
            deadman_ms=0,
            poll_s=0.02,
            stream_capability=self._capability(False),
        )
        with self.assertRaises(RuntimeError) as ctx:
            await executor.start(
                [{"kind": "goto", "x": 315.0, "y": 1600.0, "th": -90.0, "v": 300}],
                follow_path=[(315.0, 1800.0), (315.0, 1600.0)],
            )
        self.assertIn("refused", str(ctx.exception))
        self.assertEqual(self.link.stream_frames, 0)
        self.assertAlmostEqual((await self.client.odometry()).y_mm, 1800.0, delta=1.0)

    async def test_follow_run_drives_the_path_without_segments(self):
        """The mock closes the loop: streamed frames, no program segment."""
        executor = Executor(
            self.client,
            deadman_ms=1500,
            poll_s=0.02,
            stream_capability=self._capability(True),
        )
        events = []

        async def sink(event):
            events.append(event)

        executor.set_event_sink(sink)
        await executor.start(
            [{"kind": "goto", "x": 315.0, "y": 1500.0, "th": -90.0, "v": 300}],
            follow_path=[(315.0, 1800.0), (315.0, 1500.0)],
        )
        self.assertTrue(executor.last_stream)
        await executor.wait()
        self.assertEqual(executor.state, EXEC_DONE, executor.progress)
        odometry = await self.client.odometry()
        self.assertTrue(odometry.is_idle)
        self.assertAlmostEqual(odometry.y_mm, 1500.0, delta=45.0)
        # the path itself is streamed (no stop-turn segment); the ONLY
        # segment allowed after it is the small closed-loop final correction
        # when the follower stops a few mm off the planned end
        self.assertLessEqual(odometry.segments_done, 1)
        self.assertGreater(self.link.stream_frames, 3)
        self.assertEqual(self.link._deadman_ms, 0)  # armed, then cleared
        follow_events = [
            event["data"] for event in events
            if event.get("type") == "exec" and event["data"].get("mode") == "follow"
        ]
        self.assertTrue(follow_events)
        self.assertIn(follow_events[-1]["status"], ("done",))

    async def test_follow_pause_holds_and_resume_finishes(self):
        """PAUSE holds a streaming run stopped; RESUME drives it to the end."""
        executor = Executor(
            self.client,
            deadman_ms=0,
            poll_s=0.02,
            stream_capability=self._capability(True),
        )
        await executor.start(
            [{"kind": "goto", "x": 315.0, "y": 1200.0, "th": -90.0, "v": 400}],
            follow_path=[(315.0, 1800.0), (315.0, 1200.0)],
        )
        await asyncio.sleep(0.3)
        executor.pause()
        for _ in range(200):
            if executor.state == EXEC_PAUSED:
                break
            await asyncio.sleep(0.02)
        self.assertEqual(executor.state, EXEC_PAUSED)
        self.assertTrue((await self.client.odometry()).is_idle)
        executor.resume()
        await executor.wait()
        self.assertEqual(executor.state, EXEC_DONE, executor.progress)
        self.assertAlmostEqual((await self.client.odometry()).y_mm, 1200.0, delta=60.0)

    async def test_follow_stop_aborts_and_keeps_the_reason(self):
        """STOP ends a streaming run with the operator's reason, robot stopped."""
        executor = Executor(
            self.client,
            deadman_ms=0,
            poll_s=0.02,
            stream_capability=self._capability(True),
        )
        await executor.start(
            [{"kind": "goto", "x": 315.0, "y": 1000.0, "th": -90.0, "v": 400}],
            follow_path=[(315.0, 1800.0), (315.0, 1000.0)],
        )
        await asyncio.sleep(0.3)
        await executor.stop(reason="test stop")
        self.assertEqual(executor.state, EXEC_ABORTED)
        self.assertEqual(executor.progress.get("message"), "test stop")
        self.assertTrue((await self.client.odometry()).is_idle)
        self.assertIsNone(self.link._stream)


if __name__ == "__main__":
    unittest.main()
