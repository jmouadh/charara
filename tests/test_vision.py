"""Camera / vision obstacle input tests.

Covers the ``/ws/vision`` channel end to end: snapshots are accepted and
merged into every plan, TTL expiry drops stale reports, and a report that
changes the obstacle map while the robot is executing triggers the same
stop -> re-plan the remainder -> continue cycle as a hand-drawn obstacle.

File:    tests/test_vision.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

import math
import os
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ["ROBOT_TRANSPORT"] = "mock"

try:
    from fastapi.testclient import TestClient
except ImportError:  # pragma: no cover
    TestClient = None


def wait_for(predicate, timeout_s=10.0, step_s=0.05):
    """Poll ``predicate`` until it is truthy or the timeout expires."""
    deadline = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < deadline:
        last = predicate()
        if last:
            return last
        time.sleep(step_s)
    return last


# a mid-run re-plan stops the executor, computes the new plan and restarts it:
# for a moment the status reads "aborted" with this reason - a transient, not
# a mission end
REPLAN_ABORT_MESSAGE = "re-planning the remaining route (dynamic obstacle)"


def wait_mission_end(client, timeout_s=45.0):
    """Wait for the mission to really end: "done" or a sticky abort.

    The stop->re-plan->restart swap of a mid-run re-plan shows "aborted" for a
    moment; a test polling the status must not mistake that for a completion.
    """

    def probe():
        """Probe the mission status: done, sticky abort, or None while running."""
        state = client.get("/api/state").json()
        exec_state = state.get("exec", {})
        status = exec_state.get("status")
        if status == "done":
            return "done"
        if status == "aborted" and exec_state.get("message") != REPLAN_ABORT_MESSAGE:
            return "aborted: " + str(exec_state.get("message", ""))
        return None

    return wait_for(probe, timeout_s=timeout_s)


@unittest.skipIf(TestClient is None, "fastapi/httpx not installed")
class VisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Prepare the shared fixtures for this test class."""
        from software.console.main import app

        cls._ctx = TestClient(app)
        cls.client = cls._ctx.__enter__()
        wait_for(lambda: cls.client.get("/api/state").json().get("connected"), timeout_s=5.0)

    @classmethod
    def tearDownClass(cls):
        # never leave camera obstacles behind for the other suites
        """Release the shared fixtures of this test class."""
        with cls.client.websocket_connect("/ws/vision") as websocket:
            websocket.receive_json()  # hello
            websocket.send_json({"type": "clear"})
            websocket.receive_json()  # ack
        cls._ctx.__exit__(None, None, None)

    def _pose(self, x, y, theta=0.0):
        """Place the robot at (x, y) and wait for telemetry to agree."""
        self.client.post("/api/pose", json={"x_mm": x, "y_mm": y, "theta_deg": theta})
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - x) < 1.0)

    def _straight_body(self):
        """Mission body: one straight stroke at a fixed cruise speed."""
        return {
            "elements": [{"kind": "stroke", "points": [[500.0, 500.0], [2500.0, 500.0]]}],
            "options": {"speed_mm_s": 255, "goto_step_mm": 250, "avoid_obstacles": True, "smooth": False},
        }

    def test_camera_obstacles_are_merged_into_plans(self):
        """Camera obstacles are merged into plans."""
        self._pose(500.0, 500.0)
        body = self._straight_body()
        baseline = self.client.post("/api/plan", json=body).json()
        self.assertFalse(any("re-routed" in warning for warning in baseline["warnings"]))

        with self.client.websocket_connect("/ws/vision") as websocket:
            hello = websocket.receive_json()
            self.assertEqual(hello["type"], "hello")
            websocket.send_json({
                "type": "obstacles",
                "obstacles": [{"kind": "rect", "x": 1500.0, "y": 400.0, "w": 200.0, "h": 200.0}],
                "ttl_ms": 30000,
                "t": 1738073000123.4,
            })
            ack = websocket.receive_json()
            self.assertEqual(ack["type"], "ack")
            self.assertEqual(ack["count"], 1)
            self.assertTrue(ack["changed"])
            self.assertEqual(ack["t"], 1738073000123.4)   # the camera clock is echoed
            self.assertGreater(ack["t_server"], 1e12)     # and the service stamps itself

            state = self.client.get("/api/state").json()
            self.assertEqual(state["vision"]["count"], 1)
            self.assertEqual(state["vision"]["t"], 1738073000123.4)

            # the console request does not mention the camera obstacle, yet
            # the plan must respect it (merged service-side)
            replanned = self.client.post("/api/plan", json=body).json()
            self.assertTrue(
                any("re-routed" in warning for warning in replanned["warnings"]),
                replanned["warnings"],
            )
            x0, y0, x1, y1 = 1500.0, 400.0, 1700.0, 600.0
            gap = min(
                math.hypot(max(x0 - px, 0.0, px - x1), max(y0 - py, 0.0, py - y1))
                for px, py in replanned["path"]
            )
            self.assertGreater(gap, 200.0, "the plan must keep clear of the camera obstacle")

            websocket.send_json({"type": "clear"})
            ack = websocket.receive_json()
            self.assertTrue(ack["changed"])

        self.assertEqual(self.client.get("/api/state").json()["vision"]["count"], 0)
        again = self.client.post("/api/plan", json=body).json()
        self.assertFalse(any("re-routed" in warning for warning in again["warnings"]))

    def test_opponent_pose_is_a_planning_obstacle(self):
        """The opponent pose must keep planned robot motion outside its footprint."""
        self._pose(500.0, 500.0)
        body = self._straight_body()
        with self.client.websocket_connect("/ws/vision") as websocket:
            websocket.receive_json()  # hello
            websocket.send_json({
                "type": "poses",
                "opponent": {"x_mm": 1500.0, "y_mm": 500.0, "theta_deg": 0.0},
                "ttl_ms": 60000,
            })
            ack = websocket.receive_json()
            self.assertTrue(ack["seen"]["opponent"])

            result = self.client.post("/api/plan", json=body).json()
            nearest = min(math.dist((px, py), (1500.0, 500.0)) for px, py in result["path"])
            self.assertGreater(
                nearest, 430.0,
                "the planned path must clear both robots' physical footprints",
            )

    def test_camera_report_expires_with_its_ttl(self):
        """Camera report expires with its TTL."""
        with self.client.websocket_connect("/ws/vision") as websocket:
            websocket.receive_json()  # hello
            websocket.send_json({
                "type": "obstacles",
                "obstacles": [{"kind": "circle", "x": 800.0, "y": 800.0, "r": 120.0}],
                "ttl_ms": 200,
            })
            websocket.receive_json()  # ack
            self.assertEqual(self.client.get("/api/state").json()["vision"]["count"], 1)
            expired = wait_for(
                lambda: self.client.get("/api/state").json()["vision"]["count"] == 0,
                timeout_s=3.0,
            )
            self.assertTrue(expired, "the snapshot must expire with its TTL")

    def test_camera_disconnect_releases_the_channel(self):
        # the console (a second client) must be able to tell WHO owns the
        # channel: the hello carries each client's own peer id, the reports
        # carry the sender's name, and when the owner disconnects the source
        # is released (the console takes its mock camera role back)
        """Camera disconnect releases the channel."""
        with self.client.websocket_connect("/ws/vision") as websocket:
            hello = websocket.receive_json()
            self.assertTrue(hello.get("peer"), "the hello must identify the client")
            websocket.send_json({
                "type": "obstacles",
                "client": "console",
                "obstacles": [{"kind": "rect", "x": 2400.0, "y": 1600.0, "w": 300.0, "h": 300.0}],
                "ttl_ms": 60000,
            })
            websocket.receive_json()  # ack
            vision = wait_for(
                lambda: (lambda v: v if v and v["count"] == 1 else None)(
                    self.client.get("/api/state").json().get("vision")),
                timeout_s=5.0,
            )
            self.assertIsNotNone(vision, "the report must reach the service")
            self.assertEqual(vision["source"], hello["peer"])
            self.assertEqual(vision["client"], "console")

        # disconnected: snapshot AND ownership are gone
        released = wait_for(
            lambda: (lambda v: v if v and v["count"] == 0 and v["source"] == "" else None)(
                self.client.get("/api/state").json().get("vision")),
            timeout_s=5.0,
        )
        self.assertIsNotNone(released, "the disconnect must release the channel")
        self.assertEqual(released["client"], "")

    def test_reports_without_a_client_name_are_real_camera_input(self):
        # a real camera/script does not call itself "console" - that is what
        # the console watches to yield to it
        """Reports without a client name are real camera input."""
        with self.client.websocket_connect("/ws/vision") as websocket:
            websocket.receive_json()  # hello
            websocket.send_json({
                "type": "obstacles",
                "obstacles": [{"kind": "rect", "x": 1000.0, "y": 1000.0, "w": 300.0, "h": 300.0}],
                "ttl_ms": 60000,
            })
            websocket.receive_json()  # ack
            vision = wait_for(
                lambda: (lambda v: v if v and v["count"] == 1 else None)(
                    self.client.get("/api/state").json().get("vision")),
                timeout_s=5.0,
            )
            self.assertIsNotNone(vision)
            self.assertEqual(vision["client"], "")
            self.assertTrue(vision["source"], "the peer id stays the source")

    def test_camera_obstacle_mid_run_stops_and_replans(self):
        """Camera obstacle mid run stops and replans."""
        self._pose(500.0, 500.0)
        body = self._straight_body()
        plan = self.client.post("/api/plan", json=body).json()
        self.client.post("/api/execute", json={"segments": plan["segments"], "deadman_ms": 0})
        started = wait_for(
            lambda: (lambda state: state if 600.0 < state.get("x_mm", 0.0) < 1200.0
                     and state.get("exec", {}).get("status") == "running" else None)(
                self.client.get("/api/state").json()),
            timeout_s=30.0,
        )
        self.assertIsNotNone(started, "the robot never got into the run")
        first_total = self.client.get("/api/state").json().get("exec", {}).get("total")

        with self.client.websocket_connect("/ws/vision") as websocket:
            websocket.receive_json()  # hello
            websocket.send_json({
                "type": "obstacles",
                "obstacles": [{"kind": "rect", "x": 1900.0, "y": 380.0, "w": 200.0, "h": 240.0}],
                "ttl_ms": 60000,
            })
            websocket.receive_json()  # ack

            # far ahead: nothing happens ("smart" proximity) - the robot
            # keeps driving until it gets close, then the service re-checks
            # ON ITS OWN (no new camera message) and swaps the remainder:
            # the plan document is replaced (the segment COUNT can coincide,
            # so watch the document itself)
            for _ in range(10):
                state = self.client.get("/api/state").json()
                self.assertEqual(state.get("exec", {}).get("status"), "running")
                time.sleep(0.04)
            first_segments = plan["segments"]

            def rerouted():
                """Plan document replaced by the re-routed remainder, else None."""
                document = self.client.get("/api/plan").json()
                return document if document.get("segments") != first_segments else None

            routed_plan = wait_for(rerouted, timeout_s=20.0)
            self.assertIsNotNone(routed_plan, "the service never reacted when the obstacle got close")
            # the reaction is deliberately LATE (reaction ladder, 2026-10-09):
            # the robot creeps once the blockage is inside CAUTION_GAP_MM and
            # only stops/swaps inside STOP_GAP_MM, so the new plan starts well
            # past the old ~1 m horizon - and never at the mission's start
            self.assertGreater(routed_plan["path"][0][0], 1300.0)
            self.assertLess(routed_plan["path"][0][0], 1700.0)

            # the swapped remainder was re-routed around the obstacle: grab
            # the detour document as soon as it exists - the watcher may swap
            # AGAIN while the robot moves, so the LAST /api/plan document is
            # not necessarily the one that carries the detour. The detour is
            # the plan whose path keeps clear of the obstacle.
            x0, y0, x1, y1 = 1900.0, 380.0, 2100.0, 620.0

            def detour_plan():
                """Plan document whose path clears the obstacle, else None."""
                document = self.client.get("/api/plan").json()
                gap = min(
                    math.hypot(max(x0 - px, 0.0, px - x1), max(y0 - py, 0.0, py - y1))
                    for px, py in document["path"]
                )
                return document if gap > 200.0 else None

            routed_plan = wait_for(detour_plan, timeout_s=5.0)
            self.assertIsNotNone(routed_plan, "the close obstacle must re-route the remainder")
            self.assertTrue(
                any(
                    "re-routed" in warning or "kept as drawn" in warning or "squeezed" in warning
                    for warning in routed_plan["warnings"]
                ),
                routed_plan["warnings"],
            )

            # the mission still completes on the re-planned remainder
            final = wait_for(
                lambda: "done" if self.client.get("/api/state").json().get("exec", {}).get("status") == "done" else None,
                timeout_s=45.0,
            )
            self.assertEqual(final, "done")

            end = wait_for(
                lambda: (lambda state: state if abs(state.get("x_mm", 0.0) - 2500.0) < 80.0 else None)(
                    self.client.get("/api/state").json()),
                timeout_s=10.0,
            )
            self.assertIsNotNone(end, "the robot did not finish the mission")
            websocket.send_json({"type": "clear"})
            websocket.receive_json()

    def test_replanned_route_is_pushed_to_the_console(self):
        # the console map must redraw the REGENERATED route after a dynamic
        # obstacle re-plan: the service broadcasts the new plan document over
        # /ws so the shown path matches what the robot will actually drive
        """Replanned route is pushed to the console."""
        self._pose(500.0, 500.0)
        plan = self.client.post("/api/plan", json=self._straight_body()).json()
        self.client.post("/api/execute", json={"segments": plan["segments"], "deadman_ms": 0})
        started = wait_for(
            lambda: (lambda state: state if 600.0 < state.get("x_mm", 0.0) < 900.0
                     and state.get("exec", {}).get("status") == "running" else None)(
                self.client.get("/api/state").json()),
            timeout_s=30.0,
        )
        self.assertIsNotNone(started, "the robot never got into the run")

        with self.client.websocket_connect("/ws") as console:
            with self.client.websocket_connect("/ws/vision") as websocket:
                websocket.receive_json()  # hello
                websocket.send_json({
                    "type": "obstacles",
                    "obstacles": [{"kind": "rect", "x": 1400.0, "y": 380.0, "w": 200.0, "h": 240.0}],
                    "ttl_ms": 60000,
                })
                websocket.receive_json()  # ack
                # drain the console stream until the plan broadcast arrives
                # (telemetry/log messages flow in between). The reaction is
                # deliberately late now (the ladder creeps first), so give it a
                # time budget rather than a message count.
                pushed = None
                deadline = time.monotonic() + 30.0
                while time.monotonic() < deadline:
                    message = console.receive_json()
                    if message.get("type") == "plan":
                        pushed = message.get("data") or {}
                        break
                self.assertIsNotNone(pushed, "the re-planned route was never pushed to the console")
                self.assertIn("stats", pushed)
                self.assertIn("path", pushed)
                self.assertGreaterEqual(len(pushed["path"]), 2)
                self.assertNotEqual(
                    pushed["segments"], plan["segments"],
                    "the broadcast must carry the NEW program, not the old one",
                )
                websocket.send_json({"type": "clear"})
                websocket.receive_json()

        final = wait_mission_end(self.client, timeout_s=45.0)
        self.assertEqual(final, "done")

    def test_blocked_waypoint_is_waited_for_then_reached(self):
        # an opponent parked ON the mission's next waypoint: after reaching
        # the free waypoints first, the robot WAITS for the opponent to move
        # (instead of driving a compromise approach that "visits" the point
        # 27 cm short - or ignoring it forever) and then goes for it exactly
        """Blocked waypoint is waited for then reached."""
        self._pose(500.0, 500.0)
        body = {
            "elements": [
                {"kind": "waypoint", "x": 800.0, "y": 500.0, "theta_deg": 0.0},
                {"kind": "waypoint", "x": 1800.0, "y": 500.0, "theta_deg": 0.0},
            ],
            "options": {
                "speed_mm_s": 255, "goto_step_mm": 250, "avoid_obstacles": True, "smooth": False,
                "robot_width_mm": 300.0, "robot_height_mm": 300.0, "obstacle_margin_mm": 150.0,
            },
        }
        plan = self.client.post("/api/plan", json=body).json()
        self.client.post("/api/execute", json={"segments": plan["segments"], "deadman_ms": 0})
        started = wait_for(
            lambda: (lambda state: state if state.get("exec", {}).get("status") == "running"
                     and state.get("x_mm", 0.0) > 550.0 else None)(
                self.client.get("/api/state").json()),
            timeout_s=30.0,
        )
        self.assertIsNotNone(started, "the robot never got into the run")

        with self.client.websocket_connect("/ws/vision") as websocket:
            websocket.receive_json()  # hello
            # the opponent parks ON the second waypoint (1800, 500)
            websocket.send_json({
                "type": "obstacles",
                "obstacles": [{"kind": "rect", "x": 1700.0, "y": 400.0, "w": 200.0, "h": 200.0}],
                "ttl_ms": 60000,
            })
            websocket.receive_json()  # ack

            # the robot first goes to (and reaches) the FREE waypoint...
            arrived = wait_for(
                lambda: (lambda state: state if math.hypot(state.get("x_mm", 0.0) - 800.0,
                                                            state.get("y_mm", 0.0) - 500.0) < 90.0 else None)(
                    self.client.get("/api/state").json()),
                timeout_s=25.0,
            )
            self.assertIsNotNone(arrived, "the robot did not reach the free waypoint")

            # ...then WAITS: the next waypoint is occupied by the moving obstacle
            waiting = wait_for(lambda: self.client.get("/api/state").json().get("waiting"), timeout_s=15.0)
            self.assertIsNotNone(waiting, "the robot never waited for the blocked waypoint")
            aborted = wait_for(
                lambda: (lambda state: state if state.get("exec", {}).get("status") == "aborted" else None)(
                    self.client.get("/api/state").json()),
                timeout_s=10.0,
            )
            self.assertIsNotNone(aborted, "the run must stop while waiting")
            self.assertIn("waiting", aborted["exec"]["message"].lower())
            # the stop must happen CLOSE to the blocked checkpoint: a robot
            # crossing 1.4 m ahead must not freeze our robot from far away
            time.sleep(0.3)  # let the stop deceleration settle for the check
            stop = self.client.get("/api/state").json()
            self.assertLess(
                math.hypot(stop["x_mm"] - 1800.0, stop["y_mm"] - 500.0),
                750.0,
                "the robot must not stop from far away",
            )

            # the opponent moves away before the bounded wait expires: the
            # robot must still go for the waypoint exactly
            websocket.send_json({"type": "obstacles", "obstacles": [], "ttl_ms": 60000})
            websocket.receive_json()  # ack

        resumed = wait_for(
            lambda: (lambda status: status if status == "running" else None)(
                self.client.get("/api/state").json().get("exec", {}).get("status")),
            timeout_s=15.0,
        )
        self.assertEqual(resumed, "running", "the robot must resume by itself once the waypoint frees")
        final = wait_mission_end(self.client, timeout_s=45.0)
        self.assertEqual(final, "done")
        end = wait_for(
            lambda: (lambda state: state if math.hypot(state.get("x_mm", 0.0) - 1800.0,
                                                        state.get("y_mm", 0.0) - 500.0) < 80.0 else None)(
                self.client.get("/api/state").json()),
            timeout_s=15.0,
        )
        self.assertIsNotNone(end, "the robot did not reach the blocked waypoint after it freed up")

    def test_blocked_waypoint_is_skipped_after_wait_cap(self):
        # the bounded wait is NEVER forever: an opponent that stays parked on
        # the next waypoint gets it skipped after ~3 s and the mission
        # continues with the NEXT waypoint (the stop used to last until the
        # obstacle happened to move - the robot looked frozen)
        """Blocked waypoint is skipped after the wait cap."""
        self._pose(500.0, 500.0)
        body = {
            "elements": [
                {"kind": "waypoint", "x": 800.0, "y": 500.0, "theta_deg": 0.0},
                {"kind": "waypoint", "x": 1800.0, "y": 500.0, "theta_deg": 0.0},
                {"kind": "waypoint", "x": 2500.0, "y": 500.0, "theta_deg": 0.0},
            ],
            "options": {
                "speed_mm_s": 255, "goto_step_mm": 250, "avoid_obstacles": True, "smooth": False,
                "robot_width_mm": 300.0, "robot_height_mm": 300.0, "obstacle_margin_mm": 150.0,
            },
        }
        plan = self.client.post("/api/plan", json=body).json()
        self.client.post("/api/execute", json={"segments": plan["segments"], "deadman_ms": 0})
        started = wait_for(
            lambda: (lambda state: state if state.get("exec", {}).get("status") == "running"
                     and state.get("x_mm", 0.0) > 550.0 else None)(
                self.client.get("/api/state").json()),
            timeout_s=30.0,
        )
        self.assertIsNotNone(started, "the robot never got into the run")

        with self.client.websocket_connect("/ws/vision") as websocket:
            websocket.receive_json()  # hello
            # the opponent parks ON the second waypoint (1800, 500) and STAYS
            websocket.send_json({
                "type": "obstacles",
                "obstacles": [{"kind": "rect", "x": 1700.0, "y": 400.0, "w": 200.0, "h": 200.0}],
                "ttl_ms": 120000,
            })
            websocket.receive_json()  # ack

            waiting = wait_for(lambda: self.client.get("/api/state").json().get("waiting"), timeout_s=25.0)
            self.assertIsNotNone(waiting, "the robot never waited for the blocked waypoint")
            # the wait first STOPS the run (the status still reads "running"
            # for a moment when the waiting flag appears - wait for the stop)
            stopped = wait_for(
                lambda: (lambda state: state if state.get("exec", {}).get("status") == "aborted" else None)(
                    self.client.get("/api/state").json()),
                timeout_s=10.0,
            )
            self.assertIsNotNone(stopped, "the run must stop while waiting")
            # the obstacle does NOT move: the bounded wait must resolve itself
            # (skip the unreachable waypoint and continue)
            resumed = wait_for(
                lambda: (lambda status: status if status == "running" else None)(
                    self.client.get("/api/state").json().get("exec", {}).get("status")),
                timeout_s=8.0,
            )
            self.assertEqual(resumed, "running", "the bounded wait must resolve by itself")
            self.assertFalse(
                self.client.get("/api/state").json().get("waiting"),
                "the wait state must be cleared by the bounded resolution",
            )

        # the mission continues with the NEXT waypoint and the skipped one is
        # never "visited" on the way (the opponent is still parked on it)
        min_gap = float("inf")
        final = None
        deadline = time.time() + 60.0
        while time.time() < deadline:
            state = self.client.get("/api/state").json()
            gap = math.hypot(state.get("x_mm", 0.0) - 1800.0, state.get("y_mm", 0.0) - 500.0)
            min_gap = min(min_gap, gap)
            if state.get("exec", {}).get("status") == "done":
                final = "done"
                break
            time.sleep(0.2)
        self.assertEqual(final, "done", "the mission must finish after the skip")
        self.assertGreater(min_gap, 150.0, "the skipped waypoint must not be visited anyway")
        end = wait_for(
            lambda: (lambda state: state if math.hypot(state.get("x_mm", 0.0) - 2500.0,
                                                        state.get("y_mm", 0.0) - 500.0) < 80.0 else None)(
                self.client.get("/api/state").json()),
            timeout_s=15.0,
        )
        self.assertIsNotNone(end, "the robot did not continue to the next waypoint")

    def test_sealed_route_waits_then_resumes_when_cleared(self):
        # a moving obstacle that seals the way (a full-height wall) must never
        # be driven through: every detour is blocked, so the robot stops and
        # waits; when the way clears it continues on its own
        """Sealed route waits then resumes when cleared."""
        self._pose(500.0, 500.0)
        body = self._straight_body()
        plan = self.client.post("/api/plan", json=body).json()
        self.client.post("/api/execute", json={"segments": plan["segments"], "deadman_ms": 0})
        started = wait_for(
            lambda: (lambda state: state if state.get("x_mm", 0.0) > 550.0
                     and state.get("exec", {}).get("status") == "running" else None)(
                self.client.get("/api/state").json()),
            timeout_s=30.0,
        )
        self.assertIsNotNone(started, "the robot never got into the run")

        with self.client.websocket_connect("/ws/vision") as websocket:
            websocket.receive_json()  # hello
            websocket.send_json({
                "type": "obstacles",
                "obstacles": [{"kind": "rect", "x": 1480.0, "y": 0.0, "w": 240.0, "h": 2000.0}],
                "ttl_ms": 60000,
            })
            websocket.receive_json()  # ack

            waiting = wait_for(lambda: self.client.get("/api/state").json().get("waiting"), timeout_s=15.0)
            self.assertIsNotNone(waiting, "the robot did not wait for the moving obstacle")
            aborted = wait_for(
                lambda: (lambda state: state if state.get("exec", {}).get("status") == "aborted" else None)(
                    self.client.get("/api/state").json()),
                timeout_s=10.0,
            )
            self.assertIsNotNone(aborted, "the run must stop while waiting")
            self.assertIn("waiting", aborted["exec"]["message"].lower())
            # the abort status is published when STOP is sent; the board still
            # decelerates for a few mm, so measure the hold once it is IDLE
            settled = wait_for(
                lambda: (lambda state: state if state.get("state_name") == "IDLE" else None)(
                    self.client.get("/api/state").json()),
                timeout_s=10.0,
            )
            self.assertIsNotNone(settled, "the robot never settled while waiting")
            stopped_at = self.client.get("/api/state").json()["x_mm"]
            time.sleep(0.4)
            self.assertLess(
                abs(self.client.get("/api/state").json()["x_mm"] - stopped_at),
                5.0,
                "the robot must hold its position while waiting",
            )

            # the other robot moves away: the route is free and the robot goes
            websocket.send_json({"type": "obstacles", "obstacles": [], "ttl_ms": 60000})
            websocket.receive_json()  # ack
            resumed = wait_for(
                lambda: (lambda status: status if status == "running" else None)(
                    self.client.get("/api/state").json().get("exec", {}).get("status")),
                timeout_s=15.0,
            )
            self.assertEqual(resumed, "running", "the robot must resume by itself")
            final = wait_mission_end(self.client, timeout_s=45.0)
            self.assertEqual(final, "done")
            end = wait_for(
                lambda: (lambda state: state if abs(state.get("x_mm", 0.0) - 2500.0) < 80.0 else None)(
                    self.client.get("/api/state").json()),
                timeout_s=10.0,
            )
            self.assertIsNotNone(end, "the robot did not finish after the resume")

    def test_manual_stop_cancels_the_wait(self):
        # STOP while waiting must clear the waiting state: the robot must not
        # lurch off on its own later when the obstacle happens to move
        """Manual stop cancels the wait."""
        self._pose(500.0, 500.0)
        body = self._straight_body()
        plan = self.client.post("/api/plan", json=body).json()
        self.client.post("/api/execute", json={"segments": plan["segments"], "deadman_ms": 0})
        wait_for(lambda: self.client.get("/api/state").json().get("x_mm", 0.0) > 550.0, timeout_s=30.0)

        with self.client.websocket_connect("/ws/vision") as websocket:
            websocket.receive_json()  # hello
            websocket.send_json({
                "type": "obstacles",
                "obstacles": [{"kind": "rect", "x": 1480.0, "y": 0.0, "w": 240.0, "h": 2000.0}],
                "ttl_ms": 60000,
            })
            websocket.receive_json()  # ack
            waiting = wait_for(lambda: self.client.get("/api/state").json().get("waiting"), timeout_s=15.0)
            self.assertIsNotNone(waiting, "the robot did not wait")

            self.assertEqual(self.client.post("/api/stop").status_code, 200)
            cleared = wait_for(lambda: not self.client.get("/api/state").json().get("waiting"), timeout_s=3.0)
            self.assertTrue(cleared, "STOP must clear the waiting state")

            # clear the camera: the robot must stay stopped (a human stopped it)
            websocket.send_json({"type": "obstacles", "obstacles": [], "ttl_ms": 60000})
            websocket.receive_json()  # ack
            time.sleep(0.6)
            self.assertNotEqual(self.client.get("/api/state").json().get("exec", {}).get("status"), "running")

    def test_far_obstacle_on_the_route_is_watched_then_crept(self):
        # the reaction LADDER (2026-10-09): an obstacle standing on the route
        # far ahead must NOT stop the robot (another robot usually moves on;
        # re-planning from far away stopped it for a detour it never needed).
        # Inside CAUTION_GAP_MM it creeps at CAUTION_SCALE of the plan's own
        # speeds - no stop, no program swap - and only inside STOP_GAP_MM does
        # it stop and resolve the blockage.
        """A far obstacle on the route is watched, then crept past."""
        self._pose(500.0, 500.0)
        body = self._straight_body()
        plan = self.client.post("/api/plan", json=body).json()
        self.client.post("/api/execute", json={"segments": plan["segments"], "deadman_ms": 0})
        started = wait_for(
            lambda: (lambda state: state if state.get("x_mm", 0.0) > 550.0
                     and state.get("exec", {}).get("status") == "running" else None)(
                self.client.get("/api/state").json()),
            timeout_s=30.0,
        )
        self.assertIsNotNone(started, "the robot never got into the run")

        with self.client.websocket_connect("/ws/vision") as websocket:
            websocket.receive_json()  # hello
            websocket.send_json({
                "type": "obstacles",
                "obstacles": [{"kind": "rect", "x": 2100.0, "y": 380.0, "w": 200.0, "h": 240.0}],
                "ttl_ms": 60000,
            })
            websocket.receive_json()  # ack

            # 1.8 m ahead: far. The robot must keep driving on the SAME program
            deadline = time.monotonic() + 2.5
            while time.monotonic() < deadline:
                state = self.client.get("/api/state").json()
                self.assertEqual(
                    state.get("exec", {}).get("status"), "running",
                    f"the robot stopped for a far obstacle: {state.get('exec')}",
                )
                self.assertEqual(
                    self.client.get("/api/plan").json().get("segments"), plan["segments"],
                    "the program must not be swapped while the obstacle is far away",
                )
                self.assertIsNone(state.get("waiting"))
                time.sleep(0.1)

            # as it advances into CAUTION_GAP_MM: creep, still no swap and no stop
            crept = wait_for(
                lambda: (lambda state: state if state.get("exec", {}).get("status") == "running"
                         and float(state.get("exec", {}).get("speed_scale", 1.0)) < 1.0 else None)(
                    self.client.get("/api/state").json()),
                timeout_s=25.0,
            )
            self.assertIsNotNone(crept, "the robot never slowed down for the obstacle on its route")
            self.assertEqual(
                self.client.get("/api/plan").json().get("segments"), plan["segments"],
                "creeping must NOT swap the program",
            )
            self.assertLess(float(crept["exec"]["speed_scale"]), 1.0)

            websocket.send_json({"type": "obstacles", "obstacles": [], "ttl_ms": 60000})
            websocket.receive_json()  # ack
            # the obstacle is gone: the caution must lift on its own
            full = wait_for(
                lambda: (lambda state: state if float(state.get("exec", {}).get("speed_scale", 1.0)) >= 1.0
                         else None)(self.client.get("/api/state").json()),
                timeout_s=15.0,
            )
            self.assertIsNotNone(full, "the caution never lifted after the obstacle cleared")

        self.assertEqual(self.client.post("/api/stop").status_code, 200)
        wait_for(
            lambda: self.client.get("/api/state").json().get("exec", {}).get("status") != "running",
            timeout_s=10.0,
        )


if __name__ == "__main__":
    unittest.main()
