"""End-to-end API test: ASGI app + mock robot (pose -> plan -> execute).

Skipped automatically when fastapi/httpx are not installed, so the rest of
the suite keeps running on bare Python.

File:    tests/test_api.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

import math
import os
import re
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


def program_points(segments):
    """Executed polyline: GOTO nodes plus the samples carried inside ARC segments."""
    points = []
    for segment in segments:
        if segment["kind"] == "goto":
            points.append((segment["x"], segment["y"]))
        elif segment["kind"] == "arc":
            points.extend((float(p[0]), float(p[1])) for p in segment.get("points", []))
    return points


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
class ApiEndToEndTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Prepare the shared fixtures for this test class."""
        from software.console.main import app  # imported after ROBOT_TRANSPORT is forced

        cls._ctx = TestClient(app)
        cls.client = cls._ctx.__enter__()
        # Wait until the telemetry poller got its first odometry frame
        wait_for(lambda: cls.client.get("/api/state").json().get("connected"), timeout_s=5.0)

    @classmethod
    def tearDownClass(cls):
        """Release the shared fixtures of this test class."""
        cls._ctx.__exit__(None, None, None)

    def _place_robot(self, x_mm, y_mm, theta_deg):
        """Place the mock robot at a known pose and wait for it to settle.

        The self-test endpoints command motion from the CURRENT pose; the
        shared mock robot must not depend on what a previous test left
        behind (a chained run can drift out of the field).
        """
        self.client.post("/api/pose", json={"x_mm": x_mm, "y_mm": y_mm, "theta_deg": theta_deg})
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - x_mm) < 1.5)

    def test_health_and_state(self):
        """Health and state."""
        health = self.client.get("/api/health").json()
        self.assertTrue(health["ok"])
        self.assertEqual(health["transport"], "mock")
        state = self.client.get("/api/state").json()
        self.assertTrue(state["connected"])
        self.assertIn("state_name", state)

    def test_imu_endpoints_without_imu(self):
        """Mock transport has no IMU: status works, calibration answers 409."""
        status = self.client.get("/api/sensors/imu").json()
        self.assertFalse(status["available"])
        self.assertIn("status", status)
        for path in (
            "/api/sensors/imu/reset",
            "/api/sensors/imu/scale",
            "/api/sensors/imu/bias",
            "/api/sensors/imu/noise",
            "/api/sensors/imu/start",
        ):
            response = self.client.post(path, json={})
            self.assertEqual(response.status_code, 409, path)

    def test_imu_scale_endpoint_roundtrip(self):
        """The spin workflow computes and applies the scale on a healthy IMU."""
        from software.console.runtime.state import state as app_state

        class FakeImu:
            """Minimal driver stand-in for the calibration endpoints."""

            def read_gyro_bias_dps(self):
                """Return a fixed bias."""
                return 0.25

        session = app_state.session
        previous = (session.imu, session.imu_state, session.gyro_scale, session.imu_integrated_deg)
        session.imu = FakeImu()
        session.imu_state = "ready"
        session.gyro_scale = 1.0
        try:
            self.client.post("/api/sensors/imu/reset")
            session.imu_integrated_deg = 350.0
            result = self.client.post("/api/sensors/imu/scale", json={"turns_done": 1}).json()
            self.assertAlmostEqual(result["applied"], 360.0 / 350.0, places=6)
            self.assertIn("GYRO_SCALE=", result["env_line"])
            bias = self.client.post("/api/sensors/imu/bias").json()
            self.assertAlmostEqual(bias["bias_deg_s"], 0.25, places=3)
        finally:
            (
                session.imu,
                session.imu_state,
                session.gyro_scale,
                session.imu_integrated_deg,
            ) = previous

    def test_imu_stop_endpoint_with_a_fake_imu(self):
        """Stop works any time (bench test); start needs i2c + a real driver."""
        from software.console.runtime.state import state as app_state

        session = app_state.session
        previous = (
            session.imu,
            session.imu_state,
            session.gyro_stream_state,
            session._stream_task,
            session._imu_task,
        )
        session.imu = object()
        session.imu_state = "ready"
        try:
            result = self.client.post("/api/sensors/imu/stop").json()
            self.assertEqual(result["state"], "stopped")
            self.assertFalse(session.imu_status()["present"])
            # the mock transport cannot (re)start a real sampler: 409
            start = self.client.post("/api/sensors/imu/start")
            self.assertEqual(start.status_code, 409)
        finally:
            (
                session.imu,
                session.imu_state,
                session.gyro_stream_state,
                session._stream_task,
                session._imu_task,
            ) = previous

    def test_selftest_readiness_reports_checks(self):
        """Readiness returns a named check list; MOCK fails the mode check."""
        result = self.client.post("/api/selftest/readiness").json()
        self.assertTrue(result["ok"])
        self.assertEqual(result["test"], "readiness")
        names = {check["name"] for check in result["checks"]}
        self.assertIn("mode", names)
        self.assertIn("faults", names)
        self.assertIn("start pose", names)
        self.assertTrue(all(check["status"] in ("pass", "warn", "fail") for check in result["checks"]))
        mode = next(check for check in result["checks"] if check["name"] == "mode")
        self.assertEqual(mode["status"], "fail")  # MOCK transport
        self.assertFalse(result["passed"])  # a simulated robot is not match-ready

    def test_selftest_square_on_mock(self):
        """The square test runs and passes on the ideal simulator."""
        self._place_robot(1500.0, 1000.0, 0.0)
        result = self.client.post("/api/selftest/square", json={"side_mm": 150, "speed_mm_s": 300}).json()
        self.assertTrue(result["ok"])
        self.assertEqual(result["test"], "square")
        self.assertEqual(result["transport"], "mock")
        self.assertLessEqual(result["metrics"]["position_error_mm"], 20.0)
        self.assertLessEqual(result["metrics"]["heading_error_deg"], 2.0)
        self.assertTrue(result["passed"])

    def test_selftest_arc_on_mock(self):
        """The arc test evaluates both quarter arcs on the simulator."""
        self._place_robot(1500.0, 1000.0, 0.0)
        result = self.client.post("/api/selftest/arc", json={"radius_mm": 200, "speed_mm_s": 300}).json()
        self.assertTrue(result["ok"])
        self.assertEqual(result["test"], "arc")
        records = result["metrics"]["arcs"]
        self.assertEqual([record["name"] for record in records], ["left", "right"])
        for record in records:
            self.assertLessEqual(record["position_error_mm"], 20.0)
            self.assertLessEqual(record["heading_error_deg"], 2.0)
        self.assertTrue(result["passed"])

    def test_selftest_chain_on_mock(self):
        """Three tangent arcs chain without a junction stop on the simulator."""
        self._place_robot(1200.0, 800.0, 0.0)
        result = self.client.post(
            "/api/selftest/chain", json={"length_mm": 200, "radius_mm": 200, "speed_mm_s": 300}
        ).json()
        self.assertTrue(result["ok"])
        self.assertEqual(result["test"], "chain")
        self.assertEqual(result["metrics"]["stop_count"], 0, result["metrics"]["junction_stops"])
        self.assertTrue(result["passed"], result["reason"])

    def test_selftest_rest_drift_on_mock(self):
        """The drift audit passes on a standing simulated robot."""
        result = self.client.post("/api/selftest/rest-drift", json={"seconds": 3}).json()
        self.assertTrue(result["ok"])
        self.assertEqual(result["test"], "rest-drift")
        self.assertLessEqual(result["metrics"]["board_max_pos_mm"], 2.0)
        self.assertTrue(result["passed"], result["reason"])

    def test_selftest_turn_repeat_on_mock(self):
        """Two turns per direction, statistics sane on the simulator."""
        self._place_robot(1500.0, 1000.0, 0.0)
        result = self.client.post(
            "/api/selftest/turn-repeat",
            json={"count_per_dir": 2, "angle_deg": 90.0, "speed_mm_s": 300},
        ).json()
        self.assertTrue(result["ok"])
        self.assertEqual(result["test"], "turn-repeat")
        records = result["metrics"]["directions"]
        self.assertEqual([record["direction"] for record in records], ["ccw", "cw"])
        for record in records:
            self.assertLessEqual(abs(record["fused_mean_deg"]), 3.0)
            self.assertLessEqual(record["fused_sigma_deg"], 1.5)
        self.assertTrue(result["passed"], result["reason"])

    def test_selftest_loop_drift_on_mock(self):
        """One closed loop: near-zero drift and an exact tick scale."""
        self._place_robot(1500.0, 1000.0, 0.0)
        result = self.client.post(
            "/api/selftest/loop-drift", json={"loops": 1, "side_mm": 200.0, "speed_mm_s": 300}
        ).json()
        self.assertTrue(result["ok"])
        self.assertEqual(result["test"], "loop-drift")
        record = result["metrics"]["records"][-1]
        self.assertLessEqual(record["fused_pos_mm"], 60.0)
        self.assertAlmostEqual(record["tick_ratio"], 1.0, delta=0.05)
        self.assertTrue(result["passed"], result["reason"])

    def test_selftest_speed_ramp_on_mock(self):
        """The speed trace shows the commanded cruise and the distance."""
        self._place_robot(1500.0, 1000.0, 0.0)
        result = self.client.post(
            "/api/selftest/speed-ramp", json={"distance_mm": 400.0, "speed_mm_s": 400}
        ).json()
        self.assertTrue(result["ok"])
        self.assertEqual(result["test"], "speed-ramp")
        self.assertGreaterEqual(result["metrics"]["samples"], 5)
        self.assertAlmostEqual(result["metrics"]["peak_speed_mm_s"], 400.0, delta=60.0)
        self.assertLessEqual(abs(result["metrics"]["distance_error_mm"]), 60.0)
        self.assertTrue(result["passed"], result["reason"])

    def test_selftest_encoders_on_mock(self):
        """Forward/backward/turn tick behaviour passes every check."""
        self._place_robot(1500.0, 1000.0, 0.0)
        result = self.client.post(
            "/api/selftest/encoders", json={"distance_mm": 150.0, "speed_mm_s": 300}
        ).json()
        self.assertTrue(result["ok"])
        self.assertEqual(result["test"], "encoders")
        checks = result["metrics"]["checks"]
        self.assertTrue(all(checks.values()), checks)
        self.assertTrue(result["passed"], result["reason"])

    def test_selftest_fault_drill_estop_on_mock(self):
        """E-stop latch + clear rehearsal passes on the simulator."""
        result = self.client.post("/api/selftest/fault-drill", json={"kind": "estop"}).json()
        self.assertTrue(result["ok"])
        self.assertEqual(result["metrics"]["kind"], "estop")
        self.assertIn("ESTOP", result["metrics"]["latched_faults"])
        self.assertTrue(result["metrics"]["recovered"])
        self.assertTrue(result["passed"], result["reason"])

    def test_selftest_fault_drill_deadman_on_mock(self):
        """A silenced link lets the watchdog stop the creep and latch."""
        self._place_robot(1500.0, 1000.0, 0.0)
        result = self.client.post(
            "/api/selftest/fault-drill", json={"kind": "deadman"}
        ).json()
        self.assertTrue(result["ok"])
        self.assertEqual(result["metrics"]["kind"], "deadman")
        self.assertIn("DEADMAN", result["metrics"]["latched_faults"])
        self.assertTrue(result["metrics"]["stopped_short"])  # did not reach the target
        self.assertTrue(result["metrics"]["quiet"])
        self.assertTrue(result["passed"], result["reason"])

    def test_selftest_fault_drill_quiets_the_gyro_stream(self):
        """The deadman drill stops and restarts the gyro stream.

        Real-robot bug guard: the session's 50 Hz SET_GYRO frames refresh
        the board's command watchdog, so a healthy IMU would defeat the
        drill. The endpoint must stop the stream for the silence window and
        restart it afterwards.
        """
        from software.console.runtime.state import state as app_state

        session = app_state.session
        calls = []
        original_stop, original_start = session.stop_gyro_stream, session.start_gyro_stream

        async def spy_stop():
            calls.append("stop")

        async def spy_start():
            calls.append("start")

        session.stop_gyro_stream = spy_stop  # type: ignore[method-assign]
        session.start_gyro_stream = spy_start  # type: ignore[method-assign]
        try:
            self._place_robot(1500.0, 1000.0, 0.0)
            result = self.client.post(
                "/api/selftest/fault-drill", json={"kind": "deadman"}
            ).json()
            self.assertTrue(result["passed"], result["reason"])
        finally:
            session.stop_gyro_stream = original_stop  # type: ignore[method-assign]
            session.start_gyro_stream = original_start  # type: ignore[method-assign]
        self.assertEqual(calls, ["stop", "start"])

    def test_selftest_fault_drill_rejects_unknown_kind(self):
        """Only 'deadman' and 'estop' are valid drill kinds."""
        response = self.client.post("/api/selftest/fault-drill", json={"kind": "warp"})
        self.assertEqual(response.status_code, 422)

    def test_plan_continues_when_the_robot_stands_on_the_route(self):
        """A plan from a robot ON the drawing continues from its position.

        Real-robot UX bug (2026-10-01): a strategy starting with a straight
        LINE, "run" while the robot stood near the line's END - the plan
        drove it back to the line's start (two 180 deg pivots) and re-drove
        the whole line. The fresh plan must CONTINUE from the robot instead.
        """
        self.client.post("/api/pose", json={"x_mm": 675.0, "y_mm": 1775.0, "theta_deg": 0.0})
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - 675.0) < 1.5)
        body = {
            "strokes": [[[315.0, 1800.0], [700.0, 1800.0]]],
            "options": {"speed_mm_s": 255, "goto_step_mm": 250, "avoid_obstacles": False, "smooth": False},
        }
        plan = self.client.post("/api/plan", json=body).json()
        segments = plan["segments"]
        first = next(segment for segment in segments if segment["kind"] in ("goto", "arc"))
        start = (first.get("x", first.get("x0")), first.get("y", first.get("y0")))
        self.assertLess(
            math.hypot(start[0] - 675.0, start[1] - 1775.0), 90.0,
            "the plan drove back to the drawing's start instead of continuing",
        )
        self.assertFalse(
            [w for w in plan["warnings"] if "connector leg added" in w], plan["warnings"]
        )
        # only the remainder of the line, not a double pass of it
        self.assertLess(plan["stats"]["length_mm"], 300.0)

    def test_pose_plan_execute_cycle(self):
        # 1. place the robot
        """Pose plan execute cycle."""
        response = self.client.post("/api/pose", json={"x_mm": 400.0, "y_mm": 900.0, "theta_deg": 0.0})
        self.assertEqual(response.status_code, 200)
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - 400.0) < 1.0)

        # 2. plan a short drawn path
        plan_body = {
            "strokes": [[[400.0, 900.0], [800.0, 900.0]]],
            "goal": None,
            "obstacles": [],
            "options": {"speed_mm_s": 255, "goto_step_mm": 250, "avoid_obstacles": False, "smooth": False},
        }
        response = self.client.post("/api/plan", json=plan_body)
        self.assertEqual(response.status_code, 200, response.text)
        plan = response.json()
        self.assertGreaterEqual(plan["stats"]["n_segments"], 1)
        # a straight run is ONE GOTO now (the alternating flat-arc S made the
        # real robot accelerate/decelerate); its polyline still starts at the
        # drawn start, which is what the preview and the board see
        first = plan["segments"][0]
        self.assertIn(first["kind"], ("goto", "arc"))
        if first["kind"] == "goto":
            points = first.get("points") or [[first["x"], first["y"]]]
            start_x = points[0][0]
        else:
            start_x = first["x0"]
        self.assertAlmostEqual(start_x, 400.0, delta=5.0)

        # 3. execute it
        response = self.client.post("/api/execute", json={"segments": plan["segments"], "deadman_ms": 0})
        self.assertEqual(response.status_code, 200)
        final = wait_mission_end(self.client, timeout_s=20.0)
        self.assertEqual(final, "done")

        # 4. the mock robot should sit near the end of the path
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - 800.0) < 60.0)

    def test_execute_follow_path_streams_on_the_mock(self):
        """STREAMING_EXEC + follow_path -> the streaming backend drives the run.

        The mock simulates firmware 3.4, so the capability passes and the
        executor FOLLOW mode runs the plan polyline with SET_VELOCITY frames
        (one acceleration, one deceleration); the segment program is never
        sent. With the gate off the same request falls back to the segment
        backend - covered by every other execute test in this file.
        """
        from dataclasses import replace
        from software.console.runtime.state import state

        self._place_robot(400.0, 900.0, 0.0)
        plan = self.client.post(
            "/api/plan",
            json={
                "strokes": [[[400.0, 900.0], [900.0, 900.0]]],
                "goal": None,
                "obstacles": [],
                "options": {"speed_mm_s": 255, "goto_step_mm": 250, "avoid_obstacles": False, "smooth": False},
            },
        ).json()
        session = state.session
        original = session.settings
        session.settings = replace(original, streaming_exec_enabled=True)
        try:
            response = self.client.post(
                "/api/execute",
                json={"segments": plan["segments"], "deadman_ms": 0, "follow_path": plan["path"]},
            )
            self.assertEqual(response.status_code, 200, response.text)
            self.assertTrue(response.json().get("streaming"))
            final = wait_mission_end(self.client, timeout_s=20.0)
        finally:
            session.settings = original
        self.assertEqual(final, "done")
        self.assertGreater(session.client.link.stream_frames, 0)
        # the mock robot should sit near the end of the path (400 -> 900 mm)
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - 900.0) < 60.0)

    def _straight_mission_body(self):
        """Straight mission body."""
        return {
            "elements": [{"kind": "stroke", "points": [[500.0, 500.0], [2500.0, 500.0]]}],
            "options": {
                "speed_mm_s": 255, "goto_step_mm": 250, "avoid_obstacles": True, "smooth": False,
                "robot_width_mm": 300.0, "robot_height_mm": 300.0, "obstacle_margin_mm": 150.0,
            },
        }

    def test_execute_does_not_replay_stale_plan_during_obstacle_wait(self):
        """Execute retries the wait route instead of replaying the old plan."""
        from software.console.runtime.state import state as app_state

        self._place_robot(1000.0, 1000.0, 0.0)
        previous_wait = app_state.vision_waiting
        waiting = {
            "reason": "there is no way around it",
            "elements": [{"kind": "stroke", "points": [[1000.0, 1000.0], [2000.0, 1000.0]]}],
            "goal": None,
            "obstacles": [{"kind": "rect", "x": 0.0, "y": 0.0, "w": 3000.0, "h": 2000.0}],
            "options": {"avoid_obstacles": True},
            "deadman_ms": 0,
            "paused_by_user": False,
            "direct_mm": 1000.0,
            "started_at": time.monotonic(),
        }
        app_state.vision_waiting = waiting
        try:
            response = self.client.post(
                "/api/execute",
                json={"deadman_ms": 0, "segments": [{"kind": "goto", "x": 1800.0, "y": 1000.0, "th": 0.0}]},
            )
            self.assertEqual(response.status_code, 409, response.text)
            self.assertIn("still stopped", response.json()["detail"])
            self.assertIs(app_state.vision_waiting, waiting)
            state = self.client.get("/api/state").json()
            self.assertNotEqual(state["exec"]["status"], "running")
            self.assertLess(math.hypot(state["x_mm"] - 1000.0, state["y_mm"] - 1000.0), 3.0)
        finally:
            app_state.vision_waiting = previous_wait

    def test_goal_only_obstacle_replan_uses_direct_safe_route(self):
        """A goal-only mission replans around a static obstacle without waiting."""
        from software.core.mission.orchestrator import _clips_obstacle
        from software.core.navigation.options import PlanOptions

        self._place_robot(600.0, 1000.0, 0.0)
        body = {
            "elements": [],
            "goal": {"x": 2600.0, "y": 1000.0},
            "options": {
                "speed_mm_s": 255,
                "avoid_obstacles": True,
                "robot_width_mm": 300.0,
                "robot_height_mm": 300.0,
                "obstacle_margin_mm": 150.0,
            },
        }
        first = self.client.post("/api/plan", json=body).json()
        self.client.post("/api/execute", json={"segments": first["segments"], "deadman_ms": 0})
        started = wait_for(
            lambda: (lambda state: state if 900.0 < state.get("x_mm", 0.0) < 1200.0
                     and state.get("exec", {}).get("status") == "running" else None)(
                self.client.get("/api/state").json()),
            timeout_s=20.0,
        )
        self.assertIsNotNone(started)

        obstacle = {"kind": "rect", "x": 1400.0, "y": 900.0, "w": 300.0, "h": 200.0}
        replanned = self.client.post("/api/plan", json={**body, "obstacles": [obstacle]})
        self.assertEqual(replanned.status_code, 200, replanned.text)
        result = replanned.json()
        self.assertFalse(result.get("waiting"), "static obstacles must be routed around, not waited out")
        self.assertFalse(
            _clips_obstacle(
                result["path"], [obstacle], PlanOptions.from_dict(body["options"]), 3000.0, 2000.0
            ),
            "the regenerated route must clear the static obstacle",
        )
        self.assertLess(math.dist(result["path"][-1], (2600.0, 1000.0)), 5.0)
        self.assertGreater(max(point[1] for point in result["path"]), 1100.0)
        self.assertGreater(max(point[1] for point in result["path"]), min(point[1] for point in result["path"]) + 100.0)
        self.assertEqual(wait_mission_end(self.client, timeout_s=45.0), "done")

    def test_replan_while_executing_avoids_new_obstacle(self):
        # dynamic obstacle: it appears ON the route while the robot drives -
        # FAR away it must change nothing ("smart" proximity); once the robot
        # gets close the service stops it, re-routes the remainder and resumes
        """Replan while executing avoids new obstacle."""
        self.client.post("/api/pose", json={"x_mm": 500.0, "y_mm": 500.0, "theta_deg": 0.0})
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - 500.0) < 1.0)
        body = self._straight_mission_body()
        first = self.client.post("/api/plan", json=body)
        self.assertEqual(first.status_code, 200, first.text)
        first_total = len(first.json()["segments"])
        self.client.post("/api/execute", json={"segments": first.json()["segments"], "deadman_ms": 0})

        started = wait_for(
            lambda: (lambda state: state if 600.0 < state.get("x_mm", 0.0) < 1200.0
                     and state.get("exec", {}).get("status") == "running" else None)(
                self.client.get("/api/state").json()),
            timeout_s=30.0,
        )
        self.assertIsNotNone(started, "the robot never got into the run")

        # the obstacle appears ON the remaining straight line, far ahead
        obstacle = {"kind": "rect", "x": 1900.0, "y": 380.0, "w": 200.0, "h": 240.0}
        replan = self.client.post("/api/plan", json={**body, "obstacles": [obstacle]})
        self.assertEqual(replan.status_code, 200, replan.text)
        self.assertTrue(
            replan.json().get("unchanged"),
            "an obstacle this far ahead must not stop or re-plan anything",
        )
        for _ in range(12):  # same program, still driving
            state = self.client.get("/api/state").json()
            self.assertEqual(state.get("exec", {}).get("status"), "running")
            self.assertEqual(state.get("exec", {}).get("total"), first_total)
            time.sleep(0.04)

        # ...as the robot approaches, the service reacts ON ITS OWN: the plan
        # document is replaced by the re-routed remainder (the segment COUNT
        # can coincide, so watch the document itself)
        first_segments = first.json()["segments"]

        def rerouted():
            """Plan document replaced by the re-routed remainder, else None."""
            document = self.client.get("/api/plan").json()
            return document if document.get("segments") != first_segments else None

        new_plan = wait_for(rerouted, timeout_s=20.0)
        self.assertIsNotNone(new_plan, "the service never re-checked when the obstacle got close")
        self.assertTrue(any("re-routed" in warning for warning in new_plan["warnings"]))
        # the reaction happened inside the ~1 m horizon, not at the last moment
        self.assertGreater(new_plan["path"][0][0], 800.0)
        self.assertLess(new_plan["path"][0][0], 1450.0)
        self.assertGreaterEqual(
            new_plan["path"][1][0], new_plan["path"][0][0] - 2.0,
            "the replacement path must not backtrack to the pre-stop pose",
        )
        # the connector from the stop position is expected on a mid-run re-plan
        self.assertFalse(any("starts away from the robot" in warning for warning in new_plan["warnings"]))

        # the new route starts where the robot stopped: the mission is NOT
        # restarted from its beginning (what was already driven stays driven)
        robot = self.client.get("/api/state").json()
        first_point = new_plan["path"][0]
        self.assertLess(math.hypot(first_point[0] - robot["x_mm"], first_point[1] - robot["y_mm"]), 500.0)

        # the re-planned path never grazes the obstacle it had to avoid
        x0, y0 = obstacle["x"], obstacle["y"]
        x1, y1 = x0 + obstacle["w"], y0 + obstacle["h"]
        for px, py in new_plan["path"]:
            gap = math.hypot(max(x0 - px, 0.0, px - x1), max(y0 - py, 0.0, py - y1))
            self.assertGreater(gap, 100.0, f"path point ({px:.0f}, {py:.0f}) too close to the obstacle")

        # execution continues under the new program and reaches the mission end
        # (the remainder is ONE straight GOTO now, so it can legitimately be
        # done before the first poll; the end-position check below is the real
        # proof that it restarted and finished)
        running = wait_for(
            lambda: (lambda state: state if state.get("exec", {}).get("status") in ("running", "done")
                     and state.get("exec", {}).get("total", 0) >= 1 else None)(
                self.client.get("/api/state").json()),
            timeout_s=10.0,
        )
        self.assertIsNotNone(running, "the run did not restart with the new program")
        final = wait_mission_end(self.client, timeout_s=40.0)
        self.assertEqual(final, "done")
        end = wait_for(
            lambda: (lambda state: state if abs(state.get("x_mm", 0.0) - 2500.0) < 80.0
                     and abs(state.get("y_mm", 0.0) - 500.0) < 80.0 else None)(
                self.client.get("/api/state").json()),
            timeout_s=10.0,
        )
        self.assertIsNotNone(end, "the robot did not reach the end of the re-planned route")

    def test_replan_while_paused_stays_paused(self):
        # a paused run gets re-planned around the new obstacle but must NOT
        # start driving again until the operator resumes it
        """Replan while paused stays paused."""
        self.client.post("/api/pose", json={"x_mm": 500.0, "y_mm": 500.0, "theta_deg": 0.0})
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - 500.0) < 1.0)
        body = self._straight_mission_body()
        plan = self.client.post("/api/plan", json=body).json()
        self.client.post("/api/execute", json={"segments": plan["segments"], "deadman_ms": 0})
        # pause close enough to the obstacle for the service to react to it
        wait_for(lambda: 1250.0 < self.client.get("/api/state").json().get("x_mm", 0.0) < 1450.0,
                 timeout_s=30.0)
        self.assertEqual(self.client.post("/api/pause").status_code, 200)
        paused = wait_for(
            lambda: (lambda status: status if status == "paused" else None)(
                self.client.get("/api/state").json().get("exec", {}).get("status")),
            timeout_s=10.0,
        )
        self.assertEqual(paused, "paused")

        obstacle = {"kind": "rect", "x": 1900.0, "y": 380.0, "w": 200.0, "h": 240.0}
        replan = self.client.post("/api/plan", json={**body, "obstacles": [obstacle]})
        self.assertEqual(replan.status_code, 200, replan.text)
        new_total = len(replan.json()["segments"])

        re_paused = wait_for(
            lambda: (lambda state: state if state.get("exec", {}).get("status") == "paused"
                     and state.get("exec", {}).get("total") == new_total else None)(
                self.client.get("/api/state").json()),
            timeout_s=10.0,
        )
        self.assertIsNotNone(re_paused, "the re-planned run did not stay paused")
        before = self.client.get("/api/state").json().get("x_mm", 0.0)
        time.sleep(0.6)
        after = self.client.get("/api/state").json().get("x_mm", 0.0)
        self.assertLess(abs(after - before), 5.0, "the robot moved while paused")

        # resume: it drives the remaining route around the obstacle
        self.assertEqual(self.client.post("/api/resume").status_code, 200)
        final = wait_mission_end(self.client, timeout_s=40.0)
        self.assertEqual(final, "done")
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0.0) - 2500.0) < 80.0, timeout_s=10.0)

    def test_replan_on_detour_stays_on_route(self):
        # regression: while the robot is mid-detour (off the drawing), adding
        # an obstacle must NOT reattach the route to the drawing (no stop, no
        # backwards move) - the route continues where the robot is
        """Replan on detour stays on route."""
        self.client.post("/api/pose", json={"x_mm": 500.0, "y_mm": 500.0, "theta_deg": 0.0})
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - 500.0) < 1.0)
        body = self._straight_mission_body()
        on_line = {"kind": "rect", "x": 1400.0, "y": 400.0, "w": 200.0, "h": 200.0}
        plan = self.client.post("/api/plan", json={**body, "obstacles": [on_line]}).json()
        self.assertTrue(any("re-routed" in warning for warning in plan["warnings"]), plan["warnings"])
        self.client.post("/api/execute", json={"segments": plan["segments"], "deadman_ms": 0})

        # wait until the robot is climbing the detour (well above the line)
        detour = wait_for(
            lambda: (lambda state: state if state.get("y_mm", 0.0) > 700.0
                     and state.get("exec", {}).get("status") == "running" else None)(
                self.client.get("/api/state").json()),
            timeout_s=30.0,
        )
        self.assertIsNotNone(detour, "the robot never got on the detour")

        far = {"kind": "rect", "x": 2400.0, "y": 1500.0, "w": 200.0, "h": 200.0}
        response = self.client.post(
            "/api/plan", json={**body, "obstacles": [on_line, far]}
        )
        self.assertEqual(response.status_code, 200, response.text)
        self.assertTrue(
            response.json().get("unchanged"),
            "a far obstacle must not reattach the route while the robot is on a detour",
        )
        for _ in range(12):  # no stop / swap: the robot keeps driving
            self.assertEqual(self.client.get("/api/state").json().get("exec", {}).get("status"), "running")
            time.sleep(0.04)

        final = wait_mission_end(self.client, timeout_s=30.0)
        self.assertEqual(final, "done")
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0.0) - 2500.0) < 80.0,
                 timeout_s=10.0)

    def test_obstacle_behind_or_far_does_not_interrupt(self):
        # an obstacle behind the robot (already passed) or far from its
        # remaining route must NOT stop it nor change the trajectory
        """Obstacle behind or far does not interrupt."""
        self.client.post("/api/pose", json={"x_mm": 500.0, "y_mm": 500.0, "theta_deg": 0.0})
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - 500.0) < 1.0)
        body = self._straight_mission_body()
        plan = self.client.post("/api/plan", json=body).json()
        self.client.post("/api/execute", json={"segments": plan["segments"], "deadman_ms": 0})
        started = wait_for(
            lambda: (lambda state: state if 900.0 < state.get("x_mm", 0.0) < 1400.0
                     and state.get("exec", {}).get("status") == "running" else None)(
                self.client.get("/api/state").json()),
            timeout_s=30.0,
        )
        self.assertIsNotNone(started, "the robot never got into the run")

        behind = {"kind": "rect", "x": 550.0, "y": 420.0, "w": 120.0, "h": 160.0}
        response = self.client.post("/api/plan", json={**body, "obstacles": [behind]})
        self.assertEqual(response.status_code, 200, response.text)
        self.assertTrue(response.json().get("unchanged"), "an already-passed obstacle must not re-plan")
        for _ in range(15):  # and the robot is still driving (no stop / swap)
            self.assertEqual(
                self.client.get("/api/state").json().get("exec", {}).get("status"), "running"
            )
            time.sleep(0.04)

        far = {"kind": "rect", "x": 2200.0, "y": 1400.0, "w": 200.0, "h": 200.0}
        response = self.client.post("/api/plan", json={**body, "obstacles": [far]})
        self.assertEqual(response.status_code, 200, response.text)
        self.assertTrue(response.json().get("unchanged"), "a far obstacle must not re-plan")
        # tidy up: end the run so following tests start clean
        self.assertEqual(self.client.post("/api/stop").status_code, 200)
        wait_for(lambda: self.client.get("/api/state").json().get("exec", {}).get("status") == "aborted",
                 timeout_s=10.0)

    def test_obstacle_far_ahead_on_the_route_is_ignored_until_close(self):
        # "smart" proximity: a robot standing ON the path far ahead must not
        # stop or re-route anything - only when our robot gets close does the
        # service react (the proximity watchdog re-checks while driving, no
        # new request needed)
        """Obstacle far ahead on the route is ignored until close."""
        self.client.post("/api/pose", json={"x_mm": 500.0, "y_mm": 500.0, "theta_deg": 0.0})
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - 500.0) < 1.0)
        body = self._straight_mission_body()
        plan = self.client.post("/api/plan", json=body).json()
        started_exec = self.client.post("/api/execute", json={"segments": plan["segments"], "deadman_ms": 0})
        self.assertEqual(started_exec.status_code, 200, started_exec.text)
        started = wait_for(
            lambda: (lambda state: state if 600.0 < state.get("x_mm", 0.0) < 800.0
                     and state.get("exec", {}).get("status") == "running" else None)(
                self.client.get("/api/state").json()),
            timeout_s=30.0,
        )
        self.assertIsNotNone(started, "the robot never got into the run")
        self.assertGreater(1900.0 - started["x_mm"], 750.0, "the obstacle must start far away")

        # a small robot sits ON the remaining straight line, ~1.2 m ahead
        obstacle = {"kind": "rect", "x": 1900.0, "y": 420.0, "w": 200.0, "h": 100.0}
        response = self.client.post("/api/plan", json={**body, "obstacles": [obstacle]})
        self.assertEqual(response.status_code, 200, response.text)
        self.assertTrue(
            response.json().get("unchanged"),
            "an obstacle far ahead must not change the path",
        )
        for _ in range(15):  # no stop, same program: the robot keeps driving
            state = self.client.get("/api/state").json()
            self.assertEqual(state.get("exec", {}).get("status"), "running")
            self.assertEqual(state.get("exec", {}).get("total"), len(plan["segments"]))
            time.sleep(0.04)

        # the mission completes on the detour the service planned ON ITS OWN
        final = wait_mission_end(self.client, timeout_s=40.0)
        self.assertEqual(final, "done")
        swapped = self.client.get("/api/plan").json()
        # the obstacle must be visible in the last plan: either a body detour
        # ("re-routed") or a margin-aware route kept at body clearance ("kept
        # as drawn inside the obstacle margin") - both prove the re-plan ran
        self.assertTrue(
            any(
                "re-routed" in warning or "obstacle margin" in warning
                for warning in swapped["warnings"]
            ),
            "the close obstacle must have changed the planned route",
        )
        end = wait_for(
            lambda: (lambda state: state if abs(state.get("x_mm", 0.0) - 2500.0) < 80.0
                     and abs(state.get("y_mm", 0.0) - 500.0) < 80.0 else None)(
                self.client.get("/api/state").json()),
            timeout_s=10.0,
        )
        self.assertIsNotNone(end, "the robot did not finish the mission")

        final = wait_mission_end(self.client, timeout_s=30.0)
        self.assertEqual(final, "done")  # the original run completed untouched
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0.0) - 2500.0) < 80.0,
                 timeout_s=10.0)

    def test_obstacle_on_the_route_replans_a_full_metre_early(self):
        # the reaction horizon ~1 m: an obstacle ON the remaining line that
        # far ahead must re-plan RIGHT AWAY - waiting for the old ~0.7 m
        # reach let the robot drive up close and change the path at the last
        # moment (the reported session) instead of detouring early
        """Obstacle on the route replans a full metre early."""
        self.client.post("/api/pose", json={"x_mm": 500.0, "y_mm": 500.0, "theta_deg": 0.0})
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - 500.0) < 1.0)
        body = self._straight_mission_body()
        plan = self.client.post("/api/plan", json=body).json()
        self.client.post("/api/execute", json={"segments": plan["segments"], "deadman_ms": 0})
        # catch the run EARLY in the line: everything after the catch (the
        # 0.25 s watcher debounce, the stop, the fusion settle, plus the
        # telemetry lag of the reported pose) moves the robot on - the window
        # keeps the swap position well inside the asserted reach even with
        # that designed processing lag on a loaded machine
        started = wait_for(
            lambda: (lambda state: state if 560.0 < state.get("x_mm", 0.0) < 640.0
                     and state.get("exec", {}).get("status") == "running" else None)(
                self.client.get("/api/state").json()),
            timeout_s=30.0,
        )
        self.assertIsNotNone(started, "the robot never got into the run")
        gap = 1560.0 - started["x_mm"]
        self.assertGreater(gap, 712.0, "the obstacle must start beyond the OLD reach")
        self.assertLess(gap, 1012.0, "the obstacle must start within the NEW reach")

        obstacle = {"kind": "rect", "x": 1560.0, "y": 420.0, "w": 200.0, "h": 100.0}
        response = self.client.post("/api/plan", json={**body, "obstacles": [obstacle]})
        self.assertEqual(response.status_code, 200, response.text)
        self.assertFalse(
            response.json().get("unchanged"),
            "an obstacle on the route ~1 m ahead must re-plan immediately",
        )
        # the re-plan really happened: it starts where the robot IS at the
        # swap instant (the response path's first point IS the live pose the
        # swap fired on) and it re-routes. The gap is measured HERE, not at
        # the catch: the debounce + stop + settle move the robot ~100 mm on.
        replanned = response.json()
        swap_gap = 1560.0 - replanned["path"][0][0]
        self.assertGreater(swap_gap, 550.0,
                   "the swap must fire well before the close-stop band, not at the last moment")
        self.assertLess(swap_gap, 1012.0, "the swap must fire within the NEW ~1 m reach")
        self.assertTrue(any("re-routed" in w for w in replanned["warnings"]), replanned["warnings"])

        # execution continues on the re-planned remainder and reaches the end
        final = wait_mission_end(self.client, timeout_s=40.0)
        self.assertEqual(final, "done")
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0.0) - 2500.0) < 80.0,
                 timeout_s=10.0)

    def test_replan_keeps_remaining_mission_and_actions(self):
        # mid-run re-plan works from the DRAWN mission: the rest of the
        # sequence (rotation + second stroke) survives, and the controller
        # still finishes on the mission's end - not on some rebuilt tail
        """Replan keeps remaining mission and actions."""
        self.client.post("/api/pose", json={"x_mm": 500.0, "y_mm": 500.0, "theta_deg": 0.0})
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - 500.0) < 1.0)
        body = {
            "elements": [
                {"kind": "stroke", "points": [[500.0, 500.0], [1500.0, 500.0]]},
                {"kind": "rotate", "angle_deg": 90.0},
                {"kind": "stroke", "points": [[1500.0, 500.0], [1500.0, 1400.0]]},
            ],
            "options": {
                "speed_mm_s": 255, "goto_step_mm": 250, "avoid_obstacles": True, "smooth": False,
                "robot_width_mm": 300.0, "robot_height_mm": 300.0, "obstacle_margin_mm": 150.0,
            },
        }
        plan = self.client.post("/api/plan", json=body).json()
        self.client.post("/api/execute", json={"segments": plan["segments"], "deadman_ms": 0})
        # swap while the robot is still on the FIRST stroke: the detour around
        # the box is then clearly cheaper than the remaining route, so the
        # re-route branch is taken deterministically. Close to the box the
        # detour-length check can legitimately flip to the "wait" behaviour
        # (moving-obstacle policy), which this test does not exercise.
        started = wait_for(
            lambda: (lambda state: state if 620.0 < state.get("x_mm", 0.0) < 1100.0
                     and state.get("exec", {}).get("status") == "running" else None)(
                self.client.get("/api/state").json()),
            timeout_s=30.0,
        )
        self.assertIsNotNone(started, "the robot never got into the run")

        obstacle = {"kind": "rect", "x": 1400.0, "y": 900.0, "w": 200.0, "h": 240.0}
        replan = self.client.post("/api/plan", json={**body, "obstacles": [obstacle]})
        self.assertEqual(replan.status_code, 200, replan.text)
        segments = replan.json()["segments"]
        self.assertIn("turn", [segment["kind"] for segment in segments], "the rotation must survive the re-plan")

        def motion_start(program):
            """Motion start."""
            for segment in program:
                if segment["kind"] == "goto":
                    points = segment.get("points")
                    if points:
                        return (points[0][0], points[0][1])
                    return (segment["x"], segment["y"])
                if segment["kind"] == "arc":
                    return (segment["x0"], segment["y0"])
            return (float("nan"), float("nan"))

        def motion_end(program):
            """Motion end."""
            for segment in reversed(program):
                if segment["kind"] == "goto":
                    return (segment["x"], segment["y"])
                if segment["kind"] == "arc":
                    return (segment["x1"], segment["y1"])
            return (float("nan"), float("nan"))

        # the tail may be driven as native ARCs, so check where the program
        # starts and finishes, not only its GOTO nodes
        end_x, end_y = motion_end(segments)
        self.assertLess(math.hypot(end_x - 1500.0, end_y - 1400.0), 120.0,
                        "the re-plan must end on the mission's last drawn stroke")
        # the swap really happened: the new program starts at the robot, not
        # at the mission's beginning
        start_x, start_y = motion_start(segments)
        self.assertLess(
            math.hypot(start_x - started.get("x_mm", 0.0), start_y - started.get("y_mm", 0.0)),
            400.0,
            "the re-planned program must start where the robot stopped",
        )

        final = wait_mission_end(self.client, timeout_s=40.0)
        self.assertEqual(final, "done")
        end = wait_for(
            lambda: (lambda state: state if math.hypot(state.get("x_mm", 0.0) - 1500.0,
                                                        state.get("y_mm", 0.0) - 1400.0) < 120.0 else None)(
                self.client.get("/api/state").json()),
            timeout_s=10.0,
        )
        self.assertIsNotNone(end, "the robot did not finish the mission after the re-plan")

    def test_waypoint_arrival_angle_does_not_bend_the_continuation(self):
        # regression: an ARC starts from the robot's CURRENT heading, so a
        # mid-mission waypoint arrival angle must be followed by an in-place
        # alignment turn - the robot used to leave the path entirely ("went
        # completely to the right") when the continuation was an arc
        """Waypoint arrival angle does not bend the continuation."""
        self.client.post("/api/pose", json={"x_mm": 600.0, "y_mm": 900.0, "theta_deg": 0.0})
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - 600.0) < 1.0)
        body = {
            "elements": [
                {"kind": "stroke", "points": [[600.0, 900.0], [1000.0, 900.0], [1400.0, 900.0]]},
                {"kind": "waypoint", "x": 1400.0, "y": 900.0, "theta_deg": -90.0},
                {"kind": "stroke",
                 "points": [[1400.0, 900.0], [1340.0, 800.0], [1230.0, 740.0], [1120.0, 720.0]]},
            ],
            "options": {
                "speed_mm_s": 255, "goto_step_mm": 250, "avoid_obstacles": True, "smooth": False,
                "robot_width_mm": 300.0, "robot_height_mm": 300.0, "obstacle_margin_mm": 150.0,
            },
        }
        plan = self.client.post("/api/plan", json=body).json()
        # every ARC must be entered on its tangent (walk the program like the
        # board drives it) - before the fix the arc after the waypoint started
        # 22.6 deg off, bending the whole continuation away
        heading = 0.0
        for index, segment in enumerate(plan["segments"]):
            if segment["kind"] == "goto":
                heading = segment["th"]
            elif segment["kind"] == "turn":
                heading = (heading + segment["angle_deg"] + 540.0) % 360.0 - 180.0
            elif segment["kind"] == "arc":
                gap = (segment["th0_deg"] - heading + 540.0) % 360.0 - 180.0
                self.assertLess(abs(gap), 1.05, f"segment {index}: ARC entered {gap:.1f} deg off tangent")
                heading = (segment["th0_deg"] + segment["sweep_deg"] + 540.0) % 360.0 - 180.0
        started = self.client.post("/api/execute", json={"segments": plan["segments"], "deadman_ms": 0})
        self.assertEqual(started.status_code, 200, started.text)
        final = wait_mission_end(self.client, timeout_s=40.0)
        self.assertEqual(final, "done")
        end = wait_for(
            lambda: (lambda state: state if math.hypot(state.get("x_mm", 0.0) - 1120.0,
                                                        state.get("y_mm", 0.0) - 720.0) < 120.0 else None)(
                self.client.get("/api/state").json()),
            timeout_s=10.0,
        )
        self.assertIsNotNone(end, "the robot did not finish on the mission's drawn end")

    def test_estop_and_clear(self):
        """Estop and clear."""
        self.assertEqual(self.client.post("/api/estop").status_code, 200)
        wait_for(lambda: self.client.get("/api/state").json().get("state") == 6)
        self.assertEqual(self.client.post("/api/clear_faults").status_code, 200)
        wait_for(lambda: self.client.get("/api/state").json().get("state") == 0)

    def test_release_motors_only_when_idle(self):
        """The release endpoint works at rest and refuses while the robot moves."""
        self._place_robot(500.0, 500.0, 0.0)
        self.assertEqual(self.client.post("/api/release_motors").status_code, 200)

        moved = self.client.post("/api/move/drive", json={"distance_mm": 400.0, "speed_mm_s": 100})
        self.assertEqual(moved.status_code, 200, moved.text)
        refused = self.client.post("/api/release_motors")
        self.assertEqual(refused.status_code, 409)
        self.assertIn("still moving", refused.json()["detail"])

        # wait for telemetry to SEE the motion, then for its real end (an
        # immediate idle snapshot may still be the pre-command cache)
        wait_for(lambda: self.client.get("/api/state").json().get("state") != 0, timeout_s=3.0)
        wait_for(lambda: self.client.get("/api/state").json().get("state") == 0, timeout_s=8.0)
        settled = self.client.post("/api/release_motors")
        self.assertEqual(settled.status_code, 200, settled.text)

    def test_pose_while_faulted_says_clear_faults(self):
        """A latched E-STOP refuses SET_POSE with an actionable message.

        The board latches the fault and silently blocks SET_POSE: the Pose
        tool looked broken ("robot: FAULT"). The console must name the fix.
        """
        self.assertEqual(self.client.post("/api/estop").status_code, 200)
        wait_for(lambda: self.client.get("/api/state").json().get("state") == 6)
        refused = self.client.post(
            "/api/pose", json={"x_mm": 500.0, "y_mm": 500.0, "theta_deg": 0.0}
        )
        self.assertEqual(refused.status_code, 409)
        self.assertIn("Clear Faults", refused.json()["detail"])
        self.assertEqual(self.client.post("/api/clear_faults").status_code, 200)
        wait_for(lambda: self.client.get("/api/state").json().get("state") == 0)
        again = self.client.post(
            "/api/pose", json={"x_mm": 500.0, "y_mm": 500.0, "theta_deg": 0.0}
        )
        self.assertEqual(again.status_code, 200, again.text)
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0.0) - 500.0) < 3.0)

    def test_stop_when_idle(self):
        """Stop when idle."""
        self.assertEqual(self.client.post("/api/stop").status_code, 200)

    def test_plan_rejects_empty_strokes(self):
        # make sure no leftover run makes the empty request look like a
        # mid-run obstacle refresh (that path is a 200 on purpose)
        """Plan rejects empty strokes."""
        self.client.post("/api/stop")
        wait_mission_end(self.client, timeout_s=45.0)
        response = self.client.post("/api/plan", json={"strokes": [], "obstacles": [], "options": {}})
        self.assertEqual(response.status_code, 422)

    def test_plan_with_obstacles_and_goal_serializes(self):
        # regression: the goal/obstacle bodies must serialize on pydantic v1
        # (dev PC) AND v2 (docker image) - model_dump() alone 500'd on v1
        """Plan with obstacles and goal serializes."""
        self.client.post("/api/pose", json={"x_mm": 500.0, "y_mm": 500.0, "theta_deg": 0.0})
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - 500.0) < 1.0)
        body = {
            "elements": [
                {"kind": "stroke", "points": [[600.0, 500.0], [1000.0, 500.0]]},
                {"kind": "waypoint", "x": 1500.0, "y": 900.0},
            ],
            "goal": {"x": 1800.0, "y": 900.0, "theta_deg": 90.0},
            "obstacles": [{"kind": "rect", "x": 1100.0, "y": 600.0, "w": 200.0, "h": 200.0}],
            "options": {"speed_mm_s": 255, "avoid_obstacles": True, "smooth": False,
                        "robot_width_mm": 300.0, "robot_height_mm": 300.0, "obstacle_margin_mm": 150.0},
        }
        response = self.client.post("/api/plan", json=body)
        self.assertEqual(response.status_code, 200, response.text)
        data = response.json()
        # the waypoint stands inside the obstacle's FULL clearance: it is
        # moved to the closest allowed point (the robot may not stop there)
        moved = [
            warning for warning in data["warnings"]
            if "waypoint (1500, 900) mm sits inside the robot clearance zone" in warning
        ]
        self.assertTrue(moved, data["warnings"])
        safe = [float(value) for value in re.findall(r"-?\d+", moved[0])]
        safe_point = (safe[2], safe[3])  # text: waypoint (x, y) ... closest (x, y)
        # the waypoint sweep may be driven as a native ARC: measure on the
        # executed polyline (nodes plus arc samples)
        points = program_points(data["segments"])
        self.assertTrue(points)
        nearest = min(math.hypot(x - safe_point[0], y - safe_point[1]) for x, y in points)
        self.assertLess(nearest, 70.0, "the moved waypoint must be swept closely")

    def _config_value(self, key):
        """Config value."""
        data = self.client.get("/api/config/robot").json()
        return next(register for register in data["registers"] if register["key"] == key)["value"]

    def test_robot_config_endpoints(self):
        # defaults are readable, a write round-trips, save/flash/defaults/load work
        """Robot config endpoints."""
        response = self.client.get("/api/config/robot")
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data["registers"][0]["key"], "map_version")
        self.assertEqual(data["speed_limit_mm_s"], 900)
        self.assertAlmostEqual(self._config_value("wheel_diameter_mm"), 60.0, delta=0.05)
        response = self.client.post("/api/config/robot", json={"values": {"wheel_diameter_mm": 61.0}})
        self.assertEqual(response.status_code, 200, response.text)
        self.assertAlmostEqual(self._config_value("wheel_diameter_mm"), 61.0, delta=0.05)
        # out-of-range and unknown keys are rejected
        self.assertEqual(
            self.client.post("/api/config/robot", json={"values": {"max_speed_mm_s": 950}}).status_code, 422
        )
        self.assertEqual(
            self.client.post("/api/config/robot", json={"values": {"no_such_register": 1}}).status_code, 422
        )
        # save to flash, restore defaults, then reload the saved record
        self.assertEqual(self.client.post("/api/config/robot/save").status_code, 200)
        self.assertEqual(self.client.post("/api/config/robot/defaults").status_code, 200)
        self.assertAlmostEqual(self._config_value("wheel_diameter_mm"), 60.0, delta=0.05)
        self.assertEqual(self.client.post("/api/config/robot/load").status_code, 200)
        self.assertAlmostEqual(self._config_value("wheel_diameter_mm"), 61.0, delta=0.05)
        # leave the board on clean defaults for the following tests
        self.client.post("/api/config/robot/defaults")
        self.client.post("/api/config/robot/save")

    def test_board_errors_endpoint(self):
        # no new events after the latest index
        """Board errors endpoint."""
        latest = self.client.get("/api/errors").json()["latest"]
        fresh = self.client.get("/api/errors", params={"after": latest}).json()
        self.assertEqual(fresh["events"], [])
        # an e-stop is recorded by the board with its name and severity
        self.assertEqual(self.client.post("/api/estop").status_code, 200)
        wait_for(lambda: self.client.get("/api/state").json().get("state") == 6)
        self.assertEqual(self.client.post("/api/clear_faults").status_code, 200)
        wait_for(lambda: self.client.get("/api/state").json().get("state") == 0)
        result = self.client.get("/api/errors", params={"after": latest}).json()
        names = [event["name"] for event in result["events"]]
        self.assertIn("ESTOP", names)
        severity = next(event["severity"] for event in result["events"] if event["name"] == "ESTOP")
        self.assertEqual(severity, "error")

    def test_sequence_with_rotate_and_pause(self):
        """Sequence with rotate and pause."""
        self.client.post("/api/pose", json={"x_mm": 500.0, "y_mm": 500.0, "theta_deg": 0.0})
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - 500.0) < 1.0)
        body = {
            "elements": [
                {"kind": "stroke", "points": [[500.0, 500.0], [800.0, 500.0]]},
                {"kind": "rotate", "angle_deg": 45.0},
                {"kind": "pause", "seconds": 0.4},
                {"kind": "stroke", "points": [[800.0, 500.0], [800.0, 800.0]]},
            ],
            "options": {"speed_mm_s": 255, "goto_step_mm": 250, "avoid_obstacles": False, "smooth": False},
        }
        response = self.client.post("/api/plan", json=body)
        self.assertEqual(response.status_code, 200, response.text)
        kinds = [segment["kind"] for segment in response.json()["segments"]]
        self.assertIn("turn", kinds)
        self.assertIn("pause", kinds)
        started = time.monotonic()
        self.client.post("/api/execute", json={"segments": response.json()["segments"], "deadman_ms": 0})
        final = wait_for(
            lambda: (
                self.client.get("/api/state").json().get("exec", {}).get("status")
                if self.client.get("/api/state").json().get("exec", {}).get("status") in ("done", "aborted")
                else None
            ),
            timeout_s=30.0,
        )
        self.assertEqual(final, "done")
        self.assertGreaterEqual(time.monotonic() - started, 0.4)  # the pause was honoured
        wait_for(lambda: abs(self.client.get("/api/state").json().get("x_mm", 0) - 800.0) < 60.0)
        wait_for(lambda: abs(self.client.get("/api/state").json().get("y_mm", 0) - 800.0) < 60.0)

    def test_trace_endpoints(self):
        """Trace endpoints."""
        self.client.post("/api/pose", json={"x_mm": 450.0, "y_mm": 450.0, "theta_deg": 0.0})
        wait_for(lambda: self.client.get("/api/trace").json()["count"] > 0)
        trace = self.client.get("/api/trace?after=0&limit=200").json()
        directions = {entry["dir"] for entry in trace["entries"]}
        self.assertIn("tx", directions)
        self.assertIn("rx", directions)
        for entry in trace["entries"][:5]:
            self.assertIn("id", entry)
            self.assertIn("payload", entry)
        download = self.client.get("/api/trace/download")
        self.assertEqual(download.status_code, 200)
        self.assertIn("tx", download.text)
        self.assertIn("rx", download.text)
        self.assertIn("attachment", download.headers.get("content-disposition", ""))
        self.client.post("/api/trace/clear")
        # the console polls the board at 10 Hz, so one tx/rx round trip can
        # slip in between the clear and the read - the buffer must be empty
        # apart from that (a broken clear would leave the whole log behind)
        self.assertLessEqual(self.client.get("/api/trace").json()["count"], 4)

    def test_mode_switch(self):
        # idempotent when the requested mode is already active
        """Mode switch."""
        response = self.client.post("/api/mode", json={"transport": "mock"})
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.json()["transport"], "mock")
        # invalid value is rejected
        self.assertEqual(self.client.post("/api/mode", json={"transport": "serial"}).status_code, 422)
        # forcing the real board must fail gracefully when there is no I2C bus
        response = self.client.post("/api/mode", json={"transport": "i2c"})
        if response.status_code == 200:
            # a real bus exists on this machine - switch back for the other tests
            self.assertEqual(self.client.post("/api/mode", json={"transport": "mock"}).status_code, 200)
        else:
            self.assertEqual(response.status_code, 503, response.text)
            self.assertIn("could not switch", response.json()["detail"])
            self.assertEqual(self.client.get("/api/health").json()["transport"], "mock")
        # and the active link still answers
        wait_for(lambda: self.client.get("/api/state").json().get("connected") is True)

    def test_websocket_telemetry(self):
        """Websocket telemetry."""
        with self.client.websocket_connect("/ws") as websocket:
            message = websocket.receive_json()
            for _ in range(5):  # trace events may interleave with telemetry
                if message["type"] == "telemetry":
                    break
                message = websocket.receive_json()
            self.assertEqual(message["type"], "telemetry")
            self.assertTrue(message["data"]["connected"])


if __name__ == "__main__":
    unittest.main()
