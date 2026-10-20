"""Team sides (Yellow/Blue): mirroring, team switching, strategy teams, redo.

The table is symmetric about its VERTICAL centre line, so a strategy
authored for one side of a Eurobot match can be mirrored for the other one
(90 deg left becomes 90 deg right). The header selector sets WHICH side we
play; our match-start pose (and the opponent's) follow from it. The mock
"Redo" rehearsal teleports the simulator back to the exact start of the last
run so the same plan can be replayed - and if the step is SAVED instead, the
robot is put back at the run's end so the strategy continues from there.

File:    tests/test_team.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-01
"""

import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from software.core.mission.strategy import Strategy  # noqa: E402
from software.core.mission.teams import (  # noqa: E402
    TEAM_BLUE,
    TEAM_YELLOW,
    check_team,
    mirror_elements,
    mirror_goal,
    mirror_heading,
    mirror_obstacle,
    mirror_point,
    mirror_pose,
    mirror_strategy,
    other_team,
)

W = 3000.0  # field width used by every mirror test


class MirrorTests(unittest.TestCase):
    """The reflection about the vertical centre line (x -> W - x)."""

    def test_point_pose_and_heading(self):
        """Points, poses and headings mirror; mirroring twice is identity."""
        for point in ([100.0, 500.0], [2500.0, 1900.0], [315.0, 1800.0]):
            twice = mirror_point(mirror_point(point, W), W)
            self.assertAlmostEqual(twice[0], point[0], delta=1e-9)
            self.assertAlmostEqual(twice[1], point[1], delta=1e-9)
        pose = (315.0, 1800.0, -90.0)
        twice = mirror_pose(mirror_pose(pose, W), W)
        for got, want in zip(twice, pose):
            self.assertAlmostEqual(got, want, delta=1e-9)
        self.assertEqual(mirror_heading(-90.0), -90.0)  # facing "south" stays
        self.assertEqual(mirror_heading(45.0), 135.0)
        self.assertEqual(mirror_heading(120.0), 60.0)

    def test_team_helpers(self):
        """check_team normalises; other_team flips; junk is rejected."""
        self.assertEqual(check_team(" Yellow "), TEAM_YELLOW)
        self.assertEqual(other_team(TEAM_YELLOW), TEAM_BLUE)
        self.assertEqual(other_team(TEAM_BLUE), TEAM_YELLOW)
        with self.assertRaises(ValueError):
            check_team("green")

    def test_elements_mirror_rotations_and_keep_distances(self):
        """Left turns become right turns; forward stays forward; roundtrip exact."""
        elements = [
            {"kind": "stroke", "points": [[300.0, 100.0], [900.0, 1000.0]]},
            {"kind": "rotate", "angle_deg": 90.0},
            {"kind": "drive", "distance_mm": 100.0},
            {"kind": "waypoint", "x": 600.0, "y": 989.0, "theta_deg": 30.0, "combine": True},
            {"kind": "pause", "seconds": 2.0},
        ]
        mirrored = mirror_elements(elements, W)
        self.assertEqual(mirrored[0]["points"], [[2700.0, 100.0], [2100.0, 1000.0]])
        self.assertEqual(mirrored[1]["angle_deg"], -90.0)  # 90 deg left -> 90 deg right
        self.assertEqual(mirrored[2]["distance_mm"], 100.0)  # forward stays forward
        self.assertEqual(mirrored[3]["x"], 2400.0)
        self.assertEqual(mirrored[3]["theta_deg"], 150.0)
        self.assertTrue(mirrored[3]["combine"])
        self.assertEqual(mirrored[4]["seconds"], 2.0)
        back = mirror_elements(mirrored, W)
        self.assertEqual(back[0]["points"], elements[0]["points"])
        self.assertEqual(back[1]["angle_deg"], elements[1]["angle_deg"])
        self.assertEqual(back[3]["x"], elements[3]["x"])
        self.assertEqual(back[3]["theta_deg"], elements[3]["theta_deg"])

    def test_goal_and_obstacles_mirror(self):
        """Goal poses and obstacles (rect corner, circle centre) mirror."""
        goal = mirror_goal({"x": 500.0, "y": 400.0, "theta_deg": 20.0}, W)
        self.assertEqual(goal["x"], 2500.0)
        self.assertEqual(goal["theta_deg"], 160.0)
        rect = mirror_obstacle({"kind": "rect", "x": 1400.0, "y": 900.0, "w": 200.0, "h": 240.0}, W)
        self.assertEqual(rect["x"], 1400.0)
        rect2 = mirror_obstacle({"kind": "rect", "x": 100.0, "y": 900.0, "w": 200.0, "h": 240.0}, W)
        self.assertEqual(rect2["x"], 2700.0)
        circle = mirror_obstacle({"kind": "circle", "x": 600.0, "y": 400.0, "r": 50.0}, W)
        self.assertEqual(circle["x"], 2400.0)

    def test_strategy_mirror_keeps_the_authoring_tag(self):
        """Mirroring a document flips its geometry, not who wrote it."""
        strategy = Strategy(
            name="sym",
            elements=[{"kind": "rotate", "angle_deg": 45.0}],
            goal={"x": 100.0, "y": 100.0, "theta_deg": 0.0},
            team=TEAM_YELLOW,
        )
        mirror_strategy(strategy, W)
        self.assertEqual(strategy.elements[0]["angle_deg"], -45.0)
        self.assertEqual(strategy.goal["x"], 2900.0)
        self.assertEqual(strategy.team, TEAM_YELLOW)

    def test_strategy_mirror_reflects_the_recorded_start_pose(self):
        """The recorded start of a mission follows the geometry to the side."""
        strategy = Strategy(
            name="sym-start",
            elements=[],
            team=TEAM_BLUE,
            start_pose=[2685.0, 1800.0, -90.0],
        )
        mirror_strategy(strategy, W)
        self.assertEqual(strategy.start_pose, [315.0, 1800.0, -90.0])


try:
    os.environ.setdefault("ROBOT_TRANSPORT", "mock")
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


@unittest.skipIf(TestClient is None, "fastapi/httpx not installed")
class TeamApiTests(unittest.TestCase):
    """Team switching, start poses, mirrored strategies and the mock Redo."""

    @classmethod
    def setUpClass(cls):
        """Enter the console app with a private strategies directory."""
        from software.console.main import app
        from software.console.runtime.state import state

        cls._ctx = TestClient(app)
        cls.client = cls._ctx.__enter__()
        cls.state = state
        cls._strategy_dir = tempfile.mkdtemp(prefix="charara-strategies-")
        cls._saved_dir = state.strategy_dir
        state.strategy_dir = cls._strategy_dir
        wait_for(
            lambda: cls.client.get("/api/state").json().get("connected"), timeout_s=5.0
        )

    @classmethod
    def tearDownClass(cls):
        """Restore the yellow team, the strategy dir and leave the app."""
        cls.client.post("/api/team", json={"team": "yellow"})
        cls.state.strategy_dir = cls._saved_dir
        cls._ctx.__exit__(None, None, None)

    def _wait_pose(self, x_mm, y_mm, theta_deg, timeout_s=5.0):
        """Wait until /api/state reports the given pose (hard KF reset)."""

        def probe():
            state = self.client.get("/api/state").json()
            # 3 mm: a straight run drives as flat ARCs now and the S that keeps
            # the tangent chain adds up to ~2 mm of lateral error (the real
            # robot's localisation is coarser than that)
            if (
                abs(state.get("x_mm", 1e9) - x_mm) < 3.0
                and abs(state.get("y_mm", 1e9) - y_mm) < 3.0
                and abs(state.get("theta_deg", 1e9) - theta_deg) < 2.0
            ):
                return state
            return None

        self.assertIsNotNone(wait_for(probe, timeout_s=timeout_s), f"pose never reached ({x_mm}, {y_mm}, {theta_deg})")

    def test_team_switch_moves_the_mock_robot_to_the_start(self):
        """Yellow -> Blue teleports the simulator to the mirrored start."""
        self.client.post("/api/team", json={"team": "yellow"})
        info = self.client.get("/api/team").json()
        self.assertEqual(info["current"], "yellow")
        self.assertEqual(info["label"], "Yellow")
        self.assertEqual(info["start"], [315.0, 1800.0, -90.0])
        self.assertEqual(info["opponent"]["team"], "blue")
        # a missing blue start mirrors the yellow one about the vertical centre
        self.assertEqual(info["opponent"]["start"], [2685.0, 1800.0, -90.0])

        info = self.client.post("/api/team", json={"team": "blue"}).json()
        self.assertEqual(info["current"], "blue")
        self._wait_pose(2685.0, 1800.0, -90.0)

        response = self.client.post("/api/team", json={"team": "green"})
        self.assertEqual(response.status_code, 422)

        self.client.post("/api/team", json={"team": "yellow"})
        self._wait_pose(315.0, 1800.0, -90.0)

    def test_strategy_is_mirrored_when_loaded_for_the_other_team(self):
        """A Yellow strategy loads mirrored while playing Blue (and back)."""
        self.client.post("/api/team", json={"team": "yellow"})
        elements = [
            {"kind": "stroke", "points": [[300.0, 100.0], [900.0, 1000.0]]},
            {"kind": "rotate", "angle_deg": 90.0},
        ]
        response = self.client.put(
            "/api/strategies/team-mirror-test", json={"elements": elements, "options": {}}
        )
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.json()["strategy"]["team"], "yellow")

        self.client.post("/api/team", json={"team": "blue"})
        doc = self.client.get("/api/strategies/team-mirror-test").json()
        self.assertTrue(doc["mirrored"])
        self.assertEqual(doc["authored_for"], "yellow")
        self.assertEqual(doc["team"], "blue")
        self.assertEqual(doc["elements"][0]["points"], [[2700.0, 100.0], [2100.0, 1000.0]])
        self.assertEqual(doc["elements"][1]["angle_deg"], -90.0)

        self.client.post("/api/team", json={"team": "yellow"})
        doc = self.client.get("/api/strategies/team-mirror-test").json()
        self.assertFalse(doc["mirrored"])
        self.assertEqual(doc["elements"], elements)
        self.client.delete("/api/strategies/team-mirror-test")

    def test_load_places_the_mock_robot_at_the_recorded_start_mirrored(self):
        """POST /load reflects the start pose and teleports the simulator."""
        self.client.post("/api/team", json={"team": "yellow"})
        body = {
            "elements": [{"kind": "stroke", "points": [[300.0, 100.0], [900.0, 100.0]]}],
            "options": {},
            "start_pose": [400.0, 1200.0, 30.0],
        }
        self.assertEqual(
            self.client.put("/api/strategies/start-pose-test", json=body).status_code, 200
        )
        # same side: placed at the recorded pose, not mirrored
        doc = self.client.post("/api/strategies/start-pose-test/load").json()
        self.assertTrue(doc["teleported"])
        self.assertFalse(doc["mirrored"])
        self._wait_pose(400.0, 1200.0, 30.0)
        # other side: start reflected about the centre, robot placed THERE
        self.client.post("/api/team", json={"team": "blue"})
        doc = self.client.post("/api/strategies/start-pose-test/load").json()
        self.assertTrue(doc["teleported"])
        self.assertTrue(doc["mirrored"])
        self.assertAlmostEqual(doc["start_pose"][0], 2600.0, places=3)
        self.assertAlmostEqual(doc["start_pose"][2], 150.0, places=3)
        self._wait_pose(2600.0, 1200.0, 150.0)
        self.client.delete("/api/strategies/start-pose-test")

    def test_redo_replays_from_the_run_start_and_save_restores_the_end(self):
        """Redo teleports to the run start; saving puts the robot back at its end."""
        self.client.post("/api/team", json={"team": "yellow"})
        self.client.post("/api/pose", json={"x_mm": 500.0, "y_mm": 500.0, "theta_deg": 0.0})
        self._wait_pose(500.0, 500.0, 0.0)

        body = {
            "strokes": [[[500.0, 500.0], [900.0, 500.0]]],
            "options": {"speed_mm_s": 255, "goto_step_mm": 250, "avoid_obstacles": False, "smooth": False},
        }
        plan = self.client.post("/api/plan", json=body).json()
        self.assertEqual(self.client.post("/api/execute", json={"segments": plan["segments"]}).status_code, 200)

        done = wait_for(
            lambda: self.client.get("/api/state").json().get("exec", {}).get("status") == "done",
            timeout_s=30.0,
        )
        self.assertTrue(done, "the run never finished")

        info = self.client.get("/api/state").json()["redo"]
        self.assertTrue(info["available"])
        self.assertEqual(info["start"], [500.0, 500.0, 0.0])

        response = self.client.post("/api/mission/redo")
        self.assertEqual(response.status_code, 200, response.text)
        self._wait_pose(500.0, 500.0, 0.0)
        self.assertTrue(self.client.get("/api/state").json()["redo"]["stashed"])

        # saving the step resolves the redo: the mission continues from the
        # RUN'S END, not from the replayed start
        response = self.client.put(
            "/api/strategies/redo-test",
            json={"elements": [{"kind": "stroke", "points": [[500.0, 500.0], [900.0, 500.0]]}]},
        )
        self.assertEqual(response.status_code, 200, response.text)
        self._wait_pose(900.0, 500.0, 0.0)
        self.assertFalse(self.client.get("/api/state").json()["redo"]["stashed"])
        self.client.delete("/api/strategies/redo-test")


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
