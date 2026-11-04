"""Game Configuration REST API: field, artwork, colours, zones, match clock.

The endpoints are exercised through the real ASGI app (mock robot): a saved
document must survive a restart (the file is read again), a primary zone
spawn must become the team match start, the game clock/actions round-trip and
a resized table must re-clamp the zones instead of losing them.

File:    tests/test_map_api.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-23
"""

import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ["ROBOT_TRANSPORT"] = "mock"

try:
    from fastapi.testclient import TestClient
except ImportError:  # pragma: no cover
    TestClient = None

from software.core.world.map_config import MapConfig  # noqa: E402
from software.console.runtime.state import state  # noqa: E402

FIELD_DIR = Path(__file__).resolve().parents[1] / "ui" / "static" / "field"


def _wait_pose(client, x_mm, y_mm, theta_deg, timeout_s=5.0):
    """Poll ``/api/state`` until the fused pose reaches the expected pose."""
    deadline = time.monotonic() + timeout_s
    data = client.get("/api/state").json()
    while time.monotonic() < deadline:
        data = client.get("/api/state").json()
        heading = abs(((float(data.get("theta_deg", 0.0)) - theta_deg + 180.0) % 360.0) - 180.0)
        if (
            abs(float(data.get("x_mm", 0.0)) - x_mm) < 1.5
            and abs(float(data.get("y_mm", 0.0)) - y_mm) < 1.5
            and heading < 1.0
        ):
            return data
        time.sleep(0.05)
    return data


@unittest.skipIf(TestClient is None, "fastapi/httpx not installed")
class MapApiTests(unittest.TestCase):
    """PUT/GET /api/map and the start-zone lifecycle."""

    @classmethod
    def setUpClass(cls):
        """Boot one ASGI app with the mock robot for the class."""
        from software.console.main import app  # imported after ROBOT_TRANSPORT is forced

        cls._ctx = TestClient(app)
        cls.client = cls._ctx.__enter__()

    @classmethod
    def tearDownClass(cls):
        """Release the app context."""
        cls._ctx.__exit__(None, None, None)

    def setUp(self):
        """Point the service at a throwaway map document."""
        self._tmp = tempfile.TemporaryDirectory()
        self._old_config = state.map_config
        self._old_field = {
            "width_mm": state.field["width_mm"],
            "height_mm": state.field["height_mm"],
        }
        self.path = Path(self._tmp.name) / "map.json"
        state.map_config = MapConfig(
            self.path,
            FIELD_DIR,
            {"width_mm": self._old_field["width_mm"], "height_mm": self._old_field["height_mm"]},
        )
        state.apply_map()

    def tearDown(self):
        """Restore the shared service state and drop the temp document."""
        state.map_config = self._old_config
        state.field["width_mm"] = self._old_field["width_mm"]
        state.field["height_mm"] = self._old_field["height_mm"]
        state.resolve_team_starts()
        self._tmp.cleanup()

    def test_get_map_returns_the_document_and_the_artwork(self):
        """The tab can be built from one request (document + file list)."""
        data = self.client.get("/api/map").json()
        self.assertIn("field", data)
        self.assertIn("zones", data)
        names = [row["name"] for row in data["backgrounds"]]
        self.assertIn("table.png", names)
        self.assertNotIn("README.md", names)

    def test_primary_zone_spawn_owns_the_team_start(self):
        """A saved primary zone drives /api/team (and is persisted)."""
        zone = {
            "team": "yellow",
            "type": "primary",
            "x": 0.0,
            "y": 1400.0,
            "w": 700.0,
            "h": 600.0,
            "spawn": {"x": 315.0, "y": 1800.0, "theta_deg": 90.0},
        }
        response = self.client.put("/api/map", json={"zones": [zone]})
        self.assertEqual(response.status_code, 200, response.text)
        saved = response.json()["zones"]
        self.assertEqual(len(saved), 1)
        self.assertEqual(saved[0]["spawn"]["theta_deg"], 90.0)
        team = self.client.get("/api/team").json()
        self.assertEqual(team["teams"]["yellow"]["start"], [315.0, 1800.0, 90.0])
        self.assertTrue(self.path.is_file(), "the document must be saved")
        reloaded = MapConfig(
            self.path, FIELD_DIR, {"width_mm": 3000.0, "height_mm": 2000.0}
        )
        self.assertEqual(reloaded.spawn_for("yellow"), (315.0, 1800.0, 90.0))

    def test_mock_robot_is_placed_in_its_new_start_zone(self):
        """Saving our primary zone teleports the MOCK robot into it.

        The start zone IS the match start, so the rehearsal robot must stand
        there (on the REAL robot the console only prints where to place it).
        """
        # park the robot somewhere else first: the zone save must move it
        self.client.post("/api/pose", json={"x_mm": 1000.0, "y_mm": 1000.0, "theta_deg": 0.0})
        _wait_pose(self.client, 1000.0, 1000.0, 0.0)
        zone = {
            "team": "yellow",
            "type": "primary",
            "x": 0.0,
            "y": 300.0,
            "w": 700.0,
            "h": 600.0,
            "spawn": {"x": 600.0, "y": 550.0, "theta_deg": 45.0},
        }
        self.client.put("/api/map", json={"zones": [zone]})
        data = _wait_pose(self.client, 600.0, 550.0, 45.0)
        self.assertAlmostEqual(data["x_mm"], 600.0, delta=1.5)
        self.assertAlmostEqual(data["y_mm"], 550.0, delta=1.5)
        self.assertAlmostEqual(data["theta_deg"], 45.0, delta=1.0)

    def test_reset_to_start_puts_the_robot_back_on_our_start(self):
        """POST /api/pose/start returns the robot to our configured start."""
        zone = {
            "team": "yellow",
            "type": "primary",
            "x": 0.0,
            "y": 300.0,
            "w": 700.0,
            "h": 600.0,
            "spawn": {"x": 600.0, "y": 550.0, "theta_deg": 45.0},
        }
        self.client.put("/api/map", json={"zones": [zone]})
        self.client.post("/api/pose", json={"x_mm": 1800.0, "y_mm": 1200.0, "theta_deg": 0.0})
        _wait_pose(self.client, 1800.0, 1200.0, 0.0)

        info = self.client.post("/api/pose/start").json()
        self.assertEqual(info["start"], [600.0, 550.0, 45.0])
        data = _wait_pose(self.client, 600.0, 550.0, 45.0)
        self.assertAlmostEqual(data["x_mm"], 600.0, delta=1.5)
        self.assertAlmostEqual(data["y_mm"], 550.0, delta=1.5)
        self.assertAlmostEqual(data["theta_deg"], 45.0, delta=1.0)

    def test_readiness_follows_the_start_zones(self):
        """The pre-match checklist measures against the configured zones."""
        self.client.put(
            "/api/map",
            json={
                "zones": [
                    {"team": "yellow", "type": "primary", "x": 0.0, "y": 1400.0,
                     "w": 700.0, "h": 600.0,
                     "spawn": {"x": 315.0, "y": 1800.0, "theta_deg": -90.0}},
                    {"team": "blue", "type": "primary", "x": 2300.0, "y": 1400.0,
                     "w": 700.0, "h": 600.0,
                     "spawn": {"x": 2685.0, "y": 1800.0, "theta_deg": -90.0}},
                ]
            },
        )
        # the teleport is applied through the telemetry loop: let it settle
        _wait_pose(self.client, 315.0, 1800.0, -90.0)
        data = self.client.post("/api/selftest/readiness").json()
        checks = {check["name"]: check for check in data["checks"]}
        self.assertEqual(checks["start zones"]["status"], "pass", checks["start zones"])
        # the robot was placed in its zone by the mock transport
        self.assertIn("start zone", checks["start pose"]["detail"])
        self.assertEqual(checks["start pose"]["status"], "pass", checks["start pose"])

    def test_field_change_is_persisted_and_reclamps_the_zones(self):
        """Shrinking the table keeps the zones, clamped inside it."""
        self.client.put(
            "/api/map",
            json={
                "zones": [
                    {"team": "blue", "type": "secondary", "x": 2000.0, "y": 1200.0,
                     "w": 600.0, "h": 500.0}
                ]
            },
        )
        field = self.client.post(
            "/api/field", json={"width_mm": 2500.0, "height_mm": 1500.0}
        )
        self.assertEqual(field.status_code, 200, field.text)
        data = self.client.get("/api/map").json()
        self.assertEqual(data["field"], {"width_mm": 2500.0, "height_mm": 1500.0})
        zone = data["zones"][0]
        self.assertEqual((zone["x"], zone["y"]), (1900.0, 1000.0))

    def test_an_invalid_field_size_is_refused(self):
        """The play area has a sane floor (500 x 500 mm)."""
        response = self.client.put(
            "/api/map", json={"field": {"width_mm": 100.0, "height_mm": 100.0}}
        )
        self.assertEqual(response.status_code, 422)

    def test_background_and_colours_are_saved(self):
        """Artwork choice and the per-team colours round-trip."""
        response = self.client.put(
            "/api/map",
            json={"background": "field/table.png", "colors": {"yellow": "#123456"}},
        )
        self.assertEqual(response.status_code, 200, response.text)
        data = self.client.get("/api/map").json()
        self.assertEqual(data["background"], "field/table.png")
        self.assertEqual(data["colors"]["yellow"], "#123456")
        # a file that is not in the field folder is ignored, never stored
        data = self.client.put("/api/map", json={"background": "../secret.png"}).json()
        self.assertEqual(data["background"], "")

    def test_pdf_background_is_served_as_a_png(self):
        """A PDF table is rasterised by the service and served as image/png."""
        with tempfile.TemporaryDirectory() as folder:
            field = Path(folder) / "field"
            field.mkdir()
            (field / "table.pdf").write_bytes(b"%PDF-1.4\n")
            rendered = field / "rendered.png"
            rendered.write_bytes(b"\x89PNG\r\n\x1a\nstub")
            old_field = state.map_config.field_dir
            state.map_config.field_dir = field
            try:
                listing = self.client.get("/api/map/backgrounds").json()
                pdf_row = next(
                    row for row in listing["files"] if row["name"] == "table.pdf"
                )
                self.assertEqual(pdf_row["kind"], "pdf")
                self.assertTrue(pdf_row["url"].startswith("/api/map/background/table.pdf"))
                with patch(
                    "software.console.api.routes.map.ensure_background_png",
                    lambda path, cache: rendered,
                ):
                    response = self.client.get("/api/map/background/table.pdf")
                self.assertEqual(response.status_code, 200, response.text)
                self.assertEqual(response.headers["content-type"], "image/png")
                self.assertTrue(response.content.startswith(b"\x89PNG"))
                self.assertEqual(
                    self.client.get("/api/map/background/missing.pdf").status_code, 404
                )
                self.assertEqual(
                    self.client.get("/api/map/background/..%2Fmap.json").status_code, 404
                )
                with patch(
                    "software.console.api.routes.map.ensure_background_png",
                    side_effect=RuntimeError("poppler-utils missing"),
                ):
                    failed = self.client.get("/api/map/background/table.pdf")
                self.assertEqual(failed.status_code, 503)
                self.assertIn("poppler", failed.json()["detail"])
            finally:
                state.map_config.field_dir = old_field

    def test_game_clock_settings_and_runtime_round_trip(self):
        """The Game tab saves the clock/actions; start and reset drive it."""
        payload = {
            "duration_s": 120,
            "warning_s": 5,
            "actions": [
                {"id": "late", "at_s": 95, "kind": "log", "enabled": True,
                 "params": {"message": "last 25 s"}},
            ],
        }
        response = self.client.put("/api/map", json={"game": payload})
        self.assertEqual(response.status_code, 200, response.text)
        game = response.json()["game"]
        self.assertEqual(game["duration_s"], 120.0)
        self.assertEqual(game["warning_s"], 5.0)
        self.assertEqual(game["actions"][0]["at_s"], 95.0)

        # the shared app may have run a mission in an earlier test: the
        # runtime clock is process-wide, so start this check from a reset
        self.client.post("/api/game/reset")
        status = self.client.get("/api/game").json()
        self.assertEqual(status["duration_s"], 120.0)
        self.assertFalse(status["running"])

        started = self.client.post("/api/game/start").json()
        self.assertTrue(started["running"])
        self.assertEqual(started["remaining_s"], 120.0)
        reset = self.client.post("/api/game/reset").json()
        self.assertFalse(reset["running"])
        self.assertEqual(reset["elapsed_s"], 0.0)
        # the running clock is part of every telemetry snapshot
        self.assertIn("game", self.client.get("/api/state").json())

    def test_zone_can_be_deleted(self):
        """The list view's Delete removes exactly that zone."""
        data = self.client.put(
            "/api/map",
            json={
                "zones": [
                    {"team": "yellow", "type": "primary", "x": 0.0, "y": 1400.0,
                     "w": 600.0, "h": 500.0},
                    {"team": "blue", "type": "secondary", "x": 2400.0, "y": 1400.0,
                     "w": 600.0, "h": 500.0},
                ]
            },
        ).json()
        first_id = data["zones"][0]["id"]
        response = self.client.delete(f"/api/map/zones/{first_id}")
        self.assertEqual(response.status_code, 200, response.text)
        remaining = response.json()["zones"]
        self.assertEqual(len(remaining), 1)
        self.assertEqual(remaining[0]["team"], "blue")
        self.assertEqual(self.client.delete("/api/map/zones/nope").status_code, 404)


if __name__ == "__main__":
    unittest.main()
