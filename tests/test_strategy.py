"""Strategy document tests: validation, JSON round-trip, storage helpers.

File:    tests/test_strategy.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from software.core.mission.strategy import Strategy, check_name


class NameTests(unittest.TestCase):
    def test_valid_names(self) -> None:
        """Valid names."""
        for name in ("main", "Eurobot 2026", "qualif-1", "test_v2", "A"):
            self.assertEqual(check_name(name), name)

    def test_invalid_names(self) -> None:
        """Invalid names."""
        for name in ("", ".hidden", "..", "a/b", "a\\b", "x" * 65, "-lead"):
            with self.assertRaises(ValueError):
                check_name(name)


class RoundTripTests(unittest.TestCase):
    def _sample(self, name: str = "sample") -> Strategy:
        """Build the sample strategy used by the round-trip tests."""
        return Strategy(
            name=name,
            elements=[
                {"kind": "stroke", "points": [[500, 500], [900, 500]]},
                {"kind": "rotate", "angle_deg": 90.0},
                {"kind": "pause", "seconds": 2.0},
            ],
            options={"speed_mm_s": 200, "fit_tol_mm": 8.0},
            goal={"x": 900.0, "y": 500.0, "theta_deg": 90.0},
            obstacles=[{"kind": "rect", "x": 1500, "y": 400, "w": 300, "h": 200}],
            field={"width_mm": 3000.0, "height_mm": 2000.0},
            notes="qualification attempt",
        )

    def test_dict_round_trip(self) -> None:
        """Dict round trip."""
        original = self._sample()
        clone = Strategy.from_dict(original.to_dict())
        self.assertEqual(clone.name, original.name)
        self.assertEqual(clone.elements, original.elements)
        self.assertEqual(clone.options, original.options)
        self.assertEqual(clone.goal, original.goal)
        self.assertEqual(clone.obstacles, original.obstacles)
        self.assertEqual(clone.notes, original.notes)

    def test_save_load_list_delete(self) -> None:
        """Save load list delete."""
        with tempfile.TemporaryDirectory() as folder:
            self.assertEqual(Strategy.list_saved(folder), [])
            strategy = self._sample()
            path = strategy.save(folder)
            self.assertTrue(Path(path).exists())
            self.assertGreater(strategy.updated_at, 0.0)

            clone = Strategy.load(path)
            self.assertEqual(clone.name, "sample")
            self.assertEqual(len(clone.elements), 3)

            loaded = Strategy.load_named(folder, "sample")
            self.assertEqual(loaded.to_dict(), clone.to_dict())

            rows = Strategy.list_saved(folder)
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["name"], "sample")
            self.assertEqual(rows[0]["n_elements"], 3)

            self.assertTrue(Strategy.delete(folder, "sample"))
            self.assertFalse(Strategy.delete(folder, "sample"))
            self.assertEqual(Strategy.list_saved(folder), [])

    def test_broken_file_is_skipped(self) -> None:
        """Broken file is skipped."""
        with tempfile.TemporaryDirectory() as folder:
            self._sample("good").save(folder)
            (Path(folder) / "broken.json").write_text("{not json", encoding="utf-8")
            rows = Strategy.list_saved(folder)
            self.assertEqual([row["name"] for row in rows], ["good"])

    def test_json_document_fields(self) -> None:
        """JSON document fields."""
        with tempfile.TemporaryDirectory() as folder:
            path = self._sample().save(folder)
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            self.assertEqual(data["version"], 2)  # v2 adds kind + tree
            self.assertEqual(data["kind"], "elements")
            self.assertIsNone(data["tree"])
            self.assertEqual(data["name"], "sample")
            self.assertIn("updated_at", data)
            self.assertEqual(data["field"]["width_mm"], 3000.0)

    def test_from_dict_rejects_garbage(self) -> None:
        """From dict rejects garbage."""
        with self.assertRaises(ValueError):
            Strategy.from_dict({"name": "x", "elements": "not-a-list"})
        with self.assertRaises(ValueError):
            Strategy.from_dict({"name": "../escape", "elements": []})


class StrategyApiTests(unittest.TestCase):
    """PUT / GET / DELETE /api/strategies through the real FastAPI app."""

    @classmethod
    def setUpClass(cls) -> None:
        """Prepare the shared fixtures for this test class."""
        from fastapi.testclient import TestClient

        from software.console.main import app

        cls.client = TestClient(app)

    def test_save_load_list_delete(self) -> None:
        """Save load list delete."""
        from software.console.runtime.state import state

        with tempfile.TemporaryDirectory() as folder:
            previous = state.strategy_dir
            state.strategy_dir = folder
            try:
                body = {
                    "elements": [{"kind": "stroke", "points": [[500, 500], [900, 500]]}],
                    "options": {"speed_mm_s": 250},
                    "goal": None,
                    "obstacles": [{"kind": "circle", "x": 1200, "y": 800, "r": 150}],
                    "notes": "api round trip",
                }
                saved = self.client.put("/api/strategies/qualif-1", json=body)
                self.assertEqual(saved.status_code, 200, saved.text)
                self.assertEqual(saved.json()["strategy"]["name"], "qualif-1")

                listed = self.client.get("/api/strategies")
                self.assertEqual(listed.status_code, 200)
                names = [row["name"] for row in listed.json()["strategies"]]
                self.assertEqual(names, ["qualif-1"])

                loaded = self.client.get("/api/strategies/qualif-1")
                self.assertEqual(loaded.status_code, 200)
                doc = loaded.json()
                self.assertEqual(len(doc["elements"]), 1)
                self.assertEqual(doc["obstacles"][0]["kind"], "circle")
                self.assertEqual(doc["notes"], "api round trip")
                self.assertGreater(doc["field"]["width_mm"], 0.0)

                missing = self.client.get("/api/strategies/nope")
                self.assertEqual(missing.status_code, 404)
                bad = self.client.put("/api/strategies/-lead", json=body)
                self.assertEqual(bad.status_code, 422)

                removed = self.client.delete("/api/strategies/qualif-1")
                self.assertEqual(removed.status_code, 200)
                self.assertEqual(self.client.get("/api/strategies").json()["strategies"], [])
                self.assertEqual(self.client.delete("/api/strategies/qualif-1").status_code, 404)
            finally:
                state.strategy_dir = previous


if __name__ == "__main__":
    unittest.main()
