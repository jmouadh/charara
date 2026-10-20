"""Strategy REST API tests: save / list / load / delete roundtrip.

The console is where strategies are authored: each validated step's elements
are appended and PUT /api/strategies/{name} stores the document the brain
plays later. These tests pin the storage contract (elements, options, notes)
and the name validation.

File:    tests/test_strategy_api.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-30
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ["ROBOT_TRANSPORT"] = "mock"

try:
    from fastapi.testclient import TestClient
except ImportError:  # pragma: no cover
    TestClient = None


@unittest.skipIf(TestClient is None, "fastapi/httpx not installed")
class StrategyApiTests(unittest.TestCase):
    """Round-trip of the strategy document through the REST API."""

    @classmethod
    def setUpClass(cls):
        """Boot the ASGI app with the mock robot (shared by all tests)."""
        from software.console.main import app  # imported after ROBOT_TRANSPORT is forced

        cls._ctx = TestClient(app)
        cls.client = cls._ctx.__enter__()

    @classmethod
    def tearDownClass(cls):
        """Release the shared fixtures of this test class."""
        cls._ctx.__exit__(None, None, None)

    def setUp(self):
        """Redirect strategy storage to a scratch directory."""
        from software.console.runtime.state import state

        self.state = state
        self._previous_dir = state.strategy_dir
        self._tmp = tempfile.TemporaryDirectory()
        state.strategy_dir = self._tmp.name

    def tearDown(self):
        """Restore the strategy directory and clean up."""
        self.state.strategy_dir = self._previous_dir
        self._tmp.cleanup()

    def test_save_list_load_delete_roundtrip(self):
        elements = [
            {"kind": "stroke", "points": [[500.0, 300.0], [1200.0, 300.0]]},
            {"kind": "pause", "seconds": 0.5},
        ]
        body = {
            "elements": elements,
            "options": {"speed_mm_s": 250},
            "notes": "regression strategy",
        }
        saved = self.client.put("/api/strategies/unit-test", json=body)
        self.assertEqual(saved.status_code, 200, saved.text)
        doc = saved.json()["strategy"]
        self.assertEqual(doc["name"], "unit-test")
        self.assertEqual(len(doc["elements"]), 2)
        self.assertGreater(doc["updated_at"], 0.0)

        rows = self.client.get("/api/strategies").json()["strategies"]
        self.assertIn("unit-test", [row["name"] for row in rows])

        loaded = self.client.get("/api/strategies/unit-test")
        self.assertEqual(loaded.status_code, 200, loaded.text)
        self.assertEqual(loaded.json()["elements"], elements)
        self.assertEqual(loaded.json()["options"]["speed_mm_s"], 250)
        self.assertEqual(loaded.json()["notes"], "regression strategy")

        deleted = self.client.delete("/api/strategies/unit-test")
        self.assertEqual(deleted.status_code, 200, deleted.text)
        self.assertEqual(self.client.get("/api/strategies/unit-test").status_code, 404)

    def test_save_replaces_existing_document(self):
        first = {"elements": [{"kind": "stroke", "points": [[0.0, 0.0], [100.0, 0.0]]}]}
        second = {
            "elements": [
                {"kind": "stroke", "points": [[0.0, 0.0], [100.0, 0.0]]},
                {"kind": "rotate", "angle_deg": 90.0},
            ]
        }
        self.assertEqual(self.client.put("/api/strategies/append", json=first).status_code, 200)
        self.assertEqual(self.client.put("/api/strategies/append", json=second).status_code, 200)
        loaded = self.client.get("/api/strategies/append").json()
        self.assertEqual(len(loaded["elements"]), 2)
        self.assertEqual(loaded["elements"][1]["kind"], "rotate")

    def test_bt_document_roundtrip_and_validation(self):
        """A v2 (behavior tree) document round-trips; bad ones 422 at SAVE.

        The console API is the documented way to store a v2 document
        (Docs/05 section 3, Docs/08 section 8); an unpalayable one must be
        rejected here so the brain never meets it at play time.
        """
        tree = {
            "type": "sequence",
            "children": [{"type": "action", "name": "log", "message": "hi"}],
        }
        body = {"elements": [], "kind": "bt", "tree": tree, "notes": "v2 api"}
        response = self.client.put("/api/strategies/bt-doc", json=body)
        self.assertEqual(response.status_code, 200, response.text)
        loaded = self.client.get("/api/strategies/bt-doc").json()
        self.assertEqual(loaded["kind"], "bt")
        self.assertEqual(loaded["tree"], tree)
        # re-saving without kind/tree falls back to a v1 'elements' document
        again = self.client.put(
            "/api/strategies/bt-doc",
            json={"elements": [{"kind": "rotate", "angle_deg": 90.0}]},
        )
        self.assertEqual(again.json()["strategy"]["kind"], "elements")
        self.assertIsNone(again.json()["strategy"]["tree"])
        # validation at save time: bt without tree / unknown kind / stray tree
        bad = self.client.put("/api/strategies/bt-bad", json={"elements": [], "kind": "bt"})
        self.assertEqual(bad.status_code, 422)
        bad = self.client.put("/api/strategies/bt-bad", json={"elements": [], "kind": "warp"})
        self.assertEqual(bad.status_code, 422)
        bad = self.client.put(
            "/api/strategies/bt-bad", json={"elements": [], "tree": {"type": "warp"}}
        )
        self.assertEqual(bad.status_code, 422)

    def test_invalid_name_is_rejected(self):
        response = self.client.put("/api/strategies/*nope", json={"elements": []})
        self.assertEqual(response.status_code, 422, response.text)
        self.assertIn("name", response.json()["detail"])

    def test_missing_strategy_is_404(self):
        self.assertEqual(self.client.get("/api/strategies/does-not-exist").status_code, 404)

    def test_start_pose_roundtrip_and_preserved_when_omitted(self):
        """The recorded start survives re-saves that do not send it."""
        body = {"elements": [], "start_pose": [400.0, 1200.0, 30.0]}
        saved = self.client.put("/api/strategies/start-pose", json=body)
        self.assertEqual(saved.status_code, 200, saved.text)
        self.assertEqual(saved.json()["strategy"]["start_pose"], [400.0, 1200.0, 30.0])
        loaded = self.client.get("/api/strategies/start-pose").json()
        self.assertEqual(loaded["start_pose"], [400.0, 1200.0, 30.0])
        # appending a step without sending start_pose must NOT lose it
        again = self.client.put(
            "/api/strategies/start-pose",
            json={"elements": [{"kind": "rotate", "angle_deg": 90.0}]},
        )
        self.assertEqual(again.json()["strategy"]["start_pose"], [400.0, 1200.0, 30.0])
        # an explicit new value wins; a 2-value pose is accepted too
        moved = self.client.put(
            "/api/strategies/start-pose",
            json={"elements": [], "start_pose": [10.0, 20.0]},
        )
        self.assertEqual(moved.json()["strategy"]["start_pose"], [10.0, 20.0])

    def test_load_missing_strategy_is_404(self):
        self.assertEqual(self.client.post("/api/strategies/nope/load").status_code, 404)


if __name__ == "__main__":
    unittest.main()
