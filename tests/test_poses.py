"""Camera pose store tests: precision, validation, TTL and ownership.

File:    tests/test_poses.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-05
"""

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from software.core.world.poses import PoseStore  # noqa: E402


class PoseStoreTests(unittest.TestCase):
    """ArUco pose snapshot semantics (Docs/09 section 8)."""

    def test_update_keeps_raw_precision_and_wraps(self):
        """Update keeps raw precision and wraps."""
        store = PoseStore()
        changed = store.update(
            {"x_mm": 1245.2, "y_mm": 803.4, "theta_deg": 187.5, "conf": 1.4},
            {"x_mm": 2450.0, "y_mm": 1210.5, "theta_deg": -95.2},
            ttl_ms=5000,
            source="vision:1",
            t_ms=123.0,
            client="camera",
        )
        self.assertTrue(changed)
        us = store.us()
        self.assertAlmostEqual(us["x_mm"], 1245.2)  # no 10 mm quantization
        self.assertAlmostEqual(us["y_mm"], 803.4)
        self.assertAlmostEqual(us["theta_deg"], -172.5)  # wrapped to (-180, 180]
        self.assertAlmostEqual(us["conf"], 1.0)  # clamped to [0, 1]
        self.assertAlmostEqual(store.opponent()["x_mm"], 2450.0)
        self.assertEqual(store.source, "vision:1")
        self.assertEqual(store.reported_t, 123.0)
        status = store.status()
        self.assertTrue(status["fresh"])
        self.assertEqual(status["seen"], ["opponent", "us"])

    def test_identical_refresh_is_not_a_change(self):
        """Identical refresh is not a change."""
        store = PoseStore()
        entry = {"x_mm": 10.0, "y_mm": 20.0, "theta_deg": 5.0}
        self.assertTrue(store.update(entry, None, ttl_ms=5000))
        self.assertFalse(store.update(dict(entry), None, ttl_ms=5000))

    def test_missing_robot_is_dropped(self):
        """Missing robot is dropped."""
        store = PoseStore()
        store.update(None, {"x_mm": 1.0, "y_mm": 2.0, "theta_deg": 3.0}, ttl_ms=5000)
        self.assertIsNone(store.us())
        self.assertIsNotNone(store.opponent())
        self.assertEqual(store.status()["seen"], ["opponent"])

    def test_invalid_entry_is_ignored(self):
        """Invalid entry is ignored."""
        store = PoseStore()
        changed = store.update(
            {"x_mm": "not a number", "y_mm": 0.0, "theta_deg": 0.0}, None, ttl_ms=5000
        )
        self.assertFalse(changed)
        self.assertFalse(store.fresh)
        self.assertEqual(store.active(), {})

    def test_ttl_expiry_marks_the_snapshot_stale(self):
        """Ttl expiry marks the snapshot stale."""
        store = PoseStore()
        store.update({"x_mm": 1.0, "y_mm": 2.0, "theta_deg": 0.0}, None, ttl_ms=100)
        self.assertTrue(store.fresh)
        time.sleep(0.15)
        self.assertFalse(store.fresh)
        self.assertIsNone(store.us())
        self.assertEqual(store.active(), {})
        self.assertFalse(store.status()["fresh"])

    def test_release_only_by_the_owner(self):
        """Release only by the owner."""
        store = PoseStore()
        store.update({"x_mm": 1.0, "y_mm": 2.0, "theta_deg": 0.0}, None, ttl_ms=5000, source="a")
        self.assertFalse(store.release("b"))
        self.assertTrue(store.release("a"))
        self.assertFalse(store.fresh)
        self.assertEqual(store.source, "")

    def test_clear_drops_everything(self):
        """Clear drops everything."""
        store = PoseStore()
        store.update(
            {"x_mm": 1.0, "y_mm": 2.0, "theta_deg": 0.0},
            {"x_mm": 3.0, "y_mm": 4.0, "theta_deg": 0.0},
            ttl_ms=5000,
        )
        self.assertTrue(store.clear())
        self.assertFalse(store.clear())
        self.assertEqual(store.active(), {})
