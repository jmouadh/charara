"""Camera pose message tests: the ``poses`` type on ``/ws/vision``.

End-to-end: a camera client sends both robots' poses; the service acks with
the ``seen`` map and latency fields, the store reports the snapshot, and the
fusion marks the camera as an active source. The snapshot TTL expiry must
drop the source again (degraded behaviour, no explicit clearing).

File:    tests/test_poses_api.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-05
"""

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


@unittest.skipIf(TestClient is None, "fastapi/httpx not installed")
class PosesApiTests(unittest.TestCase):
    """The camera pose contract end to end (Docs/09 section 8)."""

    @classmethod
    def setUpClass(cls):
        """Prepare the shared fixtures for this test class."""
        from software.console.main import app

        cls._ctx = TestClient(app)
        cls.client = cls._ctx.__enter__()
        wait_for(lambda: cls.client.get("/api/state").json().get("connected"), timeout_s=5.0)

    @classmethod
    def tearDownClass(cls):
        # let the snapshot TTL expire so the next suites start wheel-only
        """Release the shared fixtures of this test class."""
        time.sleep(0.6)
        cls._ctx.__exit__(None, None, None)

    def test_poses_message_is_acked_stored_and_fused(self):
        """Poses message is acked stored and fused."""
        state = self.client.get("/api/state").json()
        x_mm = float(state.get("x_mm", 0.0))
        y_mm = float(state.get("y_mm", 0.0))
        theta_deg = float(state.get("theta_deg", 0.0))
        now_ms = time.time() * 1000.0

        with self.client.websocket_connect("/ws/vision") as websocket:
            hello = websocket.receive_json()
            self.assertEqual(hello["type"], "hello")
            websocket.send_json({
                "type": "poses",
                "us": {"x_mm": x_mm + 3.0, "y_mm": y_mm, "theta_deg": theta_deg, "conf": 0.9},
                "opponent": {"x_mm": 2450.0, "y_mm": 1210.0, "theta_deg": -95.0},
                "t": now_ms - 50.0,  # a 50 ms pipeline delay
                "ttl_ms": 300,
                "client": "camera",
            })
            ack = websocket.receive_json()
            self.assertEqual(ack["type"], "ack")
            self.assertTrue(ack["seen"]["us"])
            self.assertTrue(ack["seen"]["opponent"])
            self.assertTrue(ack["changed"])
            self.assertAlmostEqual(float(ack["t"]), now_ms - 50.0, delta=1.0)

            # while the client is connected the snapshot is fresh and fused
            snapshot = wait_for(
                lambda: (lambda s: s if s.get("poses", {}).get("fresh") else None)(
                    self.client.get("/api/state").json()
                ),
                timeout_s=5.0,
            )
            self.assertIsNotNone(snapshot, "the pose snapshot never became fresh")
            self.assertEqual(snapshot["poses"]["seen"], ["opponent", "us"])
            status = self.client.get("/api/state").json()
            self.assertIn("+camera", status.get("loc", {}).get("mode", ""))
            self.assertGreaterEqual(status["loc"].get("camera_samples", 0), 1)

            # TTL expiry: the source drops by itself, no explicit clearing
            time.sleep(0.6)
            expired = self.client.get("/api/state").json()
            self.assertFalse(expired["poses"]["fresh"])
            self.assertNotIn("+camera", expired.get("loc", {}).get("mode", ""))

        # disconnect releases the snapshot (a gone camera must not linger)
        released = wait_for(
            lambda: (lambda s: s if not s["poses"]["seen"] else None)(
                self.client.get("/api/state").json()
            ),
            timeout_s=5.0,
        )
        self.assertIsNotNone(released, "the snapshot was not released on disconnect")
