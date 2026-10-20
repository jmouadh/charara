"""Brain tests: the headless strategy runner on the mock board.

File:    tests/test_brain.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import asyncio
import tempfile
import unittest
from dataclasses import replace

from software.brain import Brain
from software.config import settings
from software.core.mission.strategy import Strategy


def make_strategy(name: str = "brain-test") -> Strategy:
    """A small but real mission: drive out, pause, turn, drive back."""
    return Strategy(
        name=name,
        elements=[
            {"kind": "stroke", "points": [[600, 600], [900, 600]]},
            {"kind": "rotate", "angle_deg": 90.0},
            {"kind": "pause", "seconds": 0.2},
            {"kind": "stroke", "points": [[900, 600], [900, 900]]},
        ],
        options={"speed_mm_s": 300},
    )


class BrainTests(unittest.IsolatedAsyncioTestCase):
    async def test_play_strategy_on_mock(self) -> None:
        """Play strategy on mock."""
        with tempfile.TemporaryDirectory() as folder:
            config = replace(settings, transport="mock", strategy_dir=folder)
            brain = Brain(config)
            await brain.start()
            try:
                # wait for the first telemetry sample (pose becomes available)
                for _ in range(100):
                    if brain.pose() is not None:
                        break
                    await asyncio.sleep(0.05)
                self.assertIsNotNone(brain.pose())

                final = await brain.play(make_strategy())
                self.assertEqual(final, "done")
                assert brain.ctx.executor is not None
                self.assertEqual(brain.ctx.executor.state, "done")
                self.assertIsNotNone(brain.ctx.last_plan)
                self.assertGreater(len(brain.ctx.last_plan["segments"]), 0)

                status = brain.status()
                self.assertTrue(status["connected"])
                self.assertIn("loc", status)
            finally:
                await brain.stop()

    async def test_plan_from_strategy_file(self) -> None:
        """Plan from strategy file."""
        with tempfile.TemporaryDirectory() as folder:
            path = make_strategy("saved").save(folder)
            config = replace(settings, transport="mock", strategy_dir=folder)
            brain = Brain(config)
            await brain.start()
            try:
                for _ in range(100):
                    if brain.pose() is not None:
                        break
                    await asyncio.sleep(0.05)
                strategy = Strategy.load(path)
                self.assertEqual(strategy.name, "saved")
                final = await brain.play(strategy)
                self.assertEqual(final, "done")
            finally:
                await brain.stop()

    async def test_obstacle_report_mid_run_does_not_crash(self) -> None:
        """Obstacle report mid run does not crash."""
        with tempfile.TemporaryDirectory() as folder:
            config = replace(settings, transport="mock", strategy_dir=folder)
            brain = Brain(config)
            await brain.start()
            try:
                for _ in range(100):
                    if brain.pose() is not None:
                        break
                    await asyncio.sleep(0.05)
                play = asyncio.create_task(brain.play(make_strategy()))
                await asyncio.sleep(0.5)
                # a far-away obstacle must not disturb the run
                await brain.set_obstacles(
                    [{"kind": "rect", "x": 2600, "y": 1600, "w": 200, "h": 200}], ttl_ms=5000
                )
                final = await play
                self.assertEqual(final, "done")
            finally:
                await brain.stop()

    async def test_empty_strategy_fails_cleanly(self) -> None:
        """Empty strategy fails cleanly."""
        with tempfile.TemporaryDirectory() as folder:
            config = replace(settings, transport="mock", strategy_dir=folder)
            brain = Brain(config)
            await brain.start()
            try:
                for _ in range(100):
                    if brain.pose() is not None:
                        break
                    await asyncio.sleep(0.05)
                final = await brain.play(Strategy(name="empty", elements=[]))
                self.assertEqual(final, "failed")
            finally:
                await brain.stop()


class BrainCliTests(unittest.TestCase):
    def test_list_and_missing_strategy(self) -> None:
        """List and missing strategy."""
        from software.brain.__main__ import main

        with tempfile.TemporaryDirectory() as folder:
            make_strategy("listed").save(folder)
            self.assertEqual(main(["--dir", folder, "--list"]), 0)
            self.assertEqual(main(["--dir", folder]), 2)  # no strategy given
            self.assertEqual(main(["--dir", folder, "--strategy", "nope"]), 2)  # not found


if __name__ == "__main__":
    unittest.main()
