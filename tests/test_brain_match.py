"""Brain match-layer tests (Track C, Docs/10 section 7).

Covered: strategy v2 document round-trips and validation, the leaf
catalogue (actions/conditions), full behavior-tree playback on the mock
board through the real brain, and the KPI log passthrough fix (executor
``log`` events are printed headless).

File:    tests/test_brain_match.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-09
"""

from __future__ import annotations

import asyncio
import tempfile
import time
import unittest
from dataclasses import replace

from software.brain import Brain
from software.brain.match.nodes import build_registry
from software.config import settings
from software.core.mission.bt import Status, TickContext
from software.core.mission.strategy import KIND_BT, KIND_ELEMENTS, Strategy
from software.core.world.vision import VisionStore


def make_leg(name: str = "leg") -> Strategy:
    """A small v1 document the tree's run_document leaf can play."""
    return Strategy(
        name=name,
        elements=[
            {"kind": "stroke", "points": [[600, 600], [900, 600]]},
            {"kind": "pause", "seconds": 0.2},
        ],
        options={"speed_mm_s": 300},
    )


async def _wait_for_pose(brain: Brain) -> None:
    """Wait for the first fused pose (mock telemetry settles quickly)."""
    for _ in range(100):
        if brain.pose() is not None:
            return
        await asyncio.sleep(0.05)


class DocumentV2Tests(unittest.TestCase):
    """Strategy v2: kind + tree round-trip, validation, v1 compatibility."""

    def test_bt_document_round_trip(self):
        """A bt document saves and loads with its tree intact."""
        tree = {"type": "sequence", "children": [{"type": "action", "name": "log", "message": "x"}]}
        document = Strategy(name="tree-test", kind=KIND_BT, tree=tree, notes="v2")
        with tempfile.TemporaryDirectory() as folder:
            path = document.save(folder)
            back = Strategy.load(path)
        self.assertEqual(back.kind, KIND_BT)
        self.assertEqual(back.tree, tree)
        self.assertEqual(back.version, 2)

    def test_v1_document_still_loads_as_elements(self):
        """A v1 document (no kind field) defaults to 'elements'."""
        with tempfile.TemporaryDirectory() as folder:
            path = make_leg("v1").save(folder)
            back = Strategy.load(path)
        self.assertEqual(back.kind, KIND_ELEMENTS)
        self.assertIsNone(back.tree)

    def test_bt_without_tree_is_rejected(self):
        """kind 'bt' without a tree object fails at load time."""
        data = {"name": "bad", "kind": "bt", "elements": []}
        with self.assertRaises(ValueError):
            Strategy.from_dict(data)

    def test_unknown_kind_is_rejected(self):
        """Only 'elements' and 'bt' are valid kinds."""
        with self.assertRaises(ValueError):
            Strategy.from_dict({"name": "bad", "kind": "warp", "elements": []})


class VisionFreshnessTests(unittest.TestCase):
    """The camera_fresh condition's data source."""

    def test_fresh_tracks_the_ttl(self):
        """No source -> not fresh; empty-but-fresh snapshot -> fresh."""
        store = VisionStore()
        self.assertFalse(store.fresh)
        store.update([], ttl_ms=150.0, source="camera")
        self.assertTrue(store.fresh)  # an empty snapshot is still a live camera
        time.sleep(0.25)
        self.assertFalse(store.fresh)  # TTL expired


class EventPassthroughTests(unittest.IsolatedAsyncioTestCase):
    """The brain must PRINT executor log events (KPI lines)."""

    async def test_log_events_reach_the_brain_log(self):
        """run finished / run ended lines arrive with their level."""
        brain = Brain(replace(settings, transport="mock"))
        printed = []

        async def fake_log(level, message):
            printed.append((level, message))

        brain.log = fake_log  # type: ignore[method-assign]
        line = "run finished: 6 segment(s), 3 blended without stopping"
        await brain.on_event({"type": "log", "data": {"level": "info", "message": line}})
        self.assertIn(("info", line), printed)
        await brain.on_event({"type": "log", "data": {"level": "warn", "message": "run ended 25 mm"}})
        self.assertEqual(printed[-1], ("warn", "run ended 25 mm"))
        await brain.on_event({"type": "log", "data": {"level": "info", "message": ""}})
        self.assertEqual(len(printed), 2)  # empty messages are ignored


class RegistryTests(unittest.IsolatedAsyncioTestCase):
    """Leaf catalogue semantics without a running session."""

    def _brain(self):
        """A brain on the mock transport, not started (no link needed)."""
        return Brain(replace(settings, transport="mock"))

    def test_time_left_uses_the_deadline_and_defaults_true(self):
        """No deadline -> True; passed deadline -> False."""
        brain = self._brain()
        registry = build_registry(brain)
        condition = registry["time_left"]({"seconds": 5.0})
        ctx = TickContext(blackboard={"clock": lambda: 100.0})
        self.assertTrue(condition(ctx))  # no deadline configured
        ctx.blackboard["deadline"] = 110.0
        self.assertTrue(condition(ctx))  # 10 s left >= 5 s
        ctx.blackboard["deadline"] = 102.0
        self.assertFalse(condition(ctx))  # only 2 s left

    async def test_score_actions_and_condition(self):
        """add_score raises the blackboard; score_at_least reads it."""
        brain = self._brain()
        printed = []

        async def fake_log(level, message):
            printed.append(message)

        brain.log = fake_log  # type: ignore[method-assign]
        registry = build_registry(brain)
        ctx = TickContext(blackboard={"score": 0.0})
        self.assertIs(await registry["add_score"]({"points": 3})(ctx), Status.SUCCESS)
        # conditions are plain predicates: boolean, not a coroutine
        self.assertTrue(registry["score_at_least"]({"points": 3})(ctx))
        self.assertFalse(registry["score_at_least"]({"points": 4})(ctx))

    def test_team_and_bad_parameters(self):
        """team_is compares the configured side; bad params fail at build."""
        brain = self._brain()
        registry = build_registry(brain)
        self.assertTrue(registry["team_is"]({"side": settings.team})(TickContext()))
        with self.assertRaises(ValueError):
            registry["team_is"]({"side": "green"})
        with self.assertRaises(ValueError):
            registry["at_pose"]({"x": 1.0})  # missing y
        with self.assertRaises(ValueError):
            registry["run_document"]({})  # missing doc

    def test_camera_fresh_follows_the_store(self):
        """camera_fresh mirrors VisionStore.fresh."""
        brain = self._brain()
        registry = build_registry(brain)
        condition = registry["camera_fresh"]({})
        self.assertFalse(condition(TickContext()))
        brain.ctx.vision.update([], ttl_ms=1000.0, source="camera")
        self.assertTrue(condition(TickContext()))


class MatchPlaybackTests(unittest.IsolatedAsyncioTestCase):
    """Full tree playback through Brain.play on the mock board."""

    async def test_play_bt_strategy_end_to_end(self):
        """A tree: run a mission, add score, check it, log - all on the mock."""
        with tempfile.TemporaryDirectory() as folder:
            make_leg("leg").save(folder)
            config = replace(settings, transport="mock", strategy_dir=folder)
            brain = Brain(config)
            await brain.start()
            try:
                await _wait_for_pose(brain)
                strategy = Strategy(
                    name="match",
                    kind=KIND_BT,
                    tree={
                        "type": "sequence",
                        "children": [
                            {"type": "action", "name": "run_document", "doc": "leg"},
                            {"type": "action", "name": "add_score", "points": 2},
                            {"type": "condition", "name": "score_at_least", "points": 2},
                            {"type": "action", "name": "log", "message": "mission done"},
                        ],
                    },
                    options={"speed_mm_s": 300, "match_s": 120},
                )
                final = await brain.play(strategy)
                self.assertEqual(final, "done")
                assert brain.ctx.last_plan is not None  # the mission really ran
            finally:
                await brain.stop()

    async def test_selector_falls_back_to_the_backup(self):
        """run_document(missing) fails; the selector plays the fallback leaf."""
        with tempfile.TemporaryDirectory() as folder:
            config = replace(settings, transport="mock", strategy_dir=folder)
            brain = Brain(config)
            await brain.start()
            try:
                await _wait_for_pose(brain)
                strategy = Strategy(
                    name="fallback",
                    kind=KIND_BT,
                    tree={
                        "type": "selector",
                        "children": [
                            {"type": "action", "name": "run_document", "doc": "missing"},
                            {"type": "action", "name": "log", "message": "backup"},
                        ],
                    },
                )
                self.assertEqual(await brain.play(strategy), "done")
            finally:
                await brain.stop()

    async def test_invalid_tree_fails_cleanly(self):
        """An unknown leaf fails the PLAY call, not the process."""
        with tempfile.TemporaryDirectory() as folder:
            config = replace(settings, transport="mock", strategy_dir=folder)
            brain = Brain(config)
            await brain.start()
            try:
                await _wait_for_pose(brain)
                strategy = Strategy(
                    name="broken",
                    kind=KIND_BT,
                    tree={"type": "action", "name": "nope"},
                )
                self.assertEqual(await brain.play(strategy), "failed")
            finally:
                await brain.stop()


if __name__ == "__main__":
    unittest.main(verbosity=2)
