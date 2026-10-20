"""Behavior-tree runtime tests (Track C, Docs/10 section 7).

Deterministic scheduler semantics only: composites with running-child
memory, decorators (retry budget, per-tick timeout, force/invert), the spec
loader and the tick loop. No robot, no timing flakiness (the one timing
test uses a 4x margin).
"""

from __future__ import annotations

import asyncio
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from software.core.mission.bt import (  # noqa: E402
    Action,
    Condition,
    ForceFailure,
    ForceSuccess,
    Inverter,
    Parallel,
    Retry,
    Selector,
    Sequence,
    Status,
    TickContext,
    Timeout,
    from_spec,
    status_from,
    tick_until_done,
)


def _const(value: Status):
    """Leaf returning a fixed status on every tick."""

    async def _fn(_ctx):
        return value

    return _fn


def _flip(values):
    """Leaf returning ``values`` one per tick (last value repeats)."""
    state = {"i": 0}

    async def _fn(_ctx):
        index = min(state["i"], len(values) - 1)
        state["i"] += 1
        return values[index]

    return _fn


class NormalizationTests(unittest.TestCase):
    """``status_from`` mapping rules."""

    def test_mapping(self):
        """None/True -> SUCCESS, False -> FAILURE, strings parse, Status passes."""
        self.assertIs(status_from(None), Status.SUCCESS)
        self.assertIs(status_from(True), Status.SUCCESS)
        self.assertIs(status_from(False), Status.FAILURE)
        self.assertIs(status_from("success"), Status.SUCCESS)
        self.assertIs(status_from("FAILURE"), Status.FAILURE)
        self.assertIs(status_from(Status.RUNNING), Status.RUNNING)
        self.assertIs(status_from("garbage"), Status.FAILURE)


class CompositeTests(unittest.IsolatedAsyncioTestCase):
    """Sequence / Selector / Parallel semantics."""

    async def test_sequence_runs_in_order_and_fails_fast(self):
        """Order is left-to-right; the first failure ends the tick."""
        calls = []

        def track(name, result):
            async def _fn(_ctx):
                calls.append(name)
                return result

            return _fn

        tree = Sequence(
            [Action(track("a", True)), Action(track("b", False)), Action(track("c", True))]
        )
        ctx = TickContext()
        self.assertIs(await tree.tick(ctx), Status.FAILURE)
        self.assertEqual(calls, ["a", "b"])

    async def test_sequence_resumes_the_running_child(self):
        """A RUNNING child is not restarted on the next tick."""
        starts = {"count": 0}

        async def stateful(_ctx):
            starts["count"] += 1
            return Status.SUCCESS if starts["count"] >= 3 else Status.RUNNING

        order = []

        async def second(_ctx):
            order.append("second")
            return Status.SUCCESS

        tree = Sequence([Action(stateful), Action(second)])
        ctx = TickContext()
        self.assertIs(await tree.tick(ctx), Status.RUNNING)
        self.assertIs(await tree.tick(ctx), Status.RUNNING)
        self.assertIs(await tree.tick(ctx), Status.SUCCESS)
        self.assertEqual(starts["count"], 3)  # never restarted
        self.assertEqual(order, ["second"])

    async def test_selector_takes_the_first_success(self):
        """Failing children are skipped; the first success ends the selector."""
        tree = Selector(
            [Action(_const(Status.FAILURE)), Action(_const(Status.SUCCESS)), Action(_const(Status.FAILURE))]
        )
        self.assertIs(await tree.tick(TickContext()), Status.SUCCESS)

    async def test_selector_exhausted_fails(self):
        """All children failing ends the selector with FAILURE."""
        tree = Selector([Action(_const(Status.FAILURE)), Action(_const(Status.FAILURE))])
        self.assertIs(await tree.tick(TickContext()), Status.FAILURE)

    async def test_parallel_waits_for_every_child(self):
        """RUNNING until all children finished; any failure fails it."""
        tree = Parallel([Action(_flip([Status.RUNNING, Status.SUCCESS])), Action(_const(Status.SUCCESS))])
        ctx = TickContext()
        self.assertIs(await tree.tick(ctx), Status.RUNNING)
        self.assertIs(await tree.tick(ctx), Status.SUCCESS)
        tree2 = Parallel([Action(_const(Status.SUCCESS)), Action(_const(Status.FAILURE))])
        self.assertIs(await tree2.tick(TickContext()), Status.FAILURE)


class DecoratorTests(unittest.IsolatedAsyncioTestCase):
    """Inverter / Force / Retry / Timeout."""

    async def test_inverter(self):
        """SUCCESS <-> FAILURE, RUNNING passes through."""
        self.assertIs(await Inverter(Action(_const(Status.SUCCESS))).tick(TickContext()), Status.FAILURE)
        self.assertIs(await Inverter(Action(_const(Status.FAILURE))).tick(TickContext()), Status.SUCCESS)
        self.assertIs(await Inverter(Action(_const(Status.RUNNING))).tick(TickContext()), Status.RUNNING)

    async def test_force_variants(self):
        """ForceSuccess hides failures; ForceFailure hides successes."""
        self.assertIs(
            await ForceSuccess(Action(_const(Status.FAILURE))).tick(TickContext()), Status.SUCCESS
        )
        self.assertIs(
            await ForceFailure(Action(_const(Status.SUCCESS))).tick(TickContext()), Status.FAILURE
        )

    async def test_retry_succeeds_within_budget(self):
        """Two failures then a success: RUNNING, RUNNING, SUCCESS."""
        tree = Retry(Action(_flip([Status.FAILURE, Status.FAILURE, Status.SUCCESS])), attempts=3)
        ctx = TickContext()
        self.assertIs(await tree.tick(ctx), Status.RUNNING)
        self.assertIs(await tree.tick(ctx), Status.RUNNING)
        self.assertIs(await tree.tick(ctx), Status.SUCCESS)

    async def test_retry_exhausts_the_budget(self):
        """A permanently failing child ends FAILURE after ``attempts`` ticks."""
        tree = Retry(Action(_const(Status.FAILURE)), attempts=2)
        ctx = TickContext()
        self.assertIs(await tree.tick(ctx), Status.RUNNING)
        self.assertIs(await tree.tick(ctx), Status.FAILURE)

    async def test_retry_resets_the_child(self):
        """Every attempt starts the child from a clean state."""
        seen = {"runs": 0}

        async def child(_ctx):
            seen["runs"] += 1
            return Status.FAILURE if seen["runs"] < 3 else Status.SUCCESS

        tree = Retry(Action(child), attempts=3)
        ctx = TickContext()
        while await tree.tick(ctx) is Status.RUNNING:
            pass
        self.assertEqual(seen["runs"], 3)

    async def test_timeout_cancels_a_hanging_tick(self):
        """A child tick overrunning its deadline fails quickly."""

        async def slow(_ctx):
            await asyncio.sleep(0.4)
            return Status.SUCCESS

        tree = Timeout(Action(slow), seconds=0.05)
        start = asyncio.get_running_loop().time()
        self.assertIs(await tree.tick(TickContext()), Status.FAILURE)
        self.assertLess(asyncio.get_running_loop().time() - start, 0.3)

    async def test_invalid_parameters_are_rejected(self):
        """Constructor guards: attempts >= 1, seconds > 0, children non-empty."""
        with self.assertRaises(ValueError):
            Retry(Action(_const(Status.SUCCESS)), attempts=0)
        with self.assertRaises(ValueError):
            Timeout(Action(_const(Status.SUCCESS)), seconds=0.0)
        with self.assertRaises(ValueError):
            Sequence([])
        with self.assertRaises(ValueError):
            Selector([])
        with self.assertRaises(ValueError):
            Parallel([])


class SpecLoaderTests(unittest.IsolatedAsyncioTestCase):
    """``from_spec`` builds trees from JSON-style documents."""

    def _registry(self):
        """A tiny registry with two parametric leaves."""
        calls = []

        def make_log(params):
            async def _fn(_ctx):
                calls.append(params.get("message", ""))
                return True

            return _fn

        def make_flag(params):
            def _fn(_ctx):
                return bool(params.get("value", False))

            return _fn

        return {"log": make_log, "flag": make_flag}, calls

    async def test_nested_spec_builds_and_ticks(self):
        """A sequence/selector document ticks through the registry."""
        registry, calls = self._registry()
        spec = {
            "type": "sequence",
            "children": [
                {"type": "action", "name": "log", "message": "hello"},
                {
                    "type": "selector",
                    "children": [
                        {"type": "condition", "name": "flag", "value": False},
                        {"type": "action", "name": "log", "message": "fallback"},
                    ],
                },
            ],
        }
        tree = from_spec(spec, registry)
        self.assertIs(await tree.tick(TickContext()), Status.SUCCESS)
        self.assertEqual(calls, ["hello", "fallback"])

    async def test_decorator_specs(self):
        """retry/timeout decorators are parsed from the document."""
        registry, _calls = self._registry()
        spec = {
            "type": "retry",
            "attempts": 2,
            "child": {
                "type": "timeout",
                "seconds": 0.05,
                "child": {"type": "action", "name": "log", "message": "x"},
            },
        }
        tree = from_spec(spec, registry)
        self.assertIs(await tree.tick(TickContext()), Status.SUCCESS)

    def test_unknown_leaf_or_type_is_rejected(self):
        """Validation happens at load time - never mid-match."""
        registry, _calls = self._registry()
        with self.assertRaises(ValueError):
            from_spec({"type": "action", "name": "nope"}, registry)
        with self.assertRaises(ValueError):
            from_spec({"type": "warp"}, registry)
        with self.assertRaises(ValueError):
            from_spec({"type": "sequence", "children": []}, registry)
        with self.assertRaises(ValueError):
            from_spec({"type": "retry"}, registry)


class TickLoopTests(unittest.IsolatedAsyncioTestCase):
    """``tick_until_done`` drives a tree to its final status."""

    async def test_loop_returns_the_final_status(self):
        """A quick sequence completes on its first ticks."""
        tree = Sequence([Action(_const(Status.SUCCESS)), Action(_const(Status.SUCCESS))])
        status = await tick_until_done(tree, TickContext(), poll_s=0.01)
        self.assertIs(status, Status.SUCCESS)

    async def test_loop_waits_through_running(self):
        """RUNNING ticks keep the loop alive until the tree finishes."""
        tree = Action(_flip([Status.RUNNING, Status.RUNNING, Status.SUCCESS]))
        status = await tick_until_done(tree, TickContext(), poll_s=0.01)
        self.assertIs(status, Status.SUCCESS)

    async def test_blackboard_is_shared_with_leaves(self):
        """Leaves read and write the same blackboard in one context."""

        async def write(ctx):
            ctx.blackboard["answer"] = 42
            return True

        async def read(ctx):
            return ctx.blackboard.get("answer") == 42

        tree = Sequence([Action(write), Condition(read)])
        self.assertIs(await tree.tick(TickContext()), Status.SUCCESS)


if __name__ == "__main__":
    unittest.main(verbosity=2)
