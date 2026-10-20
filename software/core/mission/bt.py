"""Behavior-tree runtime for the brain's match layer (Track C, Docs/10).

A small, dependency-free tick-based behavior tree: the brain composes match
logic from conditions, actions and decorators instead of replaying one
fixed strategy document. The runtime is pure (no board, no I/O): leaves are
plain callables injected by the host, so the whole scheduler is unit-tested
deterministically (see the tests for the exact tick semantics).

Semantics (documented once, relied on by every leaf):

* a tick returns ``RUNNING`` (still working), ``SUCCESS`` or ``FAILURE``;
* composites with state (Sequence, Selector) REMEMBER the running child and
  resume it on the next tick - a child that keeps returning RUNNING is not
  restarted;
* ``Sequence`` fails on the first failing child (resetting itself), a
  ``Selector`` tries children left to right until one succeeds;
* decorators: ``Inverter``, ``Retry(n)`` (one retry per tick), ``Timeout(s)``
  (bounds ONE child tick via asyncio.wait_for - a long action leaf is thus
  cancelled when it overruns), ``ForceSuccess`` and ``ForceFailure``;
* action leaves may be async and may run for a long time (e.g. "play this
  mission and wait for it to finish"); they must tolerate cancellation
  (``Timeout``) by leaving their services in a safe state.

The spec format used by strategy documents v2 lives in
:func:`.from_spec` - structural nodes are parsed here, leaf names are
resolved through a registry supplied by the host (``brain/match/nodes.py``).

File:    software/core/mission/bt.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-09
"""

from __future__ import annotations

import asyncio
from enum import Enum
from typing import Any, Awaitable, Callable, Dict, List, Optional, Sequence, Union

from .interfaces import LogSink


class Status(str, Enum):
    """Result of one tick (str-valued for JSON/log friendliness)."""

    RUNNING = "running"
    SUCCESS = "success"
    FAILURE = "failure"


class TickContext:
    """Everything a leaf may read or write on one tick.

    ``blackboard`` is the shared memory of the tree (budget counts, chosen
    documents, injected clock/deadline...); ``services`` carries the host
    objects a leaf needs (the brain, the client). Both are plain dicts -
    the runtime never interprets them.
    """

    def __init__(
        self,
        blackboard: Optional[Dict[str, Any]] = None,
        services: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Create a context over the given (or fresh) stores."""
        self.blackboard: Dict[str, Any] = blackboard if blackboard is not None else {}
        self.services: Dict[str, Any] = services if services is not None else {}


Leaflike = Union[Status, bool, str, None]
LeafFn = Callable[[TickContext], Union[Leaflike, Awaitable[Leaflike]]]


def status_from(result: Leaflike) -> Status:
    """Normalize a leaf result: bool/None/Status/str -> Status.

    ``True`` and ``None`` mean success (an action that did not fail), any
    other value must be a Status or one of its string values.
    """
    if isinstance(result, Status):
        return result
    if result is None or result is True:
        return Status.SUCCESS
    if result is False:
        return Status.FAILURE
    if isinstance(result, str):
        try:
            return Status(result.lower())
        except ValueError:
            return Status.FAILURE
    return Status.FAILURE


class Node:
    """Base class: one unit of behavior."""

    async def tick(self, ctx: TickContext) -> Status:
        """Advance one tick; must be overridden."""
        raise NotImplementedError

    def reset(self) -> None:
        """Forget any running state (called when a parent resets)."""
        return None


class Action(Node):
    """Leaf: run ``fn`` (sync or async) and normalize its result."""

    def __init__(self, fn: LeafFn) -> None:
        """Wrap a callable as an action leaf."""
        self._fn = fn

    async def tick(self, ctx: TickContext) -> Status:
        """Invoke the action once per tick and normalize the outcome."""
        result = self._fn(ctx)
        if asyncio.iscoroutine(result):
            result = await result
        return status_from(result)


class Condition(Node):
    """Leaf: a predicate (sync or async); True -> SUCCESS, False -> FAILURE."""

    def __init__(self, fn: LeafFn) -> None:
        """Wrap a callable as a condition leaf."""
        self._fn = fn

    async def tick(self, ctx: TickContext) -> Status:
        """Evaluate the predicate once per tick (never RUNNING)."""
        result = self._fn(ctx)
        if asyncio.iscoroutine(result):
            result = await result
        return Status.SUCCESS if result else Status.FAILURE


class Sequence(Node):
    """Run children left to right; fail fast; remember the running child."""

    def __init__(self, children: Sequence[Node]) -> None:
        """Build the sequence over its children."""
        if not children:
            raise ValueError("Sequence needs at least one child")
        self.children: List[Node] = list(children)
        self._current = 0

    def reset(self) -> None:
        """Reset the cursor and every child."""
        self._current = 0
        for child in self.children:
            child.reset()

    async def tick(self, ctx: TickContext) -> Status:
        """Tick from the remembered child; stop at the first failure."""
        while self._current < len(self.children):
            status = await self.children[self._current].tick(ctx)
            if status is Status.RUNNING:
                return Status.RUNNING
            if status is Status.FAILURE:
                self.reset()
                return Status.FAILURE
            self._current += 1
        self.reset()
        return Status.SUCCESS


class Selector(Node):
    """Try children left to right until one succeeds; remember the running one."""

    def __init__(self, children: Sequence[Node]) -> None:
        """Build the selector over its children."""
        if not children:
            raise ValueError("Selector needs at least one child")
        self.children: List[Node] = list(children)
        self._current = 0

    def reset(self) -> None:
        """Reset the cursor and every child."""
        self._current = 0
        for child in self.children:
            child.reset()

    async def tick(self, ctx: TickContext) -> Status:
        """Tick from the remembered child; success ends the selector."""
        while self._current < len(self.children):
            status = await self.children[self._current].tick(ctx)
            if status is Status.RUNNING:
                return Status.RUNNING
            if status is Status.SUCCESS:
                self.reset()
                return Status.SUCCESS
            self._current += 1
        self.reset()
        return Status.FAILURE


class Parallel(Node):
    """Tick every child each tick: all succeed -> SUCCESS, any failure -> FAILURE."""

    def __init__(self, children: Sequence[Node]) -> None:
        """Build the parallel node over its children."""
        if not children:
            raise ValueError("Parallel needs at least one child")
        self.children: List[Node] = list(children)

    def reset(self) -> None:
        """Reset every child."""
        for child in self.children:
            child.reset()

    async def tick(self, ctx: TickContext) -> Status:
        """Advance all children; finish when all succeeded or one failed."""
        results = [await child.tick(ctx) for child in self.children]
        if any(status is Status.FAILURE for status in results):
            self.reset()
            return Status.FAILURE
        if all(status is Status.SUCCESS for status in results):
            self.reset()
            return Status.SUCCESS
        return Status.RUNNING


class Inverter(Node):
    """Swap SUCCESS and FAILURE; RUNNING passes through."""

    def __init__(self, child: Node) -> None:
        """Wrap one child."""
        self.child = child

    def reset(self) -> None:
        """Reset the child."""
        self.child.reset()

    async def tick(self, ctx: TickContext) -> Status:
        """Tick the child and invert a finished result."""
        status = await self.child.tick(ctx)
        if status is Status.SUCCESS:
            return Status.FAILURE
        if status is Status.FAILURE:
            return Status.SUCCESS
        return Status.RUNNING


class ForceSuccess(Node):
    """Turn a failing child into SUCCESS; RUNNING passes through."""

    def __init__(self, child: Node) -> None:
        """Wrap one child."""
        self.child = child

    def reset(self) -> None:
        """Reset the child."""
        self.child.reset()

    async def tick(self, ctx: TickContext) -> Status:
        """Tick the child; a failure is reported as success."""
        status = await self.child.tick(ctx)
        if status is Status.FAILURE:
            self.child.reset()
            return Status.SUCCESS
        return status


class ForceFailure(Node):
    """Turn a succeeding child into FAILURE; RUNNING passes through."""

    def __init__(self, child: Node) -> None:
        """Wrap one child."""
        self.child = child

    def reset(self) -> None:
        """Reset the child."""
        self.child.reset()

    async def tick(self, ctx: TickContext) -> Status:
        """Tick the child; a success is reported as failure."""
        status = await self.child.tick(ctx)
        if status is Status.SUCCESS:
            self.child.reset()
            return Status.FAILURE
        return status


class Retry(Node):
    """Retry a failing child up to ``attempts`` times (one per tick)."""

    def __init__(self, child: Node, attempts: int = 3) -> None:
        """Wrap one child with a total attempt budget (>= 1)."""
        if attempts < 1:
            raise ValueError("attempts must be >= 1")
        self.child = child
        self.attempts = int(attempts)
        self._left = self.attempts

    def reset(self) -> None:
        """Reset the child and the attempt budget."""
        self.child.reset()
        self._left = self.attempts

    async def tick(self, ctx: TickContext) -> Status:
        """Tick the child; a failure consumes one attempt."""
        status = await self.child.tick(ctx)
        if status is Status.FAILURE:
            self.child.reset()
            self._left -= 1
            if self._left > 0:
                return Status.RUNNING  # retried on the next tick
            self.reset()
            return Status.FAILURE
        if status is Status.SUCCESS:
            self.reset()
        return status


class Timeout(Node):
    """Bound ONE child tick with ``asyncio.wait_for``; overrun -> FAILURE."""

    def __init__(self, child: Node, seconds: float) -> None:
        """Wrap one child with a per-tick deadline (seconds > 0)."""
        if seconds <= 0.0:
            raise ValueError("seconds must be > 0")
        self.child = child
        self.seconds = float(seconds)

    def reset(self) -> None:
        """Reset the child."""
        self.child.reset()

    async def tick(self, ctx: TickContext) -> Status:
        """Tick the child; a hanging tick is cancelled and fails."""
        try:
            return await asyncio.wait_for(self.child.tick(ctx), timeout=self.seconds)
        except asyncio.TimeoutError:
            self.child.reset()
            return Status.FAILURE


# ---------------------------------------------------------------- spec loader
def from_spec(spec: Dict[str, Any], registry: Dict[str, Callable[[Dict], LeafFn]]) -> Node:
    """Build a tree from a JSON-style spec and a leaf registry.

    Structural nodes: ``sequence``, ``selector``, ``parallel`` (field
    ``children``); decorators: ``invert``, ``force_success``,
    ``force_failure``, ``retry`` (``attempts``), ``timeout`` (``seconds``) -
    all with field ``child``; leaves: ``action`` / ``condition`` with a
    ``name`` resolved through ``registry`` (which receives the remaining
    fields as parameters). Unknown type or name raises ``ValueError`` - a
    strategy document is validated at load time, never mid-match.
    """
    if not isinstance(spec, dict):
        raise ValueError("tree node must be a mapping")
    kind = str(spec.get("type", "")).strip().lower()
    if kind in ("sequence", "selector", "parallel"):
        children_spec = spec.get("children")
        if not isinstance(children_spec, list) or not children_spec:
            raise ValueError(f"{kind}: 'children' must be a non-empty list")
        children = [from_spec(child, registry) for child in children_spec]
        if kind == "sequence":
            return Sequence(children)
        if kind == "selector":
            return Selector(children)
        return Parallel(children)
    if kind in ("invert", "force_success", "force_failure", "retry", "timeout"):
        if "child" not in spec:
            raise ValueError(f"{kind}: missing 'child'")
        child = from_spec(spec["child"], registry)
        if kind == "invert":
            return Inverter(child)
        if kind == "force_success":
            return ForceSuccess(child)
        if kind == "force_failure":
            return ForceFailure(child)
        if kind == "retry":
            return Retry(child, int(spec.get("attempts", 3)))
        return Timeout(child, float(spec.get("seconds", 1.0)))
    if kind in ("action", "condition"):
        name = str(spec.get("name", "")).strip()
        factory = registry.get(name)
        if factory is None:
            raise ValueError(f"unknown leaf '{name}'")
        params = {key: value for key, value in spec.items() if key not in ("type", "name")}
        wrapped = Action(factory(params)) if kind == "action" else Condition(factory(params))
        return wrapped
    raise ValueError(f"unknown node type '{spec.get('type')}'")


async def tick_until_done(
    tree: Node,
    ctx: TickContext,
    poll_s: float = 0.1,
    log: Optional[LogSink] = None,
) -> Status:
    """Tick a tree until it succeeds or fails (long-running leaves exempt).

    Between ticks the caller yields ``poll_s`` - action leaves that run
    long missions complete INSIDE their own tick, so most trees finish on
    the first few ticks. A short log line per tick change keeps the brain
    log readable.
    """
    last: Optional[Status] = None
    while True:
        status = await tree.tick(ctx)
        if status is not last and log is not None:
            await log("info", f"tree: {status.value}")
            last = status
        if status is not Status.RUNNING:
            return status
        await asyncio.sleep(poll_s)
