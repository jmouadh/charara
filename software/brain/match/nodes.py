"""Leaf catalogue of the match layer (Track C, Docs/10 section 7).

The dictionary :func:`build_registry` returns maps leaf names (as they
appear in a strategy v2 document) to factories: a factory receives the
leaf's parameters from the document and returns the callable the behavior
tree ticks. Every leaf closes over the live ``Brain``:

* **actions** drive the SAME mission engine as the console and the
  ``elements`` playback (``Brain._execute_strategy`` -> ``execute_plan`` ->
  the executor), so a tree can never invent a new motion path;
* **conditions** only READ state (fused pose, executor, vision store,
  blackboard) and never block.

Actions::

    run_document(doc)        plan + execute a saved v1 document
    wait(seconds)            pause the tree (cancellable)
    log(message)             one line into the brain log
    add_score(points)        raise the blackboard score (referee/manual)
    abort_run(reason)        stop a running mission now

Conditions::

    at_pose(x, y, theta?, tol_mm, tol_deg)   within tolerance of a pose
    executor_idle()                          no mission segment running
    camera_fresh()                           the vision snapshot is in TTL
    time_left(seconds)                       match clock has time left
    score_at_least(points)                   blackboard score reached
    team_is(side)                            'yellow' / 'blue' side check

``time_left`` reads ``blackboard["deadline"]`` (a monotonic timestamp the
host sets from the document's ``match_s`` option); when no deadline is set
the condition is TRUE - a document without a clock has no deadline either.
``score_at_least`` reads ``blackboard["score"]`` (seeded from the
document's ``score`` option, raised by ``add_score``).

File:    software/brain/match/nodes.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-09
"""

from __future__ import annotations

import asyncio
import math
import time
from typing import Callable, Dict

from software.core.mission.bt import LeafFn, Status
from software.core.mission.executor import EXEC_DONE
from software.core.mission.strategy import Strategy


def _wrap180(angle: float) -> float:
    """Wrap an angle to [-180, 180) (project convention)."""
    return (angle + 180.0) % 360.0 - 180.0


def build_registry(brain) -> Dict[str, Callable[[Dict], LeafFn]]:
    """Build the action/condition registry around a live brain.

    Raises ``ValueError`` for malformed parameters at BUILD time (when a
    strategy document is loaded), never mid-match.
    """
    registry: Dict[str, Callable[[Dict], LeafFn]] = {}

    def register(name: str) -> Callable:
        """Register one leaf factory under ``name``."""
        def _decorate(factory: Callable[[Dict], LeafFn]) -> Callable[[Dict], LeafFn]:
            registry[name] = factory
            return factory

        return _decorate

    # ------------------------------------------------------------- actions
    @register("run_document")
    def _run_document(params: Dict) -> LeafFn:
        """Plan + execute a saved v1 document; SUCCESS when it completes."""
        doc = str(params.get("doc", "")).strip()
        if not doc:
            raise ValueError("run_document needs a 'doc' name")

        async def _fn(_ctx) -> Status:
            try:
                strategy = Strategy.load_named(brain.settings.strategy_dir, doc)
            except Exception as exc:
                await brain.log("error", f"run_document '{doc}': {exc}")
                return Status.FAILURE
            state = await brain._execute_strategy(strategy)
            return Status.SUCCESS if state == EXEC_DONE else Status.FAILURE

        return _fn

    @register("wait")
    def _wait(params: Dict) -> LeafFn:
        """Pause the tree for ``seconds`` (cancellable by Timeout)."""
        seconds = max(0.0, min(3600.0, float(params.get("seconds", 1.0))))

        async def _fn(_ctx) -> Status:
            await asyncio.sleep(seconds)
            return Status.SUCCESS

        return _fn

    @register("log")
    def _log(params: Dict) -> LeafFn:
        """Write one informational line into the brain log."""
        message = str(params.get("message", ""))

        async def _fn(_ctx) -> Status:
            await brain.log("info", message)
            return Status.SUCCESS

        return _fn

    @register("add_score")
    def _add_score(params: Dict) -> LeafFn:
        """Raise the blackboard score (manual/referee input today)."""
        points = float(params.get("points", 1.0))

        async def _fn(ctx) -> Status:
            ctx.blackboard["score"] = float(ctx.blackboard.get("score", 0.0)) + points
            await brain.log("info", f"score +{points:g} -> {ctx.blackboard['score']:g}")
            return Status.SUCCESS

        return _fn

    @register("abort_run")
    def _abort_run(params: Dict) -> LeafFn:
        """Stop a running mission (executor stop + clear) right now."""
        reason = str(params.get("reason", "tree abort"))

        async def _fn(_ctx) -> Status:
            executor = brain.ctx.executor
            if executor is not None and executor.is_running():
                await executor.stop(reason=reason)
                await brain.log("warn", f"tree stopped the run: {reason}")
            return Status.SUCCESS

        return _fn

    # ---------------------------------------------------------- conditions
    @register("at_pose")
    def _at_pose(params: Dict) -> LeafFn:
        """True when the fused pose is within tolerance of (x, y [, theta])."""
        if "x" not in params or "y" not in params:
            raise ValueError("at_pose needs 'x' and 'y'")
        x = float(params["x"])
        y = float(params["y"])
        theta = float(params["theta"]) if params.get("theta") is not None else None
        tol_mm = float(params.get("tol_mm", 50.0))
        tol_deg = float(params.get("tol_deg", 5.0))

        def _fn(_ctx) -> bool:
            pose = brain.pose()
            if pose is None:
                return False
            if math.hypot(pose[0] - x, pose[1] - y) > tol_mm:
                return False
            if theta is not None and abs(_wrap180(pose[2] - theta)) > tol_deg:
                return False
            return True

        return _fn

    @register("executor_idle")
    def _executor_idle(_params: Dict) -> LeafFn:
        """True when no mission segment is running."""

        def _fn(_ctx) -> bool:
            executor = brain.ctx.executor
            return executor is None or not executor.is_running()

        return _fn

    @register("camera_fresh")
    def _camera_fresh(_params: Dict) -> LeafFn:
        """True while the vision snapshot is inside its TTL."""

        def _fn(_ctx) -> bool:
            return bool(brain.ctx.vision.fresh)

        return _fn

    @register("time_left")
    def _time_left(params: Dict) -> LeafFn:
        """True while at least ``seconds`` remain on the match clock."""
        seconds = float(params.get("seconds", 10.0))

        def _fn(ctx) -> bool:
            deadline = ctx.blackboard.get("deadline")
            if deadline is None:
                return True  # no clock configured: no deadline either
            clock = ctx.blackboard.get("clock", time.monotonic)
            return (float(deadline) - float(clock())) >= seconds

        return _fn

    @register("score_at_least")
    def _score_at_least(params: Dict) -> LeafFn:
        """True when the blackboard score reached ``points``."""
        points = float(params.get("points", 1.0))

        def _fn(ctx) -> bool:
            return float(ctx.blackboard.get("score", 0.0)) >= points

        return _fn

    @register("team_is")
    def _team_is(params: Dict) -> LeafFn:
        """True when the configured team matches ``side``."""
        side = str(params.get("side", "")).strip().lower()
        if side not in ("yellow", "blue"):
            raise ValueError("team_is needs side 'yellow' or 'blue'")

        def _fn(_ctx) -> bool:
            return str(getattr(brain.settings, "team", "")).lower() == side

        return _fn

    return registry
