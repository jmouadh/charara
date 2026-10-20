"""Headless strategy runner: robot link + localization + mission engine.

``Brain`` is the console's ``AppState`` without the web layer. It satisfies
the mission context contract (:class:`software.core.mission.interfaces.
MissionContext`) through :class:`BrainContext`, so :func:`execute_plan`,
:func:`resume_from_wait` and the watcher loops run unchanged.

Lifecycle::

    brain = Brain(settings)
    await brain.start()                    # link, telemetry, watcher loops
    await brain.play(Strategy.load(...))   # plan from the live pose, execute
    await brain.stop()

While a strategy runs, :meth:`Brain.set_obstacles` feeds the obstacle map
(the camera / future companion-card input); a change near the route triggers
the same stop -> re-plan -> continue cycle as the console.

File:    software/brain/runner.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import asyncio
import time
from typing import Dict, List, Optional, Sequence

from software.brain.match import play_tree
from software.config import Settings
from software.core.mission import watcher
from software.core.mission.executor import EXEC_DONE, EXEC_IDLE, Executor
from software.core.mission.executor import needs_in_place_actions
from software.core.mission.orchestrator import execute_plan
from software.core.mission.strategy import KIND_BT, Strategy
from software.core.world.vision import VisionStore
from software.session import RobotSession


class BrainContext:
    """The mission context the core engine drives (console's AppState twin)."""

    def __init__(self, brain: "Brain") -> None:
        """Wire the context to its brain and create the shared mission state."""
        self.plan_lock = asyncio.Lock()
        self.executor: Optional[Executor] = None
        self.vision = VisionStore()
        self.field: Dict = dict(brain.field)
        self.last_plan: Optional[Dict] = None
        self.last_plan_request: Optional[Dict] = None
        self.vision_waiting: Optional[Dict] = None
        self.vision_replan_task: Optional[asyncio.Task] = None
        self.wait_deadline_task: Optional[asyncio.Task] = None
        self.skipped_waypoints: set = set()
        self._brain = brain

    def pose(self) -> Optional[Sequence[float]]:
        """Current fused pose, or None before the first odometry sample."""
        return self._brain.pose()

    def opponent_pose(self) -> Optional[Dict]:
        """Fresh opponent pose reported by the camera, if available."""
        return self._brain.session.poses.opponent()

    async def log(self, level: str, message: str) -> None:
        """Forward a log line to the brain's logger."""
        await self._brain.log(level, message)

    async def emit(self, event: Dict) -> None:
        """Forward an event to the brain's event sink."""
        await self._brain.on_event(event)


class Brain:
    """Plays saved strategies on the real (or mock) board, no web layer."""

    def __init__(self, config: Settings) -> None:
        """Prepare the runner: context, localization and field geometry."""
        self.settings = config
        self.field = {
            "width_mm": config.field_width_mm,
            "height_mm": config.field_height_mm,
        }
        self.ctx = BrainContext(self)
        # shared runtime session (Docs/09 section 5.1): link, client,
        # executor, localization and telemetry - the same class the console
        # uses, so both hosts behave identically in every sensor mode
        self.session = RobotSession(config)
        self.session.logger = self.log
        self.session.set_event_sink(self.ctx.emit)
        self.last_event: Optional[Dict] = None  # newest exec event (progress)
        self._sweep: Optional[asyncio.Task] = None
        self._proximity: Optional[asyncio.Task] = None

    # ------------------------------------------------- the shared session
    @property
    def client(self):
        """Active board client (None before :meth:`start`)."""
        return self.session.client

    @property
    def transport(self) -> str:
        """Active link mode: 'i2c' or 'mock'."""
        return self.session.transport

    @property
    def connected(self) -> bool:
        """True while the telemetry loop reads the board successfully."""
        return self.session.connected

    @property
    def last_error(self) -> str:
        """Message of the last telemetry/link failure ('' when healthy)."""
        return self.session.last_error

    # ------------------------------------------------------------ plumbing
    def pose(self) -> Optional[Sequence[float]]:
        """Fused pose (x_mm, y_mm, theta_deg), or None before telemetry."""
        if self.session.odometry is None:
            return None
        return self.session.localization.pose()

    async def log(self, level: str, message: str) -> None:
        """Print one timestamped log line."""
        stamp = time.strftime("%H:%M:%S")
        print(f"[{stamp}] {level.upper():<5} {message}", flush=True)

    async def on_event(self, event: Dict) -> None:
        """Event sink: keep the newest exec event; print log lines.

        The console shows executor log events (`run finished: N segment(s),
        M blended without stopping`, the closing-gap audit, warnings) in its
        log panel. The brain has no panel, so it PRINTS them: headless runs
        (`docker logs`, journald) must tell the same match story - the KPI
        lines are how a field session is measured afterwards.
        """
        if event.get("type") == "log":
            data = event.get("data") or {}
            level = str(data.get("level", "info"))
            message = str(data.get("message", ""))
            if message:
                await self.log(level, message)
            return
        if event.get("type") == "exec":
            self.last_event = event
            data = event.get("data") or {}
            if data.get("status") in ("done", "aborted"):
                await self.log("info" if data["status"] == "done" else "warn", f"execution {data['status']}")

    # ----------------------------------------------------------- lifecycle
    async def start(self) -> None:
        """Open the link, start telemetry, localization and watcher loops."""
        await self.session.start()
        self.ctx.executor = self.session.executor
        self._sweep = asyncio.create_task(watcher.sweep_loop(self.ctx))
        self._proximity = asyncio.create_task(watcher.proximity_loop(self.ctx))
        # wait (briefly) for the first odometry sample: planning needs a pose
        deadline = time.monotonic() + 3.0
        while self.pose() is None and time.monotonic() < deadline:
            await asyncio.sleep(0.05)
        await self.log("info", f"brain started (transport={self.transport})")

    async def stop(self) -> None:
        """Cancel the loops and close the link."""
        for task in (self._sweep, self._proximity):
            if task is not None:
                task.cancel()
                try:
                    await task
                except (asyncio.CancelledError, Exception):
                    pass
        self._sweep = self._proximity = None
        executor = self.ctx.executor
        if executor is not None and executor.is_running():
            await executor.stop(reason="brain shutting down")
        await self.session.stop()
        await self.log("info", "brain stopped")

    # -------------------------------------------------------------- play
    async def play(self, strategy: Strategy, repeat: bool = False) -> str:
        """Plan and execute ``strategy``; returns the final executor state.

        v1 documents (``kind: "elements"``, the default) run the plan /
        execute loop below; v2 documents (``kind: "bt"``) are played by the
        match layer (`software.brain.match`) with the SAME mission engine
        behind their leaves. ``repeat`` applies to v1 documents only - a
        tree decides its own repetition, that is the point of the tree.
        """
        if strategy.kind == KIND_BT:
            if repeat:
                await self.log("warn", f"strategy '{strategy.name}': repeat is ignored for a tree")
            await self.log("info", f"strategy '{strategy.name}': behavior tree")
            try:
                state = await play_tree(self, strategy)
            except ValueError as exc:
                await self.log("error", f"strategy '{strategy.name}' cannot be played: {exc}")
                return "failed"
            if state == EXEC_DONE:
                await self.log("info", f"strategy '{strategy.name}' finished")
            else:
                await self.log("warn", f"strategy '{strategy.name}' ended early: {state}")
            return state
        await self.log(
            "info",
            f"strategy '{strategy.name}': {len(strategy.elements)} element(s), "
            f"{len(strategy.obstacles)} obstacle(s)"
            + (", repeat" if repeat else ""),
        )
        while True:
            state = await self._execute_strategy(strategy)
            if state != EXEC_DONE or not repeat:
                return state
            await asyncio.sleep(1.0)

    async def _execute_strategy(self, strategy: Strategy) -> str:
        """One plan + execute pass of a v1 document.

        Returns :data:`EXEC_DONE` on success, an early executor state on a
        fault/abort, or ``"failed"`` when the document cannot be planned.
        Shared by :meth:`play` and the match layer's ``run_document`` leaf.
        """
        try:
            result = await execute_plan(
                self.ctx,
                strategy.elements,
                strategy.goal,
                strategy.obstacles,
                strategy.options,
            )
        except ValueError as exc:
            await self.log("error", f"strategy '{strategy.name}' cannot be planned: {exc}")
            return "failed"
        self.ctx.last_plan = result
        stats = result["stats"]
        await self.log(
            "info",
            f"plan ready: {stats['n_segments']} segments, "
            f"{stats['length_mm'] / 1000.0:.2f} m, ~{stats['est_s']:.0f} s",
        )
        executor = self.ctx.executor
        assert executor is not None
        if not executor.is_running():
            # a fresh start (a mid-run re-plan restarts itself). Track B:
            # with STREAMING_EXEC on, the plan's polyline drives the run with
            # the streaming follower; the executor REFUSES when the board
            # cannot stream - the strategy then fails loudly, never silently
            # falling back to the segment backend.
            follow = None
            if self.session.settings.streaming_exec_enabled:
                path = result.get("path") or []
                if len(path) >= 2 and not needs_in_place_actions(result["segments"]):
                    follow = path
                elif len(path) >= 2:
                    await self.log(
                        "warn",
                        "streaming skipped: this plan performs an in-place rotation, "
                        "pause or arrival turn - the segment backend drives it",
                    )
            try:
                await executor.start(result["segments"], None, follow_path=follow)
            except (RuntimeError, ValueError) as exc:
                await self.log("error", f"strategy '{strategy.name}' not started: {exc}")
                return "failed"
        await executor.wait()
        if executor.state not in (EXEC_DONE, EXEC_IDLE):
            await self.log("warn", f"strategy '{strategy.name}' ended early: {executor.state}")
            return executor.state
        await self.log("info", f"strategy '{strategy.name}' finished")
        return EXEC_DONE

    # --------------------------------------------------------- obstacles
    async def set_obstacles(self, obstacles: Sequence[Dict], ttl_ms: float = 2000.0) -> bool:
        """Report what the camera currently sees (replaces the snapshot)."""
        changed = self.ctx.vision.update(list(obstacles), ttl_ms, source="camera")
        if changed:
            await watcher.notify_change(self.ctx, "camera update")
        return changed

    async def set_robot_poses(
        self,
        us: Optional[Dict] = None,
        opponent: Optional[Dict] = None,
        ttl_ms: float = 400.0,
    ) -> bool:
        """Report what the camera sees for us and the opponent (headless).

        Same contract as the console ``/ws/vision`` poses message; returns
        True when the stored snapshot changed. The fusion consumes every
        message even when the snapshot repeats.
        """
        previous_opponent = self.session.poses.opponent()
        changed = await self.session.on_camera_poses(
            us,
            opponent,
            ttl_ms=ttl_ms,
            source="camera",
            client="brain",
            t_server_ms=time.time() * 1000.0,
        )
        if changed:
            watcher.notify_opponent_change(
                self.ctx, previous_opponent, self.session.poses.opponent()
            )
        return changed

    # ------------------------------------------------------------ status
    def status(self) -> Dict:
        """Small JSON-friendly status (used by logs and tests)."""
        data: Dict = {
            "connected": self.connected,
            "transport": self.transport,
            "error": self.last_error,
            "pose": list(self.pose()) if self.pose() is not None else None,
            "loc": self.session.localization.status(),
            "waiting": self.ctx.vision_waiting is not None,
        }
        executor = self.ctx.executor
        if executor is not None:
            data["exec"] = executor.progress
        return data
