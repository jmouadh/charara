"""Process-wide console runtime state: robot link, services, telemetry cache.

The module exposes a single ``state`` instance; the API routes, the mission
layer and the WebSocket hub all talk to it. ``AppState`` is the console's
implementation of the mission context (``software.core.mission.interfaces``):
plan_lock, executor, vision store, last plan/request, pose() and log().
The background telemetry loop feeds the localization service and polls the
board diagnostics.

File:    software/console/runtime/state.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import asyncio
import time
from typing import Awaitable, Callable, Dict, List, Optional, Sequence

from software.config import Settings, settings
from software.core.mission.game_time import (
    GameClock,
    TimeActionContext,
    dispatch_time_action,
)
from software.core.mission.teams import (
    TEAMS,
    TEAM_BLUE,
    TEAM_YELLOW,
    check_team,
    other_team,
    resolve_start_pose,
)
from software.core.world.map_config import MapConfig
from software.core.world.vision import VisionStore
from software.robot import protocol as p
from software.robot.client import RobotError
from software.robot.link import LinkError
from software.robot.models import Odometry, Status
from software.session import RobotSession

from .trace import TraceBuffer

EventSink = Callable[[Dict], Awaitable[None]]
#: A repeated board warning (same code + command, e.g. the chained-ARC queue
#: answered BUSY) is logged once per window; the swallowed repeats are counted
#: and reported with the next line instead of flooding the console log.
BOARD_EVENT_REPEAT_S = 3.0


def advance_event_cursor(cursor: int, events: Sequence[Dict], latest: int) -> int:
    """Cursor after one QUERY_EVENTS drain - never skips unread events.

    The board answers at most five events per query; when a burst produced
    more, the cursor must stop at the newest READ entry so the next poll
    continues where this one stopped (jumping to ``latest`` would drop the
    events in between). Only when nothing could be read - every newer entry
    was already overwritten in the board's 16-entry ring - does the cursor
    jump to ``latest``.
    """
    for event in events:
        index = int(event["index"])
        if index > cursor:
            cursor = index
    if not events and latest > cursor:
        cursor = latest
    return cursor


class AppState:
    """Process wide state (single board, single I2C bus)."""

    def __init__(self, config: Settings) -> None:
        """Create the process-wide state with the configured defaults."""
        self.settings = config
        self.strategy_dir = config.strategy_dir  # shared with the brain
        # The shared runtime session owns the link/client/executor/
        # localization/telemetry (Docs/09 section 5.1). The console layer
        # adds the web bits and keeps the historical attribute surface for
        # the API routes and the mission context through the properties
        # below (client, executor, localization, odometry, transport, ...).
        self.session = RobotSession(config)
        self.status: Optional[Status] = None
        self.last_plan: Optional[Dict] = None
        # Map document (table size, artwork, start zones). A saved document
        # WINS over the environment defaults: the table the operator
        # configured must be the one the console starts with.
        self.map_config = MapConfig(
            path=config.map_config_file,
            field_dir=config.field_dir,
            default_field={
                "width_mm": config.field_width_mm,
                "height_mm": config.field_height_mm,
            },
        )
        self.field: Dict = {
            "width_mm": self.map_config.data["field"]["width_mm"],
            "height_mm": self.map_config.data["field"]["height_mm"],
            "defaults": {
                "speed_mm_s": config.default_speed_mm_s,
                "accel_mm_s2": config.default_accel_mm_s2,
                "decel_mm_s2": config.default_decel_mm_s2,
                "lat_accel_mm_s2": config.default_lat_accel_mm_s2,
                "goto_step_mm": config.default_goto_step_mm,
                "step_mm": config.default_step_mm,
                "fit_tol_mm": config.default_fit_tol_mm,
                "inflate_mm": config.default_inflation_mm,
                "grid_mm": config.default_grid_mm,
                "robot_width_mm": getattr(config, "robot_width_mm", 300.0),
                "robot_height_mm": getattr(config, "robot_height_mm", 300.0),
                "obstacle_margin_mm": getattr(config, "obstacle_margin_mm", 150.0),
                "waypoint_margin_mm": getattr(config, "default_waypoint_margin_mm", 0.0),
            },
        }
        # Match clock + timed actions (Game tab -> persisted in the map doc).
        # The clock is runtime-only: a service restart always starts a NEW
        # match on the next Execute; the configuration survives in map.json.
        self.game_clock = GameClock()
        self.game_clock.configure(self.map_config.data.get("game") or {})
        self.event_sink: Optional[EventSink] = None
        self.trace = TraceBuffer()
        self.plan_lock = asyncio.Lock()  # one plan/re-plan at a time
        # wire the session to the console layer: traces, lifecycle logs, the
        # per-sample hook (board diagnostics + telemetry broadcast) and the
        # failure hook (the "link down" telemetry event)
        self.session.on_trace = self.trace_add
        self.session.logger = self.log
        self.session.on_sample = self._on_sample
        self.session.on_failure = self._on_sample_failure
        self.session.set_event_sink(self.emit)
        self.vision = VisionStore()      # camera obstacle reports (/ws/vision)
        # last successful console request: used to re-plan when the camera
        # reports a changed obstacle map while the robot is executing
        self.last_plan_request: Optional[Dict] = None
        self.vision_replan_task: Optional[asyncio.Task] = None
        # set while the robot waits for a MOVING obstacle to clear: the run is
        # stopped and restarted by the vision watcher when the route is free
        self.vision_waiting: Optional[Dict] = None
        # bounded-wait timer: a wait never lasts longer than WAIT_MAX_S - the
        # timer resolves it (reach by another path, skip the checkpoint,
        # accept the detour) so a stop is never forever
        self.wait_deadline_task: Optional[asyncio.Task] = None
        # waypoints the bounded wait gave up on (still blocked, unreachable);
        # they count as done for the mission cursor until a fresh plan resets
        self.skipped_waypoints: set = set()
        # board error/event log + status (polled at ~1 Hz off the telemetry
        # loop); the console shows them in the diagnostics panels
        self.board_events: List[Dict] = []
        self.board_status: Optional[Status] = None
        self._board_event_index = 0
        #: last log time / swallowed repeats per board event (code, aux)
        self._board_event_log: Dict[tuple, tuple] = {}
        self._board_poll_divider = 0
        # Eurobot: which side we play + both match-start poses. A side's start
        # is configured (TEAM_START_YELLOW / TEAM_START_BLUE); when the blue
        # start is not set it MIRRORS the yellow one about the vertical centre
        # line of the (symmetric) table.
        try:
            self.team = check_team(config.team)
        except ValueError:
            self.team = TEAM_YELLOW
        self.team_labels: Dict[str, str] = {
            TEAMS[0]: config.team_label_yellow,
            TEAMS[1]: config.team_label_blue,
        }
        self._start_poses: Dict[str, Optional[Sequence[float]]] = {
            TEAMS[0]: tuple(config.team_start_yellow) if config.team_start_yellow else None,
            TEAMS[1]: tuple(config.team_start_blue) if config.team_start_blue else None,
        }
        self.team_starts: Dict[str, Sequence[float]] = {}
        self.resolve_team_starts()
        # mock "Redo" rehearsal: where the robot stood when Execute was pressed
        # (the redo teleports back there), and - while a redo is armed - the
        # pose to put the robot back to if the step is SAVED instead of
        # re-executed (the strategy continues from the run's end)
        self.last_run_start: Optional[Sequence[float]] = None
        self.redo_restore_pose: Optional[Sequence[float]] = None

    # ------------------------------------------------- the shared session
    @property
    def client(self):
        """Active board client (None before the first transport switch)."""
        return self.session.client

    @property
    def executor(self):
        """Active segment executor (None before the first transport switch)."""
        return self.session.executor

    @property
    def localization(self):
        """The fused-pose service owned by the session."""
        return self.session.localization

    @property
    def odometry(self):
        """Newest board odometry sample (None before the first read)."""
        return self.session.odometry

    @property
    def connected(self) -> bool:
        """True while the telemetry loop reads the board successfully."""
        return self.session.connected

    @property
    def last_error(self) -> str:
        """Message of the last telemetry/link failure ('' when healthy)."""
        return self.session.last_error

    @property
    def transport(self) -> str:
        """Active link mode: 'i2c' or 'mock'."""
        return self.session.transport

    # ----------------------------------------------------------- plumbing
    def resolve_team_starts(self) -> Dict[str, Sequence[float]]:
        """Both teams' match-start poses (blue mirrors yellow when unset).

        A PRIMARY start zone carrying a robot spawn pose OWNS the start of
        its team: the map configuration is the source of truth, the
        ``TEAM_START_*`` environment values are only the fallback. Re-run
        when the field size or the map document changes (the mirrored
        default follows the vertical centre line of the CURRENT table).
        """
        yellow = self.map_config.spawn_for(TEAM_YELLOW) or self._start_poses[TEAMS[0]]
        blue = self.map_config.spawn_for(TEAM_BLUE) or self._start_poses[TEAMS[1]]
        width = self.field["width_mm"]
        self.team_starts = {
            TEAMS[0]: resolve_start_pose(TEAM_YELLOW, yellow, blue, width),
            TEAMS[1]: resolve_start_pose(TEAM_BLUE, yellow, blue, width),
        }
        return self.team_starts

    def apply_map(self, snapshot: Optional[Dict] = None) -> Dict:
        """Adopt a map document: table size + start zones (spawn poses)."""
        data = snapshot if snapshot is not None else self.map_config.snapshot()
        self.field["width_mm"] = float(data["field"]["width_mm"])
        self.field["height_mm"] = float(data["field"]["height_mm"])
        self.resolve_team_starts()
        # Game tab changes apply live: the running clock keeps its start time
        # but adopts a new duration / warning / action list.
        self.game_clock.configure(data.get("game") or {})
        return data

    def map_info(self) -> Dict:
        """The map document plus the background artwork available on disk."""
        return {**self.map_config.snapshot(), "backgrounds": self.map_config.backgrounds()}

    # ------------------------------------------------------------ game clock
    def game_status(self) -> Dict:
        """Runtime match-clock state (config, elapsed/remaining, fired actions)."""
        status = self.game_clock.status()
        status["mock_override"] = bool(status["expired"] and self.transport == "mock")
        status["transport"] = self.transport
        return status

    def game_allows_motion(self) -> bool:
        """May a MISSION start? REAL stops at the limit; MOCK keeps running.

        Manual tool commands are deliberately still available: an operator
        can always recover the robot, and Reset clears the game state.
        """
        return self.transport == "mock" or not self.game_clock.expired

    async def game_start(self) -> Dict:
        """Start the match clock (idempotent while it is already running)."""
        started = self.game_clock.start()
        if started:
            await self.log(
                "info",
                f"game clock started: {self.game_clock.duration_s:.0f} s match, "
                f"red warning in the last {self.game_clock.warning_s:.0f} s",
            )
        elif not self.game_clock.configured:
            await self.log(
                "warn",
                "game clock not started: set a match duration in Game > Game time "
                "(0 disables the clock)",
            )
        status = self.game_status()
        await self.emit({"type": "game", "data": status})
        return status

    async def game_start_if_configured(self) -> None:
        """Start the clock on Execute only when it is configured and idle.

        A clock frozen by Pause is CONTINUED instead: a new run means the
        match time runs again (the operator started driving).
        """
        if not self.game_clock.configured:
            return
        if self.game_clock.paused:
            await self.game_resume()
        elif not self.game_clock.running:
            await self.game_start()

    async def game_pause(self) -> Dict:
        """Freeze the match clock (the console Pause button drives this)."""
        if self.game_clock.pause():
            await self.log(
                "info", "game clock paused - Resume continues the match time"
            )
        status = self.game_status()
        await self.emit({"type": "game", "data": status})
        return status

    async def game_resume(self) -> Dict:
        """Continue a paused match clock (the console Resume button)."""
        if self.game_clock.resume():
            await self.log("info", "game clock resumed")
        status = self.game_status()
        await self.emit({"type": "game", "data": status})
        return status

    async def game_reset(self) -> Dict:
        """Rewind the match clock and clear the fired-action history."""
        self.game_clock.reset()
        await self.log("info", "game clock reset - the next Execute starts a new match")
        status = self.game_status()
        await self.emit({"type": "game", "data": status})
        return status

    async def game_tick(self) -> None:
        """Dispatch due timed actions and enforce the REAL match limit."""
        for action in self.game_clock.due_actions():
            self.game_clock.mark_fired(action.get("id"))
            await dispatch_time_action(
                action,
                TimeActionContext(log=self.log, emit=self.emit, state=self),
            )
        if not self.game_clock.expire_if_due():
            return
        status = self.game_status()
        if self.transport == "mock":
            await self.log(
                "warn",
                "GAME TIME LIMIT REACHED - MOCK mode keeps running; the game "
                "timer shows the overtime warning (REAL mode stops the robot now)",
            )
        else:
            self.vision_waiting = None  # a timeout stop is final for this match
            executor = self.executor
            if executor is not None and executor.is_running():
                await executor.stop(reason="game time limit reached")
            elif self.client is not None:
                try:
                    await self.client.stop()
                except Exception:
                    pass
            await self.log(
                "error",
                f"GAME TIME LIMIT REACHED ({self.game_clock.duration_s:.0f} s) - "
                "robot stopped immediately (REAL mode); reset the game clock to run again",
            )
        await self.emit({"type": "game", "data": status})

    def start_source(self, team: str) -> str:
        """Where a team's match start comes from: ``zone`` or ``settings``.

        A PRIMARY start zone with a robot spawn owns its team's start; the
        ``TEAM_START_*`` environment values are only the fallback.
        """
        return "zone" if self.map_config.spawn_for(team) else "settings"

    def configured_start_zones(self) -> List[str]:
        """Teams whose match start comes from a primary starting zone."""
        return [side for side in TEAMS if self.map_config.spawn_for(side)]

    async def adopt_team_start(
        self,
        previous: Optional[Sequence[float]] = None,
        reason: str = "",
        force: bool = False,
        what: str = "starting zone saved",
    ) -> bool:
        """Put the robot's pose at the (new) team match start after a map change.

        The start zone IS the match start, so creating/editing a primary zone
        must place the robot frame there. In MOCK the simulator is teleported,
        exactly like a team switch. On the REAL robot the console issues
        SET_POSE (never a motion command): the robot must physically stand at
        the dedicated team start for the match, so its odometry frame is
        aligned with that point instead of keeping a stale pose. A running
        mission is never disturbed; the new start applies from the next run.
        ``force`` places the robot even when the numeric start did not move
        (saving a zone is the operator saying "start here"). Returns True
        when the pose was accepted.
        """
        start = self.team_starts[self.team]
        if not force and previous is not None and all(
            abs(float(a) - float(b)) < 0.5 for a, b in zip(previous, start)
        ):
            return False  # the start did not move: keep the robot where it is
        label = self.team_labels[self.team]
        if self.executor is not None and self.executor.is_running():
            await self.log(
                "warn",
                f"{what} - the robot keeps driving; the new match start "
                "applies from the next run",
            )
            return False
        if self.client is None:
            return False
        try:
            board = await self.client.odometry()
            if not getattr(board, "is_idle", True):
                await self.log(
                    "warn",
                    f"{what} - the robot is still moving; the match start will "
                    "be applied when it is idle (press Reset robot to start)",
                )
                return False
        except Exception:
            pass  # a client double may not expose odometry: SET_POSE still works
        try:
            await self.client.set_pose(float(start[0]), float(start[1]), float(start[2]))
        except Exception as exc:  # a broken link must not fail the map save
            await self.log(
                "warn", f"{what} but the robot pose could not be set: {exc}"
            )
            return False
        self.redo_restore_pose = None  # the start moved: a pending redo is void
        if self.transport == "mock":
            await self.log(
                "info",
                f"{what} - mock robot placed at the {label} match start "
                f"({start[0]:.0f}, {start[1]:.0f}) mm, {start[2]:.0f} deg",
            )
        else:
            await self.log(
                "info",
                f"{what} - REAL robot pose set to the {label} match start "
                f"({start[0]:.0f}, {start[1]:.0f}) mm, {start[2]:.0f} deg "
                f"({reason or 'start zones updated'})",
            )
        return True

    def team_info(self) -> Dict:
        """JSON-friendly team status: who we are + where both robots start."""
        current = self.team
        opponent = other_team(current)
        return {
            "current": current,
            "label": self.team_labels[current],
            "start": [round(float(v), 1) for v in self.team_starts[current]],
            "opponent": {
                "team": opponent,
                "label": self.team_labels[opponent],
                "start": [round(float(v), 1) for v in self.team_starts[opponent]],
            },
            "teams": {
                side: {
                    "label": self.team_labels[side],
                    "start": [round(float(v), 1) for v in self.team_starts[side]],
                }
                for side in TEAMS
            },
        }

    def redo_info(self) -> Dict:
        """Mock rehearsal status for the Redo button.

        ``available``: the robot can be teleported back to the last run's
        start (mock only - the real robot is never moved by the console).
        ``stashed``: a redo is armed; saving the strategy restores the pose
        the robot had BEFORE the redo (the run's end), so a saved step keeps
        the mission continuing from there.
        """
        executor = self.executor
        available = (
            self.transport == "mock"
            and self.odometry is not None
            and self.last_run_start is not None
            and executor is not None
            # state-based on purpose: right after the final "done" event the
            # runner task may still be unwinding for a moment (is_running()
            # true) - the rehearsal cue only needs "no ACTIVE run"
            and not (executor.is_running() and executor.state in ("running", "paused"))
        )
        return {
            "available": available,
            "stashed": self.redo_restore_pose is not None,
            "start": (
                [round(float(v), 1) for v in self.last_run_start]
                if self.last_run_start
                else None
            ),
        }

    async def switch_transport(self, mode: str) -> str:
        """Rebuild the robot link (and executor) for ``i2c`` or ``mock``.

        Delegates to the shared session (same fail-safe order: the new link
        is built first and the current transport keeps running when it
        cannot be opened - no smbus2, no bus, no board), then resets the
        console-specific caches. Used by the mode selector and at startup.
        """
        await self.session.switch_transport(mode)
        self.status = None
        self.board_events = []
        self.board_status = None
        self._board_event_index = 0
        self.last_run_start = None  # another link: no meaningful rehearsal pose
        self.redo_restore_pose = None
        # A fresh link adopts the CURRENT team's match start: MOCK teleports,
        # REAL issues SET_POSE (the robot must physically stand in its
        # dedicated start zone for the match). This is what makes the console
        # show the robot in its starting position instead of a stale pose.
        await self.adopt_team_start(
            force=True, reason=f"{mode} link ready", what="link ready"
        )
        return mode

    async def emit(self, event: Dict) -> None:
        """Publish one event to the event sink (the console websocket)."""
        if self.event_sink is None:
            return
        try:
            await self.event_sink(event)
        except Exception:
            pass

    async def log(self, level: str, message: str) -> None:
        """Publish one log line to the event sink."""
        await self.emit({"type": "log", "data": {"level": level, "message": message}})

    # --------------------------------------------------------------- trace
    def trace_add(self, entry: Dict) -> None:
        """Called (synchronously) by RobotClient for every transaction."""
        stored = self.trace.add(entry)
        try:
            asyncio.get_running_loop().create_task(self.emit({"type": "trace", "data": stored}))
        except RuntimeError:
            pass  # no event loop (unit tests) - the buffer still records it

    # ----------------------------------------------------------- telemetry
    # ----------------------------------------------------- session hooks
    def start_telemetry(self) -> None:
        """Start the session telemetry loop (console startup/recovery path)."""
        self.session.start_telemetry()

    async def stop_telemetry(self) -> None:
        """Stop the session telemetry loop and wait for it to finish."""
        await self.session.stop_telemetry()

    async def _on_sample(self, odometry: Odometry) -> None:
        """Per-sample console work: board diagnostics + telemetry broadcast.

        Runs after every successful odometry read - including the reads the
        executor makes while it supervises a segment - so the console UI is
        as fresh as the old inline telemetry loop used to be.
        """
        period = self.settings.telemetry_period_s
        board_period_ticks = max(1, int(round(1.0 / max(period, 0.01))))
        # board diagnostics ~1 Hz: event log + firmware status
        self._board_poll_divider += 1
        if self._board_poll_divider >= board_period_ticks:
            self._board_poll_divider = 0
            await self._poll_board_diagnostics()
        payload = self.snapshot()
        # the vision source travels with the telemetry so the console can
        # yield to a real camera stream the moment it appears
        payload["vision"] = self.vision.status()
        await self.emit({"type": "telemetry", "data": payload})

    async def _on_sample_failure(self, message: str) -> None:
        """Online -> offline transition: tell the consoles the link is down."""
        await self.emit({"type": "telemetry", "data": {"connected": False, "error": message}})

    async def _poll_board_diagnostics(self) -> None:
        """Read the board error/event log and status; surface new events.

        The full list is kept for the console panel; warn/error level events
        are also written to the console log so a failure is never silent.
        """
        assert self.client is not None
        try:
            self.board_status = await self.client.status()
        except (RobotError, LinkError):
            self.board_status = None
        try:
            result = await self.client.events(self._board_event_index)
        except (RobotError, LinkError):
            return
        for event in result["events"]:
            code = int(event["code"])
            entry = {
                "index": event["index"],
                "code": code,
                "aux": event["aux"],
                "name": p.event_name(code),
                "severity": p.event_severity(code),
            }
            self.board_events.append(entry)
            if len(self.board_events) > 40:
                del self.board_events[0]
            if entry["severity"] in ("warn", "error"):
                message = f"board: {entry['name']} (aux {entry['aux']})"
                if code in (
                    p.EVENT_UNKNOWN_CMD,
                    p.EVENT_BAD_PARAM,
                    p.EVENT_BUSY_REJECT,
                    p.EVENT_BAD_LEN,
                ):
                    # the aux byte is the offending command: name it, so an old
                    # firmware answering "UNKNOWN_CMD (aux 23 = QUEUE_ARC)" is
                    # recognised at a glance
                    command = p.CMD_NAMES.get(entry["aux"])
                    if command:
                        message = f"board: {entry['name']} (aux {entry['aux']} = {command})"
                # the same rejection can repeat many times per second (the
                # chained-ARC queue retried against a busy slot): log it once,
                # then summarise how many repeats were swallowed
                key = (code, entry["aux"])
                now = time.monotonic()
                last, repeats = self._board_event_log.get(key, (0.0, 0))
                if now - last < BOARD_EVENT_REPEAT_S:
                    self._board_event_log[key] = (last, repeats + 1)
                else:
                    if repeats:
                        message = f"{message} (+{repeats} more ignored)"
                    self._board_event_log[key] = (now, 0)
                    await self.log(entry["severity"], message)
        self._board_event_index = advance_event_cursor(
            self._board_event_index, result["events"], int(result.get("latest", 0))
        )

    # ------------------------------------------------------------ snapshot
    def pose(self) -> Optional[Sequence[float]]:
        """The pose consumers should use: the localized estimate, or None.

        Wheel odometry (and, later, the IMU) fused by the localization
        service - not the raw board dead reckoning, so upgrading the
        estimator never reaches into the mission code. ``None`` until the
        first odometry sample arrived (mission context contract).
        """
        if self.odometry is None:
            return None
        return self.localization.pose()

    def opponent_pose(self) -> Optional[Dict]:
        """Fresh opponent pose reported by the camera, if available."""
        return self.session.poses.opponent()

    def snapshot(self) -> Dict:
        """JSON-friendly snapshot: pose, board state, exec, vision, waiting."""
        data: Dict = {
            "connected": self.connected,
            "error": self.last_error,
            "transport": self.transport,
            "team": self.team_info(),
            "redo": self.redo_info(),
            "game": self.game_status(),
            "poses": self.session.poses.status(),
            "imu": self.session.imu_status(),
            "gyro_stream": self.session.gyro_stream_status(),
            "reanchor": self.session.reanchor_status(),
        }
        odometry = self.odometry
        if odometry is not None:
            x_mm, y_mm, theta_deg = self.localization.pose()
            data.update(
                {
                    "x_mm": x_mm,
                    "y_mm": y_mm,
                    "theta_deg": theta_deg,
                    "v_left_mm_s": odometry.v_left_mm_s,
                    "v_right_mm_s": odometry.v_right_mm_s,
                    "state": odometry.state,
                    "state_name": odometry.state_name,
                    "faults": odometry.faults,
                    "fault_names": list(p.decode_faults(odometry.faults)),
                    # the raw board dead reckoning stays visible for diagnostics
                    "board_odometry": {
                        "x_mm": odometry.x_mm,
                        "y_mm": odometry.y_mm,
                        "theta_deg": odometry.theta_deg,
                    },
                }
            )
            data["loc"] = self.localization.status()
        if self.board_events:
            data["board_events"] = list(self.board_events[-12:])
        if self.board_status is not None:
            status = self.board_status
            data["board"] = {
                "firmware": f"{status.fw_major}.{status.fw_minor}",
                "uptime_s": round(status.uptime_ms / 1000.0, 1),
                "faults": status.faults,
                "fault_names": list(p.decode_faults(status.faults)),
                "last_event": status.last_event_name,
                "reset_cause": status.reset_cause_name,
                "cfg_map_version": status.cfg_map_version,
                "cfg_dirty": status.cfg_dirty,
                "queue_depth": status.queue_depth,
                "gyro_fresh": status.gyro_fresh,
            }
        if self.executor is not None:
            data["exec"] = self.executor.progress
        if self.vision_waiting is not None:
            data["waiting"] = {
                "reason": self.vision_waiting.get("reason", ""),
                "paused": bool(self.vision_waiting.get("paused_by_user")),
            }
        return data


state = AppState(settings)
