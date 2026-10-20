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
from software.core.mission.teams import (
    TEAMS,
    TEAM_BLUE,
    TEAM_YELLOW,
    check_team,
    other_team,
    resolve_start_pose,
)
from software.core.world.vision import VisionStore
from software.robot import protocol as p
from software.robot.client import RobotError
from software.robot.link import LinkError
from software.robot.models import Odometry, Status
from software.session import RobotSession

from .trace import TraceBuffer

EventSink = Callable[[Dict], Awaitable[None]]


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
        self.field: Dict = {
            "width_mm": config.field_width_mm,
            "height_mm": config.field_height_mm,
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

        Re-run when the field size changes: the mirrored default follows the
        vertical centre line of the CURRENT table.
        """
        yellow = self._start_poses[TEAMS[0]]
        blue = self._start_poses[TEAMS[1]]
        width = self.field["width_mm"]
        self.team_starts = {
            TEAMS[0]: resolve_start_pose(TEAM_YELLOW, yellow, blue, width),
            TEAMS[1]: resolve_start_pose(TEAM_BLUE, yellow, blue, width),
        }
        return self.team_starts

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
        if mode == "mock":
            # a fresh simulator starts at the CURRENT team's match-start pose,
            # so rehearsing the other side begins from the right corner
            start = self.team_starts[self.team]
            try:
                assert self.client is not None
                await self.client.set_pose(float(start[0]), float(start[1]), float(start[2]))
            except Exception:
                pass
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
