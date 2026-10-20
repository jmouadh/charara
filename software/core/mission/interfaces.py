"""Ports the mission layer talks to - no board imports, pure typing.

The executor and the orchestrator drive a ROBOT PORT: any object with the
async methods below. ``software.robot.client.RobotClient`` implements it for
the real (or mock) board; tests implement it with a few lines. Errors raised
by the port must derive from :class:`RobotIOError` so the mission layer can
tell "robot/link trouble" from programming errors without importing the
board package.

File:    software/core/mission/interfaces.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import asyncio
from typing import Any, Awaitable, Callable, Dict, Optional, Protocol, Tuple, runtime_checkable


class RobotIOError(Exception):
    """Base class of robot port errors (link down, board rejected, ...)."""


@runtime_checkable
class RobotIO(Protocol):
    """The narrow robot port the mission layer needs.

    ``odometry()`` returns an object with ``x_mm``, ``y_mm``, ``theta_deg``,
    ``state``, ``faults``, ``is_idle`` and ``describe_faults()`` (see
    ``software.robot.models.Odometry``).
    """

    async def odometry(self) -> Any:
        """Read the current board odometry (pose, speeds, state, faults)."""
        ...

    async def set_deadman(self, timeout_ms: int) -> None:
        """Arm or clear the board command watchdog [ms, 0 = off]."""
        ...

    async def stop(self) -> None:
        """Controlled stop: pulses off, odometry and torque preserved."""
        ...

    async def estop(self) -> None:
        """Emergency stop: stop, disable the drivers, latch the e-stop fault."""
        ...

    async def release_motors(self) -> None:
        """Release the stepper drivers without latching a fault (idle only)."""
        ...

    async def goto(self, x_mm: float, y_mm: float, theta_deg: float, speed_mm_s: int) -> None:
        """Absolute move to (x, y, theta) at ``speed_mm_s``."""
        ...

    async def drive(self, distance_mm: float, speed_mm_s: int) -> None:
        """Straight move: signed distance along the current heading."""
        ...

    async def turn(self, angle_deg: float, speed_mm_s: int) -> None:
        """In-place rotation: signed angle (CCW positive)."""
        ...

    async def strafe(self, distance_mm: float, speed_mm_s: int) -> None:
        """Lateral move (dead-reckoned by the board)."""
        ...

    async def arc(self, radius_mm: float, sweep_deg: float, speed_mm_s: int) -> None:
        """Circular arc: signed radius and sweep (signs must agree)."""
        ...

    async def queue_arc(self, radius_mm: float, sweep_deg: float, speed_mm_s: int) -> None:
        """Queue a continuation ARC, blended without stopping (see client)."""
        ...

    async def stream_velocity(self, v_mm_s: float, omega_deg_s: float) -> None:
        """Silent velocity stream frame: latest-wins, no response (fw >= 3.4)."""
        ...

    async def display_text(self, line: int, text: str, offset: int = 0) -> None:
        """Write up to 9 characters on one OLED line at an offset."""
        ...


# event sink: a JSON-friendly event goes to the console / brain log
EventSink = Callable[[Dict], Awaitable[None]]
LogSink = Callable[[str, str], Awaitable[None]]


@runtime_checkable
class MissionContext(Protocol):
    """What the mission layer needs from its host process.

    The console's ``AppState`` and the brain's runner satisfy this
    structurally (no inheritance); a test double needs a handful of lines.

    ``pose()`` is the ESTIMATED pose in (x_mm, y_mm, theta_deg) - the
    localization service's output - or ``None`` before the first odometry
    sample ever arrived.
    """

    plan_lock: asyncio.Lock                      # one plan / re-plan at a time
    executor: Any                                # core.mission.executor.Executor
    vision: Any                                  # core.world.vision.VisionStore
    field: Dict                                  # {"width_mm": ..., "height_mm": ...}
    last_plan: Optional[Dict]
    last_plan_request: Optional[Dict]
    vision_waiting: Optional[Dict]
    vision_replan_task: Optional[asyncio.Task]
    wait_deadline_task: Optional[asyncio.Task]
    skipped_waypoints: set

    def pose(self) -> Optional[Tuple[float, float, float]]:
        """The estimated pose (x_mm, y_mm, theta_deg), or None before odometry."""
        ...
    def opponent_pose(self) -> Optional[Dict]:
        """The fresh camera pose for the opponent, or None when unavailable."""
        ...
    async def log(self, level: str, message: str) -> None:
        """Write one log line through the host (console or brain)."""
        ...

    async def emit(self, event: Dict) -> None:
        """Publish one event through the host (websocket or brain sink)."""
        ...
