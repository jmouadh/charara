"""Segment executor: runs a planned node list on the board with supervision.

One segment at a time (the firmware rejects concurrent motion with BUSY):
send the command, poll odometry until the state returns to IDLE, watch faults
and timeouts, and publish progress. Consecutive ARCs are pipelined with
``QUEUE_ARC`` (0x17): the board blends them without stopping - a tangent arc
chain runs with a single acceleration and a single deceleration - and its
completed-segment counter keeps exactly one arc queued ahead. PAUSE stops the
current segment with ``STOP`` and waits; RESUME continues with the next node
(all nodes are absolute, so continuing from wherever the robot stopped is
safe).

Streaming backend (Track B, optional): when the operator enables
``STREAMING_EXEC`` and the caller supplies the plan's polyline,
``start(..., follow_path=...)`` drives the whole path with continuous
velocity frames through :class:`~.follower.PathFollower` (Pure Pursuit +
``SET_VELOCITY``) - one acceleration, one deceleration, no stop at the
junctions. The start is REFUSED when the board does not accept the stream
(firmware below 3.4, gate off or no link): a requested stream never
degrades silently into the segment backend, or the field A/B comparison
would lie. Deadman, stale-fault abort, the caution speed scale and the
event stream work identically in both backends.

Dynamic obstacles: the tail of the current run can be rebuilt as planner
elements (:func:`remaining_elements`) so the API can stop the robot, re-plan
only what is left of the mission around the new obstacle and restart the
execution - paused runs stay paused.

File:    software/core/mission/executor.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-12
"""

from __future__ import annotations

import asyncio
import math
import time
from typing import Awaitable, Callable, Dict, List, Optional, Sequence, Tuple

from .follower import PathFollower
from .interfaces import EventSink, RobotIO, RobotIOError
from ..navigation.pursuit import path_length
from ...robot.protocol import FAULT_DEADMAN, ST_BUSY

EXEC_IDLE = "idle"
EXEC_RUNNING = "running"
EXEC_PAUSED = "paused"
EXEC_DONE = "done"
EXEC_ABORTED = "aborted"

#: lowest multiplier the caution reaction may ask for (200 mm/s -> 40 mm/s)
CAUTION_MIN_SCALE = 0.2
#: no motion command is ever sent slower than this (the board's own floor)
CAUTION_MIN_SPEED_MM_S = 30
#: a run that ends farther than this from the planned end gets one small
#: closed-loop correction GOTO (wheel slip / a cut arc corner used to leave
#: the robot 30-40 mm short of the drawn waypoint)
FINAL_CORRECTION_MIN_MM = 12.0
FINAL_CORRECTION_MIN_DEG = 4.0
FINAL_CORRECTION_SPEED_MM_S = 80


def remaining_drive_mm(
    segment: Dict, position: Optional[Tuple[float, float]]
) -> float:
    """Signed distance still to drive on the current DRIVE segment.

    Used when a close-waypoint reverse departure is interrupted by a
    dynamic-obstacle re-plan: the replacement route must finish the reverse
    to the turn-safe point before any rotation, or the body sweeps into the
    wall the waypoint was placed next to.
    """
    if position is None:
        return 0.0
    signed = float(segment.get("distance_mm", 0.0))
    points = [
        (float(point[0]), float(point[1]))
        for point in segment.get("points") or []
        if len(point) >= 2
    ]
    if abs(signed) < 0.5 or len(points) < 2:
        return 0.0
    total = 0.0
    along_total = 0.0
    px, py = float(position[0]), float(position[1])
    best: Optional[Tuple[float, float]] = None  # (distance to leg, along)
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            continue
        t = ((px - ax) * dx + (py - ay) * dy) / (length * length)
        t = max(0.0, min(1.0, t))
        qx, qy = ax + dx * t, ay + dy * t
        gap = math.hypot(px - qx, py - qy)
        if best is None or gap < best[0]:
            best = (gap, along_total + length * t)
        along_total += length
        total += length
    if best is None or total <= 1e-6:
        return 0.0
    fraction = max(0.0, min(1.0, best[1] / total))
    remaining = signed * (1.0 - fraction)
    return 0.0 if abs(remaining) < 1.0 else remaining


def remaining_elements(
    segments: Sequence[Dict],
    index: int,
    position: Optional[Tuple[float, float]] = None,
) -> List[Dict]:
    """Rebuild planner elements from the not-yet-executed tail.

    Used by the dynamic-obstacle re-plan: what is left of the mission is the
    part of the path the robot has not driven yet, then every rotation, pause
    or pass-through point still ahead. Consecutive GOTO nodes and ARC spans
    become a stroke (re-fitted and re-routed like a freshly drawn line); a
    lone node becomes a waypoint so the target is not lost.

    The segment being executed right now contributes only its TARGET: a GOTO
    contributes its node, and a half-driven ARC contributes its exit point
    (its start lies behind the robot - routing back to it would send the
    robot backwards into its own wake). A half-driven close-waypoint
    DEPARTURE is the exception: its remaining signed DRIVE is kept, because
    the reverse must reach the turn-safe point before the robot rotates.
    """
    elements: List[Dict] = []
    chain: List[List[float]] = []

    def flush() -> None:
        """Close the current chain into a stroke or a single waypoint."""
        nonlocal chain
        if len(chain) >= 2:
            elements.append({"kind": "stroke", "points": chain})
        elif len(chain) == 1:
            elements.append({"kind": "waypoint", "x": chain[0][0], "y": chain[0][1]})
        chain = []

    for offset, segment in enumerate(segments[index:]):
        kind = segment.get("kind", "goto")
        if kind == "goto":
            if "x" in segment and "y" in segment:
                chain.append([float(segment["x"]), float(segment["y"])])
            continue
        if kind == "arc":
            # re-fit the arc from its sampled preview points together with the
            # surrounding chain, so the re-planned tail keeps the curvature
            points = segment.get("points") or []
            exit_point = None
            if "x1" in segment and "y1" in segment:
                exit_point = [float(segment["x1"]), float(segment["y1"])]
            if offset == 0 and exit_point is not None:
                # the arc is running: only its exit is still ahead
                chain.append(exit_point)
            elif points:
                chain.extend([[float(x), float(y)] for x, y in points])
            elif exit_point is not None:
                chain.append(exit_point)
            # a segment without any geometry contributes nothing - never a
            # fabricated (0, 0) that would route the robot to the field corner
            continue
        flush()
        if kind == "turn":
            elements.append({"kind": "rotate", "angle_deg": float(segment.get("angle_deg", 0.0))})
        elif kind == "pause":
            elements.append({"kind": "pause", "seconds": float(segment.get("seconds", 1.0))})
        elif kind == "release":
            elements.append({"kind": "release"})
        elif kind == "drive":
            # a departure drive still AHEAD keeps its exact signed distance;
            # the one being executed keeps only what is left of it (a re-plan
            # that interrupts the reverse must not turn before its safe end)
            distance = float(segment.get("distance_mm", 0.0))
            if offset == 0:
                distance = (
                    remaining_drive_mm(segment, position)
                    if segment.get("departure") else 0.0
                )
            if abs(distance) >= 1.0:
                elements.append(
                    {"kind": "drive", "distance_mm": round(distance, 1)}
                )
        # strafe is a manual move, not mission geometry: skipped
    flush()
    return elements


def follow_cruise_mm_s(segments: Sequence[Dict]) -> float:
    """Cruise speed for a streaming run: the median of the planned speeds.

    The follower's speed law slows for curves and the path end by itself,
    so one cruise value is enough. The median keeps a single slow segment
    (a docking move) from dragging the whole mission down, and one raised
    speed from pushing every straight to the limit.
    """
    speeds = sorted(
        float(segment.get("v", 0.0))
        for segment in segments
        if segment.get("kind", "goto") in ("goto", "arc", "drive", "strafe")
        and float(segment.get("v", 0.0)) > 0.0
    )
    if not speeds:
        return 200.0
    middle = len(speeds) // 2
    if len(speeds) % 2:
        return speeds[middle]
    return 0.5 * (speeds[middle - 1] + speeds[middle])


def follow_final_heading_deg(segments: Sequence[Dict]) -> Optional[float]:
    """The heading the plan's LAST motion segment leaves the robot in.

    The streaming follower follows a polyline only; it cannot see the
    segment model. This is the exit heading of the last GOTO/ARC (a waypoint
    arrival angle, the goal theta, an arc's exit tangent), so the follower
    can rotate to it after reaching the path end instead of stopping a few
    degrees off (waypoint angles are a hard requirement).
    """
    for segment in reversed(segments):
        kind = segment.get("kind")
        if kind == "goto":
            return float(segment.get("th", 0.0))
        if kind == "arc":
            return float(segment.get("th0_deg", 0.0)) + float(segment.get("sweep_deg", 0.0))
    return None


def follow_final_pose(segments: Sequence[Dict]) -> Optional[Tuple[float, float, float]]:
    """The plan's final ``(x, y, heading)`` from its last motion segment."""
    for segment in reversed(segments):
        kind = segment.get("kind")
        if kind == "goto" and "x" in segment and "y" in segment:
            return (
                float(segment["x"]),
                float(segment["y"]),
                float(segment.get("th", 0.0)),
            )
        if kind == "arc" and "x1" in segment and "y1" in segment:
            return (
                float(segment["x1"]),
                float(segment["y1"]),
                float(segment.get("th0_deg", 0.0)) + float(segment.get("sweep_deg", 0.0)),
            )
    return None


def needs_in_place_actions(segments: Sequence[Dict]) -> bool:
    """Does the program need an action the path polyline cannot encode?

    The streaming backend drives the path geometry only. An explicit
    rotation/pause, a raw drive/strafe (a close-waypoint departure reverses
    out before turning), or a mid-mission GOTO whose final heading is NOT
    the direction it drives off in would be silently skipped. Such plans
    must run on the segment backend. The FINAL heading is fine: the follower
    performs it with an in-place turn after the path end.
    """
    for index, segment in enumerate(segments):
        kind = segment.get("kind")
        if kind in ("turn", "pause", "drive", "strafe", "release"):
            # a raw drive/strafe is not polyline data: a close-waypoint
            # departure (reverse out, turn, continue) must run on the segment
            # backend, never be silently dropped by the follower; the same
            # holds for a motor release (the follower only drives geometry)
            return True
        if segment.get("close_waypoint") or segment.get("close_connector"):
            # a close-waypoint final leg was validated for a STRAIGHT body
            # pose after an in-place rotation at the stand-off; Pure Pursuit
            # would cut that corner towards the obstacle
            return True
        if kind != "goto":
            continue
        outgoing: Optional[float] = None
        for following in segments[index + 1:]:
            following_kind = following.get("kind")
            if following_kind in ("turn", "pause"):
                return True
            if following_kind == "goto":
                outgoing = math.degrees(math.atan2(
                    float(following["y"]) - float(segment["y"]),
                    float(following["x"]) - float(segment["x"]),
                ))
                break
            if following_kind == "arc":
                outgoing = float(following.get("th0_deg", 0.0))
                break
        if outgoing is None:
            continue  # the last motion node: the follower aligns to it
        if abs((float(segment.get("th", 0.0)) - outgoing + 180.0) % 360.0 - 180.0) > 5.0:
            return True
    return False


#: An ARC may start this far from the previous exit without a repair.
PROGRAM_GAP_TOL_MM = 25.0
#: An ARC may be entered this far off the previous exit heading.
PROGRAM_KINK_TOL_DEG = 5.0
#: Consecutive odometry reads that may fail before a run is aborted: the
#: board's I2C recovery storms hit the bus every ~10 s on the real robot and
#: a single missed poll must not kill a healthy run (poll_s * this ~= 2.5 s).
TELEMETRY_FAILURE_LIMIT = 25


def repair_program(
    segments: Sequence[Dict],
    start_pose: Optional[Tuple[float, float]] = None,
    start_heading_deg: Optional[float] = None,
) -> Tuple[List[Dict], List[str]]:
    """Insert corrective steps where a program is geometrically discontinuous.

    The board drives every ARC from the CURRENT pose and heading (firmware
    parity: the arc's circle is built from ``self._th``), so an ARC whose
    declared start ``(x0, y0)``/``th0_deg`` does not match the previous
    segment's exit is executed from the wrong circle and lands far from the
    plan. The planner guarantees continuity; this pass is the runtime NET for
    bugs it might still have (real-robot incident 2026-10-01: the arrival
    weave left a ~300 mm hole and the mission "completed" ~450 mm short of
    the waypoint):

    * an ARC starting more than :data:`PROGRAM_GAP_TOL_MM` from the previous
      exit gets a GOTO to its declared start (arriving on its entry heading)
      in front of it;
    * an ARC entered more than :data:`PROGRAM_KINK_TOL_DEG` off the exit
      heading gets an in-place TURN in front of it.

    Nothing is inserted when the chain cannot be verified: without a pose, or
    after an unverifiable move (manual drive/strafe, a geometry-less ARC as
    used by manual programs) - a GOTO re-anchors the chain. Returns the
    (possibly repaired) list and one message per repair for the UI log; a
    clean program comes back unchanged with no messages.
    """
    repaired: List[Dict] = []
    notes: List[str] = []
    cursor: Optional[Tuple[float, float]] = start_pose
    heading: Optional[float] = start_heading_deg

    def wrap(angle: float) -> float:
        return (angle + 180.0) % 360.0 - 180.0

    for segment in segments:
        kind = segment.get("kind", "goto")
        if kind == "goto":
            cursor = (float(segment["x"]), float(segment["y"]))
            heading = float(segment.get("th", heading if heading is not None else 0.0))
            repaired.append(segment)
        elif kind == "turn":
            if heading is not None:
                heading = wrap(heading + float(segment.get("angle_deg", 0.0)))
            repaired.append(segment)
        elif kind == "pause":
            repaired.append(segment)
        elif kind == "release":
            # no geometry and no heading change: keep the chain verified
            repaired.append(segment)
        elif kind == "arc":
            if cursor is None or heading is None or "x0" not in segment or "y0" not in segment:
                # no live reference: trust the program and keep the chain
                # unverified until the next GOTO re-anchors it
                cursor = None
                heading = None
                repaired.append(segment)
                continue
            start_pt = (float(segment["x0"]), float(segment["y0"]))
            entry = float(segment["th0_deg"])
            gap_mm = math.dist(cursor, start_pt)
            kink_deg = wrap(entry - heading)
            if gap_mm > PROGRAM_GAP_TOL_MM:
                notes.append(
                    "plan discontinuity: the arc at (%.0f, %.0f) mm starts %.0f mm "
                    "from the previous exit - driving to its start first"
                    % (start_pt[0], start_pt[1], gap_mm)
                )
                repaired.append(
                    {
                        "kind": "goto",
                        "x": round(start_pt[0], 1),
                        "y": round(start_pt[1], 1),
                        "th": round(entry, 1),
                        "v": int(segment.get("v", 150)),
                    }
                )
                heading = entry
            elif abs(kink_deg) > PROGRAM_KINK_TOL_DEG:
                notes.append(
                    "plan kink: %.0f deg between the previous exit and the arc at "
                    "(%.0f, %.0f) mm - turning in place first"
                    % (abs(kink_deg), start_pt[0], start_pt[1])
                )
                repaired.append(
                    {"kind": "turn", "angle_deg": round(kink_deg, 1), "v": int(segment.get("v", 150))}
                )
                heading = entry
            repaired.append(segment)
            if "x1" in segment and "y1" in segment and "sweep_deg" in segment:
                cursor = (float(segment["x1"]), float(segment["y1"]))
                heading = wrap(entry + float(segment["sweep_deg"]))
            else:
                cursor = None
                heading = None
        else:
            # manual moves / unknown kinds: their geometry cannot be verified
            cursor = None
            heading = None
            repaired.append(segment)
    return repaired, notes


class Executor:
    """Executes GOTO/ARC/TURN/DRIVE/STRAFE segment lists with live progress.

    ``client`` is any robot port satisfying :class:`~.interfaces.RobotIO`
    (the real/mock board client, or a test double). Progress goes to the
    event sink as ``{"type": "exec", "data": {...}}`` events.
    """

    def __init__(
        self,
        client: RobotIO,
        deadman_ms: int = 1500,
        timeout_slack_s: float = 8.0,
        poll_s: float = 0.1,
        stream_capability: Optional[Callable[[], Awaitable[bool]]] = None,
    ) -> None:
        """Create the executor with its timing and deadman configuration.

        ``stream_capability`` is the host's proof that the board accepts the
        silent velocity stream (the session's ``supports_velocity_stream``).
        The FOLLOW mode of :meth:`start` REFUSES to run without it.
        """
        self._client = client
        self._deadman_ms = max(0, deadman_ms)
        self._slack_s = timeout_slack_s
        self._poll_s = poll_s
        self._stream_capability = stream_capability
        self._task: Optional[asyncio.Task] = None
        self._abort = False
        self._pause = False
        self._state = EXEC_IDLE
        self._index = 0
        self._total = 0
        self._message = ""
        self._segments: List[Dict] = []
        self._stop_reason = ""
        self._event_sink: Optional[EventSink] = None
        self._chained_indices: List[int] = []  # segments the board already runs/runs
        self._chain_extra_s = 0.0              # watchdog budget of those segments
        self._chain_supported = True           # board accepts QUEUE_ARC
        self._no_chain: set = set()            # segments the board refused to queue
        self._sent_count = 0                   # board segments sent/queued this run
        self._done_base = 0                    # board counter at the run/sync start
        self._failure = ""                     # diagnostic kept for internal aborts
        self._chained_run = 0                  # segments blended this run
        self._speed_scale = 1.0                # caution multiplier (see set_speed_scale)
        self._follow_active = False            # current/last run uses the stream
        self._stream_speed = 0.0               # its cruise speed [mm/s]
        self._follow_bounds: Optional[Tuple[float, float, float]] = None  # stream geofence

    # ----------------------------------------------------------- plumbing
    def set_event_sink(self, sink: Optional[EventSink]) -> None:
        """Attach (or clear) the sink that receives exec events."""
        self._event_sink = sink

    @property
    def state(self) -> str:
        """Current executor state (idle/running/paused/done/aborted)."""
        return self._state

    @property
    def speed_scale(self) -> float:
        """Speed multiplier currently applied to every motion command."""
        return self._speed_scale

    @property
    def last_stream(self) -> bool:
        """True when the current (or most recent) run uses the streaming backend."""
        return self._follow_active

    def set_speed_scale(self, scale: float) -> float:
        """Slow the run down (or restore it) without re-planning anything.

        The dynamic-obstacle "caution" reaction: the plan keeps its own speeds
        and the operator keeps seeing them; the board is simply asked to drive
        them slower while something is standing on the route ahead. Every
        motion command sent from now on is scaled - the queued continuation
        arc keeps the previous scale for one segment, which is harmless.
        """
        self._speed_scale = max(CAUTION_MIN_SCALE, min(1.0, float(scale)))
        return self._speed_scale

    def _scaled(self, speed: float) -> int:
        """The speed to actually send for a segment (floor 30 mm/s)."""
        scaled = float(speed) * self._speed_scale
        return max(CAUTION_MIN_SPEED_MM_S, int(round(scaled)))

    @property
    def progress(self) -> Dict:
        """JSON-friendly progress snapshot for the console."""
        return {
            "status": self._state,
            "index": self._index,
            "total": self._total,
            "message": self._message,
            "speed_scale": round(self._speed_scale, 2),
            "streaming": self._follow_active,
        }

    def is_running(self) -> bool:
        """True while the runner task is alive."""
        return self._task is not None and not self._task.done()

    def remaining_elements(
        self, position: Optional[Tuple[float, float]] = None
    ) -> List[Dict]:
        """Planner elements for the part of the mission still ahead."""
        return remaining_elements(self._segments, self._index, position)

    def remaining_departure_mm(
        self, position: Optional[Tuple[float, float]] = None
    ) -> float:
        """How much of the close-waypoint reverse is still ahead [mm].

        Covers both the reverse itself and the final straight leg into the
        close waypoint (the departure is the next segment there). 0 when no
        close-waypoint departure is current/next. Used by the mid-run
        re-plan to finish the reverse before turning.
        """
        if not (0 <= self._index < len(self._segments)):
            return 0.0
        segment = self._segments[self._index]
        if segment.get("kind") == "drive" and segment.get("departure"):
            return remaining_drive_mm(segment, position)
        # the last straight leg onto the close point: the departure is ahead
        if (
            segment.get("kind") == "goto"
            and segment.get("close_waypoint")
            and self._index + 1 < len(self._segments)
        ):
            nxt = self._segments[self._index + 1]
            if nxt.get("kind") == "drive" and nxt.get("departure"):
                return float(nxt.get("distance_mm", 0.0))
        return 0.0

    async def _emit(self, **data) -> None:
        """Send one exec event through the sink (errors swallowed)."""
        if self._event_sink is None:
            return
        try:
            await self._event_sink({"type": "exec", "data": data})
        except Exception:
            pass

    async def _warn(self, message: str) -> None:
        """Write one warning line to the console log (best effort)."""
        if self._event_sink is None:
            return
        try:
            await self._event_sink({"type": "log", "data": {"level": "warn", "message": message}})
        except Exception:
            pass

    async def _note(self, message: str) -> None:
        """Write one informational line to the console log (best effort)."""
        if self._event_sink is None:
            return
        try:
            await self._event_sink({"type": "log", "data": {"level": "info", "message": message}})
        except Exception:
            pass

    async def _report_run_summary(self, segments: Sequence[Dict]) -> None:
        """Log chaining and how far the run ended from the plan's own end node.

        The board drives ARCs relative (radius + sweep from its CURRENT pose),
        so a chained run accumulates the per-arc completion tolerance and the
        wheel slip: the closing gap is the honest accuracy number, and the RPi
        fusion carries the same wheel/gyro information so it cannot correct it.
        """
        target: Optional[Tuple[float, float, float]] = None
        for segment in reversed(segments):
            kind = segment.get("kind")
            try:
                if kind == "arc" and "x1" in segment:
                    target = (
                        float(segment["x1"]),
                        float(segment["y1"]),
                        float(segment["th0_deg"]) + float(segment["sweep_deg"]),
                    )
                    break
                if kind == "goto" and "x" in segment:
                    target = (
                        float(segment["x"]),
                        float(segment["y"]),
                        float(segment.get("th", 0.0)),
                    )
                    break
            except (KeyError, TypeError, ValueError):
                continue  # manual moves carry no plan geometry: no closing audit
        note = (
            f"run finished: {self._total} segment(s), "
            f"{self._chained_run} blended without stopping"
        )
        await self._note(note)
        if target is None:
            return
        try:
            odometry = await self._client.odometry()
        except RobotIOError:
            return
        gap = math.hypot(odometry.x_mm - target[0], odometry.y_mm - target[1])
        heading = abs((odometry.theta_deg - target[2] + 180.0) % 360.0 - 180.0)
        if gap >= 20.0 or heading >= 3.0:
            await self._warn(
                f"run ended {gap:.0f} mm / {heading:.0f} deg from the plan's last node "
                "(accumulated arc tolerance, turn tolerance and wheel slip)"
            )
        else:
            await self._note(
                f"run ended {gap:.0f} mm / {heading:.0f} deg from the plan's last node"
            )

    # ------------------------------------------------------------ control
    async def start(
        self,
        segments: Sequence[Dict],
        deadman_ms: Optional[int] = None,
        paused: bool = False,
        follow_path: Optional[Sequence[Sequence[float]]] = None,
        follow_speed_mm_s: Optional[float] = None,
        follow_bounds: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        """Begin a run; ``paused`` starts it stopped (waits for RESUME).

        ``follow_path`` selects the STREAMING backend (Track B): the given
        polyline is driven with continuous velocity frames by
        :class:`~.follower.PathFollower` instead of stop-turn programs - one
        acceleration and one deceleration for the whole path. The mode is
        gated by the host capability (``STREAMING_EXEC`` on AND firmware
        >= 3.4, or the mock); a requested stream is REFUSED when the board
        cannot take it - it never degrades silently into the segment
        backend, or the field A/B comparison would lie. ``follow_speed_mm_s``
        overrides the cruise speed otherwise derived from the segments
        (:func:`follow_cruise_mm_s`). ``follow_bounds`` is the streaming
        geofence ``(width, height, inset)`` - see :meth:`PathFollower.run`.
        """
        if self.is_running():
            raise RuntimeError("an execution is already running")
        if not segments:
            raise ValueError("no segments to execute")
        points: Optional[List[Tuple[float, float]]] = None
        speed = 0.0
        if follow_path is not None:
            points = [(float(point[0]), float(point[1])) for point in follow_path]
            if len(points) < 2:
                raise ValueError("follow_path needs at least two points")
            supported = False
            if self._stream_capability is not None:
                try:
                    supported = bool(await self._stream_capability())
                except Exception:
                    supported = False
            if not supported:
                raise RuntimeError(
                    "streaming refused: STREAMING_EXEC is off or the board firmware "
                    "is below 3.4 (set STREAMING_EXEC=0 to run the segment backend)"
                )
            speed = float(follow_speed_mm_s) if follow_speed_mm_s else follow_cruise_mm_s(segments)
            if speed <= 0.0:
                raise ValueError("follow_speed_mm_s must be positive")
        self._abort = False
        self._pause = bool(paused)
        self._stop_reason = ""
        self._index = 0
        self._total = 1 if points is not None else len(segments)
        self._message = ""
        self._failure = ""
        self._segments = list(segments)
        self._chained_indices = []
        self._chain_extra_s = 0.0
        self._chain_supported = True
        self._sent_count = 0
        self._follow_active = points is not None
        self._stream_speed = speed
        self._follow_bounds = follow_bounds
        self._state = EXEC_RUNNING
        # a fresh run starts at full speed: the caution reaction (dynamic
        # obstacle) is a mid-run decision and is re-applied within a tick
        self._speed_scale = 1.0
        hw_deadman = self._deadman_ms if deadman_ms is None else max(0, deadman_ms)
        if points is not None:
            self._task = asyncio.create_task(self._run_follow(points, speed, hw_deadman))
        else:
            self._task = asyncio.create_task(self._run(list(segments), hw_deadman))

    async def stop(self, reason: str = "") -> None:
        """Abort the run; ``reason`` replaces the "aborted" progress message."""
        self._abort = True
        self._pause = False
        self._chained_indices = []
        self._chain_extra_s = 0.0
        self._sent_count = 0
        if reason:
            self._stop_reason = reason
        await self._safe(lambda: self._client.stop())
        await self.wait()

    async def wait_idle(self, timeout_s: float = 3.0) -> bool:
        """Wait until the board reports IDLE (the robot really stopped)."""
        deadline = time.monotonic() + timeout_s
        while True:
            try:
                if (await self._client.odometry()).is_idle:
                    return True
            except RobotIOError:
                return False
            if time.monotonic() >= deadline:
                return False
            await asyncio.sleep(0.05)

    async def estop(self) -> None:
        """Emergency stop: abort, latch the board e-stop, wait for the runner."""
        self._abort = True
        self._pause = False
        self._chained_indices = []
        self._chain_extra_s = 0.0
        self._sent_count = 0
        await self._safe(lambda: self._client.estop())
        await self.wait()

    def pause(self) -> None:
        """Ask a running mission to pause at the next checkpoint."""
        if self._state == EXEC_RUNNING:
            self._pause = True

    def resume(self) -> None:
        """Resume a paused mission."""
        if self._state == EXEC_PAUSED:
            self._pause = False

    async def wait(self) -> None:
        """Wait for the runner task to finish (never raises)."""
        if self._task is not None:
            try:
                await self._task
            except Exception:
                pass

    # -------------------------------------------------------------- runner
    async def _repair(self, segments: List[Dict]) -> List[Dict]:
        """Verify the program against the live pose and repair it if needed.

        The board builds every ARC from the CURRENT pose: a program whose
        geometry jumps (planner bug) would be executed from the wrong place
        and the mission would "complete" far from the target - repair (and
        log) instead of running it as-is.
        """
        try:
            odometry = await self._client.odometry()
        except RobotIOError:
            return segments  # no live reference: run as planned
        repaired, notes = repair_program(
            segments,
            (float(odometry.x_mm), float(odometry.y_mm)),
            float(odometry.theta_deg),
        )
        for note in notes:
            await self._warn(note)
        self._total = len(repaired)  # repairs may have inserted steps
        self._segments = repaired
        return repaired

    async def _clear_stale_deadman(self) -> None:
        """Clear a DEADMAN latch left behind by the previous stop, before moving.

        A console-driven stop (a re-plan, a wait) can leave the board's deadman
        latched when the link went quiet while the wheels were still winding
        down. The next run then aborted with "robot status FAULT" and the robot
        looked dead for good - the wait timer kept trying to resume and every
        attempt failed the same way (2026-10-09). Starting a run IS the
        operator's decision to move again, so the DEADMAN bit - and only that
        bit - is cleared here, loudly. Every other fault still aborts with its
        own message, and a deadman that lapses DURING a run still stops it.
        """
        try:
            odometry = await self._client.odometry()
        except RobotIOError:
            return
        if int(getattr(odometry, "faults", 0)) != FAULT_DEADMAN:
            return
        try:
            await self._client.clear_faults()
        except RobotIOError:
            return
        await self._warn(
            "board DEADMAN latch cleared to start the run (it latched while the "
            "robot was still stopping - the console is in control again)"
        )

    async def _correct_final_pose(
        self, target: Optional[Tuple[float, float, float]], label: str = ""
    ) -> None:
        """One small closed-loop GOTO onto the planned end point.

        The segment backend and the streaming follower both stop on their own
        tolerances; on the real floor wheel slip and the arc-corner cut leave
        the robot a few centimetres short of the drawn waypoint (2026-10-18:
        37 mm). A final GOTO measured from the LIVE pose lands it exactly and
        also fixes the last heading. Only a real miss triggers it, and it is
        skipped after an abort/pause.
        """
        if target is None or self._abort or self._pause:
            return
        if self._segments and self._segments[-1].get("kind") in ("turn", "pause", "release"):
            return  # a trailing action owns the final heading / released state
        try:
            odometry = await self._client.odometry()
        except RobotIOError:
            return
        gap = math.hypot(float(target[0]) - float(odometry.x_mm), float(target[1]) - float(odometry.y_mm))
        heading = (float(target[2]) - float(odometry.theta_deg) + 180.0) % 360.0 - 180.0
        if gap < FINAL_CORRECTION_MIN_MM and abs(heading) < FINAL_CORRECTION_MIN_DEG:
            return
        await self._note(
            "%s final approach correction: %.0f mm / %.0f deg to (%.1f, %.1f) mm"
            % (label or "run", gap, heading, target[0], target[1])
        )
        await self._emit(
            status=EXEC_RUNNING,
            index=self._index,
            total=self._total,
            message="final approach correction (%.0f mm)" % gap,
        )
        armed = False
        if self._deadman_ms > 0:
            armed = await self._safe(lambda: self._client.set_deadman(self._deadman_ms))
        try:
            sent = await self._safe(
                lambda: self._client.goto(
                    float(target[0]), float(target[1]), float(target[2]), FINAL_CORRECTION_SPEED_MM_S
                )
            )
            if not sent:
                await self._warn("final approach correction could not be sent")
                return
            await self.wait_idle(6.0)
        finally:
            if armed:
                await self._safe(lambda: self._client.set_deadman(0))
        try:
            odometry = await self._client.odometry()
            gap = math.hypot(
                float(target[0]) - float(odometry.x_mm), float(target[1]) - float(odometry.y_mm)
            )
            await self._note("final approach correction left %.0f mm" % gap)
        except RobotIOError:
            return

    async def _run(self, segments: List[Dict], deadman_ms: int) -> None:
        """Run every segment with pause, fault and timeout supervision."""
        armed_deadman = False
        try:
            await self._clear_stale_deadman()
            if deadman_ms > 0:
                armed_deadman = await self._safe(lambda: self._client.set_deadman(deadman_ms))
            await self._display("PATH RUN")
            segments = await self._repair(list(segments))
            self._chained_run = 0
            await self._resync_chain()
            for i, segment in enumerate(segments):
                if self._abort:
                    break
                if self._pause:  # started paused (or re-paused): wait here
                    if await self._handle_pause() == "abort":
                        break
                if i in self._chained_indices:
                    # the board already executed this segment: it was queued as
                    # a blended continuation of an earlier one (the previous
                    # await came back only after the WHOLE chain was idle)
                    self._chained_indices.remove(i)
                    self._index = i
                    await self._emit(status=EXEC_RUNNING, index=i, total=self._total, segment=segment)
                    continue
                self._index = i
                await self._emit(status=EXEC_RUNNING, index=i, total=self._total, segment=segment)
                if segment.get("kind") == "release":
                    if not await self._do_release(i):
                        break
                    continue
                if segment.get("kind") == "pause":
                    if not await self._do_pause(segment, i):
                        break
                    continue
                if not await self._send_segment(segment):
                    break
                self._sent_count += 1
                self._chain_extra_s = 0.0
                await self._pump_chain(segments, 0)
                result = await self._await_segment(segment, segments)
                if result != "done":
                    self._chained_indices = []
                    break
            if self._abort:
                self._state = EXEC_ABORTED
                # keep the DIAGNOSTIC ("segment timeout", "fault ...") when the
                # abort came from inside - only an explicit stop/wait reason
                # may replace it (overwriting lost the real cause and the UI
                # showed a bare "aborted", 2026-10-01)
                self._message = self._stop_reason or self._failure or "aborted"
                await self._display("PATH ABORT")
            else:
                await self._correct_final_pose(follow_final_pose(segments), "segment")
                self._state = EXEC_DONE
                self._message = "completed"
                await self._display("PATH DONE")
                try:
                    await self._report_run_summary(segments)
                except Exception:
                    pass  # the closing audit must never fail a finished run
        except Exception as exc:  # keep the executor alive, report to the UI
            self._state = EXEC_ABORTED
            self._message = str(exc)
        finally:
            if armed_deadman:
                await self._safe(lambda: self._client.set_deadman(0))
            await self._emit(status=self._state, index=self._index, total=self._total, message=self._message)

    async def _run_follow(
        self, points: Sequence[Tuple[float, float]], v_cruise_mm_s: float, deadman_ms: int
    ) -> None:
        """Streaming run (Track B): drive the whole polyline without stops.

        The follower closes the Pure Pursuit loop on the board odometry; the
        executor keeps the SAME safety shell as the segment backend: deadman
        arming, stale-fault abort, OLED messages, events, the caution speed
        scale (republished to the follower every tick) and the closing audit.
        """
        armed_deadman = False
        path_mm = path_length(points)
        try:
            await self._clear_stale_deadman()
            if deadman_ms > 0:
                armed_deadman = await self._safe(lambda: self._client.set_deadman(deadman_ms))
            await self._display("PATH RUN")
            await self._note(
                "streaming mode active: the path is driven with continuous velocity "
                "frames - no stop at the junctions"
            )
            follower = PathFollower(self._client)

            async def on_progress(data: Dict) -> None:
                frames = int(data.get("frames", 0))
                if frames % 5:  # ~5 Hz: enough for the console, no event flood
                    return
                await self._emit(
                    status=EXEC_RUNNING,
                    mode="follow",
                    index=0,
                    total=1,
                    message=f"streaming - {data.get('remaining_mm', 0.0):.0f} mm left",
                    remaining_mm=round(float(data.get("remaining_mm", 0.0)), 1),
                    path_mm=round(path_mm, 1),
                    frames=frames,
                )

            result = await follower.run(
                points,
                v_cruise_mm_s,
                speed_scale=lambda: self._speed_scale,
                should_abort=lambda: self._abort,
                should_pause=lambda: self._pause,
                on_pause=self._on_follow_pause,
                on_progress=on_progress,
                bounds=self._follow_bounds,
                final_heading_deg=follow_final_heading_deg(self._segments),
            )
            if self._abort:
                self._state = EXEC_ABORTED
                self._message = self._stop_reason or self._failure or "aborted"
                await self._display("PATH ABORT")
            elif bool(result.get("ok")):
                end_target = follow_final_pose(self._segments)
                if end_target is None and points:
                    heading = follow_final_heading_deg(self._segments)
                    if heading is None and len(points) >= 2:
                        heading = math.degrees(
                            math.atan2(
                                points[-1][1] - points[-2][1],
                                points[-1][0] - points[-2][0],
                            )
                        )
                    end_target = (points[-1][0], points[-1][1], heading or 0.0)
                await self._correct_final_pose(end_target, "streaming")
                self._state = EXEC_DONE
                self._message = "completed"
                await self._display("PATH DONE")
                await self._note(
                    f"run finished: streaming follower, {result['frames']} frame(s), "
                    f"max {result['max_speed_mm_s']:.0f} mm/s"
                )
                gap = float(result.get("final_error_mm", 0.0))
                if gap >= 20.0:
                    await self._warn(
                        f"run ended {gap:.0f} mm from the path end "
                        "(tracking error, wheel slip or a cut corner)"
                    )
                else:
                    await self._note(f"run ended {gap:.0f} mm from the path end")
            else:
                self._state = EXEC_ABORTED
                self._message = str(result.get("reason") or "aborted")
                await self._display("PATH ABORT")
        except Exception as exc:  # keep the executor alive, report to the UI
            self._state = EXEC_ABORTED
            self._message = str(exc)
        finally:
            if armed_deadman:
                await self._safe(lambda: self._client.set_deadman(0))
            await self._emit(
                status=self._state,
                mode="follow",
                index=self._index,
                total=self._total,
                message=self._message,
            )

    async def _on_follow_pause(self, paused: bool) -> None:
        """Mirror the follower's pause transitions into the executor state."""
        if paused:
            self._state = EXEC_PAUSED
            self._message = "paused (streaming)"
        elif not self._abort:
            self._state = EXEC_RUNNING
            self._message = "resumed (streaming)"
        await self._emit(
            status=self._state,
            mode="follow",
            index=0,
            total=self._total,
            message=self._message,
        )

    async def _resync_chain(self) -> None:
        """Reset the chain bookkeeping against the board's segment counter.

        Called at the start of every run and after a pause/resume: the board
        counts completed motion segments there, so the pump can tell when the
        single pending slot becomes free.
        """
        self._chained_indices = []
        self._chain_extra_s = 0.0
        self._sent_count = 0
        self._chain_supported = True
        self._no_chain = set()
        try:
            odometry = await self._client.odometry()
            self._done_base = int(getattr(odometry, "segments_done", 0)) & 0xFFFF
        except RobotIOError:
            self._done_base = 0
            self._chain_supported = False

    async def _pump_chain(self, segments: Sequence[Dict], done_relative: int) -> None:
        """Keep ONE continuation ARC queued on the board while the chain runs.

        The board owns a single pending slot: once the previously queued arc
        has STARTED (its predecessor completed, so the completed-segment
        counter advanced) the slot is free and the NEXT arc may be queued -
        this keeps an arc chain of any length running without one stop in
        between. Tangent continuity is the planner's contract (explicit TURN
        segments separate the chains at every real corner). Old boards answer
        UNKNOWN_CMD: chaining is disabled for the run and the normal
        stop-send-wait flow takes over.
        """
        if not self._chain_supported:
            return
        if self._chained_indices:
            next_index = self._chained_indices[-1] + 1
        else:
            next_index = self._index + 1
        if next_index >= len(segments) or next_index in self._no_chain:
            return
        if done_relative < self._sent_count - 1:
            return  # the pending slot is still occupied by the last queued arc
        nxt = segments[next_index]
        if nxt.get("kind") != "arc":
            return
        try:
            await self._client.queue_arc(
                float(nxt["radius_mm"]), float(nxt["sweep_deg"]), self._scaled(nxt.get("v", 150))
            )
        except RobotIOError as exc:
            status = getattr(exc, "status", None)
            if status is None:
                # a TRANSPORT hiccup (I2C timeout / the board's recovery
                # storms): retry on the next poll - killing the chain here
                # turned every later segment into a stop-send-wait and a 7 s
                # plan crawled for 29 s (real run 2026-10-07)
                return
            if status == 0x04:
                return  # BUSY: the slot is momentarily full, retry next poll
            if status == 0x03:
                # UNKNOWN_CMD: an old board without QUEUE_ARC - no chaining
                self._chain_supported = False
                return
            # the board rejected THIS arc (RANGE/BAD_PARAM): never queue it
            # again, but keep chaining the rest of the run
            self._no_chain.add(next_index)
            return
        self._chained_indices.append(next_index)
        self._chained_run += 1
        self._sent_count += 1
        self._chain_extra_s += self._segment_timeout(nxt)

    async def _do_release(self, index: int) -> bool:
        """Release the stepper drivers after the robot is fully stopped.

        The board additionally rejects the command with BUSY while a segment
        is still active. There is deliberately no E-STOP here: the next
        motion command re-energizes the drivers, which is what lets a
        strategy release, wait and continue.
        """
        if not await self.wait_idle(3.0):
            self._message = "release refused: the board did not report idle"
            self._failure = self._message
            await self._emit(
                status=EXEC_ABORTED, index=index, total=self._total, message=self._message
            )
            self._abort = True
            return False
        for attempt in range(3):
            try:
                await self._client.release_motors()
                await self._note("stepper motors released - the next move re-energizes them")
                await self._emit(
                    status=EXEC_RUNNING,
                    index=index,
                    total=self._total,
                    message="motors released",
                    segment={"kind": "release"},
                )
                return True
            except RobotIOError as exc:
                if getattr(exc, "status", None) == ST_BUSY and attempt < 2:
                    # a chained segment was still winding down: wait for its
                    # real end, then retry the idle-only release
                    await self.wait_idle(3.0)
                    continue
                self._message = f"release motors: {exc}"
                self._failure = self._message
                await self._emit(
                    status=EXEC_ABORTED, index=index, total=self._total, message=self._message
                )
                self._abort = True
                return False
        return False

    async def _do_pause(self, segment: Dict, index: int) -> bool:
        """Wait ``seconds`` with a live countdown; returns False on abort."""
        seconds = max(0.1, float(segment.get("seconds", 1.0)))
        deadline = time.monotonic() + seconds
        while True:
            if self._abort:
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return True
            await self._emit(
                status=EXEC_RUNNING,
                index=index,
                total=self._total,
                message=f"pause {remaining:.1f} s",
            )
            await asyncio.sleep(min(0.5, remaining))

    async def _send_segment(self, segment: Dict) -> bool:
        """Send one segment to the robot port; False aborts the run.

        A BUSY answer means a queued continuation ARC is still winding down
        (the pump keeps the board's single pending slot filled): wait for it
        and retry - aborting here stopped healthy runs mid-chain (2026-10-01).
        """
        kind = segment.get("kind", "goto")
        for attempt in range(20):
            try:
                if kind == "goto":
                    await self._client.goto(
                        segment["x"], segment["y"], segment["th"], self._scaled(segment.get("v", 200))
                    )
                elif kind == "drive":
                    await self._client.drive(segment["distance_mm"], self._scaled(segment.get("v", 150)))
                elif kind == "turn":
                    await self._client.turn(segment["angle_deg"], self._scaled(segment.get("v", 100)))
                elif kind == "strafe":
                    await self._client.strafe(segment["distance_mm"], self._scaled(segment.get("v", 120)))
                elif kind == "arc":
                    await self._client.arc(
                        segment["radius_mm"], segment["sweep_deg"], self._scaled(segment.get("v", 150))
                    )
                else:
                    raise ValueError(f"unknown segment kind '{kind}'")
                return True
            except RobotIOError as exc:
                status = getattr(exc, "status", None)
                if status == 0x04 and attempt < 19:
                    await asyncio.sleep(0.15)
                    continue
                if status is None and attempt < 3:
                    # transient transport error (I2C recovery): a couple of
                    # retries beat aborting a healthy run mid-segment
                    await asyncio.sleep(0.2)
                    continue
                self._message = f"segment {self._index}: {exc}"
                self._failure = self._message
                await self._emit(status=EXEC_ABORTED, index=self._index, total=self._total, message=self._message)
                self._abort = True
                return False
            except (KeyError, ValueError) as exc:
                self._message = f"segment {self._index}: {exc}"
                self._failure = self._message
                await self._emit(status=EXEC_ABORTED, index=self._index, total=self._total, message=self._message)
                self._abort = True
                return False
        return False

    async def _await_segment(self, segment: Dict, segments: Sequence[Dict]) -> str:
        """Poll until the segment completes. Returns done/paused/abort.

        While polling, the chain pump keeps the next ARC queued on the board
        (segment counter permitting) so tangent arc chains keep running.
        """
        deadline = time.monotonic() + self._segment_timeout(segment)
        telemetry_failures = 0
        while True:
            if self._abort:
                await self._safe(lambda: self._client.stop())
                return "abort"
            if self._pause:
                return await self._handle_pause()
            try:
                odometry = await self._client.odometry()
                telemetry_failures = 0
            except RobotIOError as exc:
                # a TRANSPORT hiccup (the board's I2C recovery storms hit the
                # bus every ~10 s on the real robot): keep polling - aborting
                # here killed healthy runs mid-chained-arc (2026-10-07)
                telemetry_failures += 1
                if telemetry_failures < TELEMETRY_FAILURE_LIMIT:
                    await asyncio.sleep(self._poll_s)
                    continue
                self._message = f"telemetry lost: {exc}"
                self._failure = self._message
                self._abort = True
                continue
            if odometry.faults != 0:
                self._message = f"fault {', '.join(odometry.describe_faults())}"
                self._failure = self._message
                await self._display("FAULT")
                self._abort = True
                continue
            done = (int(getattr(odometry, "segments_done", 0)) - self._done_base) & 0xFFFF
            if odometry.is_idle:
                return "done"
            # pump only while the board is ACTUALLY running: queueing on an
            # idle sample races the return (the board would start the queued
            # arc right then and the next send hit BUSY)
            await self._pump_chain(segments, done)
            # the CHAIN runs past this segment: every arc the pump queued as a
            # continuation extends the window (a fixed deadline computed at
            # entry aborted healthy arc chains mid-flight - the robot stopped
            # for no reason with a bare "aborted", 2026-10-01)
            if time.monotonic() > deadline + self._chain_extra_s:
                self._message = "segment timeout"
                self._failure = self._message
                await self._safe(lambda: self._client.stop())
                self._abort = True
                continue
            await asyncio.sleep(self._poll_s)

    async def _handle_pause(self) -> str:
        """Stop, publish the paused state, wait for resume; abort reported."""
        self._chained_indices = []  # STOP clears the pending continuation
        self._chain_extra_s = 0.0
        self._sent_count = 0
        await self._safe(lambda: self._client.stop())
        # wait until the board confirms the stop
        for _ in range(30):
            try:
                if (await self._client.odometry()).is_idle:
                    break
            except RobotIOError:
                pass
            await asyncio.sleep(self._poll_s)
        self._state = EXEC_PAUSED
        self._message = f"paused at node {self._index}"
        await self._emit(status=EXEC_PAUSED, index=self._index, total=self._total, message=self._message)
        while self._pause and not self._abort:
            await asyncio.sleep(0.1)
        if self._abort:
            return "abort"
        self._state = EXEC_RUNNING
        self._message = "resumed"
        await self._emit(status=EXEC_RUNNING, index=self._index, total=self._total, message=self._message)
        await self._resync_chain()
        return "skipped"  # node was interrupted; continue with the next one

    def _segment_timeout(self, segment: Dict) -> float:
        """Per-segment watchdog budget from length and speed."""
        speed = max(40.0, float(segment.get("v", 150)))
        kind = segment.get("kind", "goto")
        if kind == "turn":
            distance = abs(float(segment.get("angle_deg", 90.0))) * math.pi / 180.0 * 72.0
        elif kind == "arc":
            distance = (
                abs(float(segment.get("sweep_deg", 90.0)))
                * math.pi
                / 180.0
                * abs(float(segment.get("radius_mm", 200.0)))
            )
        else:
            distance = abs(float(segment.get("distance_mm", 0.0))) or 300.0
        return max(10.0, (distance / speed) * 4.0 + self._slack_s)

    # ------------------------------------------------------------- helpers
    async def _safe(self, action: Callable[[], Awaitable[None]]) -> bool:
        """Run a robot-port action; False when the port failed."""
        try:
            await action()
            return True
        except RobotIOError:
            return False

    async def _display(self, text: str) -> None:
        """Best-effort OLED message on the board."""
        try:
            await self._client.display_text(0, text)
        except RobotIOError:
            pass
