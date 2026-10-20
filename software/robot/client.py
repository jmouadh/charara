"""Async RobotClient: sequencing, idempotent retries and typed commands.

Every transaction is serialized behind one lock (single I2C bus) and retried
with the same sequence number, which the board treats as idempotent - exactly
the contract described in ``Docs/03-i2c-protocol-spec.md``.

File:    software/robot/client.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import asyncio
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from software.core.mission.interfaces import RobotIOError

from . import protocol as p
from . import robot_config
from .link import LinkError
from .models import Odometry, Status


class RobotError(RobotIOError):
    """The board answered a non-OK status."""

    def __init__(self, status: int) -> None:
        """Build the error from a board status code."""
        self.status = status
        self.name = p.STATUS_NAMES.get(status, f"0x{status:02X}")
        super().__init__(f"robot status {self.name}")


class RobotClient:
    """Async facade over a byte-level link (I2CLink or MockLink)."""

    def __init__(
        self,
        link,
        retries: int = 5,
        on_trace: Optional[Callable[[Dict], None]] = None,
    ) -> None:
        """Create the client around a byte-level link."""
        self._link = link
        self._retries = retries
        self._seq = 0
        self._lock = asyncio.Lock()
        self._trace = on_trace
        self._odometry_observer: Optional[Callable[[Odometry], None]] = None

    def set_odometry_observer(self, observer: Optional[Callable[[Odometry], None]]) -> None:
        """Register a callback fed with EVERY odometry read.

        The console/brain localizer subscribes here, so the estimate also
        refreshes on the executor's own polls (``wait_idle`` after a stop):
        the pose used for a re-plan is never a whole telemetry period stale.
        """
        self._odometry_observer = observer

    @property
    def link(self):
        """The underlying byte-level link (I2CLink or MockLink)."""
        return self._link

    def _next_seq(self) -> int:
        """Next request sequence number (1..255, wraps)."""
        self._seq = 1 if self._seq >= 255 else self._seq + 1
        return self._seq

    def _emit_trace(self, entry: Dict) -> None:
        """Forward one trace entry; never raises."""
        if self._trace is not None:
            try:
                self._trace(entry)
            except Exception:
                pass

    async def transact(self, cmd: int, payload: bytes = b"", retries: Optional[int] = None) -> Tuple[int, bytes]:
        """Send one request; returns (status, payload). Raises LinkError when
        the transport keeps failing and the frame is never answered."""
        seq = self._next_seq()
        frame = p.build_request(seq, cmd, payload)
        tries = retries if retries is not None else self._retries
        last_error: Optional[Exception] = None
        async with self._lock:
            for attempt in range(1, tries + 1):
                self._emit_trace(
                    {
                        "dir": "tx",
                        "seq": seq,
                        "cmd": cmd,
                        "cmd_name": p.CMD_NAMES.get(cmd, f"0x{cmd:02X}"),
                        "len": len(payload),
                        "payload": payload.hex(" "),
                        "frame": frame.hex(" "),
                        "attempt": attempt,
                    }
                )
                try:
                    response = await asyncio.to_thread(self._link.transact, frame)
                    status, response_payload = p.parse_response(response)
                    if response[1] != seq:
                        # The board had not executed this request yet when the
                        # read phase started (it was still serving the previous
                        # response - see the spec's response-seq rule). Accept
                        # nothing that carries another seq: retry with the SAME
                        # seq, which the board treats as a duplicate after it
                        # did execute, or executes now.
                        raise ValueError(f"stale response: seq {response[1]} != {seq}")
                    self._emit_trace(
                        {
                            "dir": "rx",
                            "seq": seq,
                            "cmd": cmd,
                            "cmd_name": p.CMD_NAMES.get(cmd, f"0x{cmd:02X}"),
                            "status": status,
                            "status_name": p.STATUS_NAMES.get(status, f"0x{status:02X}"),
                            "len": len(response_payload),
                            "payload": response_payload.hex(" "),
                            "frame": response.hex(" "),
                            "attempt": attempt,
                        }
                    )
                    return status, response_payload
                except (LinkError, ValueError) as exc:
                    last_error = exc
                    self._emit_trace(
                        {
                            "dir": "err",
                            "seq": seq,
                            "cmd": cmd,
                            "cmd_name": p.CMD_NAMES.get(cmd, f"0x{cmd:02X}"),
                            "len": 0,
                            "payload": "",
                            "note": str(exc)[:160],
                            "attempt": attempt,
                        }
                    )
            raise LinkError(f"no valid answer after {tries} attempts: {last_error}")

    async def command(self, cmd: int, payload: bytes = b"") -> bytes:
        """Send a request and require an OK status."""
        status, response = await self.transact(cmd, payload)
        if status != p.ST_OK:
            raise RobotError(status)
        return response

    # ------------------------------------------------------------ queries
    async def odometry(self) -> Odometry:
        """Read board odometry and feed the registered observers."""
        result = Odometry.parse(await self.command(p.CMD_QUERY_ODOMETRY))
        if self._odometry_observer is not None:
            try:
                self._odometry_observer(result)
            except Exception:
                pass  # an observer must never break the link
        return result

    async def status(self) -> Status:
        """Read the board status block."""
        return Status.parse(await self.command(p.CMD_QUERY_STATUS))

    async def events(self, after: int = 0) -> Dict:
        """Board error/event log: events with index > ``after`` (up to 5).

        Returns ``{latest, faults, events: [{index, code, aux}, ...]}``; the
        codes and severities are decoded by ``protocol.EVENT_NAMES``.
        """
        payload = await self.command(p.CMD_QUERY_EVENTS, p.put_u16(after))
        latest = p.get_u16(payload, 0)
        faults = p.get_u16(payload, 2)
        count = p.get_u8(payload, 4) if len(payload) > 4 else 0
        events: List[Dict] = []
        for slot in range(count):
            base = 5 + 4 * slot
            if len(payload) < base + 4:
                break
            events.append(
                {
                    "index": p.get_u16(payload, base),
                    "code": p.get_u8(payload, base + 2),
                    "aux": p.get_u8(payload, base + 3),
                }
            )
        return {"latest": latest, "faults": faults, "events": events}

    async def ticks(self) -> Tuple[int, int]:
        """Raw encoder accumulators (left, right)."""
        payload = await self.command(p.CMD_QUERY_TICKS)
        return p.get_i32(payload, 0), p.get_i32(payload, 4)

    async def ping(self, echo: int = 0x12345678) -> int:
        """Echo a 32-bit value through the board (link health check)."""
        payload = await self.command(p.CMD_PING, p.put_u32(echo))
        return p.get_u32(payload, 0)

    # ------------------------------------------------------------ motion
    async def drive(self, distance_mm: float, speed_mm_s: int) -> None:
        """Straight move: signed distance [mm] at ``speed_mm_s``."""
        await self.command(p.CMD_DRIVE, p.put_i16(round(distance_mm)) + p.put_u16(speed_mm_s))

    async def turn(self, angle_deg: float, speed_mm_s: int) -> None:
        """In-place rotation: signed angle [deg] at ``speed_mm_s``."""
        await self.command(p.CMD_TURN, p.put_i16(round(angle_deg * 10.0)) + p.put_u16(speed_mm_s))

    async def goto(self, x_mm: float, y_mm: float, theta_deg: float, speed_mm_s: int) -> None:
        """Absolute GOTO to (x, y, theta) at ``speed_mm_s``."""
        await self.command(
            p.CMD_GOTO,
            p.put_i16(round(x_mm * 10.0))
            + p.put_i16(round(y_mm * 10.0))
            + p.put_i16(round(theta_deg * 10.0))
            + p.put_u16(speed_mm_s),
        )

    async def strafe(self, distance_mm: float, speed_mm_s: int) -> None:
        """Lateral move: signed distance [mm] at ``speed_mm_s``."""
        await self.command(p.CMD_STRAFE, p.put_i16(round(distance_mm)) + p.put_u16(speed_mm_s))

    async def arc(self, radius_mm: float, sweep_deg: float, speed_mm_s: int) -> None:
        """Circular arc: forward motion and rotation at the same time.

        Positive radius = arc to the robot's left (CCW), negative = right;
        ``sweep_deg`` must have the same sign. The board caps the cruise
        speed on small radii by itself.
        """
        await self.command(
            p.CMD_ARC,
            p.put_i16(round(radius_mm)) + p.put_i16(round(sweep_deg * 10.0)) + p.put_u16(speed_mm_s),
        )

    async def queue_arc(self, radius_mm: float, sweep_deg: float, speed_mm_s: int) -> None:
        """Queue a continuation ARC for seamless chaining.

        Same payload as :meth:`arc`, but accepted while a segment runs: the
        board stores it and starts it the moment the current segment
        finishes WITHOUT stopping the wheels (one continuous move through a
        tangent arc chain). When the board is idle it simply starts the arc
        now. Old firmware answers UNKNOWN_CMD and a full pending slot
        answers BUSY - the caller treats both as "no chaining" and keeps the
        normal stop-send-wait flow.
        """
        await self.command(
            p.CMD_QUEUE_ARC,
            p.put_i16(round(radius_mm)) + p.put_i16(round(sweep_deg * 10.0)) + p.put_u16(speed_mm_s),
        )

    async def stream_gyro(self, rate_deg_s: float, age_10ms: int = 0) -> None:
        """Send one silent SET_GYRO frame: write-only, never retried.

        The board executes the frame without publishing a response, so the
        command/response channel keeps serving normal commands and retries.
        A lost stream frame is harmless - the next one replaces it.
        """
        scaled = int(round(float(rate_deg_s) / p.GYRO_RATE_SCALE))
        scaled = max(-32767, min(32767, scaled))
        age = max(0, min(255, int(age_10ms)))
        frame = p.build_request(
            self._next_seq(), p.CMD_SET_GYRO, p.put_i16(scaled) + p.put_u8(age)
        )
        async with self._lock:
            await asyncio.to_thread(self._link.send_stream, frame)

    async def stream_velocity(self, v_mm_s: float, omega_deg_s: float) -> None:
        """Send one silent SET_VELOCITY frame: write-only, never retried.

        The streaming twin of :meth:`stream_gyro` (firmware >= 3.4): the
        board ramps towards the targets, applies the newest frame of a
        ``latest-wins`` stream, and leaves the streaming mode by itself when
        no frame arrives for 200 ms (controlled ramp-down, never a fault).
        Accepted while the board is idle or already streaming; silently has
        no effect during any other motion or while a fault is latched. A
        lost frame is harmless - the next one replaces it.
        """
        velocity = max(-32767, min(32767, int(round(float(v_mm_s)))))
        omega = int(round(float(omega_deg_s) / p.VELOCITY_OMEGA_SCALE))
        omega = max(-32767, min(32767, omega))
        frame = p.build_request(
            self._next_seq(), p.CMD_SET_VELOCITY, p.put_i16(velocity) + p.put_i16(omega)
        )
        async with self._lock:
            await asyncio.to_thread(self._link.send_stream, frame)

    # ------------------------------------------------------------- pose
    async def set_pose(self, x_mm: float, y_mm: float, theta_deg: float) -> None:
        """Overwrite the board pose (robot idle only)."""
        await self.command(
            p.CMD_SET_POSE,
            p.put_i16(round(x_mm * 10.0)) + p.put_i16(round(y_mm * 10.0)) + p.put_i16(round(theta_deg * 10.0)),
        )

    async def reset_odometry(self) -> None:
        """Zero the board odometry (robot idle only)."""
        await self.command(p.CMD_RESET_ODOMETRY)

    # ----------------------------------------------------------- control
    async def stop(self) -> None:
        """Controlled stop."""
        await self.command(p.CMD_STOP)

    async def estop(self) -> None:
        """Latch the board emergency stop."""
        await self.command(p.CMD_ESTOP)

    async def release_motors(self) -> None:
        """Release the stepper drivers (board idle only; no fault latched).

        The next motion command re-energizes the drivers automatically. The
        board rejects the request with BUSY while a segment is still active.
        """
        await self.command(p.CMD_RELEASE_MOTORS)

    async def clear_faults(self) -> None:
        """Clear the latched board faults."""
        await self.command(p.CMD_CLEAR_FAULTS)

    async def set_limits(self, max_speed_mm_s: int, accel_mm_s2: int, decel_mm_s2: int) -> None:
        """Update the board motion limits (ranges as in the config map)."""
        await self.command(
            p.CMD_SET_MOTION_LIMITS,
            p.put_u16(max_speed_mm_s) + p.put_u16(accel_mm_s2) + p.put_u16(decel_mm_s2),
        )

    async def set_deadman(self, timeout_ms: int) -> None:
        """Arm or clear the board command watchdog [ms; 0 = off]."""
        await self.command(p.CMD_SET_DEADMAN, p.put_u16(timeout_ms))

    # -------------------------------------------------------- robot config
    async def get_config(self, offset: int = 0, count: int = robot_config.GET_CHUNK_MAX) -> Tuple[int, List[int]]:
        """Read a run of settings registers: returns ``(offset, values)``."""
        payload = await self.command(p.CMD_GET_CONFIG, p.put_u8(offset) + p.put_u8(count))
        base = payload[0]
        number = payload[1]
        values = [p.get_u16(payload, 2 + 2 * slot) for slot in range(number)]
        return base, values

    async def get_all_config(self) -> List[int]:
        """Full settings register image (chunked to the frame limit)."""
        values: List[int] = []
        while len(values) < robot_config.REGISTER_COUNT:
            count = min(robot_config.GET_CHUNK_MAX, robot_config.REGISTER_COUNT - len(values))
            base, chunk = await self.get_config(len(values), count)
            if base != len(values) or not chunk:
                raise RobotError(p.ST_RANGE)
            values.extend(chunk)
        return values

    async def set_config(self, offset: int, values: Sequence[int]) -> None:
        """Write a run of settings registers (validated by the board)."""
        if not 1 <= len(values) <= 4:
            raise ValueError(f"one SET_CONFIG frame carries 1..4 values, got {len(values)}")
        payload = p.put_u8(offset) + p.put_u8(len(values)) + b"".join(p.put_u16(int(value)) for value in values)
        await self.command(p.CMD_SET_CONFIG, payload)

    async def save_config(self) -> None:
        """Persist the RAM settings image to the board's flash."""
        await self.command(p.CMD_SAVE_CONFIG)

    async def load_config(self) -> None:
        """Reload the settings from the board's flash."""
        await self.command(p.CMD_LOAD_CONFIG)

    async def reset_config(self) -> None:
        """Restore the compile-time defaults in RAM (not saved)."""
        await self.command(p.CMD_RESET_CONFIG)

    # ----------------------------------------------------------- display
    async def display_mode(self, mode: int) -> None:
        """Select the OLED mode (0 off, 1 auto, 2 manual)."""
        await self.command(p.CMD_DISPLAY_MODE, p.put_u8(mode))

    async def display_text(self, line: int, text: str, offset: int = 0) -> None:
        """Write up to 9 characters of ``text`` at (line, offset)."""
        raw = text.encode("ascii", errors="replace")[:9]
        await self.command(p.CMD_DISPLAY_TEXT, p.put_u8(line) + p.put_u8(offset) + raw)
