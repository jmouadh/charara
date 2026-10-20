"""Transport layer: real I2C link (smbus2) and an offline mock board.

Both classes expose the same byte-level ``transact(request) -> response``
contract, so the rest of the stack never knows whether a real STM32 is
attached. ``I2CLink`` follows the bus rules of the firmware spec: one WRITE
transaction (16 B, STOP), a short pause, then one READ transaction (32 B).
Retries reuse the same frame, which the board's sequence-number idempotency
makes safe.

File:    software/robot/link.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import math
import time
from typing import Optional

from software.core.mission.interfaces import RobotIOError

from . import protocol as p
from . import robot_config

try:  # smbus2 exists on the Pi; tests and mock mode run without it
    from smbus2 import SMBus, i2c_msg  # type: ignore
except ImportError:  # pragma: no cover - exercised only without smbus2
    SMBus = None
    i2c_msg = None


class LinkError(RobotIOError):
    """Transport level failure (NACK, timeout, bus error ...)."""


def _wrap_deg(angle_deg: float) -> float:
    """Wrap to [-180, +180) like the firmware's motion_normalize_angle()."""
    wrapped = math.fmod(angle_deg + 180.0, 360.0)
    if wrapped < 0.0:
        wrapped += 360.0
    return wrapped - 180.0


class I2CLink:
    """Blocking smbus2 link to the board. Use through RobotClient."""

    def __init__(self, bus: int, address: int, retries: int = 5, retry_delay_s: float = 0.005) -> None:
        """Open the I2C bus with the given retry policy."""
        if SMBus is None or i2c_msg is None:
            raise LinkError("smbus2 is not installed (pip install smbus2)")
        self._address = address
        self._retries = max(1, retries)
        self._retry_delay = retry_delay_s
        self._bus = SMBus(bus)

    def close(self) -> None:
        """Close the bus."""
        self._bus.close()

    def transact(self, request: bytes) -> bytes:
        """Send one request frame and return the 32-byte response frame."""
        last_error: Optional[Exception] = None
        for attempt in range(self._retries):
            try:
                self._bus.i2c_rdwr(i2c_msg.write(self._address, list(request)))
                time.sleep(0.002)  # STOP -> START gap required by the slave
                read = i2c_msg.read(self._address, p.RSP_SIZE)
                self._bus.i2c_rdwr(read)
                response = bytes(read)
                if len(response) != p.RSP_SIZE:
                    raise LinkError(f"short read: {len(response)} bytes")
                return response
            except OSError as exc:  # NACK (e.g. OLED window) or bus error
                last_error = exc
                if attempt < self._retries - 1:
                    time.sleep(self._retry_delay)
        raise LinkError(f"I2C transaction failed after {self._retries} tries: {last_error}")

    def send_stream(self, request: bytes) -> None:
        """Send one silent stream frame: write-only, never retried.

        Used for SET_GYRO: the board executes the frame without publishing a
        response, so no read follows; a lost frame is replaced by the next
        one (stream frames are perishable by design).
        """
        if len(request) != p.REQ_SIZE:
            raise LinkError(f"request size {len(request)} != {p.REQ_SIZE}")
        try:
            self._bus.i2c_rdwr(i2c_msg.write(self._address, list(request)))
        except OSError as exc:
            raise LinkError(f"I2C stream write failed: {exc}") from exc


class MockLink:
    """Virtual board: integrates the same commands the firmware would.

    Used for UI/planner development without hardware
    (``ROBOT_TRANSPORT=mock``). It is intentionally simple: straight-line
    interpolation along each commanded move (arcs follow their circle; GOTO
    runs the firmware's turn -> drive -> turn phases), min/max speed clamps,
    BUSY rejection, STOP freeze, latched E-STOP - plus the settings register
    map with a simulated flash record and the same error event log the
    firmware keeps.
    """

    MIN_SPEED = 40.0
    MAX_SPEED = float(robot_config.MAX_SPEED_LIMIT_MM_S)
    #: firmware ROBOT_TURN_TOL: skip GOTO turn phases within this heading error
    _GOTO_TURN_TOL_DEG = 0.8
    #: firmware ROBOT_STREAM_STALE_MS: no SET_VELOCITY frame for this long
    #: while streaming -> the wheels ramp down to a controlled stop
    _STREAM_STALE_S = 0.2
    #: the board accepts the silent SET_VELOCITY stream (firmware >= 3.4)
    supports_velocity_stream = True

    def __init__(self) -> None:
        """Create a fresh virtual board at the match start pose.

        The robot starts where a Eurobot match starts: (315, 1800) mm facing
        -90 deg (towards the table centre) - matching how the real robot is
        placed at the beginning of a run.
        """
        self._t0 = time.monotonic()
        self._x = 315.0
        self._y = 1800.0
        self._th = -90.0  # degrees, CCW positive
        self._state = p.STATE_IDLE
        self._faults = 0
        self._limits = 0
        self._motion: Optional[dict] = None
        self._odom_left_mm = 0.0
        self._odom_right_mm = 0.0
        self._last_adv_t = self._t0
        self._v_left = 0.0   # encoder-style wheel speeds (firmware parity)
        self._v_right = 0.0
        self._last_seq = 0
        self._last_status = p.ST_NODATA
        self._deadman_ms = 0
        self._cfg_raw = robot_config.default_raw()
        self._flash_raw: Optional[list] = None
        self._cfg_dirty = False
        self._events = []
        self._event_seq = 0
        self._goto_phases: list = []
        self._goto_speed = 0.0
        self._pending_arc: Optional[tuple] = None  # queued continuation ARC
        self.blend_count = 0                       # mock instrumentation (tests)
        self._segments_done = 0                    # completed motion segments
        # normal stepper release (0x06): only set while idle; any accepted
        # motion command re-energizes the drivers
        self.motors_released = False
        # silent gyro stream (SET_GYRO): stored for parity and tests; never
        # touches the response state or the seq bookkeeping, like the firmware
        self.last_gyro_rate_dps: Optional[float] = None
        self.last_gyro_age_10ms = 0
        self.gyro_frames = 0
        self._last_gyro_t = 0.0
        # silent velocity stream (SET_VELOCITY, fw 3.4): {"v": mm/s,
        # "w": rad/s, "stamp": seconds}; targets apply while fresh, a stale
        # stream ramps down; any new frame replaces the targets (latest-wins)
        self._stream: Optional[dict] = None
        self.stream_frames = 0
        # deadman reference: EVERY received frame refreshes it (firmware
        # MOTION_notify_request), the watchdog trips only while busy
        self._last_activity = self._t0
        self._record(p.EVENT_RESET_POR, 0)

    # ------------------------------------------------------------------ config
    def _cfg_value(self, key: str) -> float:
        """Current value of one settings register in engineering units."""
        register = robot_config.REGISTER_BY_KEY[key]
        return register.decode(self._cfg_raw[register.index])

    def _record(self, code: int, aux: int = 0) -> None:
        """Append one error/event log entry (same codes as the firmware)."""
        if code == p.EVENT_NONE:
            return
        self._event_seq += 1
        self._events.append({"index": self._event_seq, "code": code, "aux": aux})
        if len(self._events) > 32:
            del self._events[0]

    def _speed_clamp(self, speed: float) -> float:
        """Clamp like the firmware: [min_speed, configured max_speed]."""
        low = self._cfg_value("min_speed_mm_s")
        high = max(low, self._cfg_value("max_speed_mm_s"))
        return max(low, min(high, speed if speed > 0 else low))

    # ------------------------------------------------------------------ mock
    def close(self) -> None:  # interface parity with I2CLink
        """No-op: the mock owns no resources."""
        pass

    def transact(self, request: bytes) -> bytes:
        """Execute one request frame and return the 32-byte response."""
        frame = request
        seq = frame[1]
        cmd = frame[2]
        payload = frame[4 : 4 + min(frame[3], p.REQ_PAYLOAD_SIZE)]
        self._advance()
        # firmware parity: a received request refreshes the deadman AFTER the
        # watchdog check ran - a stream of frames never trips its own watch
        self._last_activity = time.monotonic()
        status, response_payload = self._execute(cmd, payload)
        self._last_seq = seq
        self._last_status = status
        return self._respond(seq, status, response_payload)

    def send_stream(self, request: bytes) -> None:
        """Execute one silent stream frame (no response, no seq bookkeeping).

        Firmware parity (SET_GYRO): the frame updates the gyro input only -
        it must not touch the last-response state nor the retry window, so
        normal command idempotency is preserved while streaming at full rate.
        """
        cmd = request[2]
        payload = request[4 : 4 + min(request[3], p.REQ_PAYLOAD_SIZE)]
        self._advance()
        self._last_activity = time.monotonic()
        if cmd == p.CMD_SET_GYRO and len(payload) >= 3:
            self.last_gyro_rate_dps = p.get_i16(payload, 0) * p.GYRO_RATE_SCALE
            self.last_gyro_age_10ms = p.get_u8(payload, 2)
            self.gyro_frames += 1
            self._last_gyro_t = time.monotonic()
        elif cmd == p.CMD_SET_VELOCITY and len(payload) >= 4:
            # Silent latest-wins stream (firmware parity, fw 3.4): accepted
            # while idle or already streaming; silently ignored while a
            # segment runs (BUSY) or a fault is latched - never an event.
            if self._faults == 0 and self._motion is None:
                if self._stream is None:
                    self._stream = {"v": 0.0, "w": 0.0, "stamp": 0.0}
                    self._state = p.STATE_STREAM
                    self.motors_released = False
                self._stream["v"] = float(p.get_i16(payload, 0))
                self._stream["w"] = math.radians(p.get_i16(payload, 2) * p.VELOCITY_OMEGA_SCALE)
                self._stream["stamp"] = time.monotonic()
                self.stream_frames += 1

    # ------------------------------------------------------------- internal
    def _advance(self) -> None:
        """Integrate motion up to 'now': deadman first, then stream/segment.

        The watchdog fires the moment it expires WHILE motion is active: the
        motion is integrated exactly up to the expiry instant (never past
        it) and frozen there, then DEADMAN latches. A run that had already
        finished before the expiry never trips - exactly the firmware
        behaviour (the watchdog only guards BUSY motion).
        """
        now = time.monotonic()
        if self._deadman_ms > 0 and (self._motion is not None or self._stream is not None):
            expiry = self._last_activity + self._deadman_ms / 1000.0
            if now >= expiry:
                self._integrate(expiry)
                if self._motion is not None or self._stream is not None:
                    self._latch_deadman()
                    return
        self._integrate(now)

    def _latch_deadman(self) -> None:
        """Freeze every motion and latch the DEADMAN fault (watchdog path)."""
        self._stream = None
        self._motion = None
        self._goto_phases = []
        self._pending_arc = None
        self._faults |= p.FAULT_DEADMAN
        self._state = p.STATE_FAULT
        self._v_left = 0.0
        self._v_right = 0.0
        self._record(p.EVENT_DEADMAN, 0)

    def _integrate(self, now: float) -> None:
        """Integrate the active motion (segment or stream) up to ``now``."""
        if self._stream is not None:
            self._integrate_stream(now, self._stream)
            return
        motion = self._motion
        if motion is None:
            self._track_speeds(now, 0.0, 0.0)
            return
        span = motion["t1"] - motion["t0"]
        f = 1.0 if span <= 0 else min(1.0, (now - motion["t0"]) / span)
        if "arc" in motion:
            # follow the circle (not the chord) so the ghost path and the
            # odometry integration match what the robot really drives
            new_th = motion["th0"] + (motion["th1"] - motion["th0"]) * f
            th_rad = math.radians(new_th)
            new_x = motion["arc"]["cx"] + motion["arc"]["r"] * math.sin(th_rad)
            new_y = motion["arc"]["cy"] - motion["arc"]["r"] * math.cos(th_rad)
        else:
            new_x = motion["x0"] + (motion["x1"] - motion["x0"]) * f
            new_y = motion["y0"] + (motion["y1"] - motion["y0"]) * f
            new_th = motion["th0"] + (motion["th1"] - motion["th0"]) * f
        # Signed centre travel: project the pose delta on the mean heading.
        # A hypot() would report forward motion while backing up, and the
        # console's wheel-odometry estimator would then push the pose the
        # wrong way at the start of every reverse move (visible forward
        # "advance" before the actual backward motion).
        heading_mid = math.radians(0.5 * (self._th + new_th))
        d_center = (new_x - self._x) * math.cos(heading_mid) + (new_y - self._y) * math.sin(heading_mid)
        d_theta = math.radians(new_th - self._th)
        odom_track = self._cfg_value("odom_track_mm")
        d_left = d_center - d_theta * odom_track / 2.0
        d_right = d_center + d_theta * odom_track / 2.0
        self._odom_left_mm += d_left
        self._odom_right_mm += d_right
        self._x, self._y, self._th = new_x, new_y, new_th
        self._track_speeds(now, d_left, d_right)
        if f >= 1.0:
            self._motion = None
            self._th = _wrap_deg(self._th)
            self._v_left = 0.0
            self._v_right = 0.0
            if self._state not in (p.STATE_FAULT, p.STATE_ESTOP):
                if self._goto_phases:
                    self._advance_goto()
                else:
                    self._segments_done += 1
                    if not self._consume_pending_arc():
                        self._state = p.STATE_IDLE

    def _track_speeds(self, now: float, d_left: float, d_right: float) -> None:
        """Encoder-style wheel speeds from the odometry deltas.

        The firmware reports MEASURED wheel speeds; a constant placeholder
        would make the RPi estimator lag behind the real motion and then
        "catch up" after a stop, which looked like drift.
        """
        dt = now - self._last_adv_t
        self._last_adv_t = now
        if dt <= 0.000001:
            return
        self._v_left = max(-32767.0, min(32767.0, d_left / dt))
        self._v_right = max(-32767.0, min(32767.0, d_right / dt))

    def _integrate_stream(self, now: float, stream: dict) -> None:
        """Integrate one step of the silent SET_VELOCITY stream.

        While frames arrive the streamed targets apply directly (the real
        board ramps towards them; the mock integrates the targets exactly -
        deterministic for the follower tests). When no frame arrived for
        ROBOT_STREAM_STALE_MS the wheels ramp down with the configured
        deceleration and the stream ends IDLE, exactly like the firmware's
        controlled stop (never a fault).
        """
        dt = now - self._last_adv_t
        if dt <= 0.000001:
            return
        if (now - stream["stamp"]) > self._STREAM_STALE_S:
            decel = self._cfg_value("decel_mm_s2")
            half_track = max(1.0, self._cfg_value("odom_track_mm") * 0.5)
            for key, limit in (("v", decel), ("w", decel / half_track)):
                value = stream[key]
                step = limit * dt
                if value > step:
                    stream[key] = value - step
                elif value < -step:
                    stream[key] = value + step
                else:
                    stream[key] = 0.0
            if stream["v"] == 0.0 and stream["w"] == 0.0:
                self._stream = None
                self._state = p.STATE_IDLE
                self._v_left = 0.0
                self._v_right = 0.0
                self._last_adv_t = now
                return
        v = stream["v"]
        w = stream["w"]  # rad/s, CCW positive
        th = math.radians(self._th)
        d_th = w * dt
        if abs(w) > 1e-9:
            radius = v / w
            self._x += radius * (math.sin(th + d_th) - math.sin(th))
            self._y -= radius * (math.cos(th + d_th) - math.cos(th))
        else:
            self._x += v * dt * math.cos(th)
            self._y += v * dt * math.sin(th)
        self._th = _wrap_deg(self._th + math.degrees(d_th))
        d_center = v * dt
        track = self._cfg_value("odom_track_mm")
        d_left = d_center - d_th * track / 2.0
        d_right = d_center + d_th * track / 2.0
        self._odom_left_mm += d_left
        self._odom_right_mm += d_right
        self._track_speeds(now, d_left, d_right)

    def _start_motion(self, kind: int, x1: float, y1: float, th1: float, speed: float,
                      state: Optional[int] = None) -> None:
        """Start a timed linear or rotation motion to the given pose.

        ``state`` overrides the reported motion state (a GOTO keeps reporting
        GOTO while its turn/drive phases run).
        """
        self.motors_released = False
        if state is None:
            # a fresh segment command supersedes any queued GOTO phases
            self._goto_phases = []
        # The firmware compares headings through a NORMALIZED error each tick
        # (motion_do_turn / motion_do_drive): a target of +180 with the robot
        # at -180 is the SAME heading. Interpolating the raw difference made
        # the mock pirouette a full 360 deg per straight chunk when driving
        # along -x (atan2 returns +180 while the stored heading is -180).
        th1 = self._th + _wrap_deg(th1 - self._th)
        v = self._speed_clamp(speed)
        track = self._cfg_value("robot_track_mm")
        if kind == p.STATE_DRIVE:
            duration = math.hypot(x1 - self._x, y1 - self._y) / v
        elif kind == p.STATE_TURN:
            duration = (abs(math.radians(th1 - self._th)) * track / 2.0) / v
        else:  # STRAFE: timed open-loop, heading held
            duration = (math.hypot(x1 - self._x, y1 - self._y) / v) + (
                abs(math.radians(th1 - self._th)) * track / 2.0 / v
            )
        now = time.monotonic()
        self._motion = {
            "x0": self._x, "y0": self._y, "th0": self._th,
            "x1": x1, "y1": y1, "th1": th1,
            "t0": now, "t1": now + max(duration, 0.05),
        }
        self._state = kind if state is None else state

    def _start_goto(self, x: float, y: float, theta: float, speed: float) -> None:
        """Start a GOTO with the firmware's phase order.

        The firmware never translates and rotates at the same time for a
        GOTO: it turns to the travel heading, drives straight, then turns to
        the final heading. A blended interpolation (translate while rotating)
        looked like the robot sliding sideways between an arc and the next
        straight leg.
        """
        self._goto_speed = speed
        theta = self._th + _wrap_deg(theta - self._th)
        dx = x - self._x
        dy = y - self._y
        distance = math.hypot(dx, dy)
        phases = []
        if distance >= 1.0:
            heading = math.degrees(math.atan2(dy, dx))
            if abs(_wrap_deg(heading - self._th)) > self._GOTO_TURN_TOL_DEG:
                phases.append(("turn", heading))
            phases.append(("drive", x, y))
            if abs(_wrap_deg(theta - heading)) > self._GOTO_TURN_TOL_DEG:
                phases.append(("turn", theta))
        elif abs(_wrap_deg(theta - self._th)) > self._GOTO_TURN_TOL_DEG:
            phases.append(("turn", theta))
        self._goto_phases = phases
        self._state = p.STATE_GOTO
        self._advance_goto()

    def _consume_pending_arc(self) -> bool:
        """Start a queued continuation ARC right now (firmware parity).

        Called when the active segment completes: the wheels never stop and
        the ramp is not reset - the result is one continuous move through a
        tangent arc chain. Returns False when nothing was queued.
        """
        if self._pending_arc is None:
            return False
        radius, sweep, speed = self._pending_arc
        self._pending_arc = None
        self.blend_count += 1
        self._start_arc(radius, sweep, speed)
        return True

    def _advance_goto(self) -> None:
        """Start the next GOTO phase; IDLE once the phase list is drained."""
        if not self._goto_phases:
            self._segments_done += 1   # the GOTO manoeuvre is one segment
            if self._consume_pending_arc():
                return
            self._state = p.STATE_IDLE
            self._v_left = 0.0
            self._v_right = 0.0
            return
        phase = self._goto_phases.pop(0)
        if phase[0] == "turn":
            target = self._th + _wrap_deg(float(phase[1]) - self._th)
            self._start_motion(p.STATE_TURN, self._x, self._y, target, self._goto_speed,
                               state=p.STATE_GOTO)
        else:
            x1, y1 = float(phase[1]), float(phase[2])
            heading = math.degrees(math.atan2(y1 - self._y, x1 - self._x))
            self._start_motion(p.STATE_DRIVE, x1, y1, heading, self._goto_speed,
                               state=p.STATE_GOTO)

    def _start_arc(self, radius: float, sweep: float, speed: float) -> None:
        """Start an arc motion; the pose is integrated along the circle."""
        self.motors_released = False
        self._goto_phases = []
        v = self._speed_clamp(speed)
        phi0 = math.radians(self._th)
        cx = self._x - radius * math.sin(phi0)
        cy = self._y + radius * math.cos(phi0)
        th1 = self._th + sweep
        th1_rad = math.radians(th1)
        x1 = cx + radius * math.sin(th1_rad)
        y1 = cy - radius * math.cos(th1_rad)
        arc_len = abs(math.radians(sweep)) * abs(radius)
        now = time.monotonic()
        self._motion = {
            "x0": self._x, "y0": self._y, "th0": self._th,
            "x1": x1, "y1": y1, "th1": th1,
            "arc": {"cx": cx, "cy": cy, "r": float(radius)},
            "t0": now, "t1": now + max(arc_len / v, 0.05),
        }
        self._state = p.STATE_ARC

    def _busy(self) -> bool:
        """True while a segment or the velocity stream is in progress."""
        return self._motion is not None or self._stream is not None

    def _execute(self, cmd: int, payload: bytes):
        # Safety commands are always accepted --------------------------------
        """Run one decoded command; returns (status, payload)."""
        if cmd == p.CMD_ESTOP:
            self._motion = None
            self._goto_phases = []
            self._pending_arc = None
            self._stream = None
            self._faults |= p.FAULT_ESTOP
            self._state = p.STATE_ESTOP
            self._record(p.EVENT_ESTOP, 0)
            return p.ST_OK, b""
        if cmd == p.CMD_STOP:
            self._motion = None
            self._goto_phases = []
            self._pending_arc = None
            self._stream = None
            self._state = p.STATE_IDLE
            return p.ST_OK, b""
        if cmd == p.CMD_CLEAR_FAULTS:
            self._faults = 0
            self._limits = 0
            self._motion = None
            self._goto_phases = []
            self._pending_arc = None
            self._stream = None
            self._state = p.STATE_IDLE
            return p.ST_OK, b""
        if cmd == p.CMD_QUERY_ODOMETRY:
            return p.ST_OK, self._odometry_payload()
        if cmd == p.CMD_QUERY_STATUS:
            return p.ST_OK, self._status_payload()
        if cmd == p.CMD_RELEASE_MOTORS:
            # firmware parity: only while fully idle; no fault is latched and
            # the next motion command re-energizes the drivers
            if self._busy() or self._goto_phases:
                self._record(p.EVENT_BUSY_REJECT, cmd)
                return p.ST_BUSY, b""
            if self._faults != 0:
                return p.ST_FAULT, b""
            self.motors_released = True
            return p.ST_OK, b""
        if cmd == p.CMD_QUERY_TICKS:
            mm_per_tick = self._cfg_value("odom_wheel_circumference_mm") / self._cfg_value("odom_ticks_per_rev")
            left = round(self._odom_left_mm / mm_per_tick)
            right = round(self._odom_right_mm / mm_per_tick)
            return p.ST_OK, p.put_i32(left) + p.put_i32(right)
        if cmd == p.CMD_QUERY_EVENTS:
            after = p.get_u16(payload, 0) if len(payload) >= 2 else 0
            # firmware parity: oldest first, up to 5 per query (the console
            # drain relies on it; newest-first would silently skip events)
            fresh = [event for event in self._events if event["index"] > after][:5]
            out = p.put_u16(self._event_seq) + p.put_u16(self._faults) + p.put_u8(len(fresh))
            for event in fresh:
                out += p.put_u16(event["index"]) + p.put_u8(event["code"]) + p.put_u8(event["aux"])
            return p.ST_OK, out
        if cmd == p.CMD_GET_CONFIG:
            offset = p.get_u8(payload, 0)
            count = p.get_u8(payload, 1)
            if count == 0 or count > robot_config.GET_CHUNK_MAX or offset + count > robot_config.REGISTER_COUNT:
                self._record(p.EVENT_BAD_PARAM, cmd)
                return p.ST_RANGE, b""
            out = p.put_u8(offset) + p.put_u8(count)
            for index in range(offset, offset + count):
                out += p.put_u16(self._cfg_raw[index])
            return p.ST_OK, out
        if cmd == p.CMD_SET_CONFIG:
            if self._busy():
                self._record(p.EVENT_BUSY_REJECT, cmd)
                return p.ST_BUSY, b""
            offset = p.get_u8(payload, 0)
            count = p.get_u8(payload, 1)
            if count == 0 or count > robot_config.SET_CHUNK_MAX or offset + count > robot_config.REGISTER_COUNT:
                self._record(p.EVENT_BAD_PARAM, cmd)
                return p.ST_RANGE, b""
            if 2 + 2 * count > len(payload):
                self._record(p.EVENT_BAD_LEN, cmd)
                return p.ST_BAD_LEN, b""
            for slot in range(count):
                register = robot_config.REGISTER_BY_INDEX[offset + slot]
                value = p.get_u16(payload, 2 + 2 * slot)
                if register.read_only or not register.min_value <= value <= register.max_value:
                    self._record(p.EVENT_BAD_PARAM, cmd)
                    return p.ST_RANGE, b""
            for slot in range(count):
                self._cfg_raw[offset + slot] = p.get_u16(payload, 2 + 2 * slot)
            self._cfg_dirty = True
            self._record(p.EVENT_CONFIG_APPLIED, offset)
            return p.ST_OK, b""
        if cmd == p.CMD_SAVE_CONFIG:
            if self._busy():
                self._record(p.EVENT_BUSY_REJECT, cmd)
                return p.ST_BUSY, b""
            self._flash_raw = list(self._cfg_raw)
            self._cfg_dirty = False
            self._record(p.EVENT_CONFIG_SAVED, 0)
            return p.ST_OK, b""
        if cmd == p.CMD_LOAD_CONFIG:
            if self._busy():
                self._record(p.EVENT_BUSY_REJECT, cmd)
                return p.ST_BUSY, b""
            self._cfg_raw = list(self._flash_raw) if self._flash_raw is not None else robot_config.default_raw()
            self._cfg_dirty = False
            return p.ST_OK, b""
        if cmd == p.CMD_RESET_CONFIG:
            if self._busy():
                self._record(p.EVENT_BUSY_REJECT, cmd)
                return p.ST_BUSY, b""
            self._cfg_raw = robot_config.default_raw()
            self._cfg_dirty = True
            self._record(p.EVENT_CONFIG_DEFAULTS, 0)
            return p.ST_OK, b""
        if cmd == p.CMD_PING:
            return p.ST_OK, payload[:4].ljust(4, b"\x00")

        # Latched faults block motion commands (like the firmware) ----------
        if self._faults != 0 and cmd in (
            p.CMD_DRIVE, p.CMD_TURN, p.CMD_GOTO, p.CMD_STRAFE, p.CMD_ARC, p.CMD_QUEUE_ARC,
            p.CMD_SET_POSE, p.CMD_RESET_ODOMETRY
        ):
            return p.ST_FAULT, b""

        if cmd in (p.CMD_DRIVE, p.CMD_TURN, p.CMD_GOTO, p.CMD_STRAFE, p.CMD_ARC) and self._busy():
            self._record(p.EVENT_BUSY_REJECT, cmd)
            return p.ST_BUSY, b""

        if cmd == p.CMD_SET_POSE:
            if self._busy():
                self._record(p.EVENT_BUSY_REJECT, cmd)
                return p.ST_BUSY, b""
            self._x = p.get_i16(payload, 0) * 0.1
            self._y = p.get_i16(payload, 2) * 0.1
            self._th = _wrap_deg(p.get_i16(payload, 4) * 0.1)
            return p.ST_OK, b""

        if cmd == p.CMD_RESET_ODOMETRY:
            if self._busy():
                self._record(p.EVENT_BUSY_REJECT, cmd)
                return p.ST_BUSY, b""
            self._x = self._y = self._th = 0.0
            self._odom_left_mm = self._odom_right_mm = 0.0
            return p.ST_OK, b""

        if cmd == p.CMD_DRIVE:
            distance = p.get_i16(payload, 0)
            speed = p.get_u16(payload, 2)
            rad = math.radians(self._th)
            x1 = self._x + distance * math.cos(rad)
            y1 = self._y + distance * math.sin(rad)
            self._start_motion(p.STATE_DRIVE, x1, y1, self._th, speed)
            return p.ST_OK, b""

        if cmd == p.CMD_TURN:
            angle = p.get_i16(payload, 0) * 0.1
            speed = p.get_u16(payload, 2)
            # firmware parity: one TURN command carries at most 180 deg
            if abs(angle) > 180.0:
                self._record(p.EVENT_BAD_PARAM, cmd)
                return p.ST_RANGE, b""
            self._start_motion(p.STATE_TURN, self._x, self._y, self._th + angle, speed)
            return p.ST_OK, b""

        if cmd == p.CMD_GOTO:
            x = p.get_i16(payload, 0) * 0.1
            y = p.get_i16(payload, 2) * 0.1
            th = p.get_i16(payload, 4) * 0.1
            speed = p.get_u16(payload, 6)
            # firmware parity: turn to the travel heading, drive straight,
            # then turn to the final heading (never translate while rotating)
            self._start_goto(x, y, th, speed)
            return p.ST_OK, b""

        if cmd == p.CMD_STRAFE:
            distance = p.get_i16(payload, 0)
            speed = p.get_u16(payload, 2)
            rad = math.radians(self._th)
            # +distance = to the robot's left (+90 deg from heading)
            x1 = self._x + distance * math.cos(rad + math.pi / 2.0)
            y1 = self._y + distance * math.sin(rad + math.pi / 2.0)
            self._start_motion(p.STATE_STRAFE, x1, y1, self._th, speed)
            return p.ST_OK, b""

        if cmd == p.CMD_ARC:
            radius = p.get_i16(payload, 0)
            sweep = p.get_i16(payload, 2) * 0.1
            speed = p.get_u16(payload, 4)
            if radius == 0 or sweep == 0 or (radius > 0) != (sweep > 0):
                self._record(p.EVENT_BAD_PARAM, cmd)
                return p.ST_RANGE, b""
            self._start_arc(float(radius), sweep, speed)
            return p.ST_OK, b""

        if cmd == p.CMD_QUEUE_ARC:
            radius = p.get_i16(payload, 0)
            sweep = p.get_i16(payload, 2) * 0.1
            speed = p.get_u16(payload, 4)
            if radius == 0 or sweep == 0 or (radius > 0) != (sweep > 0):
                self._record(p.EVENT_BAD_PARAM, cmd)
                return p.ST_RANGE, b""
            if not self._busy() and not self._goto_phases:
                # idle: the queued arc IS the next motion - start it now
                self._start_arc(float(radius), sweep, speed)
                return p.ST_OK, b""
            if self._pending_arc is not None:
                self._record(p.EVENT_BUSY_REJECT, cmd)
                return p.ST_BUSY, b""
            self._pending_arc = (float(radius), sweep, speed)
            return p.ST_OK, b""

        if cmd == p.CMD_SET_MOTION_LIMITS:
            try:
                raw = robot_config.validate_engineering(
                    {
                        "max_speed_mm_s": p.get_u16(payload, 0),
                        "accel_mm_s2": p.get_u16(payload, 2),
                        "decel_mm_s2": p.get_u16(payload, 4),
                    }
                )
            except ValueError:
                self._record(p.EVENT_BAD_PARAM, cmd)
                return p.ST_RANGE, b""
            for key, value in raw.items():
                self._cfg_raw[robot_config.REGISTER_BY_KEY[key].index] = value
            self._cfg_dirty = True
            self._record(p.EVENT_CONFIG_APPLIED, robot_config.REGISTER_BY_KEY["max_speed_mm_s"].index)
            return p.ST_OK, b""

        if cmd == p.CMD_SET_DEADMAN:
            self._deadman_ms = p.get_u16(payload, 0)
            return p.ST_OK, b""

        if cmd in (p.CMD_DISPLAY_MODE, p.CMD_DISPLAY_TEXT):
            return p.ST_OK, b""

        self._record(p.EVENT_UNKNOWN_CMD, cmd)
        return p.ST_UNKNOWN_CMD, b""

    # ----------------------------------------------------------- responses
    def _odometry_payload(self) -> bytes:
        # firmware parity: the MEASURED wheel speeds travel in every frame
        # (zero at rest, tracked from the odometry while a segment runs)
        """Build the QUERY_ODOMETRY response payload."""
        v_left = self._v_left
        v_right = self._v_right
        return (
            p.put_i32(round(self._x * 1000))
            + p.put_i32(round(self._y * 1000))
            + p.put_i32(round(_wrap_deg(self._th) * 1000))
            + p.put_i16(int(v_left))
            + p.put_i16(int(v_right))
            + p.put_u8(self._state)
            + p.put_u16(self._faults)
            + p.put_u16(self._segments_done)
        )

    def _status_payload(self) -> bytes:
        """Build the QUERY_STATUS response payload."""
        uptime = int((time.monotonic() - self._t0) * 1000)
        last = self._events[-1] if self._events else {"code": p.EVENT_NONE, "aux": 0}
        return (
            p.put_u16((3 << 8) | 5)   # firmware parity: 3.5 (+ RELEASE_MOTORS)
            + p.put_u32(uptime)
            + p.put_u8(self._state)
            + p.put_u16(self._faults)
            + p.put_u8(self._limits)
            + p.put_u8(0)
            + p.put_u8(self._last_seq)
            + p.put_u8(self._last_status)
            + p.put_u8(last["code"])
            + p.put_u8(last["aux"])
            + p.put_u8(p.EVENT_RESET_POR)
            + p.put_u8(robot_config.MAP_VERSION)
            + p.put_u8(1 if self._cfg_dirty else 0)
            + p.put_u8(self._gyro_freshness_byte())
        )

    def _gyro_freshness_byte(self) -> int:
        """1 while a recent SET_GYRO frame is being simulated (firmware parity)."""
        if self.gyro_frames == 0:
            return 0
        return 1 if (time.monotonic() - self._last_gyro_t) <= 0.2 else 0

    @staticmethod
    def _respond(seq: int, status: int, payload: bytes) -> bytes:
        """Wrap a payload into a CRC-protected 32-byte response."""
        body = bytes([p.SOF_RESPONSE, seq & 0xFF, status & 0xFF, len(payload)]) + payload.ljust(
            p.RSP_PAYLOAD_SIZE, b"\x00"
        )
        return body + bytes([p.crc8(body)])
