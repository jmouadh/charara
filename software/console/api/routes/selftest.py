"""Self-test routes: acceptance tests, readiness checks and drift audits.

This is what the console's "Acceptance & readiness" card calls:

* ``POST /selftest/square``      - the GOTO square (STM_docs/06 section 2.5)
* ``POST /selftest/arc``         - quarter arcs left and right (section 2.6)
* ``POST /selftest/chain``       - flat arc -> quarter arc -> flat arc, blended
* ``POST /selftest/rest-drift``  - stationary drill of every pose source
* ``POST /selftest/turn-repeat`` - N turns per direction, error statistics
* ``POST /selftest/loop-drift``  - repeated closed loops, per-source drift
* ``POST /selftest/speed-ramp``  - straight run, measured speed trace
* ``POST /selftest/encoders``    - encoder direction/scale check
* ``POST /selftest/fault-drill`` - guided deadman / e-stop safety drills
* ``POST /selftest/readiness``   - pre-match checklist (read-only)

Every motion test refuses while a mission executes, refuses with latched
faults, arms the board deadman while it polls (a killed request can never
leave the robot driving) and returns a result document with ``passed``, the
numbers that produced it and the thresholds used. The mock transport runs
the same simulated motions, so the whole card is rehearsable without
hardware - every result carries ``transport`` so a simulated pass is honest.

File:    software/console/api/routes/selftest.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-09
"""

from __future__ import annotations

import asyncio
import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

from fastapi import APIRouter, HTTPException

from software.core import diagnostics
from software.core.mission.executor import EXEC_DONE
from software.core.mission.interfaces import RobotIOError
from software.robot import protocol, robot_config

from ...runtime.state import state
from .deps import call, client, require_no_execution, require_no_faults
from .models import (
    ArcTestIn,
    ChainAuditIn,
    EncoderCheckIn,
    FaultDrillIn,
    LoopAuditIn,
    RestDriftIn,
    SpeedRampIn,
    SquareTestIn,
    TurnRepeatIn,
)

router = APIRouter()

#: acceptance windows (STM_docs/06 sections 2.5/2.6 and 7)
SQUARE_POS_TOL_MM = 20.0
SQUARE_HEAD_TOL_DEG = 2.0
ARC_POS_TOL_MM = 20.0
ARC_HEAD_TOL_DEG = 2.0
#: a "straight" leg of the chain audit is a very flat real ARC
CHAIN_FLAT_RADIUS_MM = 20000.0
#: rest-drift audit limits
REST_DRIFT_POS_MM = 2.0
REST_DRIFT_HEAD_DEG = 0.5
#: readiness: how close to the team start the robot must stand / rest time
READY_START_TOL_MM = 100.0
READY_START_TOL_DEG = 8.0
READY_REST_S = 30.0

#: turn-repeatability audit: windows on the per-direction heading error [deg]
TURN_MEAN_TOL_DEG = 3.0
TURN_SIGMA_TOL_DEG = 1.5
#: closed-loop drift audit: windows on the fused drift per loop [mm, deg]
LOOP_DRIFT_POS_MM = 60.0
LOOP_DRIFT_HEAD_DEG = 4.0
#: speed/ramp audit: peak within this fraction of the command, and distance
RAMP_PEAK_TOL_FRAC = 0.15
RAMP_DIST_TOL_MM = 60.0
#: encoder check: tick-count ratio window and the left/right match window
ENC_RATIO_MIN = 0.7
ENC_RATIO_MAX = 1.3
ENC_SIDE_MATCH_FRAC = 0.30
#: encoder fallback when the board config cannot be read [mm per tick]
ENC_NOMINAL_MM_PER_TICK = 0.05
#: deadman drill: extra silence after the watchdog window before reading [s]
DRILL_EXTRA_S = 0.8

#: polling while a commanded move runs (also refreshes the armed deadman)
POLL_S = 0.05
#: polling of the rest-drift audit
DRIFT_POLL_S = 0.1
#: deadman armed by the motion tests [ms]
TEST_DEADMAN_MS = 1500
#: pause after a move so the fusion settles on the stopped pose [s]
SETTLE_S = 0.3
#: the GOTO payload is signed 16-bit in 0.1 mm units - this is its window
BOARD_COORD_LIMIT_MM = 3276.0


def _clamp(value: float, low: float, high: float) -> float:
    """Clamp a numeric test parameter into its allowed window."""
    return max(low, min(high, float(value)))


def _pose() -> Optional[Tuple[float, float, float]]:
    """The fused pose (x_mm, y_mm, theta_deg), or None before telemetry."""
    pose = state.pose()
    if pose is None:
        return None
    return float(pose[0]), float(pose[1]), float(pose[2])


def _round_pose(pose: Sequence[float]) -> List[float]:
    """One pose rounded for the JSON result."""
    return [round(float(pose[0]), 1), round(float(pose[1]), 1), round(float(pose[2]), 1)]


def _move_timeout(distance_mm: float, speed_mm_s: int) -> float:
    """Watchdog for one commanded move: 3x its nominal time plus slack."""
    return max(12.0, float(distance_mm) / max(50, int(speed_mm_s)) * 3.0 + 8.0)


def _require_encodable(points: Sequence[Dict]) -> None:
    """Refuse test waypoints the 0.1 mm i16 wire format cannot carry.

    The GOTO payload is signed 16-bit in 0.1 mm units (+/-3276.7 mm); a test
    started near the field edge can leave that window, and a clear 409 beats
    a raw struct error deep in the codec.
    """
    for point in points:
        if (
            abs(float(point["x"])) > BOARD_COORD_LIMIT_MM
            or abs(float(point["y"])) > BOARD_COORD_LIMIT_MM
        ):
            raise HTTPException(
                status_code=409,
                detail="the test would leave the board's coordinate range (+/-3276 mm) - "
                "re-place the robot or reduce the size",
            )


async def _await_motion(link, timeout_s: float) -> Tuple[str, Optional[object]]:
    """Poll until the board is idle: ("done"|"fault"|"timeout", odometry)."""
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            odometry = await link.odometry()
        except RobotIOError as exc:
            raise HTTPException(status_code=503, detail=f"link lost while the test ran: {exc}") from exc
        if odometry.faults != 0:
            return "fault", odometry
        if odometry.is_idle:
            return "done", odometry
        if time.monotonic() > deadline:
            return "timeout", None
        await asyncio.sleep(POLL_S)


async def _clear_deadman(link) -> None:
    """Disarm the test deadman; never raises (cleanup path)."""
    try:
        await link.set_deadman(0)
    except Exception:
        pass


def _motion_failure(status: str, odometry) -> str:
    """Human reason for a move that ended in fault or timeout."""
    if status == "fault" and odometry is not None:
        names = ", ".join(odometry.describe_faults()) or "unknown"
        return f"board fault: {names} - press Clear Faults and retry"
    return "the move did not finish within its watchdog - check the board log"


def _result(
    test: str,
    passed: bool,
    reason: str,
    duration_s: float,
    metrics: Dict,
    thresholds: Dict,
) -> Dict:
    """Assemble the JSON result document every self-test returns."""
    return {
        "ok": True,
        "test": test,
        "passed": bool(passed),
        "reason": reason,
        "transport": state.transport,
        "duration_s": round(max(0.0, duration_s), 1),
        "thresholds": thresholds,
        "metrics": metrics,
    }


def _motion_failed(
    test: str, status: str, odometry, start: Sequence[float], t0: float, thresholds: Dict
) -> Dict:
    """A failed motion-test result (fault or timeout mid-run)."""
    return _result(
        test,
        False,
        _motion_failure(status, odometry),
        time.monotonic() - t0,
        {"start": _round_pose(start)},
        thresholds,
    )


@router.post("/selftest/square")
async def square_test(body: SquareTestIn) -> Dict:
    """GOTO square acceptance test (STM_docs/06 section 2.5).

    Runs the bench square from the CURRENT pose (four absolute GOTOs, final
    heading = start + 270 deg like the bench procedure) and evaluates the
    closing position/heading of the fused pose against 20 mm / 2 deg.
    Odometry-based: a tape measurement remains the absolute reference.
    """
    require_no_execution()
    await require_no_faults()
    link = client()
    side = _clamp(body.side_mm, 100.0, 1200.0)
    speed = int(_clamp(body.speed_mm_s, 50.0, 800.0))
    start = _pose()
    if start is None:
        raise HTTPException(status_code=409, detail="no pose yet - wait for telemetry")
    waypoints = diagnostics.square_waypoints(start[0], start[1], start[2], side)
    _require_encodable(waypoints)
    thresholds = {"pos_mm": SQUARE_POS_TOL_MM, "heading_deg": SQUARE_HEAD_TOL_DEG}
    t0 = time.monotonic()
    await call(lambda: link.set_deadman(TEST_DEADMAN_MS))
    try:
        for waypoint in waypoints:
            await call(lambda w=waypoint: link.goto(w["x"], w["y"], w["th"], speed))
            status, odometry = await _await_motion(link, _move_timeout(side, speed))
            if status != "done":
                return _motion_failed("square", status, odometry, start, t0, thresholds)
        await asyncio.sleep(SETTLE_S)
    finally:
        await _clear_deadman(link)
    end = _pose() or start
    expected = (start[0], start[1], diagnostics.wrap180(start[2] + 270.0))
    error = diagnostics.pose_error(end, expected)
    passed = error["pos_mm"] <= SQUARE_POS_TOL_MM and error["heading_deg"] <= SQUARE_HEAD_TOL_DEG
    metrics = {
        "start": _round_pose(start),
        "end": _round_pose(end),
        "position_error_mm": round(error["pos_mm"], 1),
        "heading_error_deg": round(error["heading_deg"], 2),
        "side_mm": round(side, 1),
        "speed_mm_s": speed,
    }
    board = state.odometry
    if board is not None:
        board_error = diagnostics.pose_error((board.x_mm, board.y_mm, board.theta_deg), expected)
        metrics["board_position_error_mm"] = round(board_error["pos_mm"], 1)
        metrics["board_heading_error_deg"] = round(board_error["heading_deg"], 2)
    reason = "" if passed else "outside 20 mm / 2 deg - re-run the drive/turn calibration and check for slip"
    await state.log(
        "info" if passed else "warn",
        f"self-test square: {'PASS' if passed else 'FAIL'} "
        f"({metrics['position_error_mm']} mm / {metrics['heading_error_deg']} deg)",
    )
    return _result("square", passed, reason, time.monotonic() - t0, metrics, thresholds)


@router.post("/selftest/arc")
async def arc_test(body: ArcTestIn) -> Dict:
    """Quarter-arc acceptance test (STM_docs/06 section 2.6).

    One left quarter arc, then one right quarter arc. Each is evaluated from
    the pose measured immediately BEFORE it (the arc's own start), so an
    absolute offset cannot hide a control error. PASS: both end poses within
    20 mm / 2 deg of the exact circle geometry.
    """
    require_no_execution()
    await require_no_faults()
    link = client()
    radius = _clamp(body.radius_mm, 100.0, 600.0)
    speed = int(_clamp(body.speed_mm_s, 50.0, 800.0))
    thresholds = {"pos_mm": ARC_POS_TOL_MM, "heading_deg": ARC_HEAD_TOL_DEG}
    t0 = time.monotonic()
    records: List[Dict] = []
    await call(lambda: link.set_deadman(TEST_DEADMAN_MS))
    try:
        for name, signed_radius, sweep in (("left", radius, 90.0), ("right", -radius, -90.0)):
            before = _pose()
            if before is None:
                raise HTTPException(status_code=409, detail="no pose yet - wait for telemetry")
            await call(lambda r=signed_radius, s=sweep: link.arc(r, s, speed))
            arc_length = abs(math.radians(sweep)) * radius
            status, odometry = await _await_motion(link, _move_timeout(arc_length, speed))
            if status != "done":
                records.append(
                    {"name": name, "passed": False, "reason": _motion_failure(status, odometry)}
                )
                return _result(
                    "arc",
                    False,
                    f"{name} arc: {records[-1]['reason']}",
                    time.monotonic() - t0,
                    {"arcs": records},
                    thresholds,
                )
            await asyncio.sleep(SETTLE_S)
            after = _pose() or before
            expected = diagnostics.arc_end_pose(before[0], before[1], before[2], signed_radius, sweep)
            error = diagnostics.pose_error(after, expected)
            ok = error["pos_mm"] <= ARC_POS_TOL_MM and error["heading_deg"] <= ARC_HEAD_TOL_DEG
            records.append(
                {
                    "name": name,
                    "passed": ok,
                    "start": _round_pose(before),
                    "end": _round_pose(after),
                    "expected": _round_pose(expected),
                    "position_error_mm": round(error["pos_mm"], 1),
                    "heading_error_deg": round(error["heading_deg"], 2),
                }
            )
    finally:
        await _clear_deadman(link)
    failed = [record["name"] for record in records if not record["passed"]]
    passed = not failed
    reason = "" if passed else f"{', '.join(failed)} arc outside 20 mm / 2 deg"
    await state.log("info" if passed else "warn", f"self-test arc: {'PASS' if passed else 'FAIL'}")
    return _result(
        "arc",
        passed,
        reason,
        time.monotonic() - t0,
        {"radius_mm": round(radius, 1), "speed_mm_s": speed, "arcs": records},
        thresholds,
    )


@router.post("/selftest/chain")
async def chain_test(body: ChainAuditIn) -> Dict:
    """QUEUE_ARC chain audit: flat arc -> quarter arc -> flat arc, blended.

    Runs three tangent arcs through the standard executor (the console log
    gets the usual "run finished: N segment(s), M blended" summary, the
    progress pill shows the run). A working chain shows NO board idle time
    between the segments - one acceleration, one deceleration. PASS: a
    clean completion with zero junction stops; the closing error against the
    exact arc geometry is reported for information.
    """
    require_no_execution()
    await require_no_faults()
    link = client()
    executor = state.executor
    assert executor is not None
    length = _clamp(body.length_mm, 100.0, 800.0)
    radius = _clamp(body.radius_mm, 150.0, 600.0)
    speed = int(_clamp(body.speed_mm_s, 50.0, 800.0))
    start = _pose()
    if start is None:
        raise HTTPException(status_code=409, detail="no pose yet - wait for telemetry")
    sweep = diagnostics.flat_sweep_deg(length, CHAIN_FLAT_RADIUS_MM)
    segments = [
        {"kind": "arc", "radius_mm": CHAIN_FLAT_RADIUS_MM, "sweep_deg": sweep, "v": speed},
        {"kind": "arc", "radius_mm": radius, "sweep_deg": 90.0, "v": speed},
        {"kind": "arc", "radius_mm": CHAIN_FLAT_RADIUS_MM, "sweep_deg": sweep, "v": speed},
    ]
    total_mm = 2.0 * length + math.radians(90.0) * radius
    thresholds = {"junction_stops": 0}
    t0 = time.monotonic()
    try:
        await executor.start(segments)
    except (RuntimeError, ValueError) as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    stops: List[float] = []
    idle_since: Optional[float] = None
    moving = False
    max_speed = 0.0
    task = asyncio.create_task(executor.wait())
    deadline = t0 + _move_timeout(total_mm, speed) * 1.5
    try:
        while not task.done() and time.monotonic() < deadline:
            try:
                odometry = await link.odometry()
            except RobotIOError as exc:
                await executor.stop(reason="self-test: link lost")
                raise HTTPException(status_code=503, detail=f"link lost while the test ran: {exc}") from exc
            max_speed = max(max_speed, abs(odometry.v_left_mm_s), abs(odometry.v_right_mm_s))
            if odometry.is_idle:
                if moving and idle_since is None:
                    idle_since = time.monotonic()
            else:
                moving = True
                if idle_since is not None:
                    stops.append(round(time.monotonic() - idle_since, 2))
                    idle_since = None
            await asyncio.sleep(POLL_S)
        if not task.done():
            await executor.stop(reason="self-test: chain watchdog expired")
            await task
    finally:
        if not task.done():
            task.cancel()
    await asyncio.sleep(SETTLE_S)
    end = _pose() or start
    expected: Tuple[float, float, float] = start
    for segment in segments:
        expected = diagnostics.arc_end_pose(
            expected[0], expected[1], expected[2], segment["radius_mm"], segment["sweep_deg"]
        )
    closing = diagnostics.pose_error(end, expected)["pos_mm"]
    state_ok = executor.state == EXEC_DONE
    passed = state_ok and not stops
    if not state_ok:
        reason = f"the run did not complete cleanly ({executor.progress['message'] or executor.state})"
    elif stops:
        reason = (
            f"{len(stops)} junction stop(s) detected - QUEUE_ARC chaining did not engage "
            "(check the board log)"
        )
    else:
        reason = ""
    metrics = {
        "segments": len(segments),
        "junction_stops": stops,
        "stop_count": len(stops),
        "closing_error_mm": round(closing, 1),
        "max_speed_mm_s": round(max_speed, 1),
        "end": _round_pose(end),
        "expected_end": _round_pose(expected),
        "length_mm": round(length, 1),
        "radius_mm": round(radius, 1),
        "speed_mm_s": speed,
    }
    await state.log(
        "info" if passed else "warn",
        f"self-test chain: {'PASS' if passed else 'FAIL'} - {len(stops)} junction stop(s)",
    )
    return _result("chain", passed, reason, time.monotonic() - t0, metrics, thresholds)


@router.post("/selftest/rest-drift")
async def rest_drift(body: RestDriftIn) -> Dict:
    """Stationary drift audit: board/fused pose, encoder ticks, gyro noise.

    Keep the robot standing still (wheels on the floor, nobody touching the
    table) for the whole window; the audit samples the board odometry, the
    fused pose, the raw encoder accumulators and the gyro rate, then reports
    how much each source moved. PASS: board pose <= 2 mm / 0.5 deg, no
    motion observed, no fault.
    """
    require_no_execution()
    await require_no_faults()
    link = client()
    seconds = _clamp(body.seconds, 3.0, 120.0)
    session = state.session
    thresholds = {"pos_mm": REST_DRIFT_POS_MM, "heading_deg": REST_DRIFT_HEAD_DEG}
    board_samples: List[Tuple[float, float, float, float]] = []
    fused_samples: List[Tuple[float, float, float, float]] = []
    gyro_values: List[float] = []
    tick_noise = 0
    previous_ticks: Optional[Tuple[int, int]] = None
    moved = False
    faults = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        elapsed = time.monotonic() - t0
        try:
            odometry = await link.odometry()
        except RobotIOError as exc:
            raise HTTPException(status_code=503, detail=f"link lost during the audit: {exc}") from exc
        faults = odometry.faults
        if faults != 0:
            break
        if (
            not odometry.is_idle
            or abs(odometry.v_left_mm_s) > 1.0
            or abs(odometry.v_right_mm_s) > 1.0
        ):
            moved = True
        board_samples.append(
            (elapsed, float(odometry.x_mm), float(odometry.y_mm), float(odometry.theta_deg))
        )
        pose = _pose()
        if pose is not None:
            fused_samples.append((elapsed, pose[0], pose[1], pose[2]))
        if session.imu is not None and session.imu_state == "ready":
            gyro_values.append(float(session.imu_rate_deg_s))
        try:
            ticks = await link.ticks()
        except RobotIOError:
            ticks = None
        if ticks is not None:
            if previous_ticks is not None:
                tick_noise = max(
                    tick_noise,
                    abs(ticks[0] - previous_ticks[0]),
                    abs(ticks[1] - previous_ticks[1]),
                )
            previous_ticks = ticks
        await asyncio.sleep(DRIFT_POLL_S)
    board = diagnostics.drift_stats(board_samples)
    fused = diagnostics.drift_stats(fused_samples)
    gyro = diagnostics.series_stats(gyro_values)
    if faults != 0:
        passed = False
        names = ", ".join(state.odometry.describe_faults()) if state.odometry else "unknown"
        reason = f"board fault: {names} - press Clear Faults and retry"
    elif moved:
        passed = False
        reason = "the board reported motion during the audit - did someone touch the robot?"
    elif board["max_pos_mm"] > REST_DRIFT_POS_MM or board["max_heading_deg"] > REST_DRIFT_HEAD_DEG:
        passed = False
        reason = "the board pose moved while standing still - check the wheel springs, the floor and the encoders"
    else:
        passed = True
        reason = ""
    metrics = {
        "seconds": round(time.monotonic() - t0, 1),
        "board_max_pos_mm": board["max_pos_mm"],
        "board_max_heading_deg": board["max_heading_deg"],
        "fused_max_pos_mm": fused["max_pos_mm"],
        "fused_max_heading_deg": fused["max_heading_deg"],
        "gyro_mean_deg_s": round(gyro["mean"], 3),
        "gyro_sigma_deg_s": round(gyro["sigma"], 3),
        "gyro_samples": gyro["count"],
        "tick_noise_counts": tick_noise,
    }
    await state.log(
        "info" if passed else "warn",
        f"self-test rest-drift: {'PASS' if passed else 'FAIL'} "
        f"(board drift {board['max_pos_mm']} mm over {metrics['seconds']} s)",
    )
    return _result("rest-drift", passed, reason, time.monotonic() - t0, metrics, thresholds)


async def _recent_log_verdict(link) -> Tuple[str, str]:
    """(status, detail) for the recent board event log (last ~15 entries).

    ERROR-severity entries fail (stall, timeout, limit, flash...): they are
    never acceptable before a match. WARN entries (I2C recoveries, busy
    rejections) are surfaced but do not fail - the known noisy-bus entries.
    An unreadable log is a warning, not a failure.
    """
    try:
        head = await link.events(0)
        after = max(0, int(head["latest"]) - 15)
        names_warn: List[str] = []
        names_error: List[str] = []
        seen = 0
        for _ in range(4):
            page = await link.events(after)
            page_events = page["events"]
            if not page_events:
                break
            for event in page_events:
                name = protocol.event_name(event["code"])
                severity = protocol.event_severity(event["code"])
                if severity == "error":
                    names_error.append(name)
                elif severity == "warn":
                    names_warn.append(name)
                seen += 1
            after = int(page_events[-1]["index"])
            if after >= int(page["latest"]):
                break
    except Exception:
        return "warn", "could not read the board event log"
    if names_error:
        return "fail", "recent: " + ", ".join(sorted(set(names_error)))
    if names_warn:
        return "warn", "recent warnings: " + ", ".join(sorted(set(names_warn)))
    return "pass", f"clean (last {seen} entries)"


@router.post("/selftest/readiness")
async def readiness() -> Dict:
    """Pre-match readiness checklist (read-only, also runs on MOCK).

    Checks the link and firmware, latched faults, the recent board log, the
    saved-settings flag, the IMU (bias settled + stream active) and that the
    robot stands at its match start. Warnings do not fail the overall check;
    a FAIL means: do not start a match yet.
    """
    t0 = time.monotonic()
    checks: List[Dict] = []

    def add(name: str, status: str, detail: str) -> None:
        """Append one check row (names unique per call order)."""
        checks.append({"name": name, "status": status, "detail": detail})

    add(
        "mode",
        "pass" if state.transport == "i2c" else "fail",
        "REAL board link"
        if state.transport == "i2c"
        else "MOCK transport - readiness of a simulated robot is not meaningful",
    )
    add(
        "link",
        "pass" if state.connected else "fail",
        "board answered" if state.connected else (state.last_error or "no telemetry yet"),
    )
    board = None
    if state.client is not None:
        try:
            board = await state.client.status()
        except Exception:
            board = None
    if board is None:
        add("board", "fail", "board diagnostics unreachable")
    else:
        firmware_ok = (board.fw_major, board.fw_minor) >= (3, 3)
        add(
            "firmware",
            "pass" if firmware_ok else "warn",
            f"{board.fw_major}.{board.fw_minor}"
            + ("" if firmware_ok else " - the gyro stream needs 3.3"),
        )
        faults = int(board.faults)
        if state.odometry is not None:
            faults |= int(state.odometry.faults)
        names = ", ".join(protocol.decode_faults(faults))
        add("faults", "pass" if faults == 0 else "fail", "none" if faults == 0 else names)
        add(
            "settings saved",
            "pass" if not board.cfg_dirty else "fail",
            "flash record up to date"
            if not board.cfg_dirty
            else "board RAM has unsaved edits - press Save to flash",
        )
        status, detail = await _recent_log_verdict(state.client)
        add("board log", status, detail)
    imu = state.session.imu_status()
    stream = state.session.gyro_stream_status()
    if imu.get("state") == "ready":
        add("IMU", "pass", f"ready, yaw {imu.get('yaw_axis')}, {imu.get('samples')} samples")
        age = imu.get("age_s")
        add(
            "IMU fresh",
            "pass" if (age is not None and age <= 1.0) else "warn",
            "age %.2f s" % age if age is not None else "no samples yet",
        )
        rest = imu.get("rest_s")
        if rest is not None and rest >= READY_REST_S:
            add("IMU bias settled", "pass", "rest %.0f s - bias re-centred" % rest)
        else:
            add(
                "IMU bias settled",
                "warn",
                "rest %s s - keep the robot still so the bias refines" % (rest if rest is not None else "0"),
            )
        add(
            "gyro stream",
            "pass" if stream.get("state") == "active" else "warn",
            "active" if stream.get("state") == "active" else str(stream.get("state")),
        )
    else:
        add(
            "IMU",
            "warn",
            f"state {imu.get('state')} - heading falls back to wheels (allowed, less precise)",
        )
    pose = _pose()
    info = state.team_info()
    start = info["start"]
    if pose is None:
        add("start pose", "fail", "no pose yet - wait for telemetry")
    else:
        distance = math.hypot(pose[0] - start[0], pose[1] - start[1])
        heading = abs(diagnostics.wrap180(pose[2] - start[2]))
        in_place = distance <= READY_START_TOL_MM and heading <= READY_START_TOL_DEG
        add(
            "start pose",
            "pass" if in_place else "fail",
            f"{distance:.0f} mm / {heading:.1f} deg from the {info['label']} start"
            + ("" if in_place else " - place the robot and press Set Pose"),
        )
    failed = [check for check in checks if check["status"] == "fail"]
    warned = [check for check in checks if check["status"] == "warn"]
    passed = not failed
    summary = f"{len(checks) - len(failed) - len(warned)} pass, {len(warned)} warn, {len(failed)} fail"
    await state.log("info" if passed else "warn", f"self-test readiness: {'PASS' if passed else 'FAIL'} - {summary}")
    return {
        "ok": True,
        "test": "readiness",
        "passed": passed,
        "reason": "" if passed else "readiness checks failed",
        "transport": state.transport,
        "t": time.strftime("%Y-%m-%d %H:%M:%S"),
        "duration_s": round(time.monotonic() - t0, 1),
        "summary": summary,
        "checks": checks,
    }


async def _mm_per_tick(link) -> float:
    """Encoder conversion from the board's own calibration (fallback 0.05).

    The board's odometry calibration (wheel circumference / ticks per rev)
    is the truth for its tick counters; when the config read fails the
    nominal 200 mm / 4000 ticks value is used.
    """
    try:
        values = await link.get_all_config()
        circumference = robot_config.REGISTER_BY_KEY["odom_wheel_circumference_mm"]
        ticks = robot_config.REGISTER_BY_KEY["odom_ticks_per_rev"]
        span = ticks.decode(values[ticks.index])
        if span <= 0.0:
            return ENC_NOMINAL_MM_PER_TICK
        return max(0.001, circumference.decode(values[circumference.index]) / span)
    except Exception:
        return ENC_NOMINAL_MM_PER_TICK


@router.post("/selftest/turn-repeat")
async def turn_repeat(body: TurnRepeatIn) -> Dict:
    """Turn-repeatability audit: N turns per direction, error statistics.

    Turns ``count_per_dir`` times by +angle (CCW) then the same by -angle
    (CW); every turn is measured from the pose JUST BEFORE it, for the fused
    pose and for the board odometry. Reports mean / sigma / max of the
    heading error per direction, plus the wheel-vs-fused divergence during
    the turn (wheel slip / scrub: the board heading is wheel-driven, the
    fused heading leans on the gyro). PASS: |mean| <= 3 deg and sigma
    <= 1.5 deg in BOTH directions (wheels on the floor, hands off).
    """
    require_no_execution()
    await require_no_faults()
    link = client()
    count = int(_clamp(body.count_per_dir, 1, 5))
    angle = _clamp(body.angle_deg, 15.0, 180.0)
    speed = int(_clamp(body.speed_mm_s, 50.0, 400.0))
    thresholds = {"mean_deg": TURN_MEAN_TOL_DEG, "sigma_deg": TURN_SIGMA_TOL_DEG}
    t0 = time.monotonic()
    directions: List[Dict] = []
    await call(lambda: link.set_deadman(TEST_DEADMAN_MS))
    try:
        for direction, label in ((1.0, "ccw"), (-1.0, "cw")):
            fused_errors: List[float] = []
            board_errors: List[float] = []
            scrub_errors: List[float] = []
            for _ in range(count):
                before_fused = _pose()
                if before_fused is None:
                    raise HTTPException(status_code=409, detail="no pose yet - wait for telemetry")
                before_board = await call(lambda: link.odometry())
                await call(lambda: link.turn(direction * angle, speed))
                status, odometry = await _await_motion(link, _move_timeout(2.0 * angle, speed))
                if status != "done":
                    return _motion_failed("turn-repeat", status, odometry, before_fused, t0, thresholds)
                await asyncio.sleep(SETTLE_S)
                after_fused = _pose() or before_fused
                after_board = await call(lambda: link.odometry())
                expected = diagnostics.wrap180(before_fused[2] + direction * angle)
                fused_errors.append(diagnostics.wrap180(after_fused[2] - expected))
                expected_board = diagnostics.wrap180(before_board.theta_deg + direction * angle)
                board_errors.append(diagnostics.wrap180(after_board.theta_deg - expected_board))
                scrub_errors.append(
                    abs(
                        diagnostics.wrap180(
                            (after_board.theta_deg - before_board.theta_deg)
                            - (after_fused[2] - before_fused[2])
                        )
                    )
                )
            fused = diagnostics.series_stats(fused_errors)
            board = diagnostics.series_stats(board_errors)
            scrub = diagnostics.series_stats(scrub_errors)
            ok = abs(fused["mean"]) <= TURN_MEAN_TOL_DEG and fused["sigma"] <= TURN_SIGMA_TOL_DEG
            directions.append(
                {
                    "direction": label,
                    "passed": ok,
                    "samples": count,
                    "fused_mean_deg": round(fused["mean"], 2),
                    "fused_sigma_deg": round(fused["sigma"], 2),
                    "fused_max_deg": round(max(abs(value) for value in fused_errors), 2),
                    "board_mean_deg": round(board["mean"], 2),
                    "board_sigma_deg": round(board["sigma"], 2),
                    "scrub_mean_deg": round(scrub["mean"], 2),
                }
            )
    finally:
        await _clear_deadman(link)
    failed = [record["direction"] for record in directions if not record["passed"]]
    passed = not failed
    reason = "" if passed else f"{', '.join(failed)} outside mean 3 deg / sigma 1.5 deg - check slip and the turn gain"
    metrics = {
        "angle_deg": round(angle, 1),
        "speed_mm_s": speed,
        "count_per_dir": count,
        "directions": directions,
    }
    await state.log(
        "info" if passed else "warn", f"self-test turn-repeat: {'PASS' if passed else 'FAIL'}"
    )
    return _result("turn-repeat", passed, reason, time.monotonic() - t0, metrics, thresholds)


@router.post("/selftest/loop-drift")
async def loop_drift(body: LoopAuditIn) -> Dict:
    """Closed-loop drift audit: repeated square loops, per-source drift.

    Drives the closed square loop (the GOTO square of the acceptance test)
    ``loops`` times WITHOUT correcting between rounds and compares, after
    every loop, where each source thinks the robot stands versus the start:

    * the BOARD drift is the control + wheel-slip error (open loop),
    * the FUSED drift is what the estimator believes - the difference
      between the two is the estimation divergence the camera re-anchor
      exists to remove,
    * the tick-implied distance versus the geometric loop length is the
      wheel-scale check.

    PASS: last-loop fused drift within 60 mm / 4 deg, no fault.
    """
    require_no_execution()
    await require_no_faults()
    link = client()
    loops = int(_clamp(body.loops, 1, 4))
    side = _clamp(body.side_mm, 200.0, 1200.0)
    speed = int(_clamp(body.speed_mm_s, 50.0, 400.0))
    start = _pose()
    if start is None:
        raise HTTPException(status_code=409, detail="no pose yet - wait for telemetry")
    waypoints = diagnostics.square_waypoints(start[0], start[1], start[2], side)
    _require_encodable(waypoints)
    # the bench square ends on the start POSITION rotated +270 deg (its own
    # heading sequence: theta, +90, +180, +270) - the closure reference
    expected_end = (start[0], start[1], diagnostics.wrap180(start[2] + 270.0))
    mm_per_tick = await _mm_per_tick(link)
    loop_geometry_mm = 4.0 * side
    ticks0 = await call(lambda: link.ticks())
    thresholds = {"pos_mm": LOOP_DRIFT_POS_MM, "heading_deg": LOOP_DRIFT_HEAD_DEG}
    records: List[Dict] = []
    t0 = time.monotonic()
    await call(lambda: link.set_deadman(TEST_DEADMAN_MS))
    try:
        for loop in range(1, loops + 1):
            for waypoint in waypoints:
                await call(lambda w=waypoint: link.goto(w["x"], w["y"], w["th"], speed))
                status, odometry = await _await_motion(link, _move_timeout(side, speed))
                if status != "done":
                    return _motion_failed("loop-drift", status, odometry, start, t0, thresholds)
            await asyncio.sleep(SETTLE_S)
            fused = _pose() or start
            board = await call(lambda: link.odometry())
            ticks = await call(lambda: link.ticks())
            fused_error = diagnostics.pose_error(fused, expected_end)
            board_error = diagnostics.pose_error(
                (board.x_mm, board.y_mm, board.theta_deg), expected_end
            )
            # the mean of both counters cancels the turn differential: it is
            # the CENTRE distance travelled, comparable to the loop length
            walked_mm = 0.5 * abs((ticks[0] - ticks0[0]) + (ticks[1] - ticks0[1])) * mm_per_tick
            records.append(
                {
                    "loop": loop,
                    "fused_pos_mm": round(fused_error["pos_mm"], 1),
                    "fused_heading_deg": round(fused_error["heading_deg"], 2),
                    "board_pos_mm": round(board_error["pos_mm"], 1),
                    "board_heading_deg": round(board_error["heading_deg"], 2),
                    "tick_distance_mm": round(walked_mm, 1),
                    "tick_ratio": round(walked_mm / (loop * loop_geometry_mm), 3),
                }
            )
    finally:
        await _clear_deadman(link)
    last = records[-1] if records else None
    passed = (
        last is not None
        and last["fused_pos_mm"] <= LOOP_DRIFT_POS_MM
        and last["fused_heading_deg"] <= LOOP_DRIFT_HEAD_DEG
    )
    reason = (
        ""
        if passed
        else "the fused pose drifted beyond 60 mm / 4 deg over the loops - check slip, wheel springs and the odometry calibration"
    )
    metrics = {
        "loops": loops,
        "side_mm": round(side, 1),
        "speed_mm_s": speed,
        "loop_length_mm": round(loop_geometry_mm, 1),
        "mm_per_tick": round(mm_per_tick, 4),
        "records": records,
    }
    await state.log("info" if passed else "warn", f"self-test loop-drift: {'PASS' if passed else 'FAIL'}")
    return _result("loop-drift", passed, reason, time.monotonic() - t0, metrics, thresholds)


@router.post("/selftest/speed-ramp")
async def speed_ramp(body: SpeedRampIn) -> Dict:
    """Speed/ramp audit: one long straight at cruise, sampled while it runs.

    Reports the peak and the cruise speed the board really achieved (from
    its measured wheel speeds), the time to reach 95 % of the command, the
    driven distance against the command (board and fused) and the sample
    count. PASS: peak within +/-15 % of the commanded cruise and the board
    distance within 60 mm.
    """
    require_no_execution()
    await require_no_faults()
    link = client()
    distance = _clamp(body.distance_mm, 300.0, 2500.0)
    speed = int(_clamp(body.speed_mm_s, 100.0, 800.0))
    start_board = await call(lambda: link.odometry())
    start_fused = _pose()
    thresholds = {"peak_frac": RAMP_PEAK_TOL_FRAC, "distance_mm": RAMP_DIST_TOL_MM}
    t0 = time.monotonic()
    samples: List[Tuple[float, float]] = []
    await call(lambda: link.set_deadman(TEST_DEADMAN_MS))
    try:
        await call(lambda: link.drive(distance, speed))
        deadline = time.monotonic() + _move_timeout(distance, speed)
        while True:
            odometry = await call(lambda: link.odometry())
            if odometry.faults != 0:
                return _motion_failed("speed-ramp", "fault", odometry, start_board, t0, thresholds)
            elapsed = time.monotonic() - t0
            samples.append((elapsed, 0.5 * (odometry.v_left_mm_s + odometry.v_right_mm_s)))
            if odometry.is_idle:
                break
            if time.monotonic() > deadline:
                return _motion_failed("speed-ramp", "timeout", None, start_board, t0, thresholds)
            await asyncio.sleep(POLL_S)
        await asyncio.sleep(SETTLE_S)
        end_board = await call(lambda: link.odometry())
    finally:
        await _clear_deadman(link)
    end_fused = _pose()
    moving = [value for _t, value in samples if abs(value) > 1.0]
    peak = max(moving) if moving else 0.0
    cruise_values = [value for value in moving if value >= 0.9 * peak]
    cruise = sum(cruise_values) / len(cruise_values) if cruise_values else 0.0
    t95 = next((round(t, 2) for t, value in samples if value >= 0.95 * speed), None)
    board_distance = math.hypot(end_board.x_mm - start_board.x_mm, end_board.y_mm - start_board.y_mm)
    fused_distance = None
    if start_fused is not None and end_fused is not None:
        fused_distance = math.hypot(end_fused[0] - start_fused[0], end_fused[1] - start_fused[1])
    distance_error = board_distance - distance
    peak_ok = abs(peak - speed) <= RAMP_PEAK_TOL_FRAC * speed
    passed = peak_ok and abs(distance_error) <= RAMP_DIST_TOL_MM
    if passed:
        reason = ""
    elif not peak_ok:
        reason = "the achieved speed left the +/-15 % window - check the board speed limits"
    else:
        reason = "the driven distance left the 60 mm window - check wheel slip and the odometry calibration"
    metrics = {
        "commanded_speed_mm_s": speed,
        "peak_speed_mm_s": round(peak, 1),
        "cruise_speed_mm_s": round(cruise, 1),
        "time_to_95pct_s": t95,
        "commanded_distance_mm": round(distance, 1),
        "board_distance_mm": round(board_distance, 1),
        "distance_error_mm": round(distance_error, 1),
        "fused_distance_mm": round(fused_distance, 1) if fused_distance is not None else None,
        "samples": len(samples),
    }
    await state.log("info" if passed else "warn", f"self-test speed-ramp: {'PASS' if passed else 'FAIL'}")
    return _result("speed-ramp", passed, reason, time.monotonic() - t0, metrics, thresholds)


@router.post("/selftest/encoders")
async def encoder_check(body: EncoderCheckIn) -> Dict:
    """Encoder direction/scale check: forward, backward and a CCW turn.

    Drives +d, then -d, then turns +90 deg, reading the raw tick
    accumulators after every move. Checks per encoder (using the board's
    own odom calibration): tick count matches the commanded distance
    (ratio 0.7..1.3), forward and backward counters run OPPOSITE, the CCW
    turn runs the left counter negative / right positive, and the two turn
    magnitudes match within 30 %. PASS: every check.
    """
    require_no_execution()
    await require_no_faults()
    link = client()
    distance = _clamp(body.distance_mm, 100.0, 800.0)
    speed = int(_clamp(body.speed_mm_s, 50.0, 400.0))
    mm_per_tick = await _mm_per_tick(link)
    expected = distance / mm_per_tick
    ticks: List[Tuple[int, int]] = []
    t0 = time.monotonic()
    thresholds = {
        "ratio_min": ENC_RATIO_MIN,
        "ratio_max": ENC_RATIO_MAX,
        "side_match_frac": ENC_SIDE_MATCH_FRAC,
    }
    await call(lambda: link.set_deadman(TEST_DEADMAN_MS))
    try:
        ticks.append(await call(lambda: link.ticks()))
        for label, action in (
            ("forward", lambda: link.drive(distance, speed)),
            ("backward", lambda: link.drive(-distance, speed)),
            ("turn", lambda: link.turn(90.0, speed)),
        ):
            await call(action)
            status, odometry = await _await_motion(link, _move_timeout(2.0 * distance, speed))
            if status != "done":
                return _motion_failed("encoders", status, odometry, (0.0, 0.0, 0.0), t0, thresholds)
            ticks.append(await call(lambda: link.ticks()))
    finally:
        await _clear_deadman(link)
    d_fwd = (ticks[1][0] - ticks[0][0], ticks[1][1] - ticks[0][1])
    d_back = (ticks[2][0] - ticks[1][0], ticks[2][1] - ticks[1][1])
    d_turn = (ticks[3][0] - ticks[2][0], ticks[3][1] - ticks[2][1])
    ratios = (abs(d_fwd[0]) / expected, abs(d_fwd[1]) / expected)
    turn_match = abs(abs(d_turn[0]) - abs(d_turn[1])) <= ENC_SIDE_MATCH_FRAC * max(
        abs(d_turn[0]), abs(d_turn[1]), 1
    )
    checks = {
        "forward scale left": ENC_RATIO_MIN <= ratios[0] <= ENC_RATIO_MAX,
        "forward scale right": ENC_RATIO_MIN <= ratios[1] <= ENC_RATIO_MAX,
        "forward agreement": d_fwd[0] * d_fwd[1] > 0,
        "backward direction": d_back[0] * d_fwd[0] < 0 and d_back[1] * d_fwd[1] < 0,
        "turn direction": d_turn[0] < 0 < d_turn[1],
        "turn magnitude match": turn_match,
    }
    passed = all(checks.values())
    failed = [name for name, ok in checks.items() if not ok]
    reason = "" if passed else f"failed: {', '.join(failed)} - check the wiring and encoder_invert_mask"
    metrics = {
        "distance_mm": round(distance, 1),
        "mm_per_tick": round(mm_per_tick, 4),
        "expected_ticks": round(expected, 1),
        "forward_ticks": [d_fwd[0], d_fwd[1]],
        "backward_ticks": [d_back[0], d_back[1]],
        "turn_ticks": [d_turn[0], d_turn[1]],
        "checks": checks,
    }
    await state.log("info" if passed else "warn", f"self-test encoders: {'PASS' if passed else 'FAIL'}")
    return _result("encoders", passed, reason, time.monotonic() - t0, metrics, thresholds)


@router.post("/selftest/fault-drill")
async def fault_drill(body: FaultDrillIn) -> Dict:
    """Guided safety drills: deadman expiry and e-stop rehearsal.

    ``kind="estop"`` (idle-only, no motion): latches the board e-stop,
    verifies the board reports the latch with the drivers off, clears it and
    verifies the recovery. ``kind="deadman"`` MOVES the robot: it arms the
    deadman, starts a slow creep that outlives the watchdog, SILENCES every
    board poller (telemetry AND the session's gyro stream - any received
    frame refreshes the watchdog, including silent SET_GYRO frames) and
    waits past the watchdog window without a single frame reaching the
    board - the board must stop the run BY ITSELF and latch DEADMAN. Run
    the deadman drill only with a clear floor.

    The drill clears the latched fault before returning so the robot stays
    usable; the verdict is in ``metrics`` either way.
    """
    require_no_execution()
    await require_no_faults()
    link = client()
    session = state.session
    kind = (body.kind or "").strip().lower()
    if kind not in ("deadman", "estop"):
        raise HTTPException(status_code=422, detail="kind must be 'deadman' or 'estop'")
    t0 = time.monotonic()
    if kind == "estop":
        thresholds = {"estop_latched": True, "recovered": True}
        await call(lambda: link.estop())
        await asyncio.sleep(0.4)
        latched = await call(lambda: link.odometry())
        latched_ok = bool(latched.faults & protocol.FAULT_ESTOP) and not latched.is_idle
        await call(lambda: link.clear_faults())
        await asyncio.sleep(0.2)
        after = await call(lambda: link.odometry())
        recovered = after.faults == 0 and after.is_idle
        passed = latched_ok and recovered
        reason = "" if passed else "the e-stop latch or the recovery did not behave as expected"
        metrics = {
            "kind": kind,
            "latched_faults": protocol.decode_faults(latched.faults),
            "recovered": recovered,
            "state_after": int(after.state),
        }
        await state.log("info" if passed else "warn", f"self-test fault-drill estop: {'PASS' if passed else 'FAIL'}")
        return _result("fault-drill", passed, reason, time.monotonic() - t0, metrics, thresholds)

    distance = _clamp(body.distance_mm, 100.0, 400.0)
    speed = int(_clamp(body.speed_mm_s, 60.0, 200.0))
    thresholds = {"deadman_latched": True}
    start = await call(lambda: link.odometry())
    await call(lambda: link.set_deadman(TEST_DEADMAN_MS))
    try:
        await call(lambda: link.drive(distance, speed))
        await asyncio.sleep(0.15)  # let the segment start before going quiet
        # silence EVERYTHING for the window: ANY received frame refreshes the
        # board watchdog, and the session's gyro stream sends SET_GYRO frames
        # at up to 50 Hz while the board moves. Telemetry AND the gyro
        # stream both stop; the drill restarts the stream afterwards (start
        # is idempotent and early-returns on the mock / without an IMU).
        session.pause_telemetry()
        await session.stop_gyro_stream()
        try:
            await asyncio.sleep(TEST_DEADMAN_MS / 1000.0 + DRILL_EXTRA_S)
            stopped = await call(lambda: link.odometry())
        finally:
            session.resume_telemetry()
            await session.start_gyro_stream()
            await _clear_deadman(link)
    finally:
        await call(lambda: link.clear_faults())
    latched_ok = bool(stopped.faults & protocol.FAULT_DEADMAN)
    displacement = math.hypot(stopped.x_mm - start.x_mm, stopped.y_mm - start.y_mm)
    quiet = abs(stopped.v_left_mm_s) <= 2.0 and abs(stopped.v_right_mm_s) <= 2.0
    passed = latched_ok and quiet
    if passed:
        reason = ""
    elif not latched_ok:
        reason = "the board did not latch DEADMAN - check the firmware deadman and that nothing else polls the link"
    else:
        reason = "the wheels were still turning after the deadman window"
    metrics = {
        "kind": "deadman",
        "deadman_ms": TEST_DEADMAN_MS,
        "commanded_distance_mm": round(distance, 1),
        "speed_mm_s": speed,
        "displacement_mm": round(displacement, 1),
        "stopped_short": displacement < distance - 10.0,
        "latched_faults": protocol.decode_faults(stopped.faults),
        "quiet": quiet,
    }
    await state.log("info" if passed else "warn", f"self-test fault-drill deadman: {'PASS' if passed else 'FAIL'}")
    return _result("fault-drill", passed, reason, time.monotonic() - t0, metrics, thresholds)
