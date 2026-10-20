"""Shared robot runtime session: the composition root both hosts wrap.

The console (``AppState``) and the brain (``Brain``) used to build the same
runtime twice: link, client, executor, localization service and telemetry
loop. This module owns that wiring exactly once, so the sensor work lands in
one place and the two hosts cannot diverge (Docs/09 section 5.1).

Responsibilities:

* link/client/executor construction and transport switching (i2c/mock);
* the localization service and the odometry observer feed;
* adopting the board's geometry the estimator depends on (odometry wheel
  track, settings register 7) at connect and after every settings write;
* the telemetry loop, connection state and lifecycle logging;
* the camera pose store + fusion ingestion (Docs/09 section 6);
* the MPU6050 sampler and the silent gyro forward stream (sections 7/9);
* the idle-only, camera-gated re-anchor corrections (section 7);
* optional host hooks: ``on_trace`` (I2C trace), ``logger`` (host log),
  ``on_sample`` (per successful odometry read - the console polls the board
  diagnostics and broadcasts telemetry there) and ``on_failure`` (once per
  online -> offline transition).

Every sensor is optional: an absent or failing IMU/camera only removes its
own contribution and the session behaves exactly like the wheel-only system
(Docs/09 section 10).

File:    software/session/session.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-05
"""

from __future__ import annotations

import asyncio
import math
import time
from typing import Awaitable, Callable, Dict, List, Optional, Tuple

from software.config import Settings
from software.core.localization import LocalizationService, WheelSample
from software.core.mission.executor import Executor
from software.core.mission.teams import resolve_start_pose
from software.core.world.poses import PoseStore
from software.robot import protocol as p
from software.robot import robot_config
from software.robot.client import RobotClient, RobotError
from software.robot.link import I2CLink, LinkError, MockLink
from software.robot.models import Odometry

#: host trace hook: called synchronously for every I2C transaction
TraceHook = Callable[[Dict], None]
#: host log hook: ``(level, message) -> awaitable``
Logger = Callable[[str, str], Awaitable[None]]
#: per-sample hook: called after every successful odometry read
SampleHook = Callable[[Odometry], Awaitable[None]]
#: failure hook: called once per online -> offline transition
FailureHook = Callable[[str], Awaitable[None]]
#: event sink used by the executor (console hub / brain emit)
EventSink = Callable[[Dict], Awaitable[None]]


def _wrap180(angle_deg: float) -> float:
    """Normalise an angle to (-180, 180] degrees."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


class RobotSession:
    """Owns the robot link, its services and the telemetry loop.

    One session = one board link. ``switch_transport`` rebuilds it safely
    (the new link is built first; a failure leaves the current one running).
    """

    #: consecutive IMU read failures before the sampler gives up (lost IMU)
    IMU_MAX_ERRORS = 25

    #: background link retry period [s] when the board is absent at startup
    CONNECT_RETRY_S = 5.0

    def __init__(self, settings: Settings, *, on_trace: Optional[TraceHook] = None) -> None:
        """Create the session; call :meth:`start` to open the link."""
        self.settings = settings
        self.transport = settings.transport  # active link: 'i2c' or 'mock'
        self.client: Optional[RobotClient] = None
        self.executor: Optional[Executor] = None
        self.localization = LocalizationService(
            track_mm=settings.odom_track_mm,
            sigma_gyro_deg_s=settings.gyro_sigma_deg_s,
            sigma_camera_xy_mm=settings.camera_sigma_xy_mm,
            sigma_camera_theta_deg=settings.camera_sigma_theta_deg,
            camera_gate_sigma=settings.camera_gate_sigma,
            camera_flip_deg=settings.camera_flip_deg,
        )
        self.poses = PoseStore()  # camera robot poses (us + opponent)
        self.odometry: Optional[Odometry] = None
        self.connected = False
        self.last_error = ""
        # host hooks (all optional; None = disabled)
        self.on_trace: Optional[TraceHook] = on_trace
        self.logger: Optional[Logger] = None
        self.on_sample: Optional[SampleHook] = None
        self.on_failure: Optional[FailureHook] = None
        self._event_sink: Optional[EventSink] = None
        self._telemetry: Optional[asyncio.Task] = None
        self._telemetry_paused = False  # fault drills: silence every poller
        self._connect_task: Optional[asyncio.Task] = None
        # optional MPU6050 (i2c transport only; absent is a normal state)
        self.imu = None
        self.imu_state = "off"  # off | ready | absent | lost | stopped
        self.imu_axis = "z"  # signed yaw axis in use, e.g. "z" or "-z"
        self.imu_mount: Optional[Dict] = None  # gravity mount detection
        self.imu_rate_deg_s = 0.0
        self.imu_bias_deg_s = 0.0
        self.imu_integrated_deg = 0.0  # bias-corrected yaw integral (spin check)
        self.imu_samples = 0
        self.imu_errors = 0
        self.imu_last_t: Optional[float] = None
        # live gyro scale: GYRO_SCALE seeds it, the console calibration block
        # can change it at runtime (applied by _imu_loop on the next sample)
        self.gyro_scale = float(settings.gyro_scale)
        # true while a blocking bias recalibration owns the IMU bus
        self._imu_calibrating = False
        # at-rest bias refinement state: the last "provably still" board pose
        # and when it started holding; None while any motion was seen
        self._rest_ref: Optional[Tuple[float, float, float]] = None
        self._rest_since: Optional[float] = None
        self._bias_refined_t: Optional[float] = None
        self._imu_task: Optional[asyncio.Task] = None
        # silent gyro forward stream (firmware >= 3.3 only)
        self.gyro_stream_state = "off"  # off | active | unsupported
        self.gyro_stream_frames = 0
        self.gyro_stream_errors = 0
        self._stream_task: Optional[asyncio.Task] = None
        # boundary re-anchor (SET_POSE at idle, camera-gated)
        self.reanchor_count = 0
        self.reanchor_state = "off"  # off | active
        self._last_reanchor_t = 0.0
        self._anchored_once = False
        self._reanchor_task: Optional[asyncio.Task] = None
        # match-start pose bootstrap (once per process; see the method)
        self._start_pose_done = False

    # ------------------------------------------------------------ plumbing
    def set_event_sink(self, sink: Optional[EventSink]) -> None:
        """Attach (or clear) the sink the executor publishes exec events to."""
        self._event_sink = sink
        if self.executor is not None:
            self.executor.set_event_sink(sink)

    async def start(self) -> None:
        """Open the link for the configured transport and start telemetry.

        A link that cannot be opened at startup (no smbus2, no bus, board
        absent) must never stop the host: that is a normal degraded state,
        so the session logs once and retries in the background until the
        board answers (console/brain routes report 503 in the meantime).
        """
        try:
            await self.switch_transport(self.transport)
        except Exception as exc:
            await self._log(
                "warn",
                f"robot link unavailable at startup ({exc}); "
                f"retrying every {self.CONNECT_RETRY_S:.0f} s",
            )
            self._connect_task = asyncio.create_task(self._connect_loop())

    async def _connect_loop(self) -> None:
        """Retry the configured link until it opens, then stop (quiet)."""
        while True:
            await asyncio.sleep(self.CONNECT_RETRY_S)
            try:
                await self.switch_transport(self.transport)
            except asyncio.CancelledError:
                raise
            except Exception:
                continue  # still absent: leave the session exactly as it is
            self._connect_task = None
            await self._log("info", f"robot link connected ({self.transport})")
            return

    async def _cancel_connect_retry(self) -> None:
        """Cancel a pending startup retry (idempotent, never from itself)."""
        retry = self._connect_task
        if retry is None or retry is asyncio.current_task():
            return
        self._connect_task = None
        retry.cancel()
        try:
            await retry
        except (asyncio.CancelledError, Exception):
            pass

    async def stop(self) -> None:
        """Stop the sensor tasks, telemetry and close the link."""
        await self._cancel_connect_retry()
        await self.stop_reanchor()
        await self.stop_gyro_stream()
        await self.stop_imu()
        await self.stop_telemetry()
        if self.client is not None:
            try:
                self.client.link.close()
            except Exception:
                pass
            self.client = None
        self.executor = None
        self.connected = False

    async def switch_transport(self, mode: str) -> str:
        """Rebuild the robot link (and executor) for ``i2c`` or ``mock``.

        The new link is created first: if it cannot be opened (no smbus2, no
        bus, no board) the current transport keeps running untouched. Always
        builds a fresh client/executor - after a lifespan restart the old
        asyncio primitives are bound to the previous event loop.
        """
        if mode not in ("i2c", "mock"):
            raise ValueError("transport must be 'i2c' or 'mock'")
        # a manual (or retried) switch supersedes any pending startup retry
        await self._cancel_connect_retry()
        link = MockLink() if mode == "mock" else I2CLink(
            self.settings.i2c_bus,
            self.settings.i2c_address,
            self.settings.i2c_retries,
            self.settings.i2c_retry_delay_s,
        )
        client = RobotClient(link, self.settings.i2c_retries, on_trace=self.on_trace)
        client.set_odometry_observer(self._on_odometry)
        executor = Executor(
            client,
            self.settings.deadman_ms,
            self.settings.exec_timeout_slack_s,
            stream_capability=self.supports_velocity_stream,
        )
        if self._event_sink is not None:
            executor.set_event_sink(self._event_sink)
        previous = self.client.link if self.client is not None else None
        await self.stop_reanchor()
        await self.stop_gyro_stream()
        await self.stop_imu()
        await self.stop_telemetry()
        self.client = client
        self.executor = executor
        self.odometry = None
        self.connected = False
        self.last_error = ""
        self.localization.reset()  # the new link starts a fresh pose estimate
        self.transport = mode
        if previous is not None:
            try:
                previous.close()
            except Exception:
                pass
        self.start_telemetry()
        await self.start_imu()
        await self.start_gyro_stream()
        self.start_reanchor()
        await self.sync_board_geometry()
        await self.maybe_apply_start_pose()
        return mode

    async def sync_board_geometry(self) -> bool:
        """Adopt the board's odometry wheel track into the estimator (reg 7).

        The board dead-reckons with its own odometry track and the localizer
        converts the same wheel speeds, so a calibrated board value must win
        over the ``ODOM_TRACK_MM`` environment default (that default is only
        the fallback for an unreachable board). Called at every (re)connect
        and after every settings write; a missing link, a failed read or an
        out-of-range value leaves the configured value untouched. Returns
        True when the estimator value really changed.
        """
        if self.client is None:
            return False
        try:
            spec = robot_config.REGISTER_BY_KEY["odom_track_mm"]
            raw = await self.client.get_all_config()
            value = raw[spec.index]
        except Exception:
            return False  # refinement only: never a failure path
        if not (spec.min_value <= value <= spec.max_value):
            return False
        track_mm = float(spec.decode(value))
        if abs(track_mm - self.localization.track_mm) < 0.05:
            return False
        self.localization.track_mm = max(1.0, track_mm)
        await self._log(
            "info",
            f"localizer track set from the board: {track_mm:.1f} mm (register 7)",
        )
        return True

    # ------------------------------------------------------------ telemetry
    def start_telemetry(self) -> None:
        """Start the telemetry loop unless it is already running."""
        if self._telemetry is None or self._telemetry.done():
            self._telemetry = asyncio.create_task(self._telemetry_loop())

    async def stop_telemetry(self) -> None:
        """Cancel the telemetry loop and wait for it to finish."""
        if self._telemetry is not None:
            self._telemetry.cancel()
            try:
                await self._telemetry
            except (asyncio.CancelledError, Exception):
                pass
            self._telemetry = None

    def pause_telemetry(self) -> None:
        """Suspend polling without ending the loop (fault drills).

        The deadman drill needs a window with NO I2C traffic at all: every
        request refreshes the board's command watchdog, so the poller must
        go quiet for the watchdog to expire. The loop stays alive (the flag
        is checked each period) so resuming is instant and the task is
        never recreated.
        """
        self._telemetry_paused = True

    def resume_telemetry(self) -> None:
        """Resume polling after :meth:`pause_telemetry`."""
        self._telemetry_paused = False

    async def _telemetry_loop(self) -> None:
        """Poll the board: keep the odometry cache and the connection state.

        Never dies: every failure only flips the connection flags (with one
        log line per online -> offline transition) and the loop keeps
        polling, so the link recovers by itself when the board answers again.
        """
        period = self.settings.telemetry_period_s
        while True:
            if self._telemetry_paused:
                await asyncio.sleep(period)
                continue
            try:
                assert self.client is not None
                odometry = await self.client.odometry()
                self.odometry = odometry
                self.connected = True
                if self.last_error:
                    self.last_error = ""
                if self.on_sample is not None:
                    await self.on_sample(odometry)
            except (RobotError, LinkError, AssertionError) as exc:
                await self._note_failure(exc)
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # never let the poller die
                await self._note_failure(exc)
            await asyncio.sleep(period)

    async def _note_failure(self, exc: Exception) -> None:
        """Record one failed poll; announce the transition exactly once."""
        announce = self.connected or not self.last_error
        self.connected = False
        self.last_error = str(exc)
        if not announce:
            return
        await self._log("warn", f"telemetry lost: {self.last_error}")
        if self.on_failure is not None:
            try:
                await self.on_failure(self.last_error)
            except Exception:
                pass  # a host hook must never kill the poller

    # ------------------------------------------------------------- camera
    async def on_camera_poses(
        self,
        us: Optional[Dict],
        opponent: Optional[Dict] = None,
        *,
        t_ms: Optional[float] = None,
        ttl_ms: Optional[float] = None,
        source: str = "",
        client: str = "",
        t_server_ms: Optional[float] = None,
    ) -> bool:
        """Ingest one camera pose snapshot: store it and feed the fusion.

        The store keeps the newest snapshot for status/UI and TTL semantics;
        the fusion consumes EVERY message for our robot (a fresh sample is a
        measurement even when it repeats the previous one). The pipeline
        delay (``t_server - t``) is used for latency compensation. Returns
        True when the stored snapshot really changed (ack semantics).
        """
        changed = self.poses.update(
            us, opponent, ttl_ms=ttl_ms, source=source, t_ms=t_ms, client=client
        )
        if not self.settings.camera_poses_enabled or not isinstance(us, dict):
            return changed
        latency = 0.0
        if t_ms is not None and t_server_ms is not None:
            try:
                latency = max(0.0, (float(t_server_ms) - float(t_ms)) / 1000.0)
            except (TypeError, ValueError):
                latency = 0.0
        latency = min(latency, self.settings.camera_latency_max_s)
        confidence = us.get("conf")
        try:
            self.localization.update_camera_pose(
                float(us["x_mm"]),
                float(us["y_mm"]),
                float(us["theta_deg"]),
                confidence=None if confidence is None else float(confidence),
                latency_s=latency,
            )
        except (KeyError, TypeError, ValueError):
            pass  # malformed entry: the store already dropped it
        return changed

    def release_camera_poses(self, source: str) -> bool:
        """A camera client disconnected: release its pose snapshot."""
        return self.poses.release(source)

    # ---------------------------------------------------------- match start
    async def maybe_apply_start_pose(self) -> bool:
        """Adopt the team match-start pose when the board pose is unset.

        A board that still reads (0, 0, 0) has never been placed - the field
        frame starts at the corner, no robot ever stands there - so the
        configured start of ``TEAM`` is pushed ONCE with SET_POSE: the board,
        the fusion and the console then agree with the match rules from the
        first sample instead of showing the field corner. A pose the operator
        (or a previous console run) already set is never overwritten, and
        ``START_POSE=0`` disables the behaviour. Failures are logged and never
        block the connect path.
        """
        if not self.settings.start_pose_enabled or self._start_pose_done:
            return False
        if self.client is None:
            return False
        try:
            board = await self.client.odometry()
        except Exception:
            return False  # link not ready: the next (re)connect retries
        if abs(board.x_mm) > 0.2 or abs(board.y_mm) > 0.2 or abs(board.theta_deg) > 0.2:
            self._start_pose_done = True  # already placed: leave it alone
            return False
        try:
            x, y, theta = resolve_start_pose(
                self.settings.team,
                self.settings.team_start_yellow,
                self.settings.team_start_blue,
                self.settings.field_width_mm,
            )
        except ValueError as exc:
            self._start_pose_done = True
            await self._log("warn", f"no valid team start pose ({exc})")
            return False
        try:
            await self.client.set_pose(x, y, theta)
        except Exception as exc:  # e.g. a latched fault: never a connect error
            self._start_pose_done = True
            await self._log("warn", f"could not adopt the match-start pose ({exc})")
            return False
        self._start_pose_done = True
        await self._log(
            "info",
            f"match start adopted: ({x:.0f}, {y:.0f}) mm, {theta:.0f} deg "
            f"(team '{self.settings.team}')",
        )
        return True

    # ---------------------------------------------------------------- IMU
    def _make_imu_driver(self):
        """Create the configured MPU6050 driver (overridable in tests)."""
        from software.sensors.mpu6050 import MPU6050

        axis = self.settings.imu_yaw_axis
        if axis in ("", "auto"):
            axis = "z"  # provisional: detect_orientation() refines it at boot
        return MPU6050(
            bus_number=self.settings.imu_bus,
            address=self.settings.imu_address,
            gyro_range_dps=500,
            sample_rate_hz=self.settings.imu_rate_hz,
            yaw_axis=axis,
        )

    async def start_imu(self) -> None:
        """Probe and start the MPU6050 sampler (silent no-op when absent).

        Mock transport and ``IMU=off`` never start a reader; any failure at
        probe or calibration is logged once and leaves the session exactly
        as if no IMU were mounted (Docs/09 section 10).
        """
        if not self.settings.imu_enabled or self.transport != "i2c":
            self.imu_state = "off"
            return
        driver = self._make_imu_driver()
        try:
            driver.open()
            chip = driver.probe()
            driver.configure()
            # Which sensor axis points up? Gravity answers it (a mount facing
            # down is "-z") and the mounting tilt is reported for health - it
            # needs no correction: 1 deg scales the yaw rate by cos(1 deg).
            axis_note = ""
            self.imu_mount = None
            if self.settings.imu_yaw_axis in ("", "auto"):
                try:
                    orientation = await asyncio.to_thread(driver.detect_orientation)
                    driver.yaw_axis = orientation.value
                    self.imu_mount = {
                        "axis": orientation.axis,
                        "sign": orientation.sign,
                        "tilt_deg": round(orientation.tilt_deg, 2),
                        "gravity_g": round(orientation.gravity_g, 3),
                        "mode": "auto",
                    }
                    axis_note = (
                        f", yaw {orientation.value} (tilt {orientation.tilt_deg:.1f} deg)"
                    )
                except Exception as exc:  # detection is a bonus, never fatal
                    await self._log(
                        "warn",
                        f"IMU mount detection failed ({exc}); using {driver.yaw_axis}",
                    )
            self.imu_axis = driver.yaw_axis
            # blocking calibration (~0.2 s at 100 samples): keep it off the
            # event loop so a running telemetry/streaming task is not stalled
            self.imu_bias_deg_s = await asyncio.to_thread(driver.read_gyro_bias_dps)
        except Exception as exc:  # any failure = no IMU, never a fault
            self.imu_state = "absent"
            try:
                driver.close()
            except Exception:
                pass
            await self._log("warn", f"IMU not available ({exc}); continuing without it")
            return
        self.imu = driver
        self.imu_state = "ready"
        self.imu_samples = 0
        self.imu_errors = 0
        self.imu_integrated_deg = 0.0
        self.imu_last_t = None
        self._imu_task = asyncio.create_task(self._imu_loop())
        await self._log(
            "info",
            f"IMU ready (chip 0x{chip:02X}{axis_note}, bias {self.imu_bias_deg_s:.2f} deg/s)",
        )

    async def stop_imu(self) -> None:
        """Stop the sampler and close the IMU bus (idempotent)."""
        if self._imu_task is not None:
            self._imu_task.cancel()
            try:
                await self._imu_task
            except (asyncio.CancelledError, Exception):
                pass
            self._imu_task = None
        if self.imu is not None:
            try:
                self.imu.close()
            except Exception:
                pass
            self.imu = None

    async def stop_imu_sampling(self) -> None:
        """Stop the sampler and the gyro stream on purpose (bench test).

        This is the software equivalent of unplugging the IMU (STM_docs/06
        section 8 step 4): safe to call DURING motion - the sampler and the
        frame stream stop, the board returns to wheel heading within
        ~200 ms and the fusion degrades to wheels, never a fault. The state
        becomes "stopped"; :meth:`start_imu_sampling` probes it again.
        """
        was_active = self.imu is not None or self.imu_state == "ready"
        await self.stop_gyro_stream()
        await self.stop_imu()
        if was_active:
            self.imu_state = "stopped"

    async def start_imu_sampling(self) -> None:
        """(Re)start the sampler after a manual stop (idempotent).

        ``start_imu`` re-probes the sensor and re-averages the bias, so it
        must only run while the robot stands still (the console route
        enforces it); a second call while the sampler already runs is a
        no-op.
        """
        if self._imu_task is not None and not self._imu_task.done():
            return
        await self.start_imu()
        await self.start_gyro_stream()

    async def _imu_loop(self) -> None:
        """Sample the gyro at the configured rate; exit after repeated errors.

        On exit the session keeps working wheel-only: ``imu_state`` becomes
        "lost", the fusion stops receiving gyro samples and everything else
        is untouched (the same guarantee as an IMU that was never mounted).
        """
        assert self.imu is not None
        period = 1.0 / max(1.0, self.settings.imu_rate_hz)
        consecutive = 0
        while True:
            try:
                if self._imu_calibrating:
                    # a bias recalibration is reading the bus on a worker
                    # thread: skip samples instead of sharing the bus
                    await asyncio.sleep(period)
                    continue
                now = time.monotonic()
                # GYRO_SCALE absorbs the MPU6050 scale/trim error (Docs/09
                # section 9.3); the integrated angle below is what the spin
                # calibration compares against the real rotation. A stop at
                # rest also lets the bias follow a warming gyro (section 9.3).
                raw_rate = self.imu.read_yaw_rate_dps()
                self._refine_bias_at_rest(raw_rate, now)
                rate = (raw_rate - self.imu_bias_deg_s) * self.gyro_scale
                self.imu_rate_deg_s = rate
                if self.imu_last_t is not None:
                    dt = min(0.5, max(0.0, now - self.imu_last_t))
                    self.imu_integrated_deg += rate * dt
                self.imu_last_t = now
                self.imu_samples += 1
                consecutive = 0
                self.localization.add_gyro_yaw_rate(rate)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self.imu_errors += 1
                consecutive += 1
                if consecutive >= self.IMU_MAX_ERRORS:
                    self.imu_state = "lost"
                    await self._log("warn", f"IMU lost ({exc}); continuing without it")
                    return
            await asyncio.sleep(period)

    # ------------------------------------------------- bias at rest
    #: the pose must hold still this long before the bias may refine [s]
    BIAS_REST_SETTLE_S = 5.0
    #: board pose drift still counted as "at rest" [mm] / [deg]
    BIAS_REST_POSE_MM = 2.0
    BIAS_REST_POSE_DEG = 0.5
    #: wheel speeds still counted as "at rest" [mm/s]
    BIAS_REST_SPEED_MM_S = 5.0
    #: the RAW rate must be within this gate to refine [deg/s] - a hand
    #: rotation (wheels still, real rotation) must never be learned as bias
    BIAS_REST_GATE_DEG_S = 10.0
    #: low-pass time constant of the refinement [s]
    BIAS_REFINE_TAU_S = 30.0
    #: hard clamp of the refined bias [deg/s]
    BIAS_LIMIT_DEG_S = 10.0

    def _refine_bias_at_rest(self, raw_rate: float, now: float) -> None:
        """Slowly refine the gyro bias while the robot is provably still.

        The boot average stales as the gyro warms up (the real robot read
        1.5 deg/s off after 4 h), which would bias BOTH the fusion input and
        the board's gyro stream during motion. While the board is idle, the
        wheels are stopped and the board pose has held still for
        :data:`BIAS_REST_SETTLE_S`, the bias low-passes toward the RAW
        reading with the :data:`BIAS_REFINE_TAU_S` time constant - rest
        noise averages out and a single stop is imperceptible, while a long
        idle fully re-centres the value. Any motion - a run, an execution,
        a hand push, a carry - freezes it. Silent and non-fatal by design:
        a missing link or IMU only skips the refinement.
        """
        odometry = self.odometry
        if (
            odometry is None
            or not self.connected
            or not odometry.is_idle
            or (self.executor is not None and self.executor.is_running())
            or abs(odometry.v_left_mm_s) > self.BIAS_REST_SPEED_MM_S
            or abs(odometry.v_right_mm_s) > self.BIAS_REST_SPEED_MM_S
            or abs(raw_rate) > self.BIAS_REST_GATE_DEG_S
        ):
            self._rest_ref = None
            self._rest_since = None
            self._bias_refined_t = None
            return
        pose = (float(odometry.x_mm), float(odometry.y_mm), float(odometry.theta_deg))
        if (
            self._rest_ref is None
            or math.dist(pose[:2], self._rest_ref[:2]) > self.BIAS_REST_POSE_MM
            or abs(_wrap180(pose[2] - self._rest_ref[2])) > self.BIAS_REST_POSE_DEG
        ):
            self._rest_ref = pose
            self._rest_since = now
            self._bias_refined_t = None
            return
        if self._rest_since is None or now - self._rest_since < self.BIAS_REST_SETTLE_S:
            self._bias_refined_t = None
            return
        previous = self._bias_refined_t
        self._bias_refined_t = now
        if previous is None:
            return  # first adapting sample: only start the dt clock
        alpha = min(0.05, max(0.0, (now - previous) / self.BIAS_REFINE_TAU_S))
        refined = self.imu_bias_deg_s + alpha * (raw_rate - self.imu_bias_deg_s)
        self.imu_bias_deg_s = max(-self.BIAS_LIMIT_DEG_S, min(self.BIAS_LIMIT_DEG_S, refined))

    def imu_status(self) -> Dict:
        """JSON-friendly IMU health for the telemetry/snapshot."""
        return {
            "state": self.imu_state,
            "present": self.imu is not None,
            "yaw_axis": self.imu_axis,
            "mount": self.imu_mount,
            "rate_deg_s": round(self.imu_rate_deg_s, 3),
            "bias_deg_s": round(self.imu_bias_deg_s, 3),
            "scale": round(self.gyro_scale, 6),
            "integrated_deg": round(self.imu_integrated_deg, 2),
            "samples": self.imu_samples,
            "errors": self.imu_errors,
            "calibrating": self._imu_calibrating,
            "rest_s": (
                None
                if self._rest_since is None
                else round(max(0.0, time.monotonic() - self._rest_since), 1)
            ),
            "age_s": (
                None
                if self.imu_last_t is None
                else round(time.monotonic() - self.imu_last_t, 3)
            ),
        }

    # ------------------------------------------------- IMU calibration
    #: accepted gyro scale window (sanity bound for the spin workflow)
    IMU_SCALE_MIN = 0.5
    IMU_SCALE_MAX = 2.0
    #: a spin test must integrate at least this many degrees to be usable
    IMU_SPIN_MIN_DEG = 60.0

    def imu_reset_integral(self) -> None:
        """Zero the spin-check yaw integral (start of a hand-rotation test)."""
        self.imu_integrated_deg = 0.0

    def imu_apply_scale(self, scale: float) -> Tuple[float, float]:
        """Set the LIVE gyro scale: returns ``(previous, applied)``.

        ``GYRO_SCALE`` seeds the value at boot; applying here corrects the
        MPU6050 trim without a restart (every following sample and the spin
        integral use it at once). It is NOT persisted - put the returned
        ``GYRO_SCALE=...`` line in ``docker/.env`` to survive a restart
        (Docs/09 section 9.3).
        """
        value = float(scale)
        if not math.isfinite(value) or not (self.IMU_SCALE_MIN <= value <= self.IMU_SCALE_MAX):
            raise ValueError(
                f"gyro scale must be within {self.IMU_SCALE_MIN}..{self.IMU_SCALE_MAX} (got {value})"
            )
        previous = self.gyro_scale
        self.gyro_scale = value
        return previous, value

    def imu_spin_scale(self, turns: float) -> Dict:
        """Compute the gyro scale of a finished spin test and apply it.

        ``turns`` complete hand rotations must have accumulated into
        ``imu_integrated_deg`` since the counter was reset. The new scale is
        ``current_scale * (360 * turns / integrated)`` - the integral was
        measured with the current scale applied, so the correction is
        relative. A degenerate spin (less than :data:`IMU_SPIN_MIN_DEG`
        integrated) and a correction outside the sanity window raise
        ``ValueError``.
        """
        count = float(turns)
        if not math.isfinite(count) or count <= 0.0 or count > 100.0:
            raise ValueError("turns must be a positive number (<= 100)")
        integrated = self.imu_integrated_deg
        if abs(integrated) < self.IMU_SPIN_MIN_DEG:
            raise ValueError(
                f"integrated angle too small ({integrated:.1f} deg) - "
                "reset the counter, rotate the robot by hand, then apply"
            )
        suggested = self.gyro_scale * (360.0 * count) / integrated
        previous, applied = self.imu_apply_scale(suggested)
        return {
            "previous": previous,
            "applied": applied,
            "turns": count,
            "integrated_deg": integrated,
        }

    async def imu_recalibrate_bias(self) -> float:
        """Re-average the gyro bias at rest; returns the new value [deg/s].

        The sampler pauses for the blocking driver calibration so two
        threads never share the IMU bus; the previous bias is kept when the
        driver raises (the error surfaces to the caller).
        """
        if self.imu is None or self.imu_state != "ready":
            raise ValueError(f"IMU not available (state: {self.imu_state})")
        self._imu_calibrating = True
        try:
            await asyncio.sleep(0.05)  # let an in-flight sample finish
            bias = await asyncio.to_thread(self.imu.read_gyro_bias_dps)
        finally:
            self._imu_calibrating = False
        self.imu_bias_deg_s = float(bias)
        return self.imu_bias_deg_s

    async def imu_measure_noise(self, seconds: float = 2.0) -> Dict:
        """Sample the live yaw rate at rest: mean and sigma [deg/s].

        The console uses it to suggest an honest ``GYRO_SIGMA_DEG_S``: the
        robot must stand still - motion shows up as a mean or as noise.
        """
        if self.imu is None or self.imu_state != "ready":
            raise ValueError(f"IMU not available (state: {self.imu_state})")
        span = max(0.5, min(5.0, float(seconds)))
        values: List[float] = []
        deadline = time.monotonic() + span
        while time.monotonic() < deadline:
            values.append(float(self.imu_rate_deg_s))
            await asyncio.sleep(0.02)
        # drop the first samples: they may predate the call
        values = values[2:] if len(values) > 2 else values
        if len(values) < 10:
            raise ValueError("not enough gyro samples - IMU may be stopping")
        mean = sum(values) / len(values)
        variance = sum((value - mean) ** 2 for value in values) / (len(values) - 1)
        return {
            "samples": len(values),
            "seconds": span,
            "mean_deg_s": mean,
            "sigma_deg_s": math.sqrt(variance),
        }

    # ------------------------------------------------------- gyro stream
    async def start_gyro_stream(self) -> None:
        """Start forwarding yaw rate to the board (silent, firmware >= 3.3).

        The gate is the firmware version reported by QUERY_STATUS: an older
        board must never receive SET_GYRO frames (it would answer UNKNOWN_CMD
        and flood the response channel). The stream is optional in every
        direction - no IMU or a stale stream only reverts the board to wheel
        heading (its default), never a fault.
        """
        if (
            not self.settings.gyro_stream_enabled
            or self.transport != "i2c"
            or self.client is None
            or self.imu is None
            or self.imu_state != "ready"
        ):
            return
        if self._stream_task is not None and not self._stream_task.done():
            return  # already streaming (idempotent: never two loop tasks)
        try:
            status = await self.client.status()
        except Exception:
            return  # no link yet: the next transport switch retries
        if (status.fw_major, status.fw_minor) < (p.GYRO_STREAM_FW_MAJOR, p.GYRO_STREAM_FW_MINOR):
            self.gyro_stream_state = "unsupported"
            await self._log(
                "info",
                f"gyro stream off: firmware {status.fw_major}.{status.fw_minor} "
                f"< {p.GYRO_STREAM_FW_MAJOR}.{p.GYRO_STREAM_FW_MINOR} (wheel heading)",
            )
            return
        self.gyro_stream_state = "active"
        self._stream_task = asyncio.create_task(self._gyro_stream_loop())

    async def stop_gyro_stream(self) -> None:
        """Stop the forward stream (idempotent)."""
        if self._stream_task is not None:
            self._stream_task.cancel()
            try:
                await self._stream_task
            except (asyncio.CancelledError, Exception):
                pass
            self._stream_task = None
        self.gyro_stream_state = "off"

    async def _gyro_stream_loop(self) -> None:
        """Forward the latest yaw rate while the board executes motion.

        Idle standing still: no frames (the board does not need heading
        input). Link errors are swallowed - the next frame replaces the lost
        one, and a stream that never arrives leaves the board on wheels.
        """
        assert self.client is not None
        period = 1.0 / max(1.0, self.settings.gyro_stream_hz)
        while True:
            try:
                moving = self.executor is not None and self.executor.is_running()
                if self.odometry is not None and not self.odometry.is_idle:
                    moving = True
                now = time.monotonic()
                fresh = (
                    self.imu_state == "ready"
                    and self.imu_last_t is not None
                    and (now - self.imu_last_t) <= self.settings.gyro_stale_ms / 1000.0
                )
                if moving and fresh:
                    age_10ms = int(min(255.0, (now - self.imu_last_t) * 100.0))
                    await self.client.stream_gyro(self.imu_rate_deg_s, age_10ms)
                    self.gyro_stream_frames += 1
            except asyncio.CancelledError:
                raise
            except Exception:
                self.gyro_stream_errors += 1  # replaced by the next frame
            await asyncio.sleep(period)

    def gyro_stream_status(self) -> Dict:
        """JSON-friendly forward-stream health for the telemetry/snapshot."""
        return {
            "state": self.gyro_stream_state,
            "frames": self.gyro_stream_frames,
            "errors": self.gyro_stream_errors,
        }

    # --------------------------------------------------- velocity stream
    async def supports_velocity_stream(self) -> bool:
        """True when a host may use the streaming follower backend (Track B).

        Gates, all mandatory: the ``STREAMING_EXEC`` setting (default off -
        the segment executor stays the default backend), a live link, and a
        board that accepts silent SET_VELOCITY frames (firmware >= 3.4
        reported by QUERY_STATUS; the mock simulates 3.5 and always
        supports it). Call this with the robot IDLE, before starting a
        follower run - it performs one QUERY_STATUS transaction.
        """
        if not self.settings.streaming_exec_enabled or self.client is None:
            return False
        if self.transport != "i2c":
            return True  # the mock simulates firmware 3.4 and integrates frames
        try:
            status = await self.client.status()
        except Exception:
            return False  # no link yet: the caller retries when connected
        return (status.fw_major, status.fw_minor) >= (
            p.VELOCITY_STREAM_FW_MAJOR,
            p.VELOCITY_STREAM_FW_MINOR,
        )

    # --------------------------------------------------------- re-anchor
    def start_reanchor(self) -> None:
        """Start the boundary re-anchor task (idempotent, no-op when off).

        Runs in every transport (the mock board supports SET_POSE too), so
        the rehearsal path behaves exactly like the real one.
        """
        if not self.settings.reanchor_enabled or self.client is None:
            self.reanchor_state = "off"
            return
        if self._reanchor_task is None or self._reanchor_task.done():
            self.reanchor_state = "active"
            self._reanchor_task = asyncio.create_task(self._reanchor_loop())

    async def stop_reanchor(self) -> None:
        """Stop the re-anchor task (idempotent)."""
        if self._reanchor_task is not None:
            self._reanchor_task.cancel()
            try:
                await self._reanchor_task
            except (asyncio.CancelledError, Exception):
                pass
            self._reanchor_task = None

    async def _reanchor_loop(self) -> None:
        """Low-rate policy loop; every guard failure is a silent skip."""
        period = 0.5
        while True:
            try:
                await self.maybe_reanchor()
            except asyncio.CancelledError:
                raise
            except Exception:
                pass  # a correction attempt must never kill the loop
            await asyncio.sleep(period)

    async def maybe_reanchor(self) -> bool:
        """Re-anchor the board pose when the camera disagrees at idle.

        Guards, all mandatory: a client, an idle board (never during an
        execution), a fresh accepted camera pose, and the minimum interval.
        The first anchor acts as match-start initialization; afterwards the
        anchor fires only when the fused/board discrepancy exceeds the
        configured thresholds. Returns True when SET_POSE was issued.
        """
        if self.client is None:
            return False
        if self.executor is not None and self.executor.is_running():
            return False
        if not self.localization.camera_active():
            return False
        if self.poses.us() is None:
            return False
        board = self.odometry
        if board is None or not board.is_idle:
            return False
        now = time.monotonic()
        if (now - self._last_reanchor_t) < max(0.1, self.settings.reanchor_interval_s):
            return False
        x, y, theta = self.localization.pose()
        dx = x - board.x_mm
        dy = y - board.y_mm
        dtheta = _wrap180(theta - board.theta_deg)
        need_anchor = (not self._anchored_once) or (
            math.hypot(dx, dy) > self.settings.reanchor_mm
            or abs(dtheta) > self.settings.reanchor_deg
        )
        if not need_anchor:
            return False
        await self.client.set_pose(x, y, theta)
        self._last_reanchor_t = now
        self._anchored_once = True
        self.reanchor_count += 1
        await self._log(
            "info", f"pose re-anchored at ({x:.0f}, {y:.0f}) mm, {_wrap180(theta):.1f} deg"
        )
        return True

    def reanchor_status(self) -> Dict:
        """JSON-friendly re-anchor health for the telemetry/snapshot."""
        return {
            "state": self.reanchor_state,
            "count": self.reanchor_count,
            "anchored_once": self._anchored_once,
        }

    # ------------------------------------------------------------ services
    def _on_odometry(self, odometry: Odometry) -> None:
        """Observer: every board odometry read feeds the localizer.

        Called from the telemetry loop AND from the executor's polls (segment
        supervision, wait_idle after a stop), so the fused pose is fresh
        exactly when the orchestration reads it.
        """
        self.localization.ingest(
            WheelSample(
                x_mm=odometry.x_mm,
                y_mm=odometry.y_mm,
                theta_deg=odometry.theta_deg,
                v_left_mm_s=odometry.v_left_mm_s,
                v_right_mm_s=odometry.v_right_mm_s,
            )
        )

    def pose(self) -> Optional[tuple]:
        """The pose consumers should use: the localized estimate, or None.

        ``None`` until the first odometry sample arrived (mission context
        contract). Wheel odometry is fused with the optional sensors inside
        the localization service; callers never see which sources were used.
        """
        if self.odometry is None:
            return None
        return self.localization.pose()

    def status(self) -> Dict:
        """JSON-friendly session status: transport, link health, pose mode."""
        return {
            "transport": self.transport,
            "connected": self.connected,
            "error": self.last_error,
            "loc": self.localization.status(),
            "poses": self.poses.status(),
            "imu": self.imu_status(),
            "gyro_stream": self.gyro_stream_status(),
            "reanchor": self.reanchor_status(),
        }

    async def _log(self, level: str, message: str) -> None:
        """Forward one lifecycle line to the host logger (errors swallowed)."""
        if self.logger is None:
            return
        try:
            await self.logger(level, message)
        except Exception:
            pass
