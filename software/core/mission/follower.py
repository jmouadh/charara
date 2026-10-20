"""Path follower: the streaming executor backend (Track B, Docs/10).

Drives a planned path with continuous velocity frames instead of stop-turn
GOTO programs: every control tick it reads the board odometry, runs one
Pure Pursuit step (:mod:`software.core.navigation.pursuit`) and streams the
result as a silent ``SET_VELOCITY`` frame (latest-wins, write-only). The
segment executor stays the default backend - this one is only used when the
host verified the board supports the stream (firmware >= 3.4, or the mock).

Lifecycle::

    follower = PathFollower(client)
    result = await follower.run(points, v_cruise_mm_s, deadman_ms=1500)

Result keys: ``ok``, ``reason``, ``duration_s``, ``path_mm``,
``final_error_mm``, ``frames``, ``max_speed_mm_s``. The follower brakes
with velocity frames near the path end and then issues a plain STOP to
leave the streaming mode deterministically (the board would also ramp down
on stale frames, but STOP is explicit).

Safety: the firmware keeps stall/limit/e-stop supervision while streaming;
the deadman watchdog is armed here exactly like an executor run, and every
abort path issues STOP before returning. A telemetry outage longer than
the executor's own tolerance aborts the run (the board keeps streaming for
200 ms and then stops by itself - the controlled-stale fallback (Docs/10
section 6.2).

Host hooks (all optional, wired by the executor): ``speed_scale`` re-applies
the caution multiplier every tick (omega follows the scaled speed so the
turning radius is kept); ``should_abort`` / ``should_pause`` mirror the
executor's STOP/PAUSE buttons (a pause STOPs the board and holds it; the
next frame after resume opens a fresh streaming session and pursuit
re-projects the pose on the path); ``on_pause`` publishes the paused and
resumed transitions.

File:    software/core/mission/follower.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.2.0
Date:    2026-10-12
"""

from __future__ import annotations

import asyncio
import inspect
import math
import time
from typing import Awaitable, Callable, Dict, Optional, Sequence, Tuple

from software.core.mission.interfaces import RobotIOError
from software.core.navigation.pursuit import (
    DEFAULT_PARAMS,
    PursuitParams,
    distance_to_path,
    path_length,
    pursuit_step,
)

Point = Tuple[float, float]

#: consecutive odometry read failures tolerated before the run aborts
TELEMETRY_FAILURE_LIMIT = 25
#: odometry reads while waiting for the wheels to really stop after STOP
STOP_SETTLE_TRIES = 30
#: floor for the final in-place heading alignment [rad/s]: the board's
#: per-side speed floor makes a smaller omega turn faster than commanded
ALIGN_MIN_OMEGA_RAD_S = 0.35


def _deg_norm180(angle_deg: float) -> float:
    """Wrap an angle into (-180, 180]."""
    return (angle_deg + 180.0) % 360.0 - 180.0


class PathFollower:
    """Streams (v, omega) velocity frames to follow a planned path."""

    def __init__(
        self,
        client,
        rate_hz: float = 25.0,
        params: PursuitParams = DEFAULT_PARAMS,
        capability: Optional[Callable[[], Awaitable[bool]]] = None,
    ) -> None:
        """Create the follower around a RobotClient-like port.

        ``capability`` is the HOST's proof that the board accepts the silent
        stream (the session's ``supports_velocity_stream``). When given,
        :meth:`run` refuses to send a single frame unless it returns True -
        a firmware below 3.4 (or the ``STREAMING_EXEC`` gate off) can never
        receive stream traffic by accident.
        """
        self._client = client
        self._period_s = 1.0 / max(5.0, rate_hz)
        self._params = params
        self._capability = capability

    async def run(
        self,
        points: Sequence[Point],
        v_cruise_mm_s: float,
        *,
        deadman_ms: int = 0,
        timeout_s: Optional[float] = None,
        on_progress=None,
        speed_scale: Optional[Callable[[], float]] = None,
        should_abort: Optional[Callable[[], bool]] = None,
        should_pause: Optional[Callable[[], bool]] = None,
        on_pause: Optional[Callable[[bool], Awaitable[None]]] = None,
        bounds: Optional[Tuple[float, float, float]] = None,
        final_heading_deg: Optional[float] = None,
        heading_tol_deg: float = 2.0,
    ) -> Dict:
        """Follow ``points`` at up to ``v_cruise_mm_s``; never raises.

        ``timeout_s`` defaults to 2.5x the nominal duration + 10 s (same
        spirit as the executor's chain budget). ``on_progress``, when given,
        is called with a small dict every tick (rate-limited by the loop
        period) - used by the console for live feedback.

        ``bounds`` is ``(field_width_mm, field_height_mm, inset_mm)``: a
        safety geofence around the play area. The robot's centre must stay
        ``inset`` mm inside the field (the planner uses the same region); a
        pose outside it aborts the run with the board stopped - the robot
        must never drive out of the game area, whatever the follower or the
        localization says (real-robot incident 2026-10-18).

        ``final_heading_deg`` is the pose the plan's last motion segment
        leaves the robot in (a waypoint arrival angle, the goal theta, an
        ARC's exit tangent). Pure pursuit stops within ``stop_tol`` of the
        path end and would otherwise leave the robot a few degrees (or more)
        off that heading; the follower rotates in place to it before
        reporting done.

        The host hooks (see the module docstring) let the executor driving
        this follower re-use its full supervision shell: caution scaling,
        STOP, PAUSE/RESUME. An abort always leaves the board stopped.
        """
        if len(points) < 2:
            return self._result(False, "path needs at least two points", 0.0, 0.0, 0, 0.0)
        if float(v_cruise_mm_s) <= 0.0:
            return self._result(False, "v_cruise_mm_s must be positive", 0.0, 0.0, 0, 0.0)
        if self._capability is not None:
            try:
                supported = bool(await self._capability())
            except Exception:
                supported = False
            if not supported:
                return self._result(
                    False,
                    "streaming not supported (firmware < 3.4 or STREAMING_EXEC off)",
                    0.0,
                    0.0,
                    0,
                    0.0,
                )
        path_mm = path_length(points)
        deadline = time.monotonic() + (
            float(timeout_s) if timeout_s is not None else path_mm / max(v_cruise_mm_s, 50.0) * 2.5 + 10.0
        )
        armed = False
        frames = 0
        max_speed = 0.0
        start_t = time.monotonic()
        reason = ""
        ok = False
        failures = 0
        try:
            if deadman_ms > 0:
                await self._client.set_deadman(deadman_ms)
                armed = True
            while True:
                if time.monotonic() > deadline:
                    reason = "path timeout"
                    break
                if should_abort is not None and should_abort():
                    reason = "aborted"
                    break
                if should_pause is not None and should_pause():
                    if not await self._pause_hold(should_abort, should_pause, on_pause):
                        reason = "aborted"
                        break
                try:
                    odometry = await self._client.odometry()
                    failures = 0
                except RobotIOError as exc:
                    failures += 1
                    if failures >= TELEMETRY_FAILURE_LIMIT:
                        reason = f"telemetry lost: {exc}"
                        break
                    await asyncio.sleep(self._period_s)
                    continue
                if odometry.faults != 0:
                    reason = f"fault {', '.join(odometry.describe_faults())}"
                    break
                pose = (float(odometry.x_mm), float(odometry.y_mm), float(odometry.theta_deg))
                if bounds is not None:
                    width, height, inset = bounds
                    if (
                        pose[0] < inset - 5.0
                        or pose[0] > width - inset + 5.0
                        or pose[1] < inset - 5.0
                        or pose[1] > height - inset + 5.0
                    ):
                        reason = (
                            f"safety stop: the robot reached the field edge "
                            f"({pose[0]:.0f}, {pose[1]:.0f}) mm - re-plan from its pose"
                        )
                        break
                if frames == 0:
                    off = distance_to_path(points, pose)
                    if off > self._params.start_off_path_mm:
                        reason = (
                            f"robot is {off:.0f} mm off the planned path - "
                            "re-plan from its current pose"
                        )
                        break
                cmd = pursuit_step(points, pose, v_cruise_mm_s, self._params)
                if cmd is None:
                    if final_heading_deg is not None:
                        error = _deg_norm180(float(final_heading_deg) - pose[2])
                        if abs(error) > heading_tol_deg:
                            # the path was followed to its end, but the plan's
                            # last motion segment leaves the robot facing this
                            # heading (a waypoint angle / the goal theta): do
                            # not report done a few degrees off it
                            magnitude = max(
                                ALIGN_MIN_OMEGA_RAD_S,
                                min(self._params.align_max_omega_rad_s,
                                    self._params.align_gain * math.radians(abs(error))),
                            )
                            omega = math.copysign(math.degrees(magnitude), error)
                            await self._client.stream_velocity(0.0, omega)
                            frames += 1
                            await asyncio.sleep(self._period_s)
                            continue
                    ok = True
                    reason = "done"
                    break
                scale = 1.0
                if speed_scale is not None:
                    try:
                        scale = max(0.0, min(1.0, float(speed_scale())))
                    except Exception:
                        scale = 1.0
                if cmd.turn_only:
                    # align in place: NEVER drive forward towards a target
                    # that sits behind the robot (that is how the robot left
                    # the field on 2026-10-18); the caution scale still slows
                    # the rotation
                    v = 0.0
                    omega_deg_s = math.degrees(cmd.omega_rad_s) * max(scale, 0.25)
                else:
                    # the caution scale slows the command but keeps the turn
                    # radius: omega must follow the ACTUAL speed (omega = v*kappa)
                    v = max(self._params.v_min, cmd.v_mm_s * scale)
                    omega_deg_s = (
                        math.degrees(cmd.omega_rad_s) * (v / cmd.v_mm_s)
                        if cmd.v_mm_s > 1e-9 else 0.0
                    )
                await self._client.stream_velocity(v, omega_deg_s)
                frames += 1
                max_speed = max(max_speed, v)
                if on_progress is not None:
                    try:
                        outcome = on_progress(
                            {
                                "remaining_mm": cmd.remaining_mm,
                                "v_mm_s": v,
                                "frames": frames,
                                "speed_scale": round(scale, 2),
                                "turn_only": cmd.turn_only,
                            }
                        )
                        if inspect.isawaitable(outcome):
                            await outcome  # the executor's sink emits events async
                    except Exception:
                        pass
                await asyncio.sleep(self._period_s)
        except Exception as exc:  # the follower must never take the host down
            reason = f"follower error: {exc}"
        finally:
            try:
                await self._client.stop()
            except Exception:
                pass
            await self._wait_idle()
            if armed:
                try:
                    await self._client.set_deadman(0)
                except Exception:
                    pass
        duration = time.monotonic() - start_t
        odometry = await self._safe_odometry()
        final_error = 0.0
        if odometry is not None:
            final_error = math.hypot(points[-1][0] - odometry.x_mm, points[-1][1] - odometry.y_mm)
        return self._result(ok, reason, duration, final_error, frames, max_speed)

    async def _pause_hold(self, should_abort, should_pause, on_pause) -> bool:
        """Hold the board stopped while paused; False when aborted.

        The stop is explicit (STOP) exactly like the segment executor's
        pause; the next frame on resume opens a fresh streaming session
        from rest (the firmware's documented behavior), so no bookkeeping
        is needed - pursuit re-projects the pose on the path.
        """
        try:
            await self._client.stop()
        except RobotIOError:
            pass
        if on_pause is not None:
            try:
                await on_pause(True)
            except Exception:
                pass
        while should_pause is not None and should_pause():
            if should_abort is not None and should_abort():
                break
            await asyncio.sleep(0.05)
        if on_pause is not None:
            try:
                await on_pause(False)
            except Exception:
                pass
        return not (should_abort is not None and should_abort())

    async def _wait_idle(self) -> None:
        """Best-effort wait until the board reports IDLE after the STOP."""
        for _ in range(STOP_SETTLE_TRIES):
            try:
                if (await self._client.odometry()).is_idle:
                    return
            except RobotIOError:
                return
            await asyncio.sleep(0.05)

    async def _safe_odometry(self):
        """One last odometry read for the closing error; None when it fails."""
        try:
            return await self._client.odometry()
        except RobotIOError:
            return None

    @staticmethod
    def _result(
        ok: bool, reason: str, duration_s: float, final_error_mm: float, frames: int, max_speed: float
    ) -> Dict:
        """JSON-friendly run report (same shape used by the API card)."""
        return {
            "ok": ok,
            "reason": reason,
            "duration_s": round(duration_s, 2),
            "final_error_mm": round(final_error_mm, 1),
            "frames": frames,
            "max_speed_mm_s": round(max_speed, 1),
        }
