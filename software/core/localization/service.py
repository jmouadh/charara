"""Localization service: the RPi's own pose estimate.

The STM32 still runs its on-board dead reckoning for its closed-loop motion
control. The RPi builds *its* pose from every available source:

* every telemetry sample (wheel speeds + the board's dead-reckoned pose)
  feeds :class:`PoseKalmanFilter` - wheel twist as the prediction;
* the wheel-derived yaw rate is always fused with a slip-inflated sigma;
* the **gyro** (MPU6050, Docs/09 section 9) is fused as a direct yaw-rate
  measurement with a much tighter sigma, and refines its own bias while the
  robot is known to be at rest;
* the **ArUco camera** (Docs/09 section 8) is fused as an absolute pose via
  :meth:`update_camera_pose` - gated (Mahalanobis), latency-compensated and
  outlier-safe (a large jump must be confirmed by the next sample).

Policy (Docs/09 section 6.3): while an independent source (gyro or camera)
  is fresh, the board pose is downgraded to a weak cross-check - it
  integrates the SAME encoders as this filter, so trusting it strongly would
  double count. With no independent source the service behaves exactly as
  before (board pose is the strong reference).

The service is deliberately free of asyncio/FastAPI/link code: feed it
samples, ask it for the pose. ``status()`` reports the active source mix,
uncertainty, rejection counters and the gyro bias for diagnostics.

File:    software/core/localization/service.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-05
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from .kalman import PoseKalmanFilter
from .kinematics import twist_from_wheels, wrap180


@dataclass
class WheelSample:
    """One odometry sample pushed by the board link."""

    x_mm: float
    y_mm: float
    theta_deg: float
    v_left_mm_s: float
    v_right_mm_s: float
    t: Optional[float] = None  # monotonic seconds; None -> time.monotonic()


#: a gyro sample older than this counts as absent (source mix reporting)
GYRO_FRESH_S = 1.0
#: a camera pose older than this counts as absent (fusion + corrections)
CAMERA_FRESH_S = 0.5
#: dt sanity bounds - a suspended process or a replayed log must not blow up
MIN_DT_S = 0.001
MAX_DT_S = 0.5
#: a step in the BOARD's own pose stream larger than this is a teleport
#: (SET_POSE, RESET_ODOMETRY, a board reboot): the estimate adopts it at
#: once instead of smearing it over metres. The check compares CONSECUTIVE
#: BOARD SAMPLES, never the board pose with the estimate: a heading the
#: estimate built up itself (a hand rotation, a slip) is real information
#: the gyro owns and must be kept (Docs/09 6.3).
JUMP_MM = 150.0
JUMP_DEG = 30.0
#: the same step measured as a rate, so a longer gap between samples (a link
#: hiccup) does not turn real motion into a "teleport"
JUMP_SPEED_MM_S = 1500.0
JUMP_RATE_DEG_S = 300.0
#: the gyro bias is refined only when the robot is at rest AND the measured
#: rate is small: a hand rotation (still wheels, real rotation) must never
#: be learned as bias, or the heading would drift once it stops
BIAS_REFINE_MAX_DEG_S = 5.0
#: bias refinement gain per gyro sample (100 Hz -> ~5 s time constant)
BIAS_REFINE_ALPHA = 0.002
#: a camera "jump" candidate must repeat within this radius to be adopted
CAMERA_CONFIRM_MM = 60.0
#: ... and within this heading difference [deg] (a consistent repeat)
CAMERA_CONFIRM_DEG = 15.0
#: board-pose trust while an independent source (gyro/camera) is fresh: the
#: board pose and this filter integrate the SAME encoders, so only its
#: position is used then (weak) - the heading is skipped entirely, it would
#: undo rotations the wheels missed (hand placement, scrub, slip)
WEAK_BOARD_SIGMA_XY_MM = 60.0


class LocalizationService:
    """Owns the pose estimate (wheels now; gyro and camera when present)."""

    def __init__(
        self,
        # mirrors the board's odometry track setting (settings register 7)
        track_mm: float = 170.0,
        *,
        sigma_speed_mm_s: float = 30.0,
        sigma_wheel_omega_deg_s: float = 20.0,
        sigma_pose_mm: float = 8.0,
        sigma_pose_deg: float = 0.8,
        sigma_gyro_deg_s: float = 1.5,
        sigma_camera_xy_mm: float = 10.0,
        sigma_camera_theta_deg: float = 3.0,
        camera_gate_sigma: float = 3.0,
        camera_flip_deg: float = 45.0,
        weak_board_sigma_xy_mm: float = WEAK_BOARD_SIGMA_XY_MM,
    ) -> None:
        """Create the service and its filter with the configured noise."""
        self.track_mm = max(1.0, float(track_mm))
        self.sigma_gyro = max(0.05, float(sigma_gyro_deg_s))
        self.sigma_camera_xy = max(0.5, float(sigma_camera_xy_mm))
        self.sigma_camera_theta = max(0.1, float(sigma_camera_theta_deg))
        self.camera_gate = max(0.0, float(camera_gate_sigma))
        self.camera_flip_deg = max(5.0, float(camera_flip_deg))
        self.weak_board_xy_mm = max(1.0, float(weak_board_sigma_xy_mm))
        self.filter = PoseKalmanFilter(
            sigma_speed_mm_s=sigma_speed_mm_s,
            sigma_wheel_omega_deg_s=sigma_wheel_omega_deg_s,
            sigma_pose_mm=sigma_pose_mm,
            sigma_pose_deg=sigma_pose_deg,
        )
        self._last_t: Optional[float] = None
        self._last_gyro_t: Optional[float] = None
        self._last_camera_t: Optional[float] = None
        self._was_moving = False
        self._at_rest = False
        self._last_v_mm_s = 0.0
        self._gyro_bias_deg_s = 0.0
        self._camera_pending: Optional[Tuple[float, float, float]] = None
        self._last_conf: Optional[float] = None
        self._camera_latency_s = 0.0
        #: last board sample, for the teleport (jump) check
        self._last_board_pose: Optional[Tuple[float, float, float]] = None
        self._samples = 0
        self._gyro_samples = 0
        self._camera_samples = 0
        self._camera_rejected = 0
        self._camera_flips = 0
        self._jumps = 0

    # ------------------------------------------------------------ feeding
    def reset(self, x_mm: float = 0.0, y_mm: float = 0.0, theta_deg: float = 0.0) -> None:
        """Hard reset: SET_POSE, RESET_ODOMETRY, a transport switch or boot."""
        self.filter.reset(x_mm, y_mm, theta_deg)
        self._last_t = None
        self._last_board_pose = None
        self._was_moving = False
        self._camera_pending = None
        self._samples = 0
        self._gyro_samples = 0

    def ingest(self, sample: WheelSample) -> None:
        """Fold one board odometry sample into the estimate (10 Hz typical)."""
        values = (
            sample.x_mm,
            sample.y_mm,
            sample.theta_deg,
            sample.v_left_mm_s,
            sample.v_right_mm_s,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return
        now = time.monotonic() if sample.t is None else float(sample.t)
        v_wheel, omega_wheel = twist_from_wheels(
            sample.v_left_mm_s, sample.v_right_mm_s, self.track_mm
        )
        # kept fresh even on the first sample: latency compensation and the
        # gyro bias logic read them regardless of the filter path below
        self._last_v_mm_s = v_wheel
        self._at_rest = abs(sample.v_left_mm_s) < 1.0 and abs(sample.v_right_mm_s) < 1.0
        if self._last_t is None:
            # first sample after a reset: adopt the measurement as-is
            self.filter.reset(sample.x_mm, sample.y_mm, sample.theta_deg)
            self._last_board_pose = (sample.x_mm, sample.y_mm, sample.theta_deg)
            self._last_t = now
            self._samples = 1
            return
        dt = min(MAX_DT_S, max(MIN_DT_S, now - self._last_t))
        self._last_t = now
        # discontinuity check: a step in the board's OWN pose stream is a
        # teleport (SET_POSE / RESET_ODOMETRY / a board reboot) and must be
        # adopted, not smeared over metres. Comparing the board pose with the
        # ESTIMATE would punish every rotation the wheels did not see (hand
        # placement, mecanum scrub, slip): that disagreement builds up smoothly
        # and is real information the gyro owns (Docs/09 6.3).
        reference = self._last_board_pose
        self._last_board_pose = (sample.x_mm, sample.y_mm, sample.theta_deg)
        if reference is not None:
            step_mm = math.hypot(sample.x_mm - reference[0], sample.y_mm - reference[1])
            step_deg = abs(wrap180(sample.theta_deg - reference[2]))
            if (
                step_mm > max(JUMP_MM, JUMP_SPEED_MM_S * dt)
                or step_deg > max(JUMP_DEG, JUMP_RATE_DEG_S * dt)
            ):
                self.filter.reset(sample.x_mm, sample.y_mm, sample.theta_deg)
                self._jumps += 1
                self._samples += 1
                return
        # 1. wheel prediction (forward speed; heading uses the filter's omega)
        self.filter.predict(v_wheel, dt)
        # 2. measurements: the board pose is a strong absolute reference, the
        #    wheel-derived yaw rate keeps omega honest between poses. When the
        #    robot stands still the pose is trusted tightly: dead reckoning has
        #    nothing to integrate, and the estimate must settle immediately
        #    instead of slowly catching up with the lag it built up driving.
        #    On the moving -> rest EDGE the stop is abrupt (pulses off) and the
        #    encoders are exact at zero speed, so the pose is near-snapped -
        #    otherwise a stopped robot would visibly creep for ~0.5 s.
        at_rest = abs(sample.v_left_mm_s) < 1.0 and abs(sample.v_right_mm_s) < 1.0
        stopping = at_rest and self._was_moving
        self._was_moving = not at_rest
        self._at_rest = at_rest
        self._last_v_mm_s = v_wheel
        if self.gyro_active() or self.camera_active():
            # An independent heading source is fresh and OWNS the heading. The
            # board pose integrates the SAME encoders as this filter, so only
            # its position adds information; applying its heading would undo
            # every rotation the wheels did not see (hand placement, mecanum
            # scrub, slip) and defeat the slip immunity the gyro is there for.
            self.filter.update_position(
                sample.x_mm,
                sample.y_mm,
                sigma_mm=self.weak_board_xy_mm,
            )
        else:
            self.filter.update_pose(
                sample.x_mm,
                sample.y_mm,
                sample.theta_deg,
                sigma_xy_mm=0.6 if stopping else (2.5 if at_rest else None),
                sigma_theta_deg=0.06 if stopping else (0.2 if at_rest else None),
            )
        self.filter.update_angular_rate(omega_wheel, self.filter.sigma_wheel_omega)
        self._samples += 1

    def add_gyro_yaw_rate(self, omega_deg_s: float, t: Optional[float] = None) -> None:
        """Fuse an IMU yaw-rate sample (deg/s, CCW positive).

        The gyro is a direct, low-noise measurement of the yaw-rate state, so
        it tightens heading tracking exactly where wheel slip would not be
        seen. While the robot is known to be at rest the samples also refine
        the gyro bias - slowly, and only while the measured rate is small: a
        hand rotation turns the body with the wheels still, and its rate must
        never be learned as bias (the heading would drift once it stops). The
        bias-corrected rate is what feeds the filter.
        """
        if not math.isfinite(float(omega_deg_s)):
            return
        raw = float(omega_deg_s)
        now = time.monotonic() if t is None else float(t)
        if self._at_rest and abs(raw) < BIAS_REFINE_MAX_DEG_S:
            self._gyro_bias_deg_s += BIAS_REFINE_ALPHA * (raw - self._gyro_bias_deg_s)
        self.filter.update_angular_rate(raw - self._gyro_bias_deg_s, self.sigma_gyro)
        self._last_gyro_t = now
        self._gyro_samples += 1

    def gyro_active(self) -> bool:
        """True while a fresh gyro sample is available."""
        return (
            self._last_gyro_t is not None
            and (time.monotonic() - self._last_gyro_t) <= GYRO_FRESH_S
        )

    def camera_active(self) -> bool:
        """True while a fresh accepted camera pose is available."""
        return (
            self._last_camera_t is not None
            and (time.monotonic() - self._last_camera_t) <= CAMERA_FRESH_S
        )

    # ------------------------------------------------------------- camera
    def update_camera_pose(
        self,
        x_mm: float,
        y_mm: float,
        theta_deg: float,
        *,
        confidence: Optional[float] = None,
        latency_s: float = 0.0,
    ) -> bool:
        """Fuse one absolute camera pose for OUR robot (field coordinates).

        Pipeline (Docs/09 section 6.4):

        * latency compensation: the fix is extrapolated forward by the
          measured pipeline delay using the filter's own speed/yaw rate;
        * a large discontinuity is held as a PENDING candidate and adopted
          only when the next sample confirms it (one teleport sample is an
          outlier, never the truth);
        * a Mahalanobis gate rejects implausible updates; when the heading is
          wildly off (planar-tag flip) the fix is retried position-only;
        * the optional confidence scales the measurement sigma.

        Returns True when the pose was used; rejected samples are counted in
        ``status()`` and never disturb the estimate.
        """
        try:
            x = float(x_mm)
            y = float(y_mm)
            theta = wrap180(float(theta_deg))
        except (TypeError, ValueError):
            return False
        if not all(math.isfinite(value) for value in (x, y, theta)):
            return False
        now = time.monotonic()
        sigma_xy = self.sigma_camera_xy
        sigma_theta = self.sigma_camera_theta
        if confidence is not None:
            try:
                conf = min(1.0, max(0.05, float(confidence)))
                scale = 1.0 / math.sqrt(conf)
                sigma_xy *= scale
                sigma_theta *= scale
            except (TypeError, ValueError):
                pass
        latency = 0.0 if latency_s is None else max(0.0, float(latency_s))
        latency = min(latency, MAX_DT_S)
        if latency > 0.0:
            # extrapolate the fix forward: measurement is old by ``latency``
            omega = self.filter.yaw_rate_deg_s()
            theta_mid = math.radians(theta + 0.5 * omega * latency)
            x += self._last_v_mm_s * math.cos(theta_mid) * latency
            y += self._last_v_mm_s * math.sin(theta_mid) * latency
            theta = wrap180(theta + omega * latency)
        est_x, est_y, est_theta = self.filter.pose()
        position_error = math.hypot(x - est_x, y - est_y)
        heading_error = abs(wrap180(theta - est_theta))
        # A teleport (position and heading far) or ANY gated-out sample must
        # be confirmed by a consistent follow-up before it is adopted: one
        # bad detection never moves the estimate, but a persistent correction
        # (large drift, or a re-acquisition after a gap) must not be rejected
        # forever either (Docs/09 section 6.4).
        teleport = position_error > JUMP_MM or (
            heading_error > JUMP_DEG and position_error > CAMERA_CONFIRM_MM
        )
        accepted = False
        if not teleport:
            if heading_error > self.camera_flip_deg:
                # planar-tag flip: position is usually fine, drop the heading
                accepted = self.filter.update_pose(
                    x, y, theta, sigma_xy, 1e6, gate_sigma=self.camera_gate
                )
                if accepted:
                    self._camera_flips += 1
            else:
                accepted = self.filter.update_pose(
                    x, y, theta, sigma_xy, sigma_theta, gate_sigma=self.camera_gate
                )
        if accepted:
            self._camera_pending = None
        else:
            self._camera_rejected += 1
            pending = self._camera_pending
            confirmed = (
                pending is not None
                and math.hypot(x - pending[0], y - pending[1]) <= CAMERA_CONFIRM_MM
                and abs(wrap180(theta - pending[2])) <= CAMERA_CONFIRM_DEG
            )
            if confirmed:
                # a consistent repeat: adopt the correction as the new truth
                self._camera_pending = None
                self.filter.reset(x, y, theta)
                self._jumps += 1
            else:
                self._camera_pending = (x, y, theta)
                return False
        self._last_camera_t = now
        self._camera_samples += 1
        self._last_conf = None if confidence is None else float(confidence)
        self._camera_latency_s = latency
        return True

    # ------------------------------------------------------------ reading
    def pose(self) -> Tuple[float, float, float]:
        """(x_mm, y_mm, theta_deg) - the estimate consumers should use."""
        x, y, theta = self.filter.pose()
        return x, y, wrap180(theta)

    def status(self) -> Dict:
        """Diagnostics: source mix, uncertainty, sample/rejection counts."""
        gyro_active = self.gyro_active()
        camera_active = self.camera_active()
        mode = "wheel"
        if gyro_active:
            mode += "+gyro"
        if camera_active:
            mode += "+camera"
        camera_age = (
            None
            if self._last_camera_t is None
            else round(time.monotonic() - self._last_camera_t, 3)
        )
        return {
            "mode": mode,
            "samples": self._samples,
            "gyro_samples": self._gyro_samples,
            "camera_samples": self._camera_samples,
            "camera_rejected": self._camera_rejected,
            "camera_flips": self._camera_flips,
            "camera_age_s": camera_age,
            "camera_confidence": self._last_conf,
            "camera_latency_s": round(self._camera_latency_s, 3),
            "gyro_bias_deg_s": round(self._gyro_bias_deg_s, 3),
            "jumps": self._jumps,
            "sigma_mm": round(self.filter.position_sigma_mm(), 2),
            "sigma_deg": round(self.filter.heading_sigma_deg(), 3),
            "yaw_rate_deg_s": round(self.filter.yaw_rate_deg_s(), 2),
            "track_mm": self.track_mm,
        }
