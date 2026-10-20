"""Pose Kalman filter (pure Python, no dependencies).

State vector (4): ``[x_mm, y_mm, theta_deg, omega_deg_s]``

* ``x, y``        body-centre position [mm]
* ``theta``       heading [deg], CCW positive
* ``omega``       yaw rate [deg/s] - the state the IMU will measure directly

Predict: the wheel twist (``v`` from the wheels, heading propagated with the
filter's own ``omega``) advances the pose with a midpoint step; the wheel
speed noise enters through the linearised control matrix.

Updates (any subset, any order):

* ``update_pose``     ``z = [x, y, theta]`` - the STM32 dead-reckoned pose
                      as a measurement (strong position/heading reference);
* ``update_angular_rate``  ``z = [omega]`` - the wheel-derived yaw rate today,
                      a **gyroscope** tomorrow: the same measurement function
                      takes the IMU's yaw rate with a much smaller sigma.

Only plain lists/loats are used so the filter runs anywhere (Pi, PC, tests).

File:    software/core/localization/kalman.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence

Matrix = List[List[float]]


# --------------------------------------------------------- small linalg ----
def _zeros(rows: int, cols: int) -> Matrix:
    """Return a 4x4 zero matrix."""
    return [[0.0] * cols for _ in range(rows)]


def _identity(size: int) -> Matrix:
    """Return the 4x4 identity matrix."""
    return [[1.0 if r == c else 0.0 for c in range(size)] for r in range(size)]


def _matmul(a: Matrix, b: Matrix) -> Matrix:
    """Multiply two 4x4 matrices."""
    rows, inner, cols = len(a), len(b), len(b[0])
    out = _zeros(rows, cols)
    for r in range(rows):
        row_a = a[r]
        row_out = out[r]
        for k in range(inner):
            factor = row_a[k]
            if factor == 0.0:
                continue
            row_b = b[k]
            for c in range(cols):
                row_out[c] += factor * row_b[c]
    return out


def _transpose(a: Matrix) -> Matrix:
    """Transpose a 4x4 matrix."""
    return [list(col) for col in zip(*a)]


def _add(a: Matrix, b: Matrix) -> Matrix:
    """Element-wise matrix sum."""
    return [[x + y for x, y in zip(ra, rb)] for ra, rb in zip(a, b)]


def _sub(a: Matrix, b: Matrix) -> Matrix:
    """Element-wise matrix difference."""
    return [[x - y for x, y in zip(ra, rb)] for ra, rb in zip(a, b)]


def _scale(a: Matrix, factor: float) -> Matrix:
    """Scale every element of a matrix by ``factor``."""
    return [[value * factor for value in row] for row in a]


def _inverse(a: Matrix) -> Optional[Matrix]:
    """Gauss-Jordan inverse with partial pivoting; ``None`` when singular."""
    n = len(a)
    identity = _identity(n)
    work = [list(row) + identity[i] for i, row in enumerate(a)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(work[r][col]))
        if abs(work[pivot][col]) < 1e-12:
            return None
        work[col], work[pivot] = work[pivot], work[col]
        factor = work[col][col]
        work[col] = [value / factor for value in work[col]]
        for r in range(n):
            if r == col:
                continue
            f = work[r][col]
            if f == 0.0:
                continue
            work[r] = [value - f * other for value, other in zip(work[r], work[col])]
    return [row[n:] for row in work]


# ------------------------------------------------------------- the filter --
class PoseKalmanFilter:
    """4-state pose filter (see the module docstring for the model)."""

    def __init__(
        self,
        sigma_speed_mm_s: float = 40.0,
        sigma_wheel_omega_deg_s: float = 20.0,
        sigma_pose_mm: float = 8.0,
        sigma_pose_deg: float = 0.8,
    ) -> None:
        """Initialize the filter state and its process-noise model."""
        self.sigma_speed = max(1e-3, float(sigma_speed_mm_s))
        self.sigma_wheel_omega = max(1e-3, float(sigma_wheel_omega_deg_s))
        self.sigma_pose_mm = max(1e-3, float(sigma_pose_mm))
        self.sigma_pose_deg = max(1e-3, float(sigma_pose_deg))
        self.x: List[float] = [0.0, 0.0, 0.0, 0.0]
        self.p: Matrix = _identity(4)

    # ------------------------------------------------------------- state
    def reset(self, x_mm: float = 0.0, y_mm: float = 0.0, theta_deg: float = 0.0) -> None:
        """Reset the state to the given pose with the initial covariance."""
        self.x = [float(x_mm), float(y_mm), _wrap180(float(theta_deg)), 0.0]
        # a fresh pose is a strong belief: allow the filter to move away
        # quickly once measurements contradict it
        self.p = [
            [25.0, 0.0, 0.0, 0.0],
            [0.0, 25.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 100.0],
        ]

    @property
    def position(self) -> tuple:
        """Current (x, y) position in millimetres."""
        return (self.x[0], self.x[1])

    def pose(self) -> tuple:
        """Current (x, y, theta) in millimetres and degrees."""
        return (self.x[0], self.x[1], self.x[2])

    def yaw_rate_deg_s(self) -> float:
        """Estimated yaw rate in degrees per second."""
        return self.x[3]

    def position_sigma_mm(self) -> float:
        """1-sigma position uncertainty in millimetres."""
        return math.sqrt(max(0.0, self.p[0][0] + self.p[1][1]))

    def heading_sigma_deg(self) -> float:
        """1-sigma heading uncertainty in degrees."""
        return math.sqrt(max(0.0, self.p[2][2]))

    # --------------------------------------------------------- prediction
    def predict(self, v_mm_s: float, dt_s: float) -> None:
        """Advance by ``dt_s`` with forward speed ``v_mm_s`` from the wheels."""
        if dt_s <= 0.0:
            return
        x, y, theta_deg, omega = self.x
        theta_mid = math.radians(theta_deg + 0.5 * omega * dt_s)
        cos_mid, sin_mid = math.cos(theta_mid), math.sin(theta_mid)
        self.x = [
            x + v_mm_s * cos_mid * dt_s,
            y + v_mm_s * sin_mid * dt_s,
            _wrap180(theta_deg + omega * dt_s),
            omega,
        ]
        # linearised transition around the midpoint heading
        scale = math.pi / 180.0
        jac = _identity(4)
        jac[0][2] = -v_mm_s * sin_mid * dt_s * scale
        jac[0][3] = -v_mm_s * sin_mid * dt_s * dt_s * 0.5 * scale
        jac[1][2] = v_mm_s * cos_mid * dt_s * scale
        jac[1][3] = v_mm_s * cos_mid * dt_s * dt_s * 0.5 * scale
        jac[2][3] = dt_s
        # control noise: (v, omega) uncertainty pushed through the Jacobians
        control = _zeros(4, 2)
        control[0][0] = cos_mid * dt_s
        control[0][1] = -v_mm_s * sin_mid * dt_s * dt_s * 0.5 * scale
        control[1][0] = sin_mid * dt_s
        control[1][1] = v_mm_s * cos_mid * dt_s * dt_s * 0.5 * scale
        control[2][1] = dt_s
        control[3][1] = 1.0
        # Q = control * diag(sigma_v^2, sigma_w^2) * control^T
        weighted = _zeros(4, 2)
        for r in range(4):
            weighted[r][0] = control[r][0] * self.sigma_speed * self.sigma_speed
            weighted[r][1] = control[r][1] * self.sigma_wheel_omega * self.sigma_wheel_omega
        q = _matmul(weighted, _transpose(control))
        self.p = _add(_matmul(_matmul(jac, self.p), _transpose(jac)), q)

    # ------------------------------------------------------------ updates
    def update_pose(
        self,
        x_mm: float,
        y_mm: float,
        theta_deg: float,
        sigma_xy_mm: Optional[float] = None,
        sigma_theta_deg: Optional[float] = None,
        rotation_offset_deg: Optional[float] = None,
        gate_sigma: float = 0.0,
    ) -> bool:
        """Fuse a full pose measurement (the board dead-reckoned pose).

        ``rotation_offset_deg`` can pre-rotate the measured heading - used at
        boot when the board origin differs from the world origin.
        ``gate_sigma`` > 0 rejects an implausible update (Mahalanobis gate);
        returns True when the measurement was applied.
        """
        sigma_xy = self.sigma_pose_mm if sigma_xy_mm is None else max(1e-3, float(sigma_xy_mm))
        sigma_theta = self.sigma_pose_deg if sigma_theta_deg is None else max(1e-3, float(sigma_theta_deg))
        theta = float(theta_deg) if rotation_offset_deg is None else float(theta_deg) + float(rotation_offset_deg)
        z = [float(x_mm), float(y_mm), _wrap180(theta)]
        h = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ]
        r = [
            [sigma_xy * sigma_xy, 0.0, 0.0],
            [0.0, sigma_xy * sigma_xy, 0.0],
            [0.0, 0.0, sigma_theta * sigma_theta],
        ]
        return self._update(z, h, r, gate_sigma)

    def update_position(
        self, x_mm: float, y_mm: float, sigma_mm: float, gate_sigma: float = 0.0
    ) -> bool:
        """Fuse a position-only measurement - the heading is left untouched.

        Used for the board's wheel-based pose while an independent heading
        source (gyro or camera) is fresh: the board's heading is the integral
        of the SAME encoders as this filter's prediction, so it carries no
        independent information, and correcting with it would undo every
        rotation the wheels did not see (hand placement, mecanum scrub, a
        slip) - exactly the rotations the gyro is there to keep.
        """
        sigma = max(1e-3, float(sigma_mm))
        z = [float(x_mm), float(y_mm)]
        h = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]
        r = [[sigma * sigma, 0.0], [0.0, sigma * sigma]]
        return self._update(z, h, r, gate_sigma)

    def update_angular_rate(
        self, omega_deg_s: float, sigma_deg_s: float, gate_sigma: float = 0.0
    ) -> bool:
        """Fuse a yaw-rate measurement (wheel-derived today, gyro tomorrow).

        ``gate_sigma`` > 0 rejects an implausible rate; returns True when
        the measurement was applied.
        """
        sigma = max(1e-3, float(sigma_deg_s))
        z = [float(omega_deg_s)]
        h = [[0.0, 0.0, 0.0, 1.0]]
        r = [[sigma * sigma]]
        return self._update(z, h, r, gate_sigma)

    def _update(self, z: Sequence[float], h: Matrix, r: Matrix, gate_sigma: float = 0.0) -> bool:
        """Apply one linear measurement update; False when the gate rejects it.

        ``gate_sigma`` > 0 enables a Mahalanobis gate: an update whose
        normalised innovation distance exceeds it is discarded entirely - a
        wrong absolute fix is far worse than a missing one. Returns True
        when the measurement was applied.
        """
        if not all(math.isfinite(value) for value in z):
            return False
        h_t = _transpose(h)
        s = _add(_matmul(_matmul(h, self.p), h_t), r)
        s_inv = _inverse(s)
        if s_inv is None:
            return False
        innovation = [
            float(z[i]) - sum(h[i][k] * self.x[k] for k in range(4)) for i in range(len(z))
        ]
        if len(innovation) == 3:
            innovation[2] = _wrap180(innovation[2])  # heading is circular
        elif len(innovation) == 1 and h[0][2] != 0.0:
            innovation[0] = _wrap180(innovation[0])
        if gate_sigma > 0.0:
            # d^2 = nu^T S^-1 nu (nu and S are tiny, the loops stay bounded)
            weighted = [
                sum(s_inv[i][j] * innovation[j] for j in range(len(innovation)))
                for i in range(len(innovation))
            ]
            d2 = sum(innovation[i] * weighted[i] for i in range(len(innovation)))
            if d2 > (gate_sigma * gate_sigma):
                return False
        gain = _matmul(_matmul(self.p, h_t), s_inv)
        correction = _matmul(gain, [[value] for value in innovation])
        self.x = [self.x[k] + correction[k][0] for k in range(4)]
        self.x[2] = _wrap180(self.x[2])
        if not all(math.isfinite(value) for value in self.x):
            self.x = [0.0, 0.0, 0.0, 0.0]
            self.p = _identity(4)
            return False
        # P = (I - K H) P   (Joseph form is unnecessary at this size/rate)
        identity = _identity(4)
        self.p = _matmul(_sub(identity, _matmul(gain, h)), self.p)
        return True


def _wrap180(angle_deg: float) -> float:
    """Normalise an angle to (-180, 180] degrees."""
    return ((angle_deg + 180.0) % 360.0) - 180.0
