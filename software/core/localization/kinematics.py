"""Wheel odometry kinematics (pure, portable).

The robot is a four-wheel mecanum platform with a single wheel-travel axis
per side (only left/right odometry exists), so forward motion and rotation
are observable and the body centre follows the differential-drive model:

* ``v     = (v_left + v_right) / 2``          linear speed [mm/s]
* ``omega = (v_right - v_left) / track``      yaw rate [rad/s]

The module keeps the interface in millimetres and degrees to match the board
protocol; trigonometry converts where needed. Nothing here touches I/O, the
robot link or asyncio, so it can be exercised with plain unit tests.

File:    software/core/localization/kinematics.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import math
from typing import Tuple


def wrap180(angle_deg: float) -> float:
    """Normalise an angle to (-180, 180] degrees."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def twist_from_wheels(v_left_mm_s: float, v_right_mm_s: float, track_mm: float) -> Tuple[float, float]:
    """Body twist from the two wheel speeds.

    Returns ``(v_mm_s, omega_deg_s)``. ``omega`` is positive for a
    counter-clockwise rotation (left wheel backwards / right wheel forwards).
    """
    track = max(1.0, float(track_mm))
    v = 0.5 * (float(v_left_mm_s) + float(v_right_mm_s))
    omega = math.degrees((float(v_right_mm_s) - float(v_left_mm_s)) / track)
    return v, omega


def integrate_pose(
    x_mm: float,
    y_mm: float,
    theta_deg: float,
    v_mm_s: float,
    omega_deg_s: float,
    dt_s: float,
) -> Tuple[float, float, float]:
    """Dead-reckon one step with the midpoint (2nd order) rule.

    Midpoint integration halves the heading error of the plain Euler step on
    curves and matches what the odometry documentation promises for the
    on-board integration.
    """
    theta_mid = math.radians(theta_deg + 0.5 * omega_deg_s * dt_s)
    x_next = x_mm + v_mm_s * math.cos(theta_mid) * dt_s
    y_next = y_mm + v_mm_s * math.sin(theta_mid) * dt_s
    theta_next = wrap180(theta_deg + omega_deg_s * dt_s)
    return x_next, y_next, theta_next
