"""RAMSETE-style tracking controller for a timed reference trajectory (B).

The optional refinement of :mod:`.pursuit`: where Pure Pursuit only knows
the PATH, a tracking controller also uses the reference VELOCITY of a
timed trajectory ``(x, y, theta, v, omega)``. Right law whenever the
planner (or a future vision servo) produces a full trajectory instead of a
geometric path.

Error coordinates (robot frame: x forward, y left; angles in rad)::

    e      = R(-theta) * (p_ref - p)      -> e_x along, e_y lateral
    eth    = wrap(theta_ref - theta)

Control law (linearizing form, all gains positive)::

    g_head  = 2 * zeta * omega_n
    g_lat   = omega_n^2 / max(|v_ref|, v_floor)
    v       = v_ref * cos(eth) + k_along * e_x
    omega   = omega_ref + g_lat * e_y + g_head * sin(eth)

The lateral/heading pair linearizes to ``e_y'' + g_head * e_y' +
g_lat * v_ref * e_y = 0`` - a second-order error decay with natural
frequency ``omega_n`` and damping ``zeta`` (the gain scheduling on
``v_ref`` keeps the behaviour consistent from creep to full speed). With
zero error the law degenerates to the reference twist exactly - the first
thing the tests assert.

File:    software/core/navigation/tracking.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-09
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple


@dataclass(frozen=True)
class RamseteGains:
    """Tracking-loop shaping (natural frequency and damping)."""

    #: error decay natural frequency [rad/s]
    omega_n: float = 1.2
    #: damping ratio (1 = critical; 0.7-0.8 avoids overshoot)
    zeta: float = 0.8
    #: longitudinal rejoin gain [1/s]
    k_along: float = 1.5
    #: gain-scheduling floor for the reference speed [mm/s] (a stopped
    #: reference has no meaningful v_ref; the floor keeps g_lat bounded)
    v_floor_mm_s: float = 50.0


DEFAULT_GAINS = RamseteGains()


def _wrap_rad(angle: float) -> float:
    """Wrap an angle to [-pi, +pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def ramsete_command(
    pose: Sequence[float],
    ref_pose: Sequence[float],
    ref_v_mm_s: float,
    ref_omega_rad_s: float,
    gains: RamseteGains = DEFAULT_GAINS,
) -> Tuple[float, float]:
    """One RAMSETE step: returns ``(v_mm_s, omega_rad_s)``.

    ``pose`` and ``ref_pose`` are ``(x_mm, y_mm, theta_deg)`` (theta CCW,
    project convention); the reference twist is in mm/s and rad/s. The
    returned command keeps the same units - the follower converts omega to
    deg/s for the wire.
    """
    theta = math.radians(float(pose[2]))
    theta_ref = math.radians(float(ref_pose[2]))
    dx = float(ref_pose[0]) - float(pose[0])
    dy = float(ref_pose[1]) - float(pose[1])
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    # position error expressed in the ROBOT frame (x forward, y left)
    e_x = cos_t * dx + sin_t * dy
    e_y = -sin_t * dx + cos_t * dy
    e_th = _wrap_rad(theta_ref - theta)
    g_head = 2.0 * gains.zeta * gains.omega_n
    v_scale = max(abs(ref_v_mm_s), gains.v_floor_mm_s)
    g_lat = (gains.omega_n * gains.omega_n) / v_scale
    v = ref_v_mm_s * math.cos(e_th) + gains.k_along * e_x
    omega = ref_omega_rad_s + g_lat * e_y + g_head * math.sin(e_th)
    return v, omega
