"""Curvature-aware velocity profile (trapezoid + lateral acceleration limit).

The robot firmware ramps every segment itself; the profile computed here is
used to (a) estimate the duration of a plan and (b) give each GOTO node a
cruise speed that the curves can actually be taken at.

File:    software/core/navigation/velocity.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

Point = Tuple[float, float]

MIN_SPEED_MM_S = 100.0


def curvature_at(points: Sequence[Point], index: int) -> float:
    """Discrete curvature [1/mm] at ``points[index]`` (0 at the ends)."""
    if index <= 0 or index >= len(points) - 1:
        return 0.0
    a, b, c = points[index - 1], points[index], points[index + 1]
    ab = math.dist(a, b)
    bc = math.dist(b, c)
    ca = math.dist(c, a)
    area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    denom = ab * bc * ca
    if denom < 1e-9 or area2 < 1e-9:
        return 0.0
    return 2.0 * area2 / denom


def speed_profile(
    points: Sequence[Point],
    v_max: float,
    accel: float,
    decel: float,
    lat_accel: float,
    min_speed: float = MIN_SPEED_MM_S,
) -> Tuple[List[float], float]:
    """Forward/backward pass profile. Returns (speeds per point, total seconds)."""
    n = len(points)
    if n < 2:
        return [min_speed], 0.0
    speeds: List[float] = []
    for i in range(n):
        kappa = curvature_at(points, i)
        v_curve = math.sqrt(lat_accel / kappa) if kappa > 1e-9 else math.inf
        speeds.append(max(min_speed, min(v_max, v_curve)))
    ds = [math.dist(points[i], points[i + 1]) for i in range(n - 1)]

    # Forward pass (acceleration limit)
    for i in range(1, n):
        reachable = math.sqrt(speeds[i - 1] ** 2 + 2.0 * accel * ds[i - 1])
        speeds[i] = min(speeds[i], reachable)
    # Backward pass (deceleration limit)
    for i in range(n - 2, -1, -1):
        reachable = math.sqrt(speeds[i + 1] ** 2 + 2.0 * decel * ds[i])
        speeds[i] = min(speeds[i], reachable)
    for i in range(n):
        speeds[i] = max(min_speed, speeds[i])

    total_time = 0.0
    for i in range(n - 1):
        v_avg = max(0.5 * (speeds[i] + speeds[i + 1]), 1.0)
        total_time += ds[i] / v_avg
    return speeds, total_time
