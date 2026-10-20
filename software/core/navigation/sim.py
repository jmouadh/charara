"""Deterministic kinematic simulator for path-following tests (Track B).

A bicycle-like point robot driven by the same controller the follower runs
on the real robot (:mod:`.pursuit`), integrated at a fixed time step. It
exists so the acceptance numbers of Track B (Docs/10 section 6.3) can be
measured WITHOUT the robot: tracking error on a circle, a filleted square
and an S-curve, all in milliseconds and fully deterministic.

The integration is exact for constant (v, omega) over one step - the same
closed form the firmware's odometry uses - so a straight command moves
exactly ``v * dt`` and a curvature command follows its circle exactly.

File:    software/core/navigation/sim.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-09
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from .pursuit import DEFAULT_PARAMS, PursuitParams, pursuit_step

Point = Tuple[float, float]


@dataclass
class KinematicSim:
    """Point-robot state integrated by exact (v, omega) steps."""

    x_mm: float
    y_mm: float
    theta_deg: float

    def pose(self) -> Tuple[float, float, float]:
        """Current ``(x_mm, y_mm, theta_deg)``."""
        return (self.x_mm, self.y_mm, self.theta_deg)

    def step(self, v_mm_s: float, omega_rad_s: float, dt_s: float) -> None:
        """Advance the state by one exact constant-twist step."""
        theta = math.radians(self.theta_deg)
        d_theta = omega_rad_s * dt_s
        if abs(omega_rad_s) > 1e-12:
            radius = v_mm_s / omega_rad_s
            self.x_mm += radius * (math.sin(theta + d_theta) - math.sin(theta))
            self.y_mm -= radius * (math.cos(theta + d_theta) - math.cos(theta))
        else:
            self.x_mm += v_mm_s * dt_s * math.cos(theta)
            self.y_mm += v_mm_s * dt_s * math.sin(theta)
        self.theta_deg = math.degrees(theta + d_theta)


def polyline_distance(points: Sequence[Point], x_mm: float, y_mm: float) -> float:
    """Shortest distance from a point to a polyline [mm]."""
    best = math.inf
    for i in range(len(points) - 1):
        ax, ay = points[i]
        bx, by = points[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        if seg <= 1e-9:
            dist = math.hypot(x_mm - ax, y_mm - ay)
        else:
            t = ((x_mm - ax) * (bx - ax) + (y_mm - ay) * (by - ay)) / (seg * seg)
            t = max(0.0, min(1.0, t))
            dist = math.hypot(x_mm - (ax + (bx - ax) * t), y_mm - (ay + (by - ay) * t))
        best = min(best, dist)
    if len(points) == 1:
        best = math.hypot(x_mm - points[0][0], y_mm - points[0][1])
    return best


def simulate_path(
    points: Sequence[Point],
    v_cruise_mm_s: float,
    params: PursuitParams = DEFAULT_PARAMS,
    *,
    start: Optional[Sequence[float]] = None,
    dt_s: float = 0.02,
    max_time_s: float = 120.0,
) -> Dict:
    """Close the pursuit loop over ``points`` and measure the result.

    ``start`` defaults to the first path point facing the second one (the
    follower's normal entry state). Returns a dict with the sampled
    trajectory, the maximum distance of the executed track from the path
    (``max_dev_mm``), the final distance to the last point
    (``final_error_mm``), the duration and whether the end was reached.
    """
    if len(points) < 2:
        raise ValueError("a path needs at least two points")
    if v_cruise_mm_s <= 0.0:
        raise ValueError("v_cruise_mm_s must be positive")
    if start is None:
        heading = math.degrees(
            math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0])
        )
        sim = KinematicSim(float(points[0][0]), float(points[0][1]), heading)
    else:
        sim = KinematicSim(float(start[0]), float(start[1]), float(start[2]))

    samples: List[Tuple[float, float, float, float]] = []
    done = False
    t = 0.0
    max_dev = 0.0
    while t < max_time_s:
        cmd = pursuit_step(points, sim.pose(), v_cruise_mm_s, params)
        if cmd is None:
            done = True
            break
        sim.step(cmd.v_mm_s, cmd.omega_rad_s, dt_s)
        t += dt_s
        max_dev = max(max_dev, polyline_distance(points, sim.x_mm, sim.y_mm))
        samples.append((round(t, 6), sim.x_mm, sim.y_mm, sim.theta_deg))

    final_error = math.hypot(sim.x_mm - points[-1][0], sim.y_mm - points[-1][1])
    return {
        "samples": samples,
        "max_dev_mm": max_dev,
        "final_error_mm": final_error,
        "duration_s": t,
        "done": done,
        "final_pose": sim.pose(),
    }
