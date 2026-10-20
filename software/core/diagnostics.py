"""Pure geometry and statistics for the console self-tests.

No I/O and no third-party imports: the acceptance-test endpoints in
``software/console/api/routes/selftest.py`` command the robot; these helpers
compute the expected poses (bench square, arc end pose), the pose errors and
the drift statistics they evaluate. Unit tested in ``tests/test_diagnostics.py``.

File:    software/core/diagnostics.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-09
"""

from __future__ import annotations

import math
from typing import Dict, List, Sequence, Tuple

#: a pose is (x_mm, y_mm, theta_deg) in the field frame
Pose = Tuple[float, float, float]


def wrap180(angle_deg: float) -> float:
    """Normalise an angle to [-180, 180) degrees (the project convention)."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def square_waypoints(
    x_mm: float, y_mm: float, theta_deg: float, side_mm: float
) -> List[Dict]:
    """The four GOTO targets of the bench square test (STM_docs/06 2.5).

    Corner 1 lies ``side_mm`` ahead of the start, the robot turns left at
    every corner, and the last corner snaps back to the exact start position
    with the heading rotated by +270 deg - the bench procedure's own
    sequence (headings theta, theta+90, theta+180, theta+270).
    """
    side = float(side_mm)
    corners: List[Tuple[float, float]] = [(float(x_mm), float(y_mm))]
    for turn in (0.0, 90.0, 180.0, 270.0):
        px, py = corners[-1]
        corners.append(
            (
                px + side * math.cos(math.radians(theta_deg + turn)),
                py + side * math.sin(math.radians(theta_deg + turn)),
            )
        )
    corners[-1] = (float(x_mm), float(y_mm))  # snap: float residue must not move the target
    return [
        {
            "x": corners[index + 1][0],
            "y": corners[index + 1][1],
            "th": wrap180(theta_deg + index * 90.0),
        }
        for index in range(4)
    ]


def arc_end_pose(
    x_mm: float, y_mm: float, theta_deg: float, radius_mm: float, sweep_deg: float
) -> Pose:
    """End pose of one firmware ARC from a start pose (signs like the board).

    Positive radius = centre on the robot's left (CCW sweep). The board
    drives the whole sweep as one move at constant curvature, so the end
    pose is exact geometry: the centre is ``start + R * left(theta)`` and
    the robot travels ``sweep_deg`` around it.
    """
    theta = math.radians(theta_deg)
    sweep = math.radians(sweep_deg)
    radius = float(radius_mm)
    centre_x = float(x_mm) + radius * math.cos(theta + math.pi / 2.0)
    centre_y = float(y_mm) + radius * math.sin(theta + math.pi / 2.0)
    return (
        centre_x + radius * math.cos(theta - math.pi / 2.0 + sweep),
        centre_y + radius * math.sin(theta - math.pi / 2.0 + sweep),
        wrap180(theta_deg + sweep_deg),
    )


def pose_error(actual: Sequence[float], expected: Sequence[float]) -> Dict:
    """Position [mm] and absolute heading [deg] error between two poses."""
    return {
        "pos_mm": math.hypot(
            float(actual[0]) - float(expected[0]), float(actual[1]) - float(expected[1])
        ),
        "heading_deg": abs(wrap180(float(actual[2]) - float(expected[2]))),
    }


def drift_stats(samples: Sequence[Tuple[float, float, float, float]]) -> Dict:
    """Maximum displacement/heading change of ``(t, x, y, theta)`` samples.

    Measured against the FIRST sample, so a slow monotone creep counts the
    same as a jump: the audit cares that a standing robot does not move.
    """
    if not samples:
        return {"span_s": 0.0, "max_pos_mm": 0.0, "max_heading_deg": 0.0, "samples": 0}
    _, x0, y0, th0 = samples[0]
    max_pos = 0.0
    max_heading = 0.0
    for _, x, y, theta in samples:
        max_pos = max(max_pos, math.hypot(x - x0, y - y0))
        max_heading = max(max_heading, abs(wrap180(theta - th0)))
    return {
        "span_s": round(float(samples[-1][0]) - float(samples[0][0]), 1),
        "max_pos_mm": round(max_pos, 2),
        "max_heading_deg": round(max_heading, 3),
        "samples": len(samples),
    }


def series_stats(values: Sequence[float]) -> Dict:
    """Mean and sample standard deviation of a numeric series."""
    count = len(values)
    if count == 0:
        return {"count": 0, "mean": 0.0, "sigma": 0.0}
    mean = sum(values) / count
    if count < 2:
        return {"count": count, "mean": mean, "sigma": 0.0}
    variance = sum((value - mean) ** 2 for value in values) / (count - 1)
    return {"count": count, "mean": mean, "sigma": math.sqrt(variance)}


def flat_sweep_deg(length_mm: float, radius_mm: float) -> float:
    """Sweep of a near-straight ARC that covers ``length_mm`` of travel.

    Snapped to the 0.1 deg wire quantum (the firmware re-derives its arc
    length from sweep x radius), with a 0.1 deg floor like the planner's
    near-straight segments.
    """
    sweep = math.degrees(float(length_mm) / max(1.0, float(radius_mm)))
    return max(0.1, round(sweep * 10.0) / 10.0)
