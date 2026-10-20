"""Planning options: the per-request planning knobs (pure data).

Kept apart from the trajectory code so the option schema can grow without
touching the pipeline, and so the mission/API layers can read a request
without importing the whole planner.

File:    software/core/navigation/options.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

@dataclass
class PlanOptions:
    """Per-request planning knobs (UI exposes these)."""

    speed_mm_s: float = 500.0
    turn_speed_mm_s: float = 150.0
    accel_mm_s2: float = 700.0
    decel_mm_s2: float = 700.0
    lat_accel_mm_s2: float = 300.0
    goto_step_mm: float = 250.0
    step_mm: float = 50.0
    corner_radius_mm: float = 200.0
    node_chord_tol_mm: float = 6.0
    max_node_turn_deg: float = 15.0
    min_node_step_mm: float = 40.0
    fit_tol_mm: float = 8.0
    # native arc execution: sampled chains that fit a single circle (drawn
    # arcs, corner fillets) become one firmware ARC segment (motion +
    # rotation together) instead of a string of GOTO nodes. Consecutive arcs
    # share their junction, so a chain of fitted arcs drives as consecutive
    # ARC commands with no stop-and-turn node in between
    native_arcs: bool = True
    arc_fit_tol_mm: float = 8.0
    # 60 mm: the tightest radius the firmware REALLY drives well (its own
    # floor is 50) and the same floor the junction connectors use. A hand
    # drawn corner tighter than this cannot become an ARC at all - it stays
    # GOTO nodes and the cusp pass (see _collapse_cusps) turns the stutter
    # into ONE pivot.
    arc_min_radius_mm: float = 60.0
    arc_min_sweep_deg: float = 8.0
    arc_max_sweep_deg: float = 340.0
    # straight runs are emitted as VERY flat ARCs instead of a GOTO node
    # every ``goto_step_mm``: the board brakes to a stop at every GOTO node,
    # so a node chain turned every long straight into stop-and-go. A near
    # straight ARC (radius up to 32767 mm) drives the same geometry as one
    # blended segment and chains through QUEUE_ARC like any fitted arc.
    # ``line_pair_mm`` caps one flat S-pair (the two halves alternate side so
    # the run leaves and re-joins its direction), ``line_tol_mm`` is how far
    # the samples may stray from the span chord and still count as straight
    # (anything else keeps its GOTO nodes and the stop/turn fallback).
    native_lines: bool = True
    line_pair_mm: float = 700.0
    line_tol_mm: float = 3.0
    # robot footprint: the robot centre follows the path, so the body sweeps a
    # circle of half-diagonal radius; obstacles get half-diagonal + margin,
    # the field border gets half-diagonal (stay inside the game area)
    robot_width_mm: float = 300.0
    robot_height_mm: float = 300.0
    obstacle_margin_mm: float = 150.0
    # waypoint stop clearance: extra margin added to half the robot's larger
    # body side. 0 -> a waypoint may stand half the body side from an obstacle
    # or the field border; None -> legacy (full routing clearance). A close
    # waypoint is reached by a straight final leg, with the arrival angle
    # performed at a safe stand-off first.
    waypoint_margin_mm: Optional[float] = None
    # waypoints are swept smoothly: the corner fillet stays within this
    # distance of the exact waypoint (0 = knife-point exact crossing)
    waypoint_cut_mm: float = 50.0
    # mid-mission re-plan (dynamic obstacle): the elements are the TAIL of a
    # running program - the connector from the stop position is expected, so
    # the "first path starts away from the robot" warning is not raised
    resume_mid_mission: bool = False
    inflate_mm: float = 0.0          # > 0 overrides the computed inflation
    grid_mm: float = 50.0
    min_speed_mm_s: float = 100.0
    smooth: bool = True
    fit_enabled: bool = True
    avoid_obstacles: bool = True
    max_segments: int = 600

    @classmethod
    def from_dict(cls, data: Optional[Dict]) -> "PlanOptions":
        """Build options from a request dict, applying the defaults."""
        options = cls()
        if not data:
            return options
        for key, value in data.items():
            if value is not None and hasattr(options, key):
                setattr(options, key, value)
        return options
