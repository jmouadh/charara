"""Request/response bodies of the REST API (pydantic).

File:    software/console/api/routes/models.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from pydantic import BaseModel, Field


class PoseIn(BaseModel):
    x_mm: float
    y_mm: float
    theta_deg: float = 0.0


class MoveDriveIn(BaseModel):
    distance_mm: float
    speed_mm_s: int = 150


class MoveTurnIn(BaseModel):
    angle_deg: float
    speed_mm_s: int = 100


class MoveStrafeIn(BaseModel):
    distance_mm: float
    speed_mm_s: int = 120


class MoveGotoIn(BaseModel):
    x_mm: float
    y_mm: float
    theta_deg: float = 0.0
    speed_mm_s: int = 200


class MoveArcIn(BaseModel):
    radius_mm: float
    sweep_deg: float
    speed_mm_s: int = 150


class GoalIn(BaseModel):
    x: float
    y: float
    theta_deg: Optional[float] = None


class ObstacleIn(BaseModel):
    kind: str = "rect"
    x: float = 0.0
    y: float = 0.0
    w: float = 0.0
    h: float = 0.0
    r: float = 0.0


class PlanIn(BaseModel):
    elements: List[Dict[str, Any]] = Field(default_factory=list)
    strokes: List[List[List[float]]] = Field(default_factory=list)  # legacy shorthand
    goal: Optional[GoalIn] = None
    obstacles: List[ObstacleIn] = Field(default_factory=list)
    options: Dict[str, Any] = Field(default_factory=dict)
    # re-planning a run uses it for the restarted execution (hardware watchdog)
    deadman_ms: Optional[int] = None


class ExecuteIn(BaseModel):
    segments: Optional[List[Dict[str, Any]]] = None
    deadman_ms: Optional[int] = None
    #: the plan's sampled polyline: when present and STREAMING_EXEC is on,
    #: the run drives it with the streaming follower backend (Track B)
    follow_path: Optional[List[List[float]]] = None


class LimitsIn(BaseModel):
    max_speed_mm_s: int
    accel_mm_s2: int
    decel_mm_s2: int


class DeadmanIn(BaseModel):
    timeout_ms: int


class ImuScaleIn(BaseModel):
    """Gyro scale write: a direct value, or derived from a finished spin test."""

    scale: Optional[float] = None
    turns_done: Optional[float] = None


class ImuNoiseIn(BaseModel):
    """Yaw-rate noise measurement window [s] (clamped service-side)."""

    seconds: float = 2.0


class SquareTestIn(BaseModel):
    """Square acceptance test parameters (clamped service-side)."""

    side_mm: float = 500.0
    speed_mm_s: int = 150


class ArcTestIn(BaseModel):
    """Quarter-arc acceptance test parameters (clamped service-side)."""

    radius_mm: float = 300.0
    speed_mm_s: int = 150


class ChainAuditIn(BaseModel):
    """QUEUE_ARC chain audit parameters (clamped service-side)."""

    length_mm: float = 300.0
    radius_mm: float = 300.0
    speed_mm_s: int = 150


class RestDriftIn(BaseModel):
    """Stationary drift audit window [s] (clamped 3..120 service-side)."""

    seconds: float = 30.0


class TurnRepeatIn(BaseModel):
    """Turn-repeatability audit: N turns per direction (clamped service-side)."""

    count_per_dir: int = 2
    angle_deg: float = 90.0
    speed_mm_s: int = 120


class LoopAuditIn(BaseModel):
    """Closed-loop drift audit: repeated square loops (clamped service-side)."""

    loops: int = 2
    side_mm: float = 500.0
    speed_mm_s: int = 150


class SpeedRampIn(BaseModel):
    """Speed/ramp audit: one long straight, sampled (clamped service-side)."""

    distance_mm: float = 1200.0
    speed_mm_s: int = 400


class EncoderCheckIn(BaseModel):
    """Encoder direction/scale check parameters (clamped service-side)."""

    distance_mm: float = 300.0
    speed_mm_s: int = 150


class FaultDrillIn(BaseModel):
    """Guided fault drill: ``deadman`` or ``estop`` (clamped service-side)."""

    kind: str = "estop"
    distance_mm: float = 250.0
    speed_mm_s: int = 120


class DisplayTextIn(BaseModel):
    line: int = 0
    offset: int = 0
    text: str


class DisplayModeIn(BaseModel):
    mode: int


class FieldIn(BaseModel):
    width_mm: float
    height_mm: float


class ModeIn(BaseModel):
    transport: str


class TeamIn(BaseModel):
    """Body of POST /api/team - which side we play."""

    team: str


class ConfigWriteIn(BaseModel):
    values: Dict[str, Any]  # settings key -> engineering value


class StrategyIn(BaseModel):
    """Body of PUT /api/strategies/{name} - the drawing + settings to save.

    ``kind`` selects the document kind: ``"elements"`` (v1 drawing, default)
    or ``"bt"`` (v2 behavior tree - then ``tree`` is required and validated
    at save time, so the brain never loads an unpalayable document).
    """

    elements: List[Dict[str, Any]] = Field(default_factory=list)
    options: Dict[str, Any] = Field(default_factory=dict)
    goal: Optional[Dict[str, Any]] = None
    obstacles: List[Dict[str, Any]] = Field(default_factory=list)
    start_pose: Optional[List[float]] = None  # [x, y, theta] the mission starts at
    notes: str = ""
    kind: str = "elements"
    tree: Optional[Dict[str, Any]] = None
