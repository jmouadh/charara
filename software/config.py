"""Environment configuration shared by the console and the brain.

All values can be overridden through environment variables (see
``docker/.env.example`` and ``docker/compose.yml``). Import ``settings`` -
the object is built once.

File:    software/config.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

#: Repository root (``charara/`` in the checkout) - default home of the
#: ``strategies/`` storage directory.
REPO_ROOT = Path(__file__).resolve().parents[1]


def _env_int(name: str, default: int) -> int:
    """Read environment variable ``name`` as an integer, falling back to ``default``."""
    try:
        return int(os.environ.get(name, str(default)), 0)
    except ValueError:
        return default


def _env_float(name: str, default: float) -> float:
    """Read environment variable ``name`` as a float, falling back to ``default``."""
    try:
        return float(os.environ.get(name, str(default)))
    except ValueError:
        return default


def _env_bool(name: str, default: bool) -> bool:
    """Read environment variable ``name`` as a boolean, falling back to ``default``.

    Accepts the usual spellings: ``1/0``, ``true/false``, ``yes/no``,
    ``on/off`` (case-insensitive).
    """
    text = os.environ.get(name, "").strip().lower()
    if not text:
        return default
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    return default


def _env_pose(
    name: str, default: Optional[Tuple[float, float, float]]
) -> Optional[Tuple[float, float, float]]:
    """Read ``"x,y,theta_deg"`` from the environment; ``default`` when unset."""
    text = os.environ.get(name, "").strip()
    if not text:
        return default
    try:
        parts = [float(part) for part in text.replace(";", ",").split(",")]
        if len(parts) == 3:
            return (parts[0], parts[1], parts[2])
    except ValueError:
        pass
    return default


@dataclass(frozen=True)
class Settings:
    """Runtime settings resolved from the environment."""

    # Robot link ---------------------------------------------------------
    transport: str = os.environ.get("ROBOT_TRANSPORT", "i2c").strip().lower()
    i2c_bus: int = _env_int("I2C_BUS", 1)
    i2c_address: int = _env_int("I2C_ADDRESS", 0x0A)
    i2c_retries: int = _env_int("I2C_RETRIES", 5)
    i2c_retry_delay_s: float = _env_float("I2C_RETRY_DELAY_MS", 5.0) / 1000.0

    # Field --------------------------------------------------------------
    field_width_mm: float = _env_float("FIELD_WIDTH_MM", 3000.0)
    field_height_mm: float = _env_float("FIELD_HEIGHT_MM", 2000.0)

    # Planning defaults (overridable per request) -------------------------
    default_speed_mm_s: float = _env_float("DEFAULT_SPEED_MM_S", 500.0)
    default_accel_mm_s2: float = _env_float("DEFAULT_ACCEL_MM_S2", 700.0)
    default_decel_mm_s2: float = _env_float("DEFAULT_DECEL_MM_S2", 700.0)
    default_lat_accel_mm_s2: float = _env_float("DEFAULT_LAT_ACCEL_MM_S2", 300.0)
    default_inflation_mm: float = _env_float("DEFAULT_INFLATION_MM", 180.0)
    default_grid_mm: float = _env_float("DEFAULT_GRID_MM", 50.0)
    default_goto_step_mm: float = _env_float("DEFAULT_GOTO_STEP_MM", 250.0)
    default_step_mm: float = _env_float("DEFAULT_STEP_MM", 50.0)
    default_fit_tol_mm: float = _env_float("DEFAULT_FIT_TOL_MM", 8.0)
    #: Extra clearance kept around a waypoint [mm]. 0 lets a waypoint stand
    #: half the robot body side from an obstacle / the field border; larger
    #: values keep stop points farther out. The console sends this value with
    #: every plan request (a request that omits it keeps the legacy full
    #: routing clearance).
    default_waypoint_margin_mm: float = _env_float("DEFAULT_WAYPOINT_MARGIN_MM", 0.0)

    # Telemetry / executor ------------------------------------------------
    telemetry_hz: float = _env_float("TELEMETRY_HZ", 10.0)
    deadman_ms: int = _env_int("DEADMAN_MS", 1500)
    exec_timeout_slack_s: float = _env_float("EXEC_TIMEOUT_SLACK_S", 8.0)

    # Localization ---------------------------------------------------------
    # Odometry wheel track used by the RPi-side estimator [mm]. Mirrors the
    # board's settings register 7 default (the odom encoder spacing measured
    # on the robot); a future release can sync it from the board's settings
    # store instead of the environment.
    odom_track_mm: float = _env_float("ODOM_TRACK_MM", 170.0)

    # Sensor fusion (Docs/09): camera pose reports and the optional MPU6050.
    # Every source is optional - the defaults below only take effect when the
    # corresponding hardware/stream is actually present and healthy.
    camera_poses_enabled: bool = _env_bool("CAMERA_POSES", True)
    camera_sigma_xy_mm: float = _env_float("CAMERA_SIGMA_XY_MM", 10.0)
    camera_sigma_theta_deg: float = _env_float("CAMERA_SIGMA_THETA_DEG", 3.0)
    camera_gate_sigma: float = _env_float("CAMERA_GATE_SIGMA", 3.0)
    camera_flip_deg: float = _env_float("CAMERA_FLIP_DEG", 45.0)
    camera_latency_max_s: float = _env_float("CAMERA_LATENCY_MAX_S", 0.5)
    imu_enabled: bool = _env_bool("IMU", True)
    #: Yaw axis of the MPU6050: "auto" (default) detects it from gravity at
    #: boot - an upside-down mount (sensor facing down) becomes "-z" - or
    #: force it with x, y, z, -x, -y, -z.
    imu_yaw_axis: str = os.environ.get("IMU_YAW_AXIS", "auto").strip().lower()
    #: Dedicated bus: I2C6 on GPIO22/23 (dtoverlay=i2c6 -> /dev/i2c-6).
    imu_bus: int = _env_int("IMU_BUS", 6)
    imu_address: int = _env_int("IMU_ADDRESS", 0x68)
    imu_rate_hz: float = _env_float("IMU_RATE_HZ", 100.0)
    # Spin-calibrated gyro scale (1.0 = factory trim). Measure it per robot
    # with the manual spin procedure (Docs/09 section 9.3): rotate the robot
    # by hand over N full turns and set GYRO_SCALE = 360*N / integrated_deg.
    gyro_scale: float = _env_float("GYRO_SCALE", 1.0)
    gyro_sigma_deg_s: float = _env_float("GYRO_SIGMA_DEG_S", 1.5)
    gyro_stale_ms: int = _env_int("GYRO_STALE_MS", 200)
    gyro_stream_enabled: bool = _env_bool("GYRO_STREAM", True)
    gyro_stream_hz: float = _env_float("GYRO_STREAM_HZ", 50.0)
    #: Host gate for the streaming follower backend (Track B, Docs/10).
    #: OFF (default) keeps the segment executor the only motion backend; ON
    #: makes the console and the brain run plans with the executor FOLLOW
    #: mode (the plan polyline drives with SET_VELOCITY frames) when the
    #: board firmware accepts the silent stream (>= 3.4; the mock always
    #: does). The capability check lives in the session; a requested stream
    #: on an unsupported board is refused - never silently downgraded.
    streaming_exec_enabled: bool = _env_bool("STREAMING_EXEC", False)
    reanchor_enabled: bool = _env_bool("REANCHOR", True)
    reanchor_mm: float = _env_float("REANCHOR_MM", 8.0)
    reanchor_deg: float = _env_float("REANCHOR_DEG", 0.7)
    reanchor_interval_s: float = _env_float("REANCHOR_INTERVAL_S", 1.0)

    # Strategies -----------------------------------------------------------
    # Where strategy documents live (shared by the console and the brain):
    # one JSON file per strategy, see ``software.core.mission.strategy``.
    strategy_dir: str = os.environ.get("STRATEGY_DIR", str(REPO_ROOT / "strategies"))

    # Teams ----------------------------------------------------------------
    # Eurobot plays two sides whose names/colours rotate between editions.
    # ``TEAM`` is which side WE play; the labels are display names only. The
    # match-start poses are ``x,y,theta_deg`` in mm/deg (this edition: both
    # robots sit at the top, inside their coloured corner rectangle). When
    # ``TEAM_START_BLUE`` is not set it MIRRORS the yellow start about the
    # field's vertical centre line - the symmetric default of an even table.
    #: Adopt the team match-start pose at startup when the board still
    #: reports its boot pose (0,0,0): the robot then starts inside its start
    #: rectangle instead of the field corner. Set START_POSE=0 to disable.
    start_pose_enabled: bool = _env_bool("START_POSE", True)
    team: str = os.environ.get("TEAM", "yellow").strip().lower()
    team_label_yellow: str = os.environ.get("TEAM_LABEL_YELLOW", "Yellow")
    team_label_blue: str = os.environ.get("TEAM_LABEL_BLUE", "Blue")
    team_start_yellow: Optional[Tuple[float, float, float]] = _env_pose(
        "TEAM_START_YELLOW", (315.0, 1800.0, -90.0)
    )
    team_start_blue: Optional[Tuple[float, float, float]] = _env_pose("TEAM_START_BLUE", None)

    @property
    def telemetry_period_s(self) -> float:
        """Telemetry polling period in seconds (1 / telemetry_hz)."""
        return 1.0 / max(self.telemetry_hz, 1.0)


settings = Settings()
