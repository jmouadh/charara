"""Typed views over the protocol responses (odometry, status).

File:    software/robot/models.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

from . import protocol as p


@dataclass
class Odometry:
    """QUERY_ODOMETRY payload in engineering units."""

    x_mm: float
    y_mm: float
    theta_deg: float
    v_left_mm_s: float
    v_right_mm_s: float
    state: int
    faults: int
    segments_done: int = 0  # motion segments completed by the board (chaining)

    @classmethod
    def parse(cls, payload: bytes) -> "Odometry":
        """Decode the QUERY_ODOMETRY payload."""
        return cls(
            x_mm=p.get_i32(payload, 0) / 1000.0,
            y_mm=p.get_i32(payload, 4) / 1000.0,
            theta_deg=p.get_i32(payload, 8) / 1000.0,
            v_left_mm_s=float(p.get_i16(payload, 12)),
            v_right_mm_s=float(p.get_i16(payload, 14)),
            state=p.get_u8(payload, 16),
            faults=p.get_u16(payload, 17),
            segments_done=p.get_u16(payload, 19) if len(payload) >= 21 else 0,
        )

    @property
    def state_name(self) -> str:
        """Human-readable motion state name."""
        return p.STATE_NAMES.get(self.state, f"STATE_{self.state}")

    @property
    def is_idle(self) -> bool:
        """True when the board reports no motion in progress."""
        return self.state == p.STATE_IDLE

    def describe_faults(self) -> List[str]:
        """Names of the active fault bits (empty when healthy)."""
        return list(p.decode_faults(self.faults))


@dataclass
class Status:
    """QUERY_STATUS payload (protocol v3)."""

    fw_major: int
    fw_minor: int
    uptime_ms: int
    state: int
    faults: int
    limits: int
    queue_depth: int
    last_seq: int
    last_status: int
    last_event: int
    last_event_aux: int
    reset_cause: int
    cfg_map_version: int
    cfg_dirty: bool
    gyro_fresh: int = 0

    @classmethod
    def parse(cls, payload: bytes) -> "Status":
        """Decode the QUERY_STATUS payload."""
        version = p.get_u16(payload, 0)
        return cls(
            fw_major=(version >> 8) & 0xFF,
            fw_minor=version & 0xFF,
            uptime_ms=p.get_u32(payload, 2),
            state=p.get_u8(payload, 6),
            faults=p.get_u16(payload, 7),
            limits=p.get_u8(payload, 9),
            queue_depth=p.get_u8(payload, 10),
            last_seq=p.get_u8(payload, 11),
            last_status=p.get_u8(payload, 12),
            last_event=p.get_u8(payload, 13),
            last_event_aux=p.get_u8(payload, 14),
            reset_cause=p.get_u8(payload, 15),
            cfg_map_version=p.get_u8(payload, 16),
            cfg_dirty=bool(p.get_u8(payload, 17)),
            gyro_fresh=p.get_u8(payload, 18) if len(payload) > 18 else 0,
        )

    @property
    def state_name(self) -> str:
        """Human-readable motion state name."""
        return p.STATE_NAMES.get(self.state, f"STATE_{self.state}")

    @property
    def last_event_name(self) -> str:
        """Name of the most recent board event."""
        return p.event_name(self.last_event) if self.last_event else "NONE"

    @property
    def reset_cause_name(self) -> str:
        """Name of the recorded boot cause."""
        return p.event_name(self.reset_cause) if self.reset_cause else "UNKNOWN"
