"""Robot pose store: ArUco camera reports for us and the opponent.

The camera system (Docs/09 section 8) sends ONE message per detection cycle
with both robots' poses in field coordinates::

    {"type": "poses",
     "us":       {"x_mm": 1245.2, "y_mm": 803.4, "theta_deg": 87.5, "conf": 0.9},
     "opponent": {"x_mm": 2450.0, "y_mm": 1210.5, "theta_deg": -95.2},
     "t": 1759680000123.4,
     "ttl_ms": 400}

Unlike :class:`~.vision.VisionStore` (obstacles), poses are kept at **raw
precision**: the obstacle store quantizes to 10 mm to suppress re-plan
churn, which would inject steps into the Kalman filter. Here a millimetre
is meaningful.

Freshness: a snapshot is ``fresh`` while ``ttl_ms`` has not elapsed since it
was received; a stale snapshot still exists (for diagnostics) but
:meth:`active` and :meth:`fresh` report it as absent, so the fusion and the
planner policies degrade without any explicit clearing.

File:    software/core/world/poses.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-05
"""

from __future__ import annotations

import math
import time
from typing import Dict, Optional

#: robots a snapshot may carry (keys of the message)
ROBOT_KEYS = ("us", "opponent")


def _wrap180(angle_deg: float) -> float:
    """Normalise an angle to (-180, 180] degrees."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def _clean_robot(raw: object) -> Optional[Dict]:
    """Validate one robot entry; None when it is unusable.

    A malformed entry is dropped (better no measurement than a wrong one);
    ``conf`` is optional and clamped to [0, 1]; the heading wraps to
    (-180, 180].
    """
    if not isinstance(raw, dict):
        return None
    try:
        entry = {
            "x_mm": float(raw["x_mm"]),
            "y_mm": float(raw["y_mm"]),
            "theta_deg": _wrap180(float(raw["theta_deg"])),
        }
    except (TypeError, ValueError, KeyError):
        return None
    if not all(math.isfinite(value) for value in entry.values()):
        return None
    confidence = raw.get("conf")
    if confidence is not None:
        try:
            entry["conf"] = min(1.0, max(0.0, float(confidence)))
        except (TypeError, ValueError):
            pass
    return entry


class PoseStore:
    """Newest robot-pose snapshot reported by the camera system."""

    DEFAULT_TTL_MS = 400.0
    MIN_TTL_MS = 100.0

    def __init__(self) -> None:
        """Create an empty pose store."""
        self._poses: Dict[str, Dict] = {}
        self._ttl_ms = self.DEFAULT_TTL_MS
        self._updated_at = 0.0
        self._updated_epoch_ms = 0.0
        self._reported_t: Optional[float] = None  # the camera's own timestamp
        self._source = ""
        self._source_client = ""  # what the sender calls itself (e.g. "camera")
        self.version = 0  # bumped on every effective change

    # -- writes -----------------------------------------------------------
    def update(
        self,
        us: object = None,
        opponent: object = None,
        ttl_ms: Optional[float] = None,
        source: str = "",
        t_ms: Optional[float] = None,
        client: str = "",
    ) -> bool:
        """Replace the snapshot; returns True when something really changed.

        ``us`` / ``opponent`` are the raw message entries (or None when that
        robot is not currently visible - a missing entry means "no fresh
        measurement now", the previous values are dropped).
        """
        cleaned: Dict[str, Dict] = {}
        for key, raw in (("us", us), ("opponent", opponent)):
            entry = _clean_robot(raw)
            if entry is not None:
                cleaned[key] = entry
        if ttl_ms is not None:
            try:
                self._ttl_ms = max(self.MIN_TTL_MS, float(ttl_ms))
            except (TypeError, ValueError):
                pass
        changed = cleaned != self._poses
        self._poses = cleaned
        self._updated_at = time.monotonic()
        self._updated_epoch_ms = time.time() * 1000.0
        if t_ms is not None:
            try:
                self._reported_t = float(t_ms)
            except (TypeError, ValueError):
                pass
        if source:
            self._source = source
            self._source_client = str(client or "")
        if changed:
            self.version += 1
        return changed

    def clear(self) -> bool:
        """Drop the snapshot; True when something was removed."""
        changed = bool(self._poses)
        self._poses = {}
        if changed:
            self.version += 1
        return changed

    def release(self, source: str) -> bool:
        """A camera client disconnected: its poses must not linger.

        Only the client that owns the snapshot can release it; when it does,
        its ownership is forgotten too.
        """
        if source and self._source and self._source != source:
            return False
        changed = bool(self._poses)
        self._poses = {}
        self._source = ""
        self._source_client = ""
        if changed:
            self.version += 1
        return changed

    # -- reads ------------------------------------------------------------
    @property
    def fresh(self) -> bool:
        """True while the snapshot is within its TTL and carries a pose."""
        return bool(self._poses) and (self.age_s * 1000.0) <= self._ttl_ms

    @property
    def age_s(self) -> float:
        """Seconds since the snapshot was received (large when never)."""
        if not self._updated_at:
            return float("inf")
        return max(0.0, time.monotonic() - self._updated_at)

    @property
    def ttl_ms(self) -> float:
        """Current TTL of the snapshot in milliseconds."""
        return self._ttl_ms

    @property
    def reported_t(self) -> Optional[float]:
        """The camera's own timestamp of the newest snapshot (echoed in acks)."""
        return self._reported_t

    @property
    def source(self) -> str:
        """Peer string of the client that owns the snapshot ('' when none)."""
        return self._source

    def us(self) -> Optional[Dict]:
        """Our robot pose (a copy), or None when absent/stale."""
        if not self.fresh:
            return None
        entry = self._poses.get("us")
        return dict(entry) if entry is not None else None

    def opponent(self) -> Optional[Dict]:
        """Opponent pose (a copy), or None when absent/stale."""
        if not self.fresh:
            return None
        entry = self._poses.get("opponent")
        return dict(entry) if entry is not None else None

    def active(self) -> Dict:
        """Fresh poses keyed by robot ({} when stale or empty)."""
        if not self.fresh:
            return {}
        return {key: dict(value) for key, value in self._poses.items()}

    def status(self) -> Dict:
        """JSON-friendly store status for the telemetry/snapshot."""
        return {
            "source": self._source,
            "client": self._source_client,
            "fresh": self.fresh,
            "age_s": None if not self._updated_at else round(self.age_s, 3),
            "ttl_ms": self._ttl_ms,
            "seen": sorted(self._poses.keys()),
            "poses": self._poses if self.fresh else {},
            "reported_t": self._reported_t,
            "version": self.version,
        }
