"""The camera obstacle store (pure, no I/O).

A vision client connects to ``/ws/vision`` and keeps pushing the obstacles it
currently sees. The snapshot is replaced as a whole every message and
**expires** ``ttl_ms`` after the last message, so a camera only ever reports
what it sees right now - nothing has to track removals. Coordinates are
quantised to 10 mm: sensor jitter below that is not a change (no re-plan
churn), while a real move publishes a new version.

The store knows nothing about WebSockets, planning or the robot: the API
layer feeds it, the mission layer reads it.

File:    software/core/world/vision.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional, Sequence


def _quantize(value: float, step: float = 10.0) -> float:
    """Snap to a coarse grid: a camera jittering by millimetres is not a change."""
    return round(value / step) * step


class VisionStore:
    """Newest obstacle snapshot reported by a vision (camera) client."""

    DEFAULT_TTL_MS = 2000.0
    MIN_TTL_MS = 100.0
    QUANT_MM = 10.0
    MAX_OBSTACLES = 64

    def __init__(self) -> None:
        """Create an empty obstacle store."""
        self._obstacles: List[Dict] = []
        self._ttl_ms = self.DEFAULT_TTL_MS
        self._updated_at = 0.0
        self._updated_epoch_ms = 0.0
        self._reported_t: Optional[float] = None  # the camera's own timestamp
        self._source = ""
        self._source_client = ""  # what the sender calls itself (e.g. "console")
        self.version = 0  # bumped on every effective change

    # -- writes -----------------------------------------------------------
    def update(
        self,
        obstacles: Sequence[Dict],
        ttl_ms: Optional[float] = None,
        source: str = "",
        t_ms: Optional[float] = None,
        client: str = "",
    ) -> bool:
        """Replace the snapshot; returns True when something really changed."""
        cleaned: List[Dict] = []
        for raw in list(obstacles)[: self.MAX_OBSTACLES]:
            try:
                if str(raw.get("kind", "rect")) == "circle":
                    cleaned.append({
                        "kind": "circle",
                        "x": _quantize(float(raw["x"])),
                        "y": _quantize(float(raw["y"])),
                        "r": max(1.0, _quantize(float(raw.get("r", 0.0)))),
                    })
                else:
                    cleaned.append({
                        "kind": "rect",
                        "x": _quantize(float(raw.get("x", 0.0))),
                        "y": _quantize(float(raw.get("y", 0.0))),
                        "w": max(1.0, _quantize(float(raw.get("w", 0.0)))),
                        "h": max(1.0, _quantize(float(raw.get("h", 0.0)))),
                    })
            except (TypeError, ValueError, KeyError):
                continue
        if ttl_ms is not None:
            try:
                self._ttl_ms = max(self.MIN_TTL_MS, float(ttl_ms))
            except (TypeError, ValueError):
                pass
        changed = cleaned != self._obstacles
        self._obstacles = cleaned
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
        changed = bool(self._obstacles)
        self._obstacles = []
        if changed:
            self.version += 1
        return changed

    def release(self, source: str) -> bool:
        """A vision client disconnected: its snapshot must not linger.

        Only the client that owns the snapshot can release it; when it does,
        its ownership is forgotten too - the console watching ``source`` can
        then take its mock camera role back.
        """
        if source and self._source and self._source != source:
            return False
        changed = bool(self._obstacles)
        self._obstacles = []
        self._source = ""
        self._source_client = ""
        if changed:
            self.version += 1
        return changed

    def prune(self) -> bool:
        """Drop the snapshot when its TTL expired; True when it did."""
        if self._obstacles and (time.monotonic() - self._updated_at) * 1000.0 > self._ttl_ms:
            self._obstacles = []
            self.version += 1
            return True
        return False

    # -- reads ------------------------------------------------------------
    def active(self) -> List[Dict]:
        """The currently valid obstacle list."""
        self.prune()
        return [dict(obstacle) for obstacle in self._obstacles]

    @property
    def fresh(self) -> bool:
        """True while the newest snapshot is inside its TTL.

        A fresh snapshot with an empty obstacle list is valid (the camera
        sees nothing) - only an ABSENT camera or an expired snapshot is not
        fresh. Used by the match layer's ``camera_fresh`` condition.
        """
        return bool(self._source) and (time.monotonic() - self._updated_at) * 1000.0 <= self._ttl_ms

    @property
    def ttl_ms(self) -> float:
        """Time-to-live of the current snapshot in milliseconds."""
        return self._ttl_ms

    @property
    def source(self) -> str:
        """Name of the client owning the snapshot ('' when none)."""
        return self._source

    @property
    def source_client(self) -> str:
        """What the active source calls itself ("console" for the console's
        mock camera role, empty for a real camera/script)."""
        return self._source_client

    @property
    def reported_t(self) -> Optional[float]:
        """The client's own timestamp of the last snapshot (epoch ms), if given."""
        return self._reported_t

    def status(self) -> Dict:
        """JSON-friendly store status for the API and the console."""
        return {
            "count": len(self._obstacles),
            "ttl_ms": self._ttl_ms,
            "source": self._source,
            "client": self._source_client,
            "obstacles": self.active(),
            "t": self._reported_t,                       # camera clock, echoed
            "t_server": round(self._updated_epoch_ms, 1),  # service receive time
        }


def merged_obstacles(console_obstacles: Sequence[Dict], vision: VisionStore) -> List[Dict]:
    """The map a plan is built from: console obstacles + the camera's live set.

    Both sources are always respected; the camera's obstacles are appended so
    the console drawing keeps precedence when the two overlap (the planner
    handles duplicates as one wall anyway).
    """
    merged = list(console_obstacles)
    merged.extend(vision.active())
    return merged
