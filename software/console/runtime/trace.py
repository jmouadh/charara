"""I2C transaction trace: ring buffer for the UI monitor and log export.

Entries are produced by ``RobotClient`` (every TX attempt and every valid RX,
plus transport errors), pushed to the WebSocket hub by ``AppState`` and
exposed through ``/api/trace`` (recent entries) and ``/api/trace/download``
(text log).

File:    software/console/runtime/trace.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import time
from collections import deque
from typing import Deque, Dict, List, Optional


class TraceBuffer:
    """Bounded history of I2C transactions (newest last)."""

    def __init__(self, capacity: int = 4000) -> None:
        """Create the empty ring buffer."""
        self._items: Deque[Dict] = deque(maxlen=capacity)
        self._next_id = 1

    def add(self, entry: Dict) -> Dict:
        """Store one trace entry, assigning its sequence id."""
        entry = dict(entry)
        entry["id"] = self._next_id
        self._next_id += 1
        entry.setdefault("t", time.time() * 1000.0)
        self._items.append(entry)
        return entry

    def recent(self, after_id: int = 0, limit: int = 500) -> List[Dict]:
        """Entries after ``after`` (at most ``limit``), oldest first."""
        limit = max(1, min(int(limit), 2000))
        items = [item for item in self._items if item["id"] > after_id]
        return items[:limit]

    @property
    def last_id(self) -> int:
        """Sequence id of the newest entry (0 when empty)."""
        return self._items[-1]["id"] if self._items else 0

    @property
    def count(self) -> int:
        """Number of entries currently buffered."""
        return len(self._items)

    def clear(self) -> None:
        """Drop every buffered entry."""
        self._items.clear()

    def to_text(self) -> str:
        """Full log export (one line per entry, fixed columns)."""
        lines = [
            "# Charara RPI I2C trace",
            "# time                      dir  seq  command       status  bytes  payload / note",
        ]
        for item in self._items:
            timestamp = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(item["t"] / 1000.0))
            millis = int(item["t"] % 1000.0)
            seq = item.get("seq")
            cmd = item.get("cmd_name") or (f"0x{item['cmd']:02X}" if item.get("cmd") is not None else "-")
            status = item.get("status_name") or ""
            payload = item.get("payload") or ""
            note = item.get("note") or ""
            line = (
                f"{timestamp}.{millis:03d}  {item['dir']:<4} "
                f"{seq if seq is not None else '-':>3}  {cmd:<12}  {status:<6} "
                f"{item.get('len', 0):>5}  {payload}{note}"
            )
            lines.append(line.rstrip())
            frame = item.get("frame")
            if frame:
                lines.append(f"{'':<26}frame: {frame}")
        return "\n".join(lines) + "\n"
