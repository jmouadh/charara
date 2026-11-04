"""Game/map configuration document: table, artwork, zones, match clock.

The console must adapt to ANY game: the play area, the artwork behind it,
the starting rectangles of both teams and the match clock are configuration,
not code. This module owns that document:

* ``field``      - ``width_mm`` / ``height_mm`` of the play area,
* ``background`` - the artwork file under ``ui/static/field`` ('' = auto),
* ``colors``     - one colour per team, used to paint the zones on the map,
* ``zones``      - the starting rectangles: team, primary/secondary type and
  (primary only) the exact robot spawn pose inside the rectangle,
* ``game``       - match duration, red-warning window and timed actions.

Storage is one plain JSON file (``MAP_CONFIG_FILE``, default
``<repo>/map/map.json``; ``/data/map/map.json`` in docker). The file is
written atomically, so a crash can never leave a half-written map behind.
The document is deliberately forgiving: a broken or hand-edited entry is
normalised (clamped to the table, unknown team/type dropped) instead of
taking the console down.

File:    software/core/world/map_config.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-23
"""

from __future__ import annotations

import json
import math
import os
import re
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple
from urllib.parse import quote

from ..mission.teams import TEAM_BLUE, TEAM_YELLOW, TEAMS

#: Document format version - bump when the schema changes in a breaking way.
VERSION = 1

#: Zone types: a PRIMARY zone holds the main robot and carries its spawn
#: pose; a SECONDARY one is the small-robot rectangle (no spawn pose).
ZONE_PRIMARY = "primary"
ZONE_SECONDARY = "secondary"
ZONE_TYPES = (ZONE_PRIMARY, ZONE_SECONDARY)

#: Default per-team colours when the document (or the request) does not set
#: one. They match the map's historic start-ghost colours.
DEFAULT_COLORS = {TEAM_YELLOW: "#efc13c", TEAM_BLUE: "#4b7bec"}

#: A zone smaller than this is a stray click, not a rectangle [mm].
MIN_ZONE_SIDE_MM = 50.0
#: Refuse to grow the document without limit (a broken client loop).
MAX_ZONES = 64

#: Game-clock defaults: the Eurobot match is 100 s. ``duration_s = 0``
#: disables the clock entirely (no timeout stop, no timed actions).
DEFAULT_GAME_DURATION_S = 100.0
DEFAULT_GAME_WARNING_S = 10.0
#: Refuse an unbounded action list (a broken client loop).
MAX_GAME_ACTIONS = 64
#: ``log`` is implemented now; ``rest`` is the reserved actuator bridge.
GAME_ACTION_KINDS = ("log", "rest")

_IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg", ".svg", ".webp", ".gif")
#: Eurobot publishes the table as a PDF: it is a valid background too, the
#: service rasterises its first page for the map.
_BACKGROUND_SUFFIXES = _IMAGE_SUFFIXES + (".pdf",)
_HEX_RE = re.compile(r"^#[0-9a-fA-F]{6}$")


def _number(value: Any, default: float = 0.0) -> float:
    """Finite float or ``default`` (rejects NaN/inf/strings that are not numbers)."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _clamp(value: float, low: float, high: float) -> float:
    """Clamp ``value`` into ``[low, high]`` (``high`` wins on an inverted range)."""
    return max(low, min(high, value))


def _norm180(angle: float) -> float:
    """Wrap an angle into (-180, 180]."""
    value = ((angle % 360.0) + 540.0) % 360.0 - 180.0
    return 180.0 if value == -180.0 else value


def _color(value: Any, fallback: str) -> str:
    """``#rrggbb`` colour or ``fallback``."""
    text = str(value or "").strip()
    return text if _HEX_RE.match(text) else fallback


def _round(value: float) -> float:
    """One decimal is the map's working resolution."""
    return round(float(value), 1)


def normalize_background(value: Any, field_dir: Path | str) -> str:
    """Normalise a background selection to ``field/<name>`` ('' = auto).

    Accepts ``table.png``, ``field/table.png`` or an absolute path: only the
    FILE NAME is kept (no directory traversal), the suffix must be an image
    and the file must really exist in the field folder. Anything else falls
    back to the automatic artwork search of the UI.
    """
    text = str(value or "").strip().replace("\\", "/")
    if not text:
        return ""
    name = Path(text).name
    if not name or name.startswith("."):
        return ""
    if Path(name).suffix.lower() not in _BACKGROUND_SUFFIXES:
        return ""
    if not (Path(field_dir) / name).is_file():
        return ""
    return f"field/{name}"


def field_size(raw: Any, default: Dict[str, float]) -> Dict[str, float]:
    """Validated table size: at least 500 x 500 mm, else the default."""
    data = raw if isinstance(raw, dict) else {}
    width = _number(data.get("width_mm"), _number(default.get("width_mm"), 3000.0))
    height = _number(data.get("height_mm"), _number(default.get("height_mm"), 2000.0))
    return {
        "width_mm": _round(max(500.0, width)),
        "height_mm": _round(max(500.0, height)),
    }


def normalize_zone(
    raw: Any, field_w: float, field_h: float, fallback_id: str
) -> Optional[Dict[str, Any]]:
    """One starting zone, clamped to the table; ``None`` when unusable.

    A zone is a rectangle (``x``, ``y`` = lower-left corner in the console's
    mm frame) plus its team and type. A PRIMARY zone may carry the robot
    spawn pose; the pose is clamped INSIDE the rectangle, so a hand-edited
    file can never place the robot outside its own start box.
    """
    if not isinstance(raw, dict):
        return None
    team = str(raw.get("team", "")).strip().lower()
    if team not in TEAMS:
        return None
    kind = str(raw.get("type") or raw.get("zone_type") or ZONE_PRIMARY).strip().lower()
    if kind not in ZONE_TYPES:
        kind = ZONE_PRIMARY
    width = _clamp(_number(raw.get("w"), MIN_ZONE_SIDE_MM), MIN_ZONE_SIDE_MM, field_w)
    height = _clamp(_number(raw.get("h"), MIN_ZONE_SIDE_MM), MIN_ZONE_SIDE_MM, field_h)
    x = _clamp(_number(raw.get("x")), 0.0, max(0.0, field_w - width))
    y = _clamp(_number(raw.get("y")), 0.0, max(0.0, field_h - height))
    zone: Dict[str, Any] = {
        "id": str(raw.get("id") or fallback_id),
        "team": team,
        "type": kind,
        "x": _round(x),
        "y": _round(y),
        "w": _round(width),
        "h": _round(height),
        "spawn": None,
    }
    spawn = raw.get("spawn")
    if kind == ZONE_PRIMARY and isinstance(spawn, dict):
        zone["spawn"] = {
            "x": _round(_clamp(_number(spawn.get("x"), x + width / 2.0), x, x + width)),
            "y": _round(_clamp(_number(spawn.get("y"), y + height / 2.0), y, y + height)),
            "theta_deg": _round(_norm180(_number(spawn.get("theta_deg"), 0.0))),
        }
    return zone


def normalize_game(raw: Any) -> Dict[str, Any]:
    """The match clock + timed-action list, clamped to a sane range.

    ``duration_s`` is the total match time (0 = clock disabled). The clock
    turns the header timer red when ``warning_s`` remains, and a timed action
    fires once when the ELAPSED time reaches ``at_s``. Action ids are unique
    within the document; the runtime keys "already fired" on them.

    ``kind`` selects the handler: ``log`` writes one event-log line (the
    implemented reference action), ``rest`` is the reserved bridge for a
    future actuator REST call (the handler is pluggable - see
    ``software/core/mission/game_time.py`` and Docs/11).
    """
    data = raw if isinstance(raw, dict) else {}
    duration = _clamp(
        _number(data.get("duration_s"), DEFAULT_GAME_DURATION_S), 0.0, 3600.0
    )
    warning = _clamp(
        _number(data.get("warning_s"), DEFAULT_GAME_WARNING_S), 0.0, duration
    )
    actions: List[Dict[str, Any]] = []
    seen: set = set()
    raw_actions = data.get("actions")
    if isinstance(raw_actions, list):
        for index, entry in enumerate(raw_actions[:MAX_GAME_ACTIONS]):
            if not isinstance(entry, dict):
                continue
            kind = str(entry.get("kind") or "log").strip().lower()
            if kind not in GAME_ACTION_KINDS:
                kind = "log"
            base = str(entry.get("id") or f"action-{index + 1}").strip()[:64]
            if not base:
                base = f"action-{index + 1}"
            action_id = base
            suffix = 2
            while action_id in seen:
                action_id = f"{base}-{suffix}"
                suffix += 1
            seen.add(action_id)
            params = entry.get("params")
            actions.append({
                "id": action_id,
                "at_s": _round(_clamp(_number(entry.get("at_s"), 0.0), 0.0, duration)),
                "kind": kind,
                "enabled": bool(entry.get("enabled", True)),
                "params": dict(params) if isinstance(params, dict) else {},
            })
    return {
        "duration_s": _round(duration),
        "warning_s": _round(warning),
        "actions": actions,
    }


class MapConfig:
    """The persisted map document (table, artwork, colours, start zones)."""

    def __init__(
        self,
        path: Path | str,
        field_dir: Path | str,
        default_field: Dict[str, float],
        default_background: str = "",
    ) -> None:
        """Create the document; an existing file is loaded and normalised."""
        self.path = Path(path)
        self.field_dir = Path(field_dir)
        self._default_field = {
            "width_mm": _number(default_field.get("width_mm"), 3000.0),
            "height_mm": _number(default_field.get("height_mm"), 2000.0),
        }
        self._default_background = default_background
        self.data = self._defaults()
        self.load()

    # ------------------------------------------------------------- document
    def _defaults(self) -> Dict[str, Any]:
        """An empty document with the configured table size and colours."""
        return {
            "version": VERSION,
            "updated_at": 0.0,
            "field": dict(self._default_field),
            "background": "",
            "colors": dict(DEFAULT_COLORS),
            "zones": [],
            "game": normalize_game({}),
        }

    def _normalize(self, raw: Dict[str, Any]) -> Dict[str, Any]:
        """Coerce any document/request into the canonical shape."""
        if not isinstance(raw, dict):
            return self._defaults()
        field = field_size(raw.get("field"), self._default_field)
        field_w, field_h = field["width_mm"], field["height_mm"]
        colors = dict(DEFAULT_COLORS)
        raw_colors = raw.get("colors")
        if isinstance(raw_colors, dict):
            for team in TEAMS:
                colors[team] = _color(raw_colors.get(team), colors[team])
        zones: List[Dict[str, Any]] = []
        seen: set = set()
        raw_zones = raw.get("zones")
        if isinstance(raw_zones, list):
            for index, entry in enumerate(raw_zones[:MAX_ZONES]):
                zone = normalize_zone(entry, field_w, field_h, f"zone-{index + 1}")
                if zone is None:
                    continue
                base = zone["id"]
                suffix = 2
                while zone["id"] in seen:  # ids key the map/list + popup edits
                    zone["id"] = f"{base}-{suffix}"
                    suffix += 1
                seen.add(zone["id"])
                zones.append(zone)
        return {
            "version": VERSION,
            "updated_at": _number(raw.get("updated_at"), 0.0),
            "field": field,
            "background": normalize_background(raw.get("background"), self.field_dir),
            "colors": colors,
            "zones": zones,
            "game": normalize_game(raw.get("game")),
        }

    # -------------------------------------------------------------- storage
    def load(self) -> bool:
        """Read the file; ``False`` when it is missing or unreadable."""
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            self.data = self._defaults()
            return False
        self.data = self._normalize(raw)
        return True

    def save(self) -> None:
        """Write the document atomically (temps file then rename)."""
        self.data["updated_at"] = time.time()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.path.with_suffix(self.path.suffix + ".tmp")
        temp.write_text(json.dumps(self.data, indent=2) + "\n", encoding="utf-8")
        os.replace(temp, self.path)

    def snapshot(self) -> Dict[str, Any]:
        """Deep-enough copy for the API (callers must not mutate the store)."""
        return json.loads(json.dumps(self.data))

    def update(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Merge a partial request, normalise it and persist the result."""
        merged = self.snapshot()
        if isinstance(payload, dict):
            for key in ("field", "background", "colors", "zones", "game"):
                if key in payload and payload[key] is not None:
                    merged[key] = payload[key]
        self.data = self._normalize(merged)
        self.save()
        return self.snapshot()

    # ------------------------------------------------------------- queries
    def backgrounds(self) -> List[Dict[str, str]]:
        """Background artwork available in the field folder, sorted by name.

        Each entry carries the value to STORE (``field/<name>``) and the URL
        the browser loads: images are served as files, a PDF through the
        service, which rasterises its first page (``/api/map/background/...``).
        """
        folder = self.field_dir
        if not folder.is_dir():
            return []
        rows: List[Dict[str, str]] = []
        try:
            entries: Sequence[Path] = sorted(folder.iterdir(), key=lambda entry: entry.name.lower())
        except OSError:
            return []
        for entry in entries:
            if not entry.is_file() or entry.name.startswith("."):
                continue
            suffix = entry.suffix.lower()
            if suffix not in _BACKGROUND_SUFFIXES:
                continue
            value = f"field/{entry.name}"
            try:  # cache-buster: a replaced table file must redraw
                stamp = str(int(entry.stat().st_mtime))
            except OSError:
                stamp = "0"
            if suffix == ".pdf":
                url = f"/api/map/background/{quote(entry.name)}"
                kind = "pdf"
            else:
                url = f"field/{quote(entry.name)}"
                kind = "image"
            rows.append(
                {
                    "name": entry.name,
                    "value": value,
                    "url": f"{url}?v={stamp}",
                    "kind": kind,
                }
            )
        return rows

    def background_path(self, name: str) -> Optional[Path]:
        """Safe path of one background file (``None`` when missing/unsafe)."""
        text = str(name or "").strip().replace("\\", "/")
        if not text or "/" in text or text in (".", ".."):
            return None
        path = self.field_dir / text
        if not path.is_file():
            return None
        if path.suffix.lower() not in _BACKGROUND_SUFFIXES:
            return None
        return path

    def zones_for(self, team: str, zone_type: Optional[str] = None) -> List[Dict[str, Any]]:
        """The zones of one team (optionally only primary/secondary)."""
        return [
            zone for zone in self.data["zones"]
            if zone["team"] == team and (zone_type is None or zone["type"] == zone_type)
        ]

    def spawn_for(self, team: str) -> Optional[Tuple[float, float, float]]:
        """Spawn pose of the team's PRIMARY zone (the first one), if any."""
        for zone in self.zones_for(team, ZONE_PRIMARY):
            spawn = zone.get("spawn")
            if isinstance(spawn, dict):
                return (
                    float(spawn["x"]),
                    float(spawn["y"]),
                    float(spawn["theta_deg"]),
                )
        return None
