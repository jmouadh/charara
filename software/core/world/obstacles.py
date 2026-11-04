"""Obstacle model helpers: permanent fixtures vs named game elements.

The table map carries two kinds of obstacle:

* ``permanent`` - walls, pillars, the table itself. They are part of the
  setup and, once a strategy is being planned, cannot be created, moved or
  deleted.
* ``game_element`` - a movable game piece. It carries a unique ``name`` so
  the strategy can reference it from ``erase`` / ``spawn`` actions. The
  planner removes it from the routing grid at an ``erase`` action and adds
  it back (with the geometry captured by the action) at a ``spawn``.

The module is pure data/validation (no console, no planner): the API, the
planner, the strategy loader, the brain and the tests all share the same
rules. Legacy documents without ``type`` load as permanent obstacles, so the
whole existing corpus keeps working.

File:    software/core/world/obstacles.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.3.0
Date:    2026-10-28
"""

from __future__ import annotations

import json
import re
from typing import Any, Dict, List, Optional, Sequence

TYPE_PERMANENT = "permanent"
TYPE_GAME_ELEMENT = "game_element"
OBSTACLE_TYPES = (TYPE_PERMANENT, TYPE_GAME_ELEMENT)

KIND_RECT = "rect"
KIND_CIRCLE = "circle"
KIND_POLYGON = "polygon"
OBSTACLE_KINDS = (KIND_RECT, KIND_CIRCLE, KIND_POLYGON)

#: short, user-visible identifier shown on the map and in the sequence
ELEMENT_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._ -]{0,31}$")

#: legacy spellings accepted for a type (documents from before 2026-10-28)
_TYPE_ALIASES = {
    "": TYPE_PERMANENT,
    "permanent": TYPE_PERMANENT,
    "static": TYPE_PERMANENT,
    "fixed": TYPE_PERMANENT,
    "fixture": TYPE_PERMANENT,
    "game_element": TYPE_GAME_ELEMENT,
    "game-element": TYPE_GAME_ELEMENT,
    "gameelement": TYPE_GAME_ELEMENT,
    "element": TYPE_GAME_ELEMENT,
    "dynamic": TYPE_GAME_ELEMENT,
}


def check_element_name(name: Any) -> str:
    """Validate a game-element name; returns the stripped name."""
    text = str(name or "").strip()
    if not ELEMENT_NAME_RE.match(text):
        raise ValueError(
            "a game element needs a unique name starting with a letter/digit, "
            "using letters, digits, '.', '_', '-' and spaces (max 32 chars)"
        )
    return text


def obstacle_type(obstacle: Dict[str, Any]) -> str:
    """Normalised type of one obstacle (legacy = permanent)."""
    raw = str((obstacle or {}).get("type", "")).strip().lower()
    kind = _TYPE_ALIASES.get(raw)
    if kind is None:
        raise ValueError(f"unknown obstacle type '{raw}' (permanent / game_element)")
    return kind


def is_game_element(obstacle: Dict[str, Any]) -> bool:
    """True when the obstacle is a named, movable game element."""
    try:
        return obstacle_type(obstacle) == TYPE_GAME_ELEMENT
    except ValueError:
        return False


def normalize_obstacle(obstacle: Dict[str, Any]) -> Dict[str, Any]:
    """Return a validated copy with an explicit type/name/kind.

    Geometry is left to the planner (it clamps drawn shapes to the table);
    the shape KIND and the identity fields are validated here because the
    runtime actions reference them.
    """
    if not isinstance(obstacle, dict):
        raise ValueError("an obstacle must be a JSON object")
    item = dict(obstacle)
    item["type"] = obstacle_type(item)
    kind = str(item.get("kind") or KIND_RECT).strip().lower()
    if kind not in OBSTACLE_KINDS:
        raise ValueError(f"unknown obstacle kind '{kind}' (rect / circle / polygon)")
    item["kind"] = kind
    if item["type"] == TYPE_GAME_ELEMENT:
        item["name"] = check_element_name(item.get("name"))
    else:
        # permanents are anonymous: an old name must not turn them into an
        # action target by accident
        item.pop("name", None)
    return item


def normalize_obstacles(obstacles: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Validate a whole obstacle list; game-element names must be unique."""
    out: List[Dict[str, Any]] = []
    seen: Dict[str, str] = {}
    for raw in obstacles or []:
        item = normalize_obstacle(raw)
        if item["type"] == TYPE_GAME_ELEMENT:
            key = item["name"].casefold()
            if key in seen:
                raise ValueError(
                    f"duplicate game element name '{item['name']}' "
                    f"(already used by '{seen[key]}')"
                )
            seen[key] = item["name"]
        out.append(item)
    return out


def game_element_names(obstacles: Sequence[Dict[str, Any]]) -> List[str]:
    """Names of the game elements in ``obstacles`` (document order)."""
    return [str(o.get("name")) for o in obstacles if is_game_element(o)]


def find_game_element(
    obstacles: Sequence[Dict[str, Any]], name: str
) -> Optional[Dict[str, Any]]:
    """The game element called ``name`` (case-insensitive), or None."""
    key = str(name or "").strip().casefold()
    for obstacle in obstacles:
        if is_game_element(obstacle) and str(obstacle.get("name", "")).casefold() == key:
            return obstacle
    return None


def erase_game_element(
    obstacles: Sequence[Dict[str, Any]], name: str
) -> List[Dict[str, Any]]:
    """Copy of ``obstacles`` without the named game element (idempotent)."""
    key = str(name or "").strip().casefold()
    return [
        dict(obstacle) for obstacle in obstacles
        if not (is_game_element(obstacle) and str(obstacle.get("name", "")).casefold() == key)
    ]


def spawn_game_element(
    obstacles: Sequence[Dict[str, Any]], name: str, obstacle: Dict[str, Any]
) -> List[Dict[str, Any]]:
    """Copy of ``obstacles`` with the named game element placed/updated.

    An active element with the same name is REPLACED (the runtime action is
    idempotent: replaying a strategy after its last run must not fail only
    because the piece already stands at its final spot).
    """
    checked = check_element_name(name)
    if not isinstance(obstacle, dict):
        raise ValueError(f"spawn '{checked}' needs an 'obstacle' object")
    item = dict(obstacle)
    item["type"] = TYPE_GAME_ELEMENT
    item["name"] = checked
    item["kind"] = str(item.get("kind") or KIND_RECT).strip().lower()
    if item["kind"] not in OBSTACLE_KINDS:
        raise ValueError(f"unknown obstacle kind '{item['kind']}' (rect / circle / polygon)")
    return erase_game_element(obstacles, checked) + [item]


def validate_spawn_obstacle(obstacle: Dict[str, Any]) -> None:
    """Reject a spawn action whose geometry cannot be an obstacle at all."""
    kind = str(obstacle.get("kind") or KIND_RECT).strip().lower()
    if kind == KIND_RECT:
        if float(obstacle.get("w", 0.0) or 0.0) <= 0.0 or float(
            obstacle.get("h", 0.0) or 0.0
        ) <= 0.0:
            raise ValueError("a spawned box needs a positive width and height")
    elif kind == KIND_CIRCLE:
        if float(obstacle.get("r", 0.0) or 0.0) <= 0.0:
            raise ValueError("a spawned zone needs a positive radius")
    elif kind == KIND_POLYGON:
        points = [
            point for point in (obstacle.get("points") or [])
            if isinstance(point, (list, tuple)) and len(point) >= 2
        ]
        if len(points) < 3:
            raise ValueError("a spawned polygon needs at least 3 points")
    else:
        raise ValueError(f"unknown obstacle kind '{kind}' (rect / circle / polygon)")


def validate_actions(
    elements: Sequence[Dict[str, Any]], obstacles: Sequence[Dict[str, Any]]
) -> None:
    """Validate every ``erase`` / ``spawn`` action of a mission.

    * the action needs a name;
    * that name may not be an active permanent obstacle (permanents cannot
      be modified by a strategy);
    * a ``spawn`` carries a usable geometry;
    * erasing an absent element and re-spawning an active one are allowed
      (idempotent replay from any current map state).
    """
    permanents = {
        str(obstacle.get("name", "")).strip().casefold()
        for obstacle in obstacles
        if not is_game_element(obstacle) and obstacle.get("name")
    }
    for index, element in enumerate(elements or []):
        kind = str(element.get("kind", "stroke")).strip().lower()
        if kind not in ("erase", "spawn"):
            continue
        name = check_element_name(element.get("name"))
        if name.casefold() in permanents:
            raise ValueError(
                f"element {index + 1}: '{name}' is a permanent obstacle - a "
                "strategy cannot erase or re-spawn it"
            )
        if kind == "spawn":
            raw = element.get("obstacle")
            if not isinstance(raw, dict):
                raise ValueError(
                    f"element {index + 1}: spawn '{name}' needs an 'obstacle' object"
                )
            validate_spawn_obstacle(raw)


def apply_action(
    obstacles: Sequence[Dict[str, Any]],
    kind: str,
    name: str,
    obstacle: Optional[Dict[str, Any]] = None,
) -> List[Dict[str, Any]]:
    """Apply one strategy action to an obstacle list (pure copy)."""
    if str(kind) == "erase":
        return erase_game_element(obstacles, name)
    if str(kind) == "spawn":
        if obstacle is None:
            raise ValueError(f"spawn '{name}' needs an 'obstacle' object")
        return spawn_game_element(obstacles, name, obstacle)
    raise ValueError(f"unknown game-element action '{kind}'")


def _canonical(obstacle: Dict[str, Any]) -> Dict[str, Any]:
    """Rounding-stable identity of one obstacle for the permanent lock."""

    def number(value: Any) -> float:
        try:
            return round(float(value), 1)
        except (TypeError, ValueError):
            return 0.0

    item: Dict[str, Any] = {
        "kind": str(obstacle.get("kind") or KIND_RECT),
    }
    if item["kind"] == KIND_POLYGON:
        item["points"] = [
            [number(point[0]), number(point[1])]
            for point in (obstacle.get("points") or [])
            if isinstance(point, (list, tuple)) and len(point) >= 2
        ]
    elif item["kind"] == KIND_CIRCLE:
        item.update({"x": number(obstacle.get("x")), "y": number(obstacle.get("y")),
                     "r": number(obstacle.get("r"))})
    else:
        item.update({"x": number(obstacle.get("x")), "y": number(obstacle.get("y")),
                     "w": number(obstacle.get("w")), "h": number(obstacle.get("h"))})
    return item


def permanent_signature(obstacles: Sequence[Dict[str, Any]]) -> str:
    """Stable identity of the permanent obstacle set (lock baseline)."""
    permanents = [
        _canonical(obstacle) for obstacle in obstacles if not is_game_element(obstacle)
    ]
    return json.dumps(permanents, sort_keys=True, separators=(",", ":"))


def permanents_kept(
    baseline: Sequence[Dict[str, Any]], current: Sequence[Dict[str, Any]]
) -> bool:
    """Are all baseline table fixtures still present, unchanged?

    Used by the mid-run lock: a camera/dynamic obstacle may APPEAR in a
    re-plan request (it is merged into the map like a vision report), but a
    fixture that was part of the setup may not be moved or deleted while the
    mission executes. New anonymous obstacles are accepted as transient.
    """
    current_items = [
        _canonical(obstacle) for obstacle in current if not is_game_element(obstacle)
    ]
    for obstacle in baseline:
        if is_game_element(obstacle):
            continue
        if _canonical(obstacle) not in current_items:
            return False
    return True


def spawn_geometry(obstacle: Dict[str, Any]) -> Dict[str, Any]:
    """Copy of a game element suitable for a ``spawn`` action payload."""
    item = dict(obstacle)
    item["type"] = TYPE_GAME_ELEMENT
    item.pop("name", None)
    if item.get("kind") == KIND_POLYGON:
        item["points"] = [
            [float(point[0]), float(point[1])]
            for point in (item.get("points") or [])
            if isinstance(point, (list, tuple)) and len(point) >= 2
        ]
    for key in ("x", "y", "w", "h", "r"):
        if key in item:
            item[key] = float(item[key])
    return item
