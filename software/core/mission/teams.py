"""Team side (Yellow / Blue) and table mirroring for symmetric Eurobot fields.

Each edition names the two sides differently (this season: **Yellow** and
**Blue**) and the match rules place every robot inside the start rectangle of
its colour. The field items are symmetric about the VERTICAL centre line, so
any strategy authored for one side can be REFLECTED to be played by the other
side - "100 mm forward, then 90 deg left" becomes "100 mm forward, then
90 deg right", exactly as a mirror image of the table would show.

The mirror is a pure reflection about ``x = field_width / 2``:

* points: ``(x, y) -> (W - x, y)``
* headings (direction of travel): ``theta -> 180 - theta``
* in-place rotations: ``alpha -> -alpha``
* rect obstacles (corner + size): ``x -> W - (x + w)``
* circle obstacles (centre + radius): ``x -> W - x``

Applying the mirror twice is the identity (up to float rounding), so the SAME
code converts Yellow -> Blue and Blue -> Yellow: save a strategy while playing
Yellow, switch the team to Blue and load it again - the plan is re-computed
from the robot's (new) start position for the mirrored drawing.

File:    software/core/mission/teams.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-01
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Dict, List, Optional, Sequence, Tuple

if TYPE_CHECKING:  # pragma: no cover - typing only
    from .strategy import Strategy

#: The two sides of a Eurobot match. The DISPLAY names ("Yellow"/"Blue") are
#: configuration (``TEAM_LABEL_YELLOW``/``TEAM_LABEL_BLUE``): an edition may
#: name or colour its sides differently, the code only knows yellow/blue.
TEAM_YELLOW = "yellow"
TEAM_BLUE = "blue"
TEAMS = (TEAM_YELLOW, TEAM_BLUE)

#: Yellow match-start pose used when ``TEAM_START_YELLOW`` is not configured
#: (Eurobot 2026 table: inside the top-left start rectangle, facing down).
DEFAULT_START_YELLOW: Tuple[float, float, float] = (315.0, 1800.0, -90.0)


def check_team(team: str) -> str:
    """Normalise a team name; ``ValueError`` for anything but yellow/blue."""
    name = (team or "").strip().lower()
    if name not in TEAMS:
        raise ValueError("team must be 'yellow' or 'blue'")
    return name


def other_team(team: str) -> str:
    """The opposing side of ``team``."""
    return TEAM_BLUE if check_team(team) == TEAM_YELLOW else TEAM_YELLOW


def norm180(angle: float) -> float:
    """Wrap an angle to (-180, 180]."""
    return (angle + 180.0) % 360.0 - 180.0


def mirror_point(point: Sequence[float], field_width_mm: float) -> Tuple[float, float]:
    """Reflect a point about the field's vertical centre line."""
    return (float(field_width_mm) - float(point[0]), float(point[1]))


def mirror_heading(theta_deg: float) -> float:
    """Reflect a heading (direction of travel) about the vertical centre."""
    return norm180(180.0 - float(theta_deg))


def mirror_pose(
    pose: Sequence[float], field_width_mm: float
) -> Tuple[float, float, float]:
    """Reflect a full pose ``(x, y, theta_deg)`` for the other side."""
    return (
        float(field_width_mm) - float(pose[0]),
        float(pose[1]),
        mirror_heading(pose[2]),
    )


def resolve_start_pose(
    team: str,
    yellow: Optional[Sequence[float]] = None,
    blue: Optional[Sequence[float]] = None,
    field_width_mm: float = 3000.0,
) -> Tuple[float, float, float]:
    """The effective match-start pose of ``team``.

    Yellow defaults to :data:`DEFAULT_START_YELLOW`; Blue uses its configured
    pose when set and otherwise the Yellow start mirrored about the field's
    vertical centre line (the symmetric-table rule). One home for that rule:
    the console's team panel and the session's start-pose bootstrap both
    call it (``RobotSession.maybe_apply_start_pose``).
    """
    base = tuple(float(value) for value in (yellow or DEFAULT_START_YELLOW))
    if check_team(team) == TEAM_YELLOW:
        return (base[0], base[1], base[2])
    if blue:
        return (float(blue[0]), float(blue[1]), float(blue[2]))
    return mirror_pose(base, field_width_mm)


def mirror_elements(
    elements: Sequence[Dict[str, Any]], field_width_mm: float
) -> List[Dict[str, Any]]:
    """Reflect a planner element list for the OTHER side of the table.

    Strokes are mirrored point by point, waypoints (position + arrival
    heading) reflected, rotations negated; a straight advance follows the
    heading chain, so its distance is mirror-invariant and stays as it is,
    and pauses do not move at all.
    """
    mirrored: List[Dict[str, Any]] = []
    for element in elements:
        item: Dict[str, Any] = {key: value for key, value in element.items()}
        kind = str(item.get("kind", "stroke"))
        if kind == "stroke":
            item["points"] = [
                [round(field_width_mm - float(p[0]), 3), round(float(p[1]), 3)]
                for p in (item.get("points") or [])
                if len(p) >= 2
            ]
        elif kind == "waypoint":
            if item.get("x") is not None:
                item["x"] = round(field_width_mm - float(item["x"]), 3)
            if item.get("theta_deg") is not None:
                item["theta_deg"] = round(mirror_heading(item["theta_deg"]), 3)
        elif kind == "rotate":
            if item.get("angle_deg") is not None:
                item["angle_deg"] = round(-float(item["angle_deg"]), 3)
        mirrored.append(item)
    return mirrored


def mirror_goal(
    goal: Optional[Dict[str, Any]], field_width_mm: float
) -> Optional[Dict[str, Any]]:
    """Reflect a goal element (position + optional arrival heading)."""
    if not goal:
        return goal
    item: Dict[str, Any] = dict(goal)
    if item.get("x") is not None:
        item["x"] = round(field_width_mm - float(item["x"]), 3)
    if item.get("theta_deg") is not None:
        item["theta_deg"] = round(mirror_heading(item["theta_deg"]), 3)
    return item


def mirror_obstacle(
    obstacle: Dict[str, Any], field_width_mm: float
) -> Dict[str, Any]:
    """Reflect one obstacle (rect = corner+size, circle = centre+radius)."""
    item: Dict[str, Any] = dict(obstacle)
    if str(item.get("kind", "rect")) == "circle":
        item["x"] = round(field_width_mm - float(item["x"]), 3)
    else:
        item["x"] = round(
            field_width_mm - (float(item["x"]) + float(item.get("w", 0.0))), 3
        )
    return item


def mirror_obstacles(
    obstacles: Sequence[Dict[str, Any]], field_width_mm: float
) -> List[Dict[str, Any]]:
    """Reflect a whole obstacle list for the other side."""
    return [mirror_obstacle(obstacle, field_width_mm) for obstacle in obstacles]


def mirror_strategy(strategy: "Strategy", field_width_mm: float) -> "Strategy":
    """Reflect a whole :class:`Strategy` document for the other side (in place).

    The stored ``team`` tag is kept - it records who AUTHORED the document;
    callers flip the team tag themselves if they persist the copy.
    """
    strategy.elements = mirror_elements(strategy.elements, field_width_mm)
    strategy.goal = mirror_goal(strategy.goal, field_width_mm)
    strategy.obstacles = mirror_obstacles(strategy.obstacles, field_width_mm)
    if strategy.start_pose and len(strategy.start_pose) >= 2:
        strategy.start_pose = list(mirror_pose(strategy.start_pose, field_width_mm))
    return strategy
