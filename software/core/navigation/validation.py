"""Load-time mission geometry validation (leg-indexed clearance report).

A strategy saved months ago can name a playfield that no longer matches the
table, or carry a stroke that runs over the border. The planner already
re-routes such a mission at PLAN time, but the operator only finds out after
pressing Plan - and a mission whose FIRST leg leaves the field is exactly
the case the old bug (issue #10) described.

This module is pure: it walks the drawing, numbers every motion element as a
LEG and reports the places where the geometry leaves the configured
playfield or where the rotated body would cross the border. It never rejects
a document (old strategies must stay loadable) - the console/brain log the
lines so the operator can fix the drawing before running it.

Obstacles are deliberately not part of this check: the planner routes each
leg around them, and a warehouse of game elements changes during a strategy.
The field border is fixed, so it is the honest thing to validate up front.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence

from .clearance import body_clear_at, body_geometry, first_body_clip
from .options import PlanOptions

#: Default table used when a document does not record its field size.
DEFAULT_FIELD_MM = {"width_mm": 3000.0, "height_mm": 2000.0}

#: How far the rotated body may touch/overhang the wall before the load-time
#: report complains [mm]. A start zone IS the wall: a 300 mm body turned by a
#: few tenths of a degree around x = 150 mm pokes ~1 mm through it, and that
#: is the normal way a strategy is authored - warning there would fire on
#: every real document. Anything beyond a real overhang is still reported.
LOAD_BORDER_TOLERANCE_MM = 5.0


def _field_size(field: Optional[Dict]) -> tuple:
    """(width, height) of the playfield from a strategy field block."""
    data = field if isinstance(field, dict) else {}
    try:
        width = float(data.get("width_mm") or DEFAULT_FIELD_MM["width_mm"])
        height = float(data.get("height_mm") or DEFAULT_FIELD_MM["height_mm"])
    except (TypeError, ValueError):
        width, height = DEFAULT_FIELD_MM["width_mm"], DEFAULT_FIELD_MM["height_mm"]
    return max(1.0, width), max(1.0, height)


def mission_geometry_warnings(
    elements: Sequence[Dict],
    options_dict: Optional[Dict] = None,
    field: Optional[Dict] = None,
    start_pose: Optional[Sequence[float]] = None,
) -> List[str]:
    """Leg-indexed warnings for a mission loaded from disk (never raises).

    Legs are the motion elements in mission order (a stroke is one leg, a
    waypoint/goal is one leg); actions (pause, rotate, drive, release,
    erase, spawn) do not add a leg number, exactly like the operator reads
    the sequence. Reports:

    * a stroke/waypoint point outside the playfield,
    * a leg whose rotated body crosses the field border,
    * a recorded start pose whose body does not fit on the table.

    The connector from the recorded start pose to the first target is NOT
    checked: a start zone is the wall, so that leg legitimately clips there
    and the planner already projects its start ("path start inside the robot
    clearance zone - moved to…"). Reporting it would fire on every real
    document and tell the operator nothing they can act on.
    """
    options = PlanOptions.from_dict(dict(options_dict or {}))
    width, height = _field_size(field)
    body_w, body_h = body_geometry(options)
    warnings: List[str] = []

    def outside(point: Sequence[float]) -> bool:
        """Is the point outside the playfield rectangle?"""
        x, y = float(point[0]), float(point[1])
        tol = LOAD_BORDER_TOLERANCE_MM
        return not (-tol <= x <= width + tol and -tol <= y <= height + tol)

    def border_clip(points: Sequence[Sequence[float]]) -> Optional[tuple]:
        """First sample of the leg whose rotated body crosses the border."""
        if len(points) < 2:
            return None
        return first_body_clip(
            points,
            [],
            body_w,
            body_h,
            width,
            height,
            border_pad_mm=-LOAD_BORDER_TOLERANCE_MM,
        )

    leg = 0
    for element in elements or []:
        if not isinstance(element, dict):
            continue
        kind = str(element.get("kind", "stroke")).strip().lower()
        if kind == "stroke":
            points = []
            for raw in element.get("points", []):
                if not isinstance(raw, (list, tuple)) or len(raw) < 2:
                    continue
                try:
                    points.append((float(raw[0]), float(raw[1])))
                except (TypeError, ValueError):
                    continue  # a hand-edited document must stay loadable
            if len(points) < 2:
                continue
            leg += 1
            for index, point in enumerate(points):
                if outside(point):
                    warnings.append(
                        "leg {leg} (stroke): point {point} ({x:.0f}, {y:.0f}) mm is "
                        "outside the playfield ({w:.0f} x {h:.0f} mm)".format(
                            leg=leg, point=index + 1, x=point[0], y=point[1],
                            w=width, h=height,
                        )
                    )
                    break  # one report per leg is enough to fix the drawing
            clip = border_clip(points)
            if clip is not None:
                distance, point = clip
                warnings.append(
                    "leg {leg} (stroke): the body crosses the field border at "
                    "({x:.0f}, {y:.0f}) mm ({distance:.0f} mm into the leg) - the "
                    "planner will re-route it, but the drawn geometry should be fixed".format(
                        leg=leg, x=point[0], y=point[1], distance=distance,
                    )
                )
        elif kind in ("waypoint", "goal"):
            try:
                point = (float(element.get("x", 0.0)), float(element.get("y", 0.0)))
            except (TypeError, ValueError):
                continue
            leg += 1
            if outside(point):
                warnings.append(
                    "leg {leg} ({kind}): ({x:.0f}, {y:.0f}) mm is outside the "
                    "playfield ({w:.0f} x {h:.0f} mm)".format(
                        leg=leg, kind=kind, x=point[0], y=point[1],
                        w=width, h=height,
                    )
                )
                continue
            theta = element.get("theta_deg")
            if theta is not None:
                try:
                    heading = float(theta)
                except (TypeError, ValueError):
                    heading = 0.0
                if not body_clear_at(
                    point,
                    heading,
                    [],
                    body_w,
                    body_h,
                    width,
                    height,
                    border_pad_mm=-LOAD_BORDER_TOLERANCE_MM,
                ):
                    warnings.append(
                        "leg {leg} ({kind}): the body at the arrival angle "
                        "({heading:.0f} deg) does not fit inside the playfield at "
                        "({x:.0f}, {y:.0f}) mm".format(
                            leg=leg, kind=kind, heading=heading,
                            x=point[0], y=point[1],
                        )
                    )

    if start_pose is not None and len(start_pose) >= 2:
        try:
            start = (float(start_pose[0]), float(start_pose[1]))
            start_theta = float(start_pose[2]) if len(start_pose) > 2 else 0.0
        except (TypeError, ValueError):
            start = None
            start_theta = 0.0
        if start is not None:
            if outside(start) or not body_clear_at(
                start,
                start_theta,
                [],
                body_w,
                body_h,
                width,
                height,
                border_pad_mm=-LOAD_BORDER_TOLERANCE_MM,
            ):
                warnings.insert(
                    0,
                    "start pose ({x:.0f}, {y:.0f}) mm at {theta:.0f} deg leaves the "
                    "body outside the playfield ({w:.0f} x {h:.0f} mm)".format(
                        x=start[0], y=start[1], theta=start_theta, w=width, h=height,
                    ),
                )
    return warnings
