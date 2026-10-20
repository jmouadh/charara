"""Strategy routes: save / load / list / delete the saved strategies.

The console is where strategies are EDITED (draw the mission, set the
options) and TESTED (plan / execute once). ``PUT /api/strategies/{name}``
stores the current drawing; the brain (``software.brain``) loads exactly this
file later and plays it. Storage is one JSON file per strategy in
``state.strategy_dir`` (``STRATEGY_DIR``, default ``<repo>/strategies``).

File:    software/console/api/routes/strategies.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

from typing import Dict

from fastapi import APIRouter, HTTPException

from software.core.mission.strategy import Strategy
from software.core.mission.teams import mirror_strategy

from ...runtime.state import state
from .deps import require_no_execution
from .models import StrategyIn

router = APIRouter()


@router.get("/strategies")
async def list_strategies() -> Dict:
    """Saved strategies, newest first (name, updated_at, size, notes)."""
    return {
        "strategies": Strategy.list_saved(state.strategy_dir),
        "dir": state.strategy_dir,
    }


@router.get("/strategies/{name}")
async def get_strategy(name: str) -> Dict:
    """Full strategy document (elements, options, goal, obstacles).

    The document is adapted to the CURRENT team: one authored for the other
    side is mirrored about the table's vertical centre line (strokes,
    waypoints, rotations, goal, obstacles), so the same saved strategy can
    be rehearsed by either side. The response reports ``authored_for`` (the
    stored team tag) and ``mirrored``.
    """
    try:
        strategy = Strategy.load_named(state.strategy_dir, name)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=f"no strategy named '{name}'") from exc
    authored_for = strategy.team
    mirrored = bool(authored_for) and authored_for != state.team
    if mirrored:
        mirror_strategy(strategy, state.field["width_mm"])
    doc = strategy.to_dict()
    doc["authored_for"] = authored_for
    doc["team"] = state.team
    doc["mirrored"] = mirrored
    return doc


@router.post("/strategies/{name}/load")
async def load_strategy(name: str) -> Dict:
    """Load a strategy FOR THE CURRENT TEAM and place the robot at its start.

    Same mirroring rules as :func:`get_strategy` (a document authored for
    the other side is reflected). Additionally, when the strategy recorded
    the pose its first step was run from, the MOCK robot is teleported there
    (mirrored for this side) - the mission starts where it was recorded
    instead of "dancing" across the field first. On the real robot the
    console never moves the board: it only reports the pose to place by
    hand. The response carries ``teleported`` and the applied ``start_pose``.
    """
    require_no_execution()
    try:
        strategy = Strategy.load_named(state.strategy_dir, name)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=f"no strategy named '{name}'") from exc
    authored_for = strategy.team
    mirrored = bool(authored_for) and authored_for != state.team
    if mirrored:
        mirror_strategy(strategy, state.field["width_mm"])
    doc = strategy.to_dict()
    doc["authored_for"] = authored_for
    doc["team"] = state.team
    doc["mirrored"] = mirrored
    teleported = False
    start = strategy.start_pose
    if start and len(start) >= 2:
        x = float(start[0])
        y = float(start[1])
        theta = float(start[2]) if len(start) > 2 else 0.0
        if state.transport == "mock" and state.client is not None:
            await state.client.set_pose(x, y, theta)
            state.vision_waiting = None  # a placement supersedes a wait state
            state.redo_restore_pose = None
            teleported = True
            await state.log(
                "info",
                f"strategy \"{name}\" loaded - robot placed at its recorded start "
                f"({x:.0f}, {y:.0f}) mm, {theta:.0f} deg",
            )
        else:
            await state.log(
                "info",
                f"strategy \"{name}\" starts at ({x:.0f}, {y:.0f}) mm, {theta:.0f} deg - "
                "place the robot there and press Set Pose, then Run",
            )
    else:
        await state.log(
            "info",
            f"strategy \"{name}\" has no recorded start pose - the mission starts "
            "from wherever the robot stands",
        )
    doc["teleported"] = teleported
    return doc


@router.put("/strategies/{name}")
async def save_strategy(name: str, body: StrategyIn) -> Dict:
    """Store the given drawing (v1) or behavior tree (v2) under ``name``.

    The document is tagged with the CURRENT team; saving also resolves a
    pending mock Redo (the robot is put back at the pose it had before the
    redo - the run's end - so the strategy continues from there). v2 bodies
    carry ``kind: "bt"`` + a ``tree`` (validated here, Docs/08 section 8).
    """
    try:
        previous: Strategy | None = None
        try:
            previous = Strategy.load_named(state.strategy_dir, name)
        except (ValueError, FileNotFoundError):
            previous = None
        # the strategy's recorded start: the console sends it after the first
        # validated step; a re-save without the field must NOT lose it
        start_pose = body.start_pose
        if start_pose is None and previous is not None and previous.start_pose:
            start_pose = list(previous.start_pose)
        if start_pose is not None and len(start_pose) >= 2:
            start_pose = [float(value) for value in start_pose[:3]]
        else:
            start_pose = None
        # v2 documents: validate the kind/tree pair HERE so an unpalayable
        # tree is rejected at save time, exactly like the model validates on
        # load - the brain must never meet a document it cannot play
        kind = str(body.kind or "elements").strip().lower()
        tree = body.tree
        if kind not in ("elements", "bt"):
            raise ValueError(f"unknown strategy kind '{kind}'")
        if kind == "bt" and not isinstance(tree, dict):
            raise ValueError("a 'bt' strategy requires a 'tree' object")
        if kind == "elements" and isinstance(tree, dict):
            raise ValueError("a 'tree' is only valid with kind 'bt'")
        strategy = Strategy(
            name=name,
            elements=body.elements,
            options=body.options,
            goal=body.goal,
            obstacles=body.obstacles,
            field={
                "width_mm": state.field["width_mm"],
                "height_mm": state.field["height_mm"],
            },
            team=state.team,
            start_pose=start_pose,
            notes=body.notes,
            kind=kind,
            tree=dict(tree) if isinstance(tree, dict) else None,
        )
        path = strategy.save(state.strategy_dir)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    pose = state.redo_restore_pose
    if pose is not None:
        state.redo_restore_pose = None
        if state.transport == "mock" and state.client is not None:
            try:
                await state.client.set_pose(float(pose[0]), float(pose[1]), float(pose[2]))
                await state.log(
                    "info",
                    f"redo was pending: robot put back at the run's end "
                    f"({pose[0]:.0f}, {pose[1]:.0f}) mm, {pose[2]:.0f} deg - the saved "
                    f"strategy continues from there",
                )
            except Exception:
                pass
    await state.log("info", f"strategy '{strategy.name}' saved for team {state.team} ({path.name})")
    return {"ok": True, "strategy": strategy.to_dict()}


@router.delete("/strategies/{name}")
async def delete_strategy(name: str) -> Dict:
    """Delete a saved strategy; 404 when it does not exist."""
    try:
        removed = Strategy.delete(state.strategy_dir, name)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    if not removed:
        raise HTTPException(status_code=404, detail=f"no strategy named '{name}'")
    await state.log("info", f"strategy '{name}' deleted")
    return {"ok": True}
