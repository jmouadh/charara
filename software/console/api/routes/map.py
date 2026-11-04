"""Game/map configuration routes: table, artwork, start zones, match clock.

The console's **Game Configuration** tab owns everything that makes the
service fit a different game: the play-area size, the artwork drawn behind
the map, the match clock + timed actions and the starting rectangles of both
teams (with the exact robot spawn pose of a primary zone).

Storage is the map document (``MAP_CONFIG_FILE``, see
:class:`software.core.world.map_config.MapConfig`) - a plain JSON file that
survives restarts. Background images are listed from ``FIELD_DIR`` (the
``ui/static/field`` folder the web app serves), so dropping a new image in
that folder makes it selectable after a page refresh.

File:    software/console/api/routes/map.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-23
"""

from __future__ import annotations

import asyncio
from typing import Dict

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse

from software.core.world.pdf_background import cache_path, ensure_background_png

from ...runtime.state import state
from .deps import to_dict
from .models import MapIn

router = APIRouter()


async def _publish() -> None:
    """Push the new document to every connected console."""
    await state.emit({"type": "map", "data": state.map_info()})


@router.get("/map")
async def get_map() -> Dict:
    """The map document plus the background images available on disk."""
    return state.map_info()


@router.get("/map/backgrounds")
async def get_backgrounds() -> Dict:
    """Background artwork found in the field folder (name + URL to load)."""
    return {
        "dir": str(state.settings.field_dir),
        "files": state.map_config.backgrounds(),
        "selected": state.map_config.data["background"],
    }


@router.get("/map/background/{name}")
async def get_background_image(name: str) -> FileResponse:
    """The background to draw: an image file, or a PDF rendered to a PNG.

    Eurobot publishes the table as a PDF, so a PDF in the field folder is
    accepted: its FIRST page is rasterised (poppler ``pdftoppm``, ~1 px/mm),
    cached under ``MAP_CACHE_DIR`` and served here. The browser only ever
    receives a PNG - no PDF plugin and no 7 MB vector download.
    """
    path = state.map_config.background_path(name)
    if path is None:
        raise HTTPException(status_code=404, detail=f"no background '{name}' in the field folder")
    if path.suffix.lower() != ".pdf":
        return FileResponse(path, headers={"Cache-Control": "no-store"})
    cache_dir = state.settings.map_cache_dir
    cached = cache_path(cache_dir, path).is_file()
    try:
        png = await asyncio.to_thread(ensure_background_png, path, cache_dir)
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=f"no background '{name}'") from exc
    except RuntimeError as exc:  # poppler missing / unreadable PDF
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    if not cached:
        await state.log("info", f"PDF background rendered: {name} -> {png.name}")
    return FileResponse(png, media_type="image/png", headers={"Cache-Control": "no-store"})


@router.put("/map")
async def put_map(body: MapIn) -> Dict:
    """Update (partially) and persist the map document.

    Accepted fields: ``field`` (``width_mm``/``height_mm``), ``background``
    (an image from the field folder, '' = automatic), ``colors`` (one
    ``#rrggbb`` per team) and ``zones`` (the complete starting-zone list; a
    primary zone may carry its robot spawn pose). Omitted fields are kept,
    so a small PATCH-like request is enough for a single change.

    The document is normalised before it is stored: zones are clamped to the
    play area, a spawn is clamped inside its zone, unknown teams/zones are
    dropped. Applying it also updates the field size and the teams' match
    starts (a primary zone spawn owns its team's start).
    """
    payload = {key: value for key, value in to_dict(body).items() if value is not None}
    if "field" in payload:
        field = payload["field"]
        width = float(field.get("width_mm", 0.0) or 0.0)
        height = float(field.get("height_mm", 0.0) or 0.0)
        if width < 500.0 or height < 500.0:
            raise HTTPException(status_code=422, detail="field must be at least 500 x 500 mm")
    previous_start = tuple(state.team_starts[state.team])
    try:
        snapshot = state.map_config.update(payload)
    except OSError as exc:  # unwritable volume: never pretend it was saved
        raise HTTPException(status_code=503, detail=f"could not save the map: {exc}") from exc
    state.apply_map(snapshot)
    # a primary zone IS the match start: put the robots in their new zones
    # (saving zones always places them, even when the start numbers did not
    # change - the operator is saying "the match starts HERE")
    await state.adopt_team_start(
        previous_start, "start zones updated", force="zones" in payload
    )
    zones = snapshot["zones"]
    await state.log(
        "info",
        f"map configuration saved: {snapshot['field']['width_mm']:.0f} x "
        f"{snapshot['field']['height_mm']:.0f} mm, {len(zones)} start zone(s), "
        f"background {snapshot['background'] or 'auto'}",
    )
    await _publish()
    return {**state.map_info()}


@router.delete("/map/zones/{zone_id}")
async def delete_zone(zone_id: str) -> Dict:
    """Remove one starting zone by id (the list view's Delete action)."""
    zones = [zone for zone in state.map_config.data["zones"] if zone["id"] != zone_id]
    if len(zones) == len(state.map_config.data["zones"]):
        raise HTTPException(status_code=404, detail=f"no start zone '{zone_id}'")
    previous_start = tuple(state.team_starts[state.team])
    snapshot = state.map_config.update({"zones": zones})
    state.apply_map(snapshot)
    # the team falls back to its configured start: put the robot there too
    await state.adopt_team_start(previous_start, "start zone removed", force=True)
    await state.log("info", f"start zone removed ({zone_id})")
    await _publish()
    return {**state.map_info()}
