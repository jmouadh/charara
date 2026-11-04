"""Rasterise a PDF (the published table artwork) into the map background.

Eurobot publishes the table as a PDF, so the console must accept a PDF in
``ui/static/field`` and draw it behind the map like any image. The raster is
produced on the service (the browser must not download a 7 MB vector PDF and
a PDF renderer), cached next to the map document and served as a PNG.

The first page is the table (the same convention the UI image already uses
when it converts ``table_FINALE_1.0.pdf`` at container start). Rendering uses
``pdftoppm`` from poppler-utils (installed in the console image); the raster
is scaled to ~1 px/mm of the play area, which is what the bundled
``table.png`` uses, so the field artwork keeps its detail without shipping a
7000 px wide bitmap.

File:    software/core/world/pdf_background.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-23
"""

from __future__ import annotations

import os
import shutil
import subprocess
import threading
from pathlib import Path
from typing import Optional

#: Width of the rendered raster [px] - ~1 px per mm on a 3 m field. The UI
#: stretches the image to the configured play area, so this is a detail/
#: weight trade-off, not a geometry decision.
DEFAULT_WIDTH_PX = 3000
#: pdftoppm can be slow on a Pi for a big vector table: bound the wait.
RENDER_TIMEOUT_S = 180

#: one render at a time (a double refresh must not race on the cache file)
_render_lock = threading.Lock()


def tool_path() -> Optional[str]:
    """Path of ``pdftoppm``, or ``None`` when poppler is not installed."""
    return shutil.which("pdftoppm")


def cache_path(cache_dir: Path | str, pdf_path: Path | str, width_px: int = DEFAULT_WIDTH_PX) -> Path:
    """Stable cache file for (PDF, mtime, size, width) - invalidates on change."""
    folder = Path(cache_dir)
    pdf = Path(pdf_path)
    try:
        stat = pdf.stat()
        stamp = f"{stat.st_mtime_ns}-{stat.st_size}"
    except OSError:
        stamp = "missing"
    return folder / f"{pdf.stem}-{stamp}-{int(width_px)}.png"


def ensure_background_png(
    pdf_path: Path | str,
    cache_dir: Path | str,
    width_px: int = DEFAULT_WIDTH_PX,
) -> Path:
    """Render the first PDF page to a cached PNG and return its path.

    Raises ``RuntimeError`` with an actionable message when poppler is not
    installed or the PDF cannot be read; the caller maps that to HTTP 503/422.
    """
    pdf = Path(pdf_path)
    if not pdf.is_file():
        raise FileNotFoundError(str(pdf))
    target = cache_path(cache_dir, pdf, width_px)
    if target.is_file() and target.stat().st_size > 0:
        return target
    binary = tool_path()
    if binary is None:
        raise RuntimeError(
            "PDF backgrounds need poppler-utils (pdftoppm) in the console image "
            "- rebuild the api container (docker compose up -d --build api)"
        )
    target.parent.mkdir(parents=True, exist_ok=True)
    with _render_lock:
        if target.is_file() and target.stat().st_size > 0:
            return target  # another request rendered it while we waited
        # pdftoppm writes "<stem>.png"; render to a temp stem then rename, so
        # a concurrent request never sees a half-written raster
        temp_stem = target.with_name(f".{target.stem}.tmp")
        rendered = Path(str(temp_stem) + ".png")
        try:
            rendered.unlink()
        except OSError:
            pass
        command = [
            binary,
            "-png",
            "-singlefile",
            "-scale-to-x",
            str(int(width_px)),
            "-scale-to-y",
            "-1",
            str(pdf),
            str(temp_stem),
        ]
        try:
            process = subprocess.run(
                command, capture_output=True, timeout=RENDER_TIMEOUT_S
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"PDF rendering timed out after {RENDER_TIMEOUT_S} s") from exc
        if process.returncode != 0 or not rendered.is_file() or rendered.stat().st_size == 0:
            detail = (process.stderr or b"").decode("utf-8", "replace").strip()
            try:
                rendered.unlink()
            except OSError:
                pass
            raise RuntimeError(f"PDF rendering failed: {detail or 'empty raster'}")
        os.replace(rendered, target)
        _prune(folder=target.parent, keep=target, stem=pdf.stem)
        return target


def _prune(folder: Path, keep: Path, stem: str) -> None:
    """Drop older renders of the same PDF (a re-exported table changes mtime)."""
    prefix = f"{stem}-"
    try:
        entries = list(folder.iterdir())
    except OSError:
        return
    for entry in entries:
        if entry == keep or not entry.name.startswith(prefix):
            continue
        if entry.suffix.lower() != ".png":
            continue
        try:
            entry.unlink()
        except OSError:
            pass
