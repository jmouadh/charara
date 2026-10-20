"""Strategy document: what the RPi plays, independent from any plan.

A strategy is the SAVED mission: the drawing (planner elements), the plan
options, an optional goal, the obstacles that belong to the table setup and
free-form notes. The trajectory itself is computed at play time - the console
plans it for testing, the brain plans it when it starts playing - so the same
strategy adapts to where the robot actually is and to what the camera sees.

Storage: one JSON file per strategy (``<dir>/<name>.json``), shared by the
console (save / load / delete through the REST API) and the brain
(``--strategy <name>``). The format is deliberately plain JSON so a strategy
can be diffed, reviewed and hand-edited.

File:    software/core/mission/strategy.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import json
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

#: Document format version - bump when the schema changes in a breaking way.
#: v2 adds the optional behavior-tree layer (``kind`` + ``tree``): v1
#: documents (elements only) load unchanged - ``kind`` defaults to
#: "elements" and the brain plays them exactly as before.
VERSION = 2
#: document kinds the brain can play (see Docs/10 section 7)
KIND_ELEMENTS = "elements"
KIND_BT = "bt"

_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._ -]{0,63}$")


def check_name(name: str) -> str:
    """Validate a strategy name (also the file name stem)."""
    text = (name or "").strip()
    if not _NAME_RE.match(text):
        raise ValueError(
            "strategy name must start with a letter/digit, use only letters, "
            "digits, '.', '_', '-' and spaces, and be at most 64 characters"
        )
    return text


@dataclass
class Strategy:
    """A saved mission: drawing + options + table obstacles + metadata."""

    name: str
    elements: List[Dict[str, Any]] = field(default_factory=list)
    options: Dict[str, Any] = field(default_factory=dict)
    goal: Optional[Dict[str, Any]] = None
    obstacles: List[Dict[str, Any]] = field(default_factory=list)
    # field size at save time - the brain warns when the table does not match
    field: Dict[str, Any] = field(default_factory=dict)
    # which side AUTHORED this document ('yellow' / 'blue'; '' = unknown,
    # legacy): loading it while playing the other side mirrors the geometry
    # (the table is symmetric), see ``software.core.mission.teams``
    team: str = ""
    # pose the robot held when the FIRST validated step was run (recorded by
    # the console): loading the strategy puts the robot back exactly there -
    # the mission starts where it was recorded instead of driving to that
    # place from wherever the robot stands (mirrored for the other side)
    start_pose: Optional[List[float]] = None
    notes: str = ""
    version: int = VERSION
    updated_at: float = 0.0  # epoch seconds (0 = never saved)
    #: document kind: "elements" (v1 drawing, the default) or "bt" (v2 tree)
    kind: str = KIND_ELEMENTS
    #: behavior-tree spec of a "bt" document (parsed by core.mission.bt)
    tree: Optional[Dict[str, Any]] = None

    # ------------------------------------------------------------- document
    def to_dict(self) -> Dict[str, Any]:
        """JSON-friendly document of the whole strategy."""
        return {
            "name": self.name,
            "version": self.version,
            "updated_at": self.updated_at,
            "field": dict(self.field),
            "goal": self.goal,
            "obstacles": [dict(o) for o in self.obstacles],
            "options": dict(self.options),
            "elements": [dict(e) for e in self.elements],
            "kind": self.kind,
            "tree": self.tree,
            "team": self.team,
            "start_pose": list(self.start_pose) if self.start_pose else None,
            "notes": self.notes,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Strategy":
        """Build a validated strategy from a JSON-style mapping."""
        if not isinstance(data, dict):
            raise ValueError("strategy document must be a JSON object")
        name = check_name(str(data.get("name", "")))
        elements = data.get("elements") or []
        if not isinstance(elements, list):
            raise ValueError("'elements' must be a list")
        kind = str(data.get("kind", KIND_ELEMENTS)).strip().lower()
        if kind not in (KIND_ELEMENTS, KIND_BT):
            raise ValueError(f"unknown strategy kind '{kind}'")
        tree = data.get("tree")
        if kind == KIND_BT and not isinstance(tree, dict):
            raise ValueError("a 'bt' strategy requires a 'tree' object")
        if tree is not None and not isinstance(tree, dict):
            raise ValueError("'tree' must be a JSON object")
        raw_start = data.get("start_pose")
        start_pose: Optional[List[float]] = None
        if isinstance(raw_start, (list, tuple)) and len(raw_start) >= 2:
            start_pose = [float(value) for value in raw_start[:3]]
        return cls(
            name=name,
            elements=[dict(e) for e in elements],
            options=dict(data.get("options") or {}),
            goal=dict(data["goal"]) if data.get("goal") else None,
            obstacles=[dict(o) for o in (data.get("obstacles") or [])],
            field=dict(data.get("field") or {}),
            team=str(data.get("team", "")).strip().lower(),
            start_pose=start_pose,
            notes=str(data.get("notes", "")),
            version=int(data.get("version", VERSION)),
            updated_at=float(data.get("updated_at", 0.0)),
            kind=kind,
            tree=dict(tree) if isinstance(tree, dict) else None,
        )

    # ------------------------------------------------------------- storage
    def save(self, directory: Path | str) -> Path:
        """Write ``<directory>/<name>.json`` (creates the directory)."""
        folder = Path(directory)
        folder.mkdir(parents=True, exist_ok=True)
        self.updated_at = time.time()
        path = folder / f"{check_name(self.name)}.json"
        path.write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")
        return path

    @classmethod
    def load(cls, path: Path | str) -> "Strategy":
        """Read a strategy document from an explicit path."""
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        strategy = cls.from_dict(data)
        if not strategy.name:
            strategy.name = Path(path).stem
        return strategy

    @classmethod
    def load_named(cls, directory: Path | str, name: str) -> "Strategy":
        """Read ``<directory>/<name>.json``."""
        return cls.load(Path(directory) / f"{check_name(name)}.json")

    @classmethod
    def list_saved(cls, directory: Path | str) -> List[Dict[str, Any]]:
        """Summary of the stored strategies, newest first."""
        folder = Path(directory)
        if not folder.is_dir():
            return []
        rows: List[Dict[str, Any]] = []
        for path in sorted(folder.glob("*.json")):
            try:
                strategy = cls.load(path)
            except (ValueError, OSError, json.JSONDecodeError):
                continue  # a broken file must not hide the others
            rows.append(
                {
                    "name": strategy.name,
                    "updated_at": strategy.updated_at,
                    "n_elements": len(strategy.elements),
                    "kind": strategy.kind,
                    "team": strategy.team,
                    "notes": strategy.notes,
                }
            )
        rows.sort(key=lambda row: row["updated_at"], reverse=True)
        return rows

    @classmethod
    def delete(cls, directory: Path | str, name: str) -> bool:
        """Remove ``<directory>/<name>.json``; True when a file was removed."""
        path = Path(directory) / f"{check_name(name)}.json"
        try:
            path.unlink()
            return True
        except FileNotFoundError:
            return False
