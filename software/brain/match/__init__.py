"""Match layer: behavior-tree playback for the brain (Track C, Docs/10).

``play_tree`` is the v2 twin of ``Brain.play``: it builds the leaf registry
around the live brain (``nodes.build_registry``), parses the document's
``tree`` spec (failing FAST on an invalid document, never mid-match) and
ticks it to completion with ``tick_until_done``.

Document options read here:

* ``match_s`` - match duration in seconds; sets the monotonic deadline the
  ``time_left`` condition consults (absent = no deadline);
* ``score``   - initial blackboard score for ``score_at_least`` (raised at
  runtime by the ``add_score`` action).

Returns the executor-style state string (``"done"`` / ``"failed"``) so the
brain keeps ONE return contract for both document kinds.

File:    software/brain/match/__init__.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.1.0
Date:    2026-10-09
"""

from __future__ import annotations

import time

from software.core.mission.bt import Status, TickContext, from_spec, tick_until_done
from software.core.mission.executor import EXEC_DONE

from .nodes import build_registry

#: state string returned when a tree ends in failure (Brain.play contract)
MATCH_FAILED = "failed"


async def play_tree(brain, strategy, poll_s: float = 0.1) -> str:
    """Play one behavior-tree strategy; returns :data:`EXEC_DONE` or failed.

    Raises ``ValueError`` when the document's tree is invalid - the caller
    logs it and reports ``"failed"``, exactly like an unplannable v1
    document.
    """
    registry = build_registry(brain)
    tree = from_spec(dict(strategy.tree or {}), registry)
    options = dict(strategy.options or {})
    blackboard = {
        "clock": time.monotonic,
        "score": float(options.get("score", 0.0) or 0.0),
    }
    match_s = float(options.get("match_s", 0.0) or 0.0)
    if match_s > 0.0:
        blackboard["deadline"] = time.monotonic() + match_s
    ctx = TickContext(blackboard=blackboard, services={"brain": brain})
    status = await tick_until_done(tree, ctx, poll_s=poll_s, log=brain.log)
    return EXEC_DONE if status is Status.SUCCESS else MATCH_FAILED
