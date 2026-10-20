"""Mission layer: ports, executor, orchestration, strategy documents.

* ``interfaces``   - the ROBOT PORT the executor drives (``RobotIO``) and the
                     error base class implementations derive from.
* ``executor``     - runs a motion program segment by segment, with pause,
                     resume, e-stop, timeouts and progress events.
* ``orchestrator`` - high level plan / mid-run re-plan / waiting decisions
                     over a mission context (console or brain).
* ``watcher``      - reactions to obstacle map changes: broadcast, debounced
                     re-plan, proximity re-checks, TTL sweeps.
* ``strategy``     - the saved strategy document the RPi plays (elements,
                     options, goal, obstacles) + JSON persistence.

File:    software/core/mission/__init__.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from .executor import (
    EXEC_ABORTED,
    EXEC_DONE,
    EXEC_IDLE,
    EXEC_PAUSED,
    EXEC_RUNNING,
    Executor,
    remaining_elements,
)
from .strategy import Strategy

__all__ = [
    "EXEC_ABORTED",
    "EXEC_DONE",
    "EXEC_IDLE",
    "EXEC_PAUSED",
    "EXEC_RUNNING",
    "Executor",
    "Strategy",
    "remaining_elements",
]
