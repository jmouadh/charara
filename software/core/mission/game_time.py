"""Game clock and time-triggered actions (pure runtime helpers).

The match clock is configured with the rest of the game document (see
:func:`software.core.world.map_config.normalize_game`). It is deliberately
kept OUT of the mission executor: a match clock runs independently of what
the robot happens to be doing, so it can stop a REAL run at the limit, warn
in MOCK, and fire global actions while a plan drives.

Lifecycle
---------
* :meth:`GameClock.start` marks the match begin (called by ``Execute`` and by
  the Game tab's Start button). Starting an already running clock is a no-op.
* :meth:`GameClock.pause` / :meth:`GameClock.resume` freeze and continue the
  elapsed time (the console's Pause / Resume buttons drive both the robot and
  the match clock). A paused clock fires no timed action and cannot expire.
* :meth:`GameClock.status` is what every telemetry snapshot reports.
* :meth:`GameClock.due_actions` returns the enabled actions whose ELAPSED
  time has been reached and that have not fired yet.
* :meth:`GameClock.expire_if_due` latches the limit exactly once.

Time-action architecture
------------------------
An action is a small JSON object::

    {"id": "action-1", "at_s": 95.0, "kind": "log", "enabled": true,
     "params": {"message": "release the harvester"}}

``kind`` selects a :class:`TimeActionHandler`. Two handlers ship today:

``log``
    writes the message to the console event log (the implemented reference),
``rest``
    is the RESERVED actuator bridge. It deliberately performs no network I/O
    yet: it logs that the action is configured and emits a ``time_action``
    event with ``status = "pending"``. A future REST/actuator integration
    registers a real handler under the same kind (or a new kind) with
    :func:`register_time_action_handler` - no scheduler change is needed.

File:    software/core/mission/game_time.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 1.0.0
Date:    2026-11-03
"""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Dict, List, Optional

#: Default tick of the console game loop [s]. 0.1 s keeps the REAL timeout
#: stop close enough to "immediately" without loading the Pi.
GAME_TICK_S = 0.1


class GameClock:
    """The match clock: start, elapsed/remaining, warning and fired actions.

    ``now`` is injectable so tests can drive the clock without sleeping. The
    clock keeps RUNNING after the limit (``expired`` is latched) so MOCK can
    show overtime; ``start`` is refused until :meth:`reset`.
    """

    def __init__(self, now: Callable[[], float] = time.monotonic) -> None:
        """Create a stopped clock with duration 0 (disabled)."""
        self._now = now
        self.duration_s = 0.0
        self.warning_s = 0.0
        self.actions: List[Dict[str, Any]] = []
        self._started_at: Optional[float] = None
        self._paused_at: Optional[float] = None
        self._paused_total = 0.0
        self._fired: set = set()
        self.expired = False

    # ------------------------------------------------------------ config
    def configure(self, game: Optional[Dict[str, Any]]) -> None:
        """Apply a normalized game document (duration, warning, actions).

        Called whenever the Game tab saves. Actions already fired keep that
        state while their id survives; a new id is eligible again.
        """
        data = game if isinstance(game, dict) else {}
        self.duration_s = max(0.0, float(data.get("duration_s") or 0.0))
        self.warning_s = max(0.0, min(self.duration_s, float(data.get("warning_s") or 0.0)))
        self.actions = [
            dict(action) for action in data.get("actions") or []
            if isinstance(action, dict)
        ]
        if self.duration_s <= 0.0:
            # disabling the clock also stops an in-flight match cleanly
            self.reset()
            return
        known = {str(action.get("id")) for action in self.actions}
        self._fired &= known

    @property
    def configured(self) -> bool:
        """True when a positive match duration is configured."""
        return self.duration_s > 0.0

    @property
    def running(self) -> bool:
        """True from :meth:`start` until :meth:`reset` (also past the limit)."""
        return self._started_at is not None

    @property
    def paused(self) -> bool:
        """True while :meth:`pause` holds the elapsed time (Resume continues)."""
        return self._paused_at is not None

    # ---------------------------------------------------------- lifecycle
    def start(self) -> bool:
        """Start the match clock; ``False`` when disabled or already running."""
        if not self.configured or self._started_at is not None:
            return False
        self._started_at = self._now()
        self._paused_at = None
        self._paused_total = 0.0
        self._fired = set()
        self.expired = False
        return True

    def pause(self) -> bool:
        """Freeze the elapsed time; ``False`` when stopped or already paused."""
        if self._started_at is None or self._paused_at is not None:
            return False
        self._paused_at = self._now()
        return True

    def resume(self) -> bool:
        """Continue a paused clock where :meth:`pause` left it."""
        if self._started_at is None or self._paused_at is None:
            return False
        self._paused_total += max(0.0, self._now() - self._paused_at)
        self._paused_at = None
        return True

    def reset(self) -> None:
        """Stop and rewind the clock (a new match can be started)."""
        self._started_at = None
        self._paused_at = None
        self._paused_total = 0.0
        self._fired = set()
        self.expired = False

    def elapsed_s(self) -> float:
        """Seconds since the match began (0 while stopped, frozen while paused)."""
        if self._started_at is None:
            return 0.0
        now = self._now()
        paused = self._paused_total
        if self._paused_at is not None:
            paused += max(0.0, now - self._paused_at)
        return max(0.0, now - self._started_at - paused)

    def remaining_s(self) -> float:
        """Seconds left in the match (negative in MOCK overtime)."""
        if not self.configured:
            return 0.0
        if self._started_at is None:
            return self.duration_s
        return self.duration_s - self.elapsed_s()

    # ------------------------------------------------------------ queries
    def status(self) -> Dict[str, Any]:
        """JSON-friendly clock + action state for the console snapshot."""
        elapsed = self.elapsed_s()
        remaining = self.remaining_s()
        warning = bool(
            self.configured
            and self._started_at is not None
            and (self.expired or remaining <= self.warning_s)
        )
        return {
            "configured": self.configured,
            "duration_s": round(self.duration_s, 1),
            "warning_s": round(self.warning_s, 1),
            "running": self.running,
            "paused": self.paused,
            "expired": self.expired,
            "elapsed_s": round(elapsed, 2),
            "remaining_s": round(remaining, 2),
            "warning": warning,
            "actions": [
                {
                    **action,
                    "fired": str(action.get("id")) in self._fired,
                }
                for action in self.actions
            ],
        }

    def due_actions(self) -> List[Dict[str, Any]]:
        """Enabled, not-yet-fired actions whose ``at_s`` has been reached."""
        if not self.running or self.expired or self.paused:
            return []
        elapsed = self.elapsed_s()
        due: List[Dict[str, Any]] = []
        for action in self.actions:
            action_id = str(action.get("id"))
            if not action.get("enabled", True) or action_id in self._fired:
                continue
            try:
                at_s = float(action.get("at_s", 0.0))
            except (TypeError, ValueError):
                continue
            if at_s <= elapsed:
                due.append(action)
        return due

    def mark_fired(self, action_id: Any) -> None:
        """Remember that an action was dispatched (it fires once per match)."""
        self._fired.add(str(action_id))

    def expire_if_due(self) -> bool:
        """Latch ``expired`` once when the match duration is reached."""
        if not self.running or self.expired or not self.configured or self.paused:
            return False
        if self.elapsed_s() >= self.duration_s:
            self.expired = True
            return True
        return False


# --------------------------------------------------------- action handlers -
@dataclass
class TimeActionContext:
    """What a time-action handler may use (no direct service import).

    ``state`` is the console runtime for integrations that need it; the two
    built-in handlers only use ``log``/``emit``. A future REST handler can
    read configuration or the current pose through ``state`` without this
    module depending on the console.
    """

    log: Callable[[str, str], Awaitable[None]]
    emit: Callable[[Dict[str, Any]], Awaitable[None]]
    state: Any = None


class TimeActionHandler:
    """Base class for one ``kind`` of timed action."""

    kind = ""

    async def run(
        self, action: Dict[str, Any], context: TimeActionContext
    ) -> Dict[str, Any]:
        """Execute the action and return ``{status, message}``."""
        raise NotImplementedError


class LogTimeAction(TimeActionHandler):
    """Write one event-log line (the implemented reference action)."""

    kind = "log"

    async def run(
        self, action: Dict[str, Any], context: TimeActionContext
    ) -> Dict[str, Any]:
        """Log the configured message at its match time."""
        params = action.get("params") if isinstance(action.get("params"), dict) else {}
        message = str(params.get("message") or "timed action").strip()
        await context.log(
            "info",
            f"game time {float(action.get('at_s', 0.0)):.1f} s - {message}",
        )
        return {"status": "done", "message": message}


class RestTimeAction(TimeActionHandler):
    """Reserved bridge for a future actuator REST call.

    The scheduler, config document and event contract are already in place.
    The actual HTTP call is intentionally NOT implemented: on the real table
    an actuator must be validated against its own safety rules first. A future
    integration registers a handler with kind ``rest`` (see Docs/11) and this
    stub stops being used.
    """

    kind = "rest"

    async def run(
        self, action: Dict[str, Any], context: TimeActionContext
    ) -> Dict[str, Any]:
        """Report that the REST bridge is configured but not active yet."""
        params = action.get("params") if isinstance(action.get("params"), dict) else {}
        target = str(params.get("url") or "(no URL)").strip()
        message = (
            f"REST timed action at {float(action.get('at_s', 0.0)):.1f} s is "
            f"configured for {target} - no REST handler is implemented yet"
        )
        await context.log("warn", message)
        return {"status": "pending", "message": message}


_HANDLERS: Dict[str, TimeActionHandler] = {
    handler.kind: handler for handler in (LogTimeAction(), RestTimeAction())
}


def register_time_action_handler(handler: TimeActionHandler) -> None:
    """Register (or replace) the handler for ``handler.kind``.

    This is the only hook a future REST/actuator integration needs: create a
    :class:`TimeActionHandler` subclass, call this once at service start-up,
    and every configured action of that kind is dispatched to it. See
    Docs/11-game-time.md for a complete example.
    """
    if not getattr(handler, "kind", ""):
        raise ValueError("a time-action handler needs a non-empty kind")
    _HANDLERS[str(handler.kind)] = handler


async def dispatch_time_action(
    action: Dict[str, Any], context: TimeActionContext
) -> Dict[str, Any]:
    """Dispatch one due action to its handler and emit the outcome event."""
    kind = str(action.get("kind") or "log").strip().lower()
    handler = _HANDLERS.get(kind)
    if handler is None:
        result = {
            "status": "unsupported",
            "message": f"unknown timed-action kind '{kind}'",
        }
        await context.log("warn", result["message"])
    else:
        try:
            result = await handler.run(action, context)
        except Exception as exc:  # an action must never kill the game loop
            result = {"status": "error", "message": str(exc)}
            await context.log(
                "error",
                f"timed action '{action.get('id')}' failed: {exc}",
            )
    await context.emit({
        "type": "time_action",
        "data": {
            "id": action.get("id"),
            "at_s": action.get("at_s"),
            "kind": kind,
            **result,
        },
    })
    return result


async def game_tick_loop(tick: Callable[[], Awaitable[None]],
                         period_s: float = GAME_TICK_S) -> None:
    """Run ``tick`` forever (the console starts this in its lifespan)."""
    while True:
        await asyncio.sleep(period_s)
        try:
            await tick()
        except asyncio.CancelledError:
            raise
        except Exception:
            # the game loop must survive a transient board/link failure: the
            # next tick retries, exactly like the vision watchdog
            pass
