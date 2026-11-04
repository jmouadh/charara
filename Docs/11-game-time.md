# 11. Game time and timed actions

The console has a **match clock** that is independent of the mission
executor. It is configured in the **Game** tab (the former Map tab) and is
stored with the game/map document, so it survives a service restart.

```
Game tab
├── Field                (table size, background artwork)
├── Game time            (duration, red-warning window, clock controls)
├── Timed actions        (fire at elapsed seconds; log / REST placeholder)
├── Starting zones       (teams, spawn poses)
└── Zone colours
```

## 1. Document model

The clock is part of `MAP_CONFIG_FILE` (`/data/map/map.json` in docker):

```json
{
  "game": {
    "duration_s": 100.0,
    "warning_s": 10.0,
    "actions": [
      {
        "id": "action-1",
        "at_s": 95.0,
        "kind": "log",
        "enabled": true,
        "params": {"message": "release the second harvester"}
      }
    ]
  }
}
```

| Field | Meaning |
|---|---|
| `duration_s` | Total match time in seconds. `0` disables the clock, timeout stop and action scheduling. Default `100` (Eurobot match). |
| `warning_s` | The header timer turns red when this many seconds remain. Default `10`; clamped to `duration_s`. |
| `actions` | Timed actions, at most 64. `enabled: false` keeps an action configured but dormant. |

Action rules:

* `at_s` is **elapsed match time** (not remaining). `95` in a `100 s` match
  fires during the last 5 s.
* An action fires **once per clock run**. The fired flag is runtime-only;
  `Reset` (or a service restart) re-arms every action.
* `id` is the stable identity of the action and is used for the fired state.
  The document normaliser makes ids unique.

## 2. Clock lifecycle

| Event | Result |
|---|---|
| **Execute** (or **Run** a strategy) | Starts the clock automatically when a duration is configured and the clock is not already running. A clock frozen by Pause is **continued** instead. |
| **Start clock** button | Starts the clock now (idempotent while running). |
| **Reset** button | Stops and rewinds the clock and clears the fired-action history. |
| **Pause** | **Freezes** the match clock (Pause also stops the robot). Elapsed/remaining stop moving, no timed action fires and the limit cannot trigger while frozen. |
| **Resume** | Continues the frozen clock exactly where Pause left it, then the run goes on. |
| **Redo** (mock rehearsal) | Ends a paused run, teleports the robot to the run start and **rewinds the clock**: both counters (task timer and match clock) restart with the next Execute. |
| **Service restart** | The configuration survives; the runtime clock starts stopped. |

The header pill shows the remaining time. It is slate before the match,
teal while running, **amber while paused**, **red inside the configured
warning window** and solid red at the limit. The existing task-timer pill
(planned duration / task elapsed) pauses and resumes with the same buttons.

## 3. REAL vs MOCK

| Mode | At `duration_s` |
|---|---|
| **REAL** | The service sends an immediate stop (`executor.stop`, or `link.stop` when idle), clears obstacle waits and blocks a mission restart until **Reset**. The event log records `GAME TIME LIMIT REACHED`. |
| **MOCK** | The robot keeps running; the game pill turns red and shows the `!` overtime mark. The event log warns that REAL would have stopped. |

Mission starts are guarded in three places:

1. `POST /api/execute` and `POST /api/resume` refuse after a REAL timeout,
2. the mission orchestrator's `_start_verified` guard blocks an automatic
   vision re-plan / wait resume,
3. the clock is restarted only by an explicit **Reset**.

Manual jog commands (`/api/move/*`) remain available for recovery; the game
limit stops the match, it does not lock the hardware. Use **Reset** before a
new run.

> **Scope:** the clock is hosted by the console service, like the map and
> vision channels. A headless `brain` run has no console state and therefore
> does not enforce the game clock; run the console for a timed match.

## 4. REST API

| Method | Path | Meaning |
|---|---|---|
| `GET` | `/api/game` | Runtime status: config, `running`, `paused`, `expired`, `elapsed_s`, `remaining_s`, `warning`, fired actions, `mock_override`. Also embedded as `game` in every `GET /api/state` snapshot. |
| `POST` | `/api/game/start` | Start the configured clock now (no body). |
| `POST` | `/api/game/reset` | Rewind and clear the fired actions. |
| `PUT` | `/api/map` | Save `{"game": {...}}` (partial document update). |

Example:

```text
PUT /api/map {"game": {"duration_s": 100, "warning_s": 10, "actions": []}}
POST /api/game/start
GET /api/game
```

## 5. Timed-action architecture (future REST/actuator bridge)

The scheduler, document model and event contract are already in place. The
implementation lives in `software/core/mission/game_time.py`.

```
game_tick_loop (0.1 s)             console lifespan
        │
        ▼
GameClock.due_actions()            enabled, at_s <= elapsed, not fired
        │
        ▼
dispatch_time_action() ──► TimeActionHandler registry
        │                     ├── "log"  (implemented reference)
        │                     └── "rest" (reserved placeholder)
        ▼
{type: "time_action", data: {id, at_s, kind, status, message}}
```

A handler implements one small interface:

```python
class TimeActionHandler:
    kind = "rest"

    async def run(self, action, context):
        # action["params"] holds the document parameters
        # context.log(level, message) / context.emit(event) / context.state
        return {"status": "done", "message": "actuator triggered"}
```

Register it once at service start-up:

```python
from software.core.mission.game_time import register_time_action_handler

register_time_action_handler(MyRestAction())
```

No scheduler or UI change is required: the **REST call** action kind already
exists in the Game tab, and every configured action is dispatched through
the registry. The shipped `RestTimeAction` is deliberately a safe no-op that
logs "no REST handler is implemented yet" and emits
`status = "pending"`; replace it (or register a new kind) when the actuator
endpoint, authentication and fail-safe rules are defined.

For an actuator, validate before the call and keep the timeout shorter than
the remaining match time. A failed action must never stop the game loop: the
dispatcher catches handler exceptions and emits `status = "error"` without
affecting the clock.

## 6. REAL start position

A REAL robot adopts the primary start-zone spawn of its team with
`SET_POSE` when the link becomes ready, when the team changes, when a start
zone is saved and with **Reset robot to start**. `SET_POSE` only aligns the
odometry frame - it never drives the robot - and is skipped while the robot
is moving or a mission is executing. This is why the console opens with the
robot in its dedicated start zone instead of a stale corner pose.
