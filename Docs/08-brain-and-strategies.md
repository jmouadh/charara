# 08 - Brain & strategies

The **console** is the workshop: configure, calibrate, test, plan. The
**brain** is what actually plays a match: it loads a saved **strategy** and
runs it headless - plan from the live pose, execute, react to obstacles,
repeat. Both compose the same `software.core` engine, so what was tested in
the console is what drives the robot.

```
console (browser + FastAPI)  --save-->  strategies/<name>.json  --load-->  brain (RPi)
        draw, test, calibrate               one file per strategy            plan -> execute -> avoid
```

## 1. The strategy document

A strategy stores the DRAWING, not a computed trajectory - the plan is built
at play time from where the robot actually is and what the camera sees.

| Field | Meaning |
|---|---|
| `name` | document name (also the file stem); letters/digits/`.`/`_`/`-`/space, starts alphanumeric, max 64 |
| `version` | format version (currently 1) |
| `updated_at` | epoch seconds, set on every save |
| `field` | table size at save time (`width_mm`, `height_mm`) - reference for the operator |
| `elements` | the mission drawing, same element list as `POST /api/plan` (stroke / waypoint / rotate / pause / release) |
| `options` | any `PlanOptions` field (speeds, tolerances, ...), see [03 section 1](03-planner-algorithms.md) |
| `goal` | optional final pose `{x, y, theta_deg}` |
| `obstacles` | obstacles that belong to the TABLE SETUP (rect/circle); camera obstacles are live and never saved |
| `notes` | free text for the team |
| `kind` | `"elements"` (v1 drawing, the default) or `"bt"` (v2 behavior tree - section 8) |
| `tree` | the tree spec when `kind` is `"bt"`; validated at load time |

Example (`strategies/demo.json`):

```json
{
  "name": "demo",
  "version": 1,
  "updated_at": 0,
  "field": {"width_mm": 3000.0, "height_mm": 2000.0},
  "goal": null,
  "obstacles": [],
  "options": {"speed_mm_s": 250},
  "elements": [
    {"kind": "stroke", "points": [[400, 300], [1100, 300]]},
    {"kind": "pause", "seconds": 0.5},
    {"kind": "release"},
    {"kind": "pause", "seconds": 1.0},
    {"kind": "stroke", "points": [[1100, 300], [1100, 800]]}
  ],
  "notes": "Sample strategy: small L-shaped drive near the start corner."
}
```

Storage: one JSON file per strategy in `STRATEGY_DIR` (default
`<repo>/strategies/`). The files are plain JSON - diffable, reviewable and
hand-editable. The document model lives in
`software/core/mission/strategy.py` (`Strategy.save/load/load_named/
list_saved/delete`).

## 2. Saving from the console

The console exposes the storage as REST (full reference:
[05 section 3](05-api-reference.md)):

| Endpoint | Effect |
|---|---|
| `GET /api/strategies` | list saved strategies (newest first) |
| `GET /api/strategies/{name}` | the full document |
| `PUT /api/strategies/{name}` | save the current drawing/options (create or replace) |
| `DELETE /api/strategies/{name}` | remove it |

Typical workflow:

1. Draw the mission in the console, set the options, plan it and execute it
   on the mock (or the robot) until the behaviour is right.
2. `PUT /api/strategies/main` with the elements/options/goal/obstacles - the
   field size is recorded automatically.
3. On the robot, the brain plays `main`. Re-visit the console whenever the
   strategy needs tuning; the robot is not involved in editing.

```bash
curl -s -X PUT localhost:8090/api/strategies/main \
     -H "Content-Type: application/json" \
     -d '{"elements":[{"kind":"stroke","points":[[500,500],[1500,500]]}],
          "options":{"speed_mm_s":700}, "notes":"qualification"}'
```

## 3. The brain

`software/brain/runner.py`:

* `BrainContext` - the mission context (same contract as the console's
  `AppState`): plan lock, executor, vision store, last plan/request,
  `pose()`, `log()`, `emit()`.
* `Brain` - owns the robot link (i2c or mock), the localization service, the
  telemetry loop and the watcher loops (`sweep_loop`, `proximity_loop`).
  * `await brain.start()` - open the link, start the loops, wait briefly
    for the first pose.
  * `await brain.play(strategy, repeat=False)` - plan from the live pose
    around the live obstacle map, execute, wait; returns the final executor
    state (`done`, `aborted`, `failed`). With `repeat`, the strategy is
    replayed in a loop (each round re-plans from the robot's position).
  * `await brain.set_obstacles(obstacles, ttl_ms)` - the camera input: it
    replaces the live snapshot and triggers the same reaction the console's
    `/ws/vision` does (re-plan or wait when the remaining route is affected).
  * `brain.status()` - JSON-friendly status (pose, localization health,
    exec progress, waiting flag) for logs and tests.
  * `await brain.stop()` - stop the run if needed, cancel the loops, close
    the link.

A strategy that cannot be planned (empty drawing, impossible geometry)
returns `"failed"` and logs the reason; the process stays alive. A v2
document (`kind: "bt"`) is played by the match layer instead - the same
engine behind its leaves, with decisions between missions (section 8).

## 4. CLI

```bash
python -m software.brain --strategy <name|path> [--repeat] [--plan-only]
                         [--transport i2c|mock] [--dir <strategies dir>]
                         [--list]
```

| Option | Meaning |
|---|---|
| `--strategy NAME` | a name in the strategies dir, or a path to a `.json` file (`STRATEGY` env is the fallback) |
| `--list` | list saved strategies and exit |
| `--plan-only` | plan and print the stats - no motion (great for checking a strategy; 'elements' documents - a tree plans inside its `run_document` leaves when played) |
| `--repeat` | replay until stopped (Ctrl+C) |
| `--transport` | `i2c` (default) or `mock` |
| `--dir` | strategies directory (default `STRATEGY_DIR` or `<repo>/strategies`) |

Examples:

```bash
# see what is saved
python -m software.brain --list

# verify the plan without moving (mock)
python -m software.brain --strategy demo --transport mock --plan-only

# play it for real
python -m software.brain --strategy main

# rehearse on the mock, looping
python -m software.brain --strategy main --transport mock --repeat
```

Exit codes: `0` strategy done (or `--list`/`--plan-only` ok), `1` the run
ended early, `2` no strategy given / not found. Logs go to stdout with a
timestamp and level, so `journalctl`/docker logs show the whole match story
(re-plans, waits, faults, completions) without a browser.

## 5. Running with docker

```
cd docker
docker compose --profile brain up -d --build
```

Set `STRATEGY` in `docker/.env` (name or path inside the container):

```
STRATEGY=main
```

> Remote access: the same stack runs the `vpn` Tailscale sidecar - set
> `TS_AUTHKEY` in `docker/.env` and manage everything from any of your
> devices at `http://<tailnet-name>:8080` (see `../docker/README.md`).

The `brain` service mounts the same `strategies` volume as the console, so a
strategy saved in the console web UI is immediately playable. The
`console.Dockerfile` / `brain.Dockerfile` images install the same `charara`
package - the brain image simply does not include FastAPI usage.

> The console and the brain are separate processes and both own
> `/dev/i2c-1`. Run the brain while the console is NOT driving the robot
> (stop the `api` service or use the console in mock mode).

## 6. Extending the brain

* **Obstacle sources (camera, companion cards)** - everything that can
  deliver obstacle lists should end up in `brain.set_obstacles(...)`; the
  watcher (debounce, proximity re-checks, TTL sweep) is already wired. A
  future sensor wrapper (socket, shared file, MQTT, ...) is a thin adapter in
  `software/brain/` and nothing else changes.
* **IMU** - feed yaw-rate samples into
  `brain._localization.add_gyro_yaw_rate(omega, t)` (the same hook the
  console uses); the estimator then reports mode `wheel+gyro` and heading is
  fused from the beginning. No mission code changes.
* **Strategy selection** - `--strategy` already accepts any saved document;
  a match manager can start the brain with the right one (env/docker), or a
  future brain server can expose `play()` over a small API.
* **Companion cards** - commands to other boards belong next to the brain's
  robot port: add methods to a `RobotIO`-style port and call them from
  strategy elements or the brain loop; keep the core engine unaware of the
  wiring.

## 7. Testing the brain

`tests/test_brain.py` runs the whole runner on the mock board: start ->
telemetry -> plan -> execute -> stop, a strategy loaded from a file, an
obstacle report mid-run (a far obstacle must not disturb the run) and the
empty-strategy failure path. The CLI list/missing-strategy paths are covered
too. `tests/test_strategy.py` covers the document model and the REST round
trip.

```bash
# from the repository root
python -m unittest discover -s tests -p "test_brain.py" -v
python -m unittest discover -s tests -p "test_strategy.py" -v
```

## 8. Behavior-tree strategies (v2 documents, Track C)

A v2 document keeps the same envelope (name, options, obstacles, notes) but
replaces the drawing with a **behavior tree**: the drawing stays the mission
description, the tree describes the DECISIONS around it (which mission to
run, what to do when one fails, when to stop). The brain plays both kinds -
`kind` decides; v1 documents load unchanged (`kind` defaults to
`"elements"`).

```json
{
  "name": "match-2026",
  "version": 2,
  "kind": "bt",
  "options": {"match_s": 100, "score": 0},
  "tree": {
    "type": "sequence",
    "children": [
      {"type": "action", "name": "run_document", "doc": "collect-main"},
      {"type": "selector", "children": [
        {"type": "action", "name": "run_document", "doc": "collect-backup"},
        {"type": "action", "name": "log", "message": "backup path failed"}
      ]},
      {"type": "action", "name": "add_score", "points": 2}
    ]
  }
}
```

Structural nodes: `sequence`, `selector`, `parallel` (field `children`).
Decorators: `invert`, `force_success`, `force_failure`, `retry` (`attempts`),
`timeout` (`seconds` - bounds ONE child tick via asyncio cancellation, so a
long action leaf can be cut). Leaves: `action` / `condition` with a `name`
resolved through the match registry (`software/brain/match/nodes.py`).
Unknown types or names fail at LOAD time - a document is validated before
play, never mid-match.

Leaf catalogue (v1, implemented in `brain/match/nodes.py`):

| Actions | Meaning |
|---|---|
| `run_document(doc)` | plan + execute a saved v1 document; SUCCESS when it completes |
| `wait(seconds)` | pause the tree (cancellable by `timeout`) |
| `log(message)` | one line into the brain log |
| `add_score(points)` | raise the blackboard score (manual/referee today) |
| `abort_run(reason)` | stop a running mission right now |

| Conditions | Meaning |
|---|---|
| `at_pose(x, y, theta?, tol_mm?, tol_deg?)` | fused pose within tolerance |
| `executor_idle()` | no mission segment running |
| `camera_fresh()` | the vision snapshot is inside its TTL |
| `time_left(seconds)` | `match_s` deadline has time left (no deadline = True) |
| `score_at_least(points)` | blackboard score reached |
| `team_is(side)` | configured `TEAM` matches `yellow` / `blue` |

Engine parity rule: every action leaf calls the SAME mission engine as the
console and `play()` (`Brain._execute_strategy` -> `execute_plan` -> the
executor) - a tree can never invent a new motion path, and one plan runs at a
time (`plan_lock`). STOP/E-STOP stay independent of the tree.

> Team mirroring covers v1 GEOMETRY only (strokes, waypoints, rotations,
> goal, obstacles). A v2 TREE is played exactly as authored - absolute
> coordinates inside leaves (e.g. `at_pose`) are NOT mirrored; guard the
> branch with `team_is` or keep one document per side.

Testing: `tests/test_bt.py` (scheduler semantics - deterministic, no
robot) and `tests/test_brain_match.py` (catalogue + full tree playback on
the mock board).
