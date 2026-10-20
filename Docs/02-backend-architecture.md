# 02 - Software architecture

The robot software is ONE Python package, `software/`, split into two
applications and two libraries. The two applications are the **console**
(FastAPI service used to configure, calibrate, test and plan) and the
**brain** (the RPi's main job: play a saved strategy headless). They compose
the same libraries, so exactly the code that is tested in the console is the
code that drives the robot in a match.

```
charara/                  the RPi repository / checkout root - docker/, Docs/,
                          strategies/, tests/, ui/ and the Python package
`-- software/             the Python package
```

## 1. Repository layout

```
charara/                            (repository root)
|-- README.md
|-- pyproject.toml                  one package: charara; extras: console, hardware, dev
|-- .dockerignore
|-- docker/                         every container file lives here
|   |-- compose.yml                 api + ui (+ brain behind the "brain" profile)
|   |-- compose.dev.yml             mock/demo stack on any machine
|   |-- console.Dockerfile          the console service image
|   |-- brain.Dockerfile            the brain image
|   |-- ui.Dockerfile               nginx + static console
|   |-- .env.example                copy to docker/.env
|   `-- README.md                   build/run recipes
|-- strategies/                     saved strategy documents (JSON), shared
|   `-- demo.json                   sample strategy
|-- tests/                          unittest suite for the whole package
|-- ui/                             the web console (static, no CDN)
|   |-- nginx.conf / entrypoint.sh
|   `-- static/                     index.html, css/, js/, field/
|-- Docs/                           this documentation
`-- software/                       THE PYTHON PACKAGE
    |-- config.py                   Settings from env (shared by console + brain)
    |-- core/                       pure domain - no I/O, no third-party imports
    |   |-- navigation/             geometry, grid, clearance, options,
    |   |                           trajectory, replan, velocity
    |   |                           pursuit, tracking, sim (path following, B)
    |   |-- localization/           kinematics, kalman, service (fused pose:
    |   |                           wheels + gyro + camera, gating, status)
    |   |-- world/                  VisionStore (obstacles) + PoseStore (robots)
    |   `-- mission/
    |       |-- interfaces.py       ports: RobotIO, RobotIOError, MissionContext
    |       |-- executor.py         runs a motion program on a robot port
    |       |-- follower.py         streaming (SET_VELOCITY) follower backend (B)
    |       |-- bt.py               behavior-tree runtime (strategy v2, C)
    |       |-- orchestrator.py     plan / mid-run re-plan / wait decisions
    |       |-- watcher.py          obstacle reactions: broadcast, re-plan, sweeps
    |       `-- strategy.py         strategy document (v1 elements / v2 trees)
    |-- robot/                      board access (protocol, links, client, models)
    |-- sensors/                    RPi-local sensors: MPU6050 driver (optional,
    |                               absent = today's wheel-only behaviour)
    |-- session/                    RobotSession: shared composition root used
    |                               by the console and the brain (Docs/09 5.1)
    |-- console/                    the FastAPI console service
    |   |-- main.py                 app factory, lifespan, static UI mount
    |   |-- api/routes/             REST, one module per domain
    |   |   |-- deps.py             client access + error mapping
    |   |   |-- models.py           pydantic request/response bodies
    |   |   |-- system.py           health, mode, state, field, pose, board log, display, trace
    |   |   |-- motion.py           manual moves, execution control, E-STOP
    |   |   |-- planning.py         plan / re-plan, last plan
    |   |   |-- settings.py         limits, watchdog, settings registers
    |   |   |-- sensors.py          IMU / gyro calibration + health (Docs/09)
    |   |   |-- selftest.py         acceptance card: tests, readiness, audits, fault drills
    |   |   `-- strategies.py       save / load / list / delete strategies
    |   |-- api/ws.py               /ws fan-out + /ws/vision protocol
    |   `-- runtime/                AppState (mission context, wraps RobotSession) + trace
    `-- brain/                      the headless strategy player
        |-- runner.py               Brain + BrainContext
        |-- match/                  behavior-tree match layer (nodes, play_tree)
        `-- __main__.py             CLI: python -m software.brain
```

## 2. Layering rule

Dependencies point downwards, always:

```
console --+                  brain --+
          |--> core  <-- robot ------+   (robot imports only core.mission.interfaces)
          |      ^
          +------+  console and brain IMPORT core + robot;
                    core imports NOTHING from robot/console/brain.
```

* **`software.core`** is pure Python: no file I/O in the algorithms, no
  third-party packages, no knowledge of I2C, FastAPI or asyncio tasks other
  than locks. It can be unit-tested anywhere and reused as-is by tooling.
* **`software.robot`** owns the wire protocol and the transports. Its errors
  derive from `core.mission.interfaces.RobotIOError` so the mission layer can
  react to robot trouble without importing the board package.
* **`software.console`** and **`software.brain`** are compositions: they wire
  a robot link, a localization service, an executor and a mission context
  together, then either serve the API (console) or play strategies (brain).

The mission layer talks to the robot through a PORT
(`core.mission.interfaces.RobotIO`, structural typing): any object with the
async methods `odometry/set_deadman/stop/estop/goto/drive/turn/strafe/arc/
display_text`. `robot.client.RobotClient` implements it for the real or mock
board; tests and the brain use the same port.

## 3. The core domain

### 3.1 Navigation (`core/navigation/`) - pure planning math

See [03 - Planner & algorithms](03-planner-algorithms.md) for the formulas.
Modules: `geometry` (RDP, circle fitting, line/arc segmentation, fillets,
resampling), `grid` (occupancy grid + Theta*), `clearance` (obstacle merge,
body inflation, grid build), `options` (`PlanOptions`, every planner knob),
`trajectory` (`plan()`: elements -> chained ARC/GOTO/TURN/PAUSE program),
`replan` (remaining-path slicing, path fraction, blockage checks),
`velocity` (trapezoidal profile with lateral limit).

### 3.2 Localization (`core/localization/`)

The pose the rest of the system consumes is **fused**, not the raw board
odometry:

* `kinematics.py` - `wrap180`, `twist_from_wheels`, `integrate_pose`
  (midpoint integration).
* `kalman.py` - `PoseKalmanFilter`: constant-velocity 4-state filter
  `[x, y, theta, omega]` (mm, deg). `predict` uses the linearized midpoint
  model, `Q` derives from the control noise; `update_pose` fuses a board
  pose reading; `update_angular_rate` is the **IMU hook** (a gyro is a direct
  omega measurement). All dense 4x4 math is plain Python - no numpy.
* `service.py` - `LocalizationService`: `ingest(WheelSample)` predicts then
  updates; the first sample is adopted, a discontinuity above 150 mm /
  30 deg (SET_POSE, odometry reset, reboot) is adopted immediately, and at
  rest (|v| < 1 mm/s) the measurement sigma tightens (2.5 mm / 0.2 deg) so a
  standing robot cannot drift. `add_gyro_yaw_rate(omega, t)` switches
  `status().mode` from `wheel` to `wheel+gyro` when gyro data is fresh.

### 3.3 World model (`core/world/vision.py`)

`VisionStore` keeps the newest camera snapshot (obstacles + TTL + version +
quantised fingerprint so identical refreshes are not "changes") and
`merged_obstacles(console_obstacles, store)` merges console-drawn and camera
obstacles for every plan.

### 3.4 Mission (`core/mission/`)

* `interfaces.py` - the ports: `RobotIO` (see section 2), `RobotIOError` and
  `MissionContext`. A mission context is the host process seen by the
  engine: `plan_lock`, `executor`, `vision`, `field`, `last_plan`,
  `last_plan_request`, `vision_waiting`, `vision_replan_task`, `pose()`,
  `log()`, `emit()`. The console's `AppState` and the brain's `BrainContext`
  both satisfy it - the engine itself is host-agnostic.
* `executor.py` - runs a segment list on the robot port one segment at a
  time (the firmware rejects concurrent motion): send, poll odometry until
  idle, watch faults and per-segment timeouts, publish progress events.
  Pause = STOP + wait (nodes are absolute, so resume is safe). Stop reasons
  replace the "aborted" message (the console shows "re-planning ..." instead).
  `set_speed_scale()` slows every motion command without touching the plan
  (the dynamic-obstacle caution; a fresh run resets it to 1.0) and is
  published as `exec.speed_scale`. Starting a run clears a **DEADMAN** latch
  left by the previous stop (and only that bit, loudly): a console-driven stop
  could leave the watchdog latched while the wheels were winding down, and
  every later restart then aborted with `robot status FAULT`.
  `remaining_elements()` rebuilds planner elements from the not-yet-driven
  tail - the fallback mission source for a re-plan. The optional **FOLLOW
  mode** (Track B) drives the whole plan polyline with continuous
  `SET_VELOCITY` frames through `follower.PathFollower` instead of stop-turn
  programs (`start(..., follow_path=...)`): one acceleration, one
  deceleration, PAUSE = STOP + hold, and the caution `speed_scale` is
  republished to the follower every tick. The start is REFUSED when the
  session's capability check fails (`STREAMING_EXEC` off, firmware < 3.4 or
  no link) - a requested stream never silently degrades into the segment
  backend; `last_stream` records which backend runs so a mid-run re-plan
  restarts in the same one. On either backend, a successful run that ends
  more than 12 mm / 4 deg from the plan's last motion node sends **one**
  small closed-loop correction GOTO (`FINAL_CORRECTION_*`) measured from the
  live pose: wheel slip or a cut arc corner used to leave the robot 30-40 mm
  short of the drawn waypoint. It is skipped after an abort/pause or when a
  trailing TURN/PAUSE owns the final heading.
* `orchestrator.py` - `execute_plan(ctx, elements, goal, obstacles, options,
  deadman_ms)`, the single planning entry point (console and brain):
  * robot idle: plain plan (returns segments, preview path, stats);
  * robot executing: **dynamic re-plan** - a candidate of the REMAINING
    route (what the robot actually drives, detours included) is planned on
    the new obstacle map and compared with the current remaining length. Only
    a real difference (> max(100 mm, 2 %)) stops the robot and swaps the
    program; an edited mission goes back to the drawing (sliced at the
    robot). The response is a LADDER by distance (2026-10-09): beyond
    `CAUTION_GAP_MM` (700 mm) along the route a known clip is only watched;
    inside it every motion command is slowed to `CAUTION_SCALE` (45 %, no
    stop, no swap); inside `STOP_GAP_MM` (250 mm) - or at any distance when
    the blockage point is tight (border/static obstacle) - the robot stops and
    resolves: a candidate that still drives through the obstacle and every
    long detour favour "wait" (`resume_from_wait` restarts when the map
    clears, verifies the restart survived, and gives up loudly after
    `WAIT_RESUME_MAX_FAILS`; a paused run stays paused). The planner itself
    runs in a worker thread so the event loop (and the telemetry that keeps
    the board's deadman fed) never stalls.
* `watcher.py` - reactions to obstacle-map changes: `notify_change` (log +
  broadcast + debounced re-plan), `schedule_replan` (coalesced, 0.25 s),
  `proximity_loop` (re-checks as the robot advances, quiet when nothing is
  blocked), `sweep_loop` (TTL expiry). All take the mission context.
* `follower.py` - the streaming path-following backend (Track B, Docs/10):
  follows a planned polyline with Pure Pursuit over silent SET_VELOCITY
  frames, supervising faults, telemetry and a path timeout; same deadman
  arming as an executor run, explicit STOP before returning. Safety rules
  added after the 2026-10-18 field session: a lookahead behind the robot
  becomes a **turn in place** (never full speed with a tiny curvature - that
  drove the robot straight out of the field), the run is refused when the
  live pose is > 300 mm off the whole path, a **field-edge geofence**
  (`bounds`, the planner's legal region) stops the board before the robot can
  leave the game area, and the plan's **final heading** (waypoint angle /
  goal theta / ARC exit) is performed with an in-place turn after the path
  end. A plan with an explicit rotation, pause or mid-route arrival turn
  cannot be encoded as a polyline: `/execute` (and the brain runner) uses the
  segment backend for it, loudly, instead of silently skipping the action.
  Driven by the executor's FOLLOW mode, which provides the caution
  scale, STOP and PAUSE/RESUME hooks; a false capability check refuses before
  any frame.
* `bt.py` - the behavior-tree runtime of strategy v2 (Track C, Docs/10):
  Status, composites with running-child memory, decorators
  (invert/force/retry/timeout), a JSON-style spec loader and a tick loop.
  Pure and host-agnostic - the brain injects the leaf registry.
* `strategy.py` - the strategy document (elements, options, goal, obstacles,
  field size, notes, version) + JSON persistence (`save/load/load_named/
  list_saved/delete`, name validation). See
  [08 - Brain & strategies](08-brain-and-strategies.md).

## 4. Board access (`robot/`)

* `protocol.py` - codec for firmware protocol v3 (16-byte requests / 32-byte
  responses, CRC-8 poly 0x07, LE helpers, command/state/fault/event tables).
* `robot_config.py` - the 17-register settings map mirroring the firmware
  (validation, defaults, chunked GET/SET, speed limit).
* `link.py` - `I2CLink` (smbus2, optional import; the board on /dev/i2c-1)
  and `MockLink` (whole board simulated: motion interpolation, busy
  rejection, latched e-stop, deadman, odometry from wheel integration,
  register store with flash simulation and the event log).
* `client.py` - `RobotClient`: async lock + `to_thread` so the event loop
  never blocks on I2C; typed commands; trace callback. Errors: `RobotError`
  (board answered non-OK) and `LinkError` (transport), both `RobotIOError`.
* `models.py` - `Odometry` (`is_idle`, `describe_faults()` helpers) and
  `Status` dataclasses parsed from the protocol.

## 5. Console service (`console/`)

* `main.py` - FastAPI app: lifespan builds the link (env `ROBOT_TRANSPORT`),
  starts the watcher sweep + proximity loops, mounts `ui/static` when
  present (`UI_DIR` or the sibling `ui/static`).
* `api/routes/` - one module per domain (system, motion, planning, settings,
  strategies) + `deps.py` (client access, `require_no_execution`, error
  mapping) and `models.py` (pydantic v1+v2 compatible bodies; use
  `deps.to_dict()`).
* `api/ws.py` - `/ws` fan-out hub (telemetry, exec progress, logs, I2C
  trace, vision events) and the `/ws/vision` protocol (parse, ack, drop a
  disconnecting client's obstacles -> `watcher.notify_change`).
* `runtime/state.py` - `AppState`: the console's **mission context**. Holds
  the client/executor, the localization service, the vision store, the
  field, the last plan/request, the trace buffer and the event sink. The
  telemetry loop (10 Hz) reads odometry, feeds `localization.ingest()`,
  polls board diagnostics (~1 Hz) and broadcasts `state.snapshot()`: fused
  `x_mm/y_mm/theta_deg`, raw `board_odometry`, `loc` health block, board
  events/status, exec progress and the vision source.
* `runtime/trace.py` - 4000-entry I2C ring buffer (the console's monitor;
  `GET /api/trace`, `/api/trace/download`, live `trace` events).

## 6. Brain runtime (`brain/`)

`Brain` (runner.py) is the console's `AppState` without the web layer: link +
telemetry + localization + executor + `BrainContext`, and the same watcher
loops. `play(strategy)` plans from the LIVE pose around the LIVE obstacle
map, executes, and loops on request; `set_obstacles()` is the camera input
point (replaces the snapshot, triggers the usual reaction). A v2 document
(`kind: "bt"`) is played by the match layer instead (`brain/match/`, Track
C): the same mission engine behind its leaves, decisions between missions.
The CLI (`python -m software.brain`) selects a strategy by name or path and
can run it once (`--plan-only`), normally, or with `--repeat` (v1 documents
only - a tree decides its own repetition). Details:
[08 - Brain & strategies](08-brain-and-strategies.md).

## 7. Strategies

A strategy is what the RPi plays: the drawing (planner elements), plan
options, an optional goal, the table obstacles and notes - independent from
any plan output, so the trajectory is computed at play time from the current
pose. The console saves/loads them (`/api/strategies`, storage in
`STRATEGY_DIR`, default `strategies/`); the brain consumes the same JSON.

## 8. Configuration (`config.py`)

`Settings` is built once from the environment (shared by console and brain):

| Variable | Default | Meaning |
|---|---|---|
| `ROBOT_TRANSPORT` | `i2c` | boot mode (`mock` for simulation) |
| `I2C_BUS` / `I2C_ADDRESS` | `1` / `0x0A` | board on the bus |
| `I2C_RETRIES` / `I2C_RETRY_DELAY_MS` | `5` / `5` | retry policy |
| `FIELD_WIDTH_MM` / `FIELD_HEIGHT_MM` | `3000` / `2000` | play area |
| `DEFAULT_SPEED_MM_S` / `DEFAULT_ACCEL_MM_S2` / `DEFAULT_DECEL_MM_S2` / `DEFAULT_LAT_ACCEL_MM_S2` | `500` / `700` / `700` / `300` | speed / accel / decel / lateral defaults |
| `DEFAULT_GOTO_STEP_MM` / `DEFAULT_FIT_TOL_MM` | `250` / `8` | planner defaults (also `DEFAULT_INFLATION_MM` 180, `DEFAULT_GRID_MM` 50, `DEFAULT_STEP_MM` 50) |
| `TELEMETRY_HZ` | `10` | odometry poll rate (also the deadman feed) |
| `DEADMAN_MS` / `EXEC_TIMEOUT_SLACK_S` | `1500` / `8` | hardware watchdog / timeout slack |
| `ODOM_TRACK_MM` | `170` | wheel track for the localization model - fallback only: the board's register 7 value is adopted at connect and after every settings write |
| `GYRO_SCALE` | `1.0` | spin-calibrated MPU6050 scale (see Docs/09 section 9.3) |
| `IMU_BUS` / `IMU_ADDRESS` | `6` / `0x68` | MPU6050 on the dedicated I2C6 bus (GPIO22/23, Docs/09 section 9.2) |
| `IMU_YAW_AXIS` | `auto` | gyro axis + sign for yaw; `auto` detects it from gravity at boot (facing down -> `-z`, Docs/09 section 9.1) |
| `START_POSE` | `1` | adopt the team match-start pose at startup when the board still reads (0,0,0) |
| `STRATEGY_DIR` | `<repo>/strategies` | shared strategy storage |
| `UI_DIR` | sibling `ui/static` (`/srv/ui/static` in docker) | static console mount - read by `console/main.py` directly (console only) |

## 9. Concurrency model

One asyncio loop per process. REST handlers are async; planning is CPU-bound
(~50-300 ms) and serialised by `ctx.plan_lock`. Each host runs ONE executor
task, one telemetry task and the watcher loops; pose fusion runs inline in
the telemetry loop (microseconds). I2C calls are off-loaded to threads by
`RobotClient`. The console and the brain are separate processes and must not
drive the same board at the same time (both own `/dev/i2c-1`).

## 10. Deployment (`docker/`)

| Service | Image | Role |
|---|---|---|
| `api` | `docker/console.Dockerfile` | console REST/WS, port 8000 |
| `ui` | `docker/ui.Dockerfile` | nginx static console, port 8080 |
| `vpn` | `tailscale/tailscale:latest` | Tailscale node (host network): `http://<tailnet-name>:8080` reaches the console |
| `brain` | `docker/brain.Dockerfile` | plays `STRATEGY` (profile `brain`) |

```
cd docker
docker compose up -d --build                    # console + ui + vpn (tailnet)
docker compose --profile brain up -d            # + brain (set STRATEGY first)
```

`docker/compose.dev.yml` runs a mock console + ui on any machine. Without
docker: `python -m uvicorn software.console.main:app --port 8090` (from the
repository root) and `python -m software.brain --strategy demo --transport
mock`. The strategies directory is a shared volume in docker so a strategy
saved in the console is what the brain plays.

With `TS_AUTHKEY` set in `docker/.env`, the `vpn` service authenticates the
Pi to your tailnet, so every device of yours reaches the console at
`http://<tailnet-name>:8080` (console 8080, API 8000) - see `docker/README.md`.
