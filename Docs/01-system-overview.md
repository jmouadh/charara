# 01 - System overview

## 1. What the robot is

Charara is a **differential-drive Eurobot-style robot**. Four stepper motors
(two per side), quadrature encoders, limit switches, an OLED display and an
RGB status LED are driven by an **STM32F722** board (firmware v3.5,
bare-metal + IAR). A **Raspberry Pi 4** runs this service and is the only
thing that talks to the board; a browser console is the operator interface.

```
                       +---------------------------------------------+
   browser console     |  Raspberry Pi 4                             |
 +---------------+     |                                             |
 | field canvas  |<-->|  +----------+   +-----------+    +-------+  |
 | sequence      | WS  |  | console  |-->| core pkgs |-->|robot  |  |
 | mission btns  |-----|  | REST/WS  |   | nav, loc, |    |client |  |
 +---------------+     |  +----------+   | world,    |    +---+---+  |
                       |                 | mission   |       | I2C   |
   camera / detector   |                 +-----------+       |       |
 +---------------+     |                                     v       |
 | obstacles at  |-----|-- /ws/vision              +--------------+  |
 | (x, y) + size |     |                           | STM32F722    |  |
 +---------------+     |                           | motors/OLED  |  |
                       +---------------------------|--------------+  |
                                                   +-----------------+
```

## 2. Components

### 2.1 STM32 board (firmware)

* Protocol **v3**: 16-byte requests / 32-byte responses, CRC-8 (poly 0x07),
  sequence-id idempotency, commands `0x01...0x7F`. Speeds are u16 mm/s with an
  absolute ceiling of 900 mm/s.
* Motion commands: `GOTO(x, y, theta)` (executed as *turn in place -> drive ->
  turn in place*), `ARC(radius, sweep, speed)` (one continuous circular
  sweep - the primitive behind smooth combined moves), `DRIVE`, `TURN`,
  `STRAFE`, plus `STOP`, `E-STOP`,
  `SET_POSE`, `RESET_ODOMETRY`, `SET_MOTION_LIMITS`, `SET_DEADMAN`,
  display commands, queries (`PING`, odometry, status, tick counts, board
  event log `QUERY_EVENTS`).
* **Settings store:** `GET_CONFIG` / `SET_CONFIG` / `SAVE_CONFIG` /
  `LOAD_CONFIG` / `RESET_CONFIG` read and write 17 calibration registers
  (wheel geometry, tracks, speed limits, ramps, gains, direction masks);
  `SAVE_CONFIG` writes them to the board flash and they are reloaded at
  boot.
* **Error reporting:** `QUERY_STATUS` and `QUERY_ODOMETRY` carry 16-bit
  fault masks; every failure (faults, bad frames, rejected parameters, I2C
  and flash errors, config fallbacks, reset causes) is also recorded in the
  board event log readable with `QUERY_EVENTS`, with a code and a context
  byte.
* Safety: limit switches stop the motors in the EXTI ISR, faults latch until
  `CLEAR_FAULTS`, a deadman timeout (`SET_DEADMAN`) brakes if the Pi stops
  polling.
* The board never accepts streamed trajectories - this is *why* the planner
  minimises per-node turning (fillets, curvature-aware node spacing, native
  arc segments).

The firmware and its spec documents live in the firmware project (not part of
this repository); the Python codec that mirrors the wire protocol is
`software/robot/protocol.py`.

### 2.2 Robot software (`../software`) - console + brain

ONE Python package with two applications that compose the same libraries:

* **console** (FastAPI, `software/console/main.py`) - the workshop, with
  three faces:
  1. **REST API** under `/api` - plan, execute, telemetry snapshot, manual
     moves, configuration, I2C trace, strategies
     (`software/console/api/routes/`, one module per domain).
  2. **Console WebSocket** `/ws` - server -> browser fan-out (telemetry, exec
     progress, logs, I2C frames, vision obstacle updates).
  3. **Vision/camera WebSocket** `/ws/vision` - input channel where a camera
     or detector pushes the obstacles it currently sees.
* **brain** (`software/brain/`) - the RPi's main job: play a saved strategy
  headless (plan -> execute -> avoid obstacles -> repeat). No web layer; it
  offers the same planning / re-plan / avoidance engine as the console.

Internals: pure planning math and mission engine (`software/core/navigation`,
`software/core/mission`), fused pose and kinematics
(`software/core/localization`), camera obstacle store (`software/core/world`),
board link & protocol (`software/robot`), console service
(`software/console`). See
[02 - Software architecture](02-backend-architecture.md).

### 2.3 Web console (`../ui/static`)

Vanilla-JS single page (no CDN): canvas field with the Eurobot 2026 table
artwork, drawing tools, mission sequence editor, plan/execute controls,
manual drive pad, I2C monitor, configuration tabs and a **Calibration tab**
(drive/turn tests, measured-value corrections, board error log). See
[04 - Console guide](04-console-guide.md).

### 2.4 Camera / vision detector (external)

Any process that can open a WebSocket pushes the obstacles it sees:

```json
{"type": "obstacles",
 "obstacles": [{"kind": "rect", "x": 1500, "y": 400, "w": 300, "h": 200}],
 "ttl_ms": 2000}
```

The service merges them into every plan (they are drawn as dashed outlines in
the console) and, **if the robot is executing**, it re-checks the *remaining*
route: only when that route is actually affected does the robot stop, re-route
and continue. See [06 - Vision integration](06-vision-camera-integration.md).

## 3. Coordinate frames and units

| Frame | Origin | Axes | Angle |
|---|---|---|---|
| Field / robot | bottom-left corner of the play field | +x right, +y up | theta CCW positive, degrees in all APIs |
| Screen (console) | top-left of the canvas | +x right, **+y down** | display only - the console converts |

* All distances are **millimetres**, speeds mm/s, angles degrees.
* The robot position is the **body centre**; the body sweeps a circle of
  `half-diagonal` radius around the path (default 300x300 mm -> 212.1 mm).
* Field default 3000 x 2000 mm, configurable (`POST /api/field`).
* Firmware wire format uses 0.1 mm units - the Python protocol layer handles
  the conversion.

## 4. Data flow of a mission

1. **Draw** - the operator draws strokes on the console; each stroke is a list
   of `[x, y]` points. Waypoints/rotations/pauses are inserted in order.
2. **Plan** - `POST /api/plan` sends the ordered elements + obstacles + goal +
   options. The pipeline fits the drawing (lines/arcs), routes it around
   physical obstacles (drawn-first), rounds corners, samples nodes, computes
   speeds, and returns:
   * `segments` - the program: `goto` / `arc` / `turn` / `pause`, executed in order;
   * `path` / `reference` - the preview polylines;
   * `stats` - length, estimated time, counts;
   * `warnings` - every deviation from the drawing, in plain words.
3. **Execute** - `POST /api/execute` hands the segments to the **executor**,
   which sends them one at a time, polls odometry at 10 Hz, arms the hardware
   deadman, broadcasts progress, and honours PAUSE/RESUME/STOP/E-STOP.
4. **Dynamic obstacles** - while executing, a new plan request (console edit)
   or a camera report that changes the obstacle map triggers a *re-check*: the
   remaining mission is re-planned only if the new map affects it; otherwise
   the robot keeps driving its route (log: *"trajectory unchanged"*).

## 5. Modes: real board vs simulator

| Mode | Env | Behaviour |
|---|---|---|
| **REAL** | `ROBOT_TRANSPORT=i2c` (default) | the service talks to the board on `/dev/i2c-1` (address `0x0A`) |
| **MOCK** | `ROBOT_TRANSPORT=mock` | the service simulates the whole board (virtual motion, odometry, faults, settings registers with flash simulation, board event log) - no hardware needed |

The console header has a REAL/MOCK selector (`POST /api/mode`) - switching is
refused while a mission executes and gracefully falls back when the board
cannot be opened (e.g. no `smbus2` on a dev PC). Every telemetry snapshot
carries `transport`, and the mock mode shows a watermark on the field.

Both modes behave identically for planning and execution, so a mission can be
developed on the PC and run unchanged on the robot.

## 6. Deployment shapes

| Shape | Command | UI served by |
|---|---|---|
| Dev PC (mock) | `python -m uvicorn software.console.main:app --port 8090` from the repository root | the FastAPI app itself (`ui/static` is auto-mounted) |
| Docker on the Pi | `docker compose up -d --build` | nginx (`ui` container) proxying `/api` + `/ws` to the `api` container |
| Remote (Tailscale) | any tailnet device opens `http://<tailnet-name>:8080` | same nginx console; the `vpn` sidecar puts the Pi on the tailnet (set `TS_AUTHKEY` in `docker/.env`) |
| Docker dev | `docker/compose.dev.yml` | mock mode, no `/dev/i2c` mapping |

See `../README.md` for the complete commands, and
[07 - Development guide](07-development-guide.md) for the test setup.

## 7. Glossary

| Term | Meaning |
|---|---|
| **element** | one entry of the mission: `stroke`, `waypoint`, `rotate`, `pause` |
| **run** | a merged chain of strokes the planner treats as one continuous path (corners between them get filleted) |
| **GOTO node** | one absolute waypoint sent to the board; the plan samples the path into nodes |
| **body clearance** | half-diagonal + small buffer - the hard physical limit; routing never violates it |
| **obstacle margin** | `obstacle_margin_mm` (default 150) - soft clearance; the drawn line may use it |
| **waypoint margin** | `waypoint_margin_mm` (default 0) - extra stop clearance over half the robot body side; close waypoints use a straight final leg and pre-rotate |
| **drawn-first routing** | the drawn geometry is kept; only *physically blocked* runs are re-routed |
| **slice** | at a mid-run re-plan, the mission is cut at the robot's stop position and only the remainder is planned |
| **TTL** | time-to-live of a camera obstacle snapshot |
