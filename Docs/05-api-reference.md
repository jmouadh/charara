# 05 - API reference

Base URL: `http://<host>:<port>/api` (dev: `:8090`; docker: `:8080` via nginx,
or `:8000` on the `api` container directly).
All bodies are JSON; all angles are degrees (CCW positive); all lengths are
millimetres. No authentication (trusted team network).

Error shape (FastAPI): `{"detail": "<message>"}` with
`400/404/409/422/503` as noted per endpoint. `409` = conflict with the current
state (e.g. an execution is running), `422` = the request cannot be planned,
`503` = the robot link/board is unavailable.

---

## 1. Status & state

### `GET /api/health`
```json
{"ok": true, "transport": "mock", "connected": true,
 "error": "", "version": "3.2.0", "build": "20261019.1336",
 "firmware": "3.5"}
```
`version` is the console package version, `build` the Docker image build
stamp (empty when running from a plain checkout), and `firmware` the version
reported by the board (`QUERY_STATUS`); the latter reads `"unknown"` until
the first diagnostics poll has answered.

### `GET /api/state`
Telemetry snapshot + execution progress + camera obstacle status:
```json
{
  "connected": true, "error": "", "transport": "mock",
  "x_mm": 1234.5, "y_mm": 678.9, "theta_deg": -45.0,
  "v_left_mm_s": 0.0, "v_right_mm_s": 0.0,
  "state": 0, "state_name": "IDLE", "faults": 0, "fault_names": [],
  "loc": {"mode": "wheel", "sigma_mm": 2.1, "sigma_deg": 0.2, "samples": 512,
          "gyro_samples": 0, "jumps": 0, "yaw_rate_deg_s": 0.0, "track_mm": 170.0},
  "imu": {"state": "off", "present": false, "yaw_axis": "z",
          "mount": {"axis": "z", "sign": -1, "tilt_deg": 1.2, "gravity_g": 1.0, "mode": "auto"},
          "rate_deg_s": 0.0, "bias_deg_s": 0.0,
          "scale": 1.0, "integrated_deg": 0.0, "samples": 0, "errors": 0, "age_s": null},
  "gyro_stream": {"state": "off", "frames": 0, "errors": 0},
  "board_odometry": {"x_mm": 1234.5, "y_mm": 678.9, "theta_deg": -45.0},
  "board": {"firmware": "3.0", "uptime_s": 412.7, "faults": 0,
            "fault_names": [], "last_event": "RESET_POWER_ON",
            "reset_cause": "RESET_POWER_ON", "cfg_map_version": 1,
            "cfg_dirty": 0, "queue_depth": 0},
  "board_events": [{"index": 1, "code": 18, "aux": 0,
                     "name": "RESET_POWER_ON", "severity": "info"}],
  "exec": {"status": "idle", "index": 0, "total": 0, "message": ""},
  "vision": {"count": 1, "ttl_ms": 2000.0, "source": "vision:127.0.0.1:5678",
             "obstacles": [{"kind": "rect", "x": 1000.0, "y": 1000.0, "w": 200.0, "h": 200.0}],
             "t": 1738073000123.4, "t_server": 1738073000147.9}
}
```
`exec.status` in `idle | running | paused | done | aborted`.
`exec.speed_scale` is the current speed multiplier (1.0 normal, down to 0.2
while the dynamic-obstacle caution creeps past a blockage on the route).
`state` values: 0 IDLE, 1 DRIVE, 2 TURN, 3 STRAFE, 4 GOTO, 5 FAULT, 6 ESTOP, 7 ARC.
`faults` is a **16-bit mask** (`0x0001` ESTOP, `0x0002` LIMIT, `0x0004` STALL,
`0x0008` TIMEOUT, `0x0010` DEADMAN); `fault_names` decodes it. The `board`
block carries the firmware version, boot cause, settings version / dirty flag
and the last board event; `board_events` is the most recent event batch from
the 1 Hz diagnostics poll (`code`, `aux` context byte, name, severity).
`x_mm`/`y_mm`/`theta_deg` are the **fused (localized) pose**;
`board_odometry` keeps the raw board dead reckoning for comparison and `loc`
the estimator health (mode `wheel` | `wheel+gyro` | `wheel+camera` |
`wheel+gyro+camera`, sigmas, sample counters, computed jumps, yaw rate, the
wheel track in use - the board's settings register 7 value when it is
readable, otherwise `ODOM_TRACK_MM`).

### `POST /api/mode` - switch the robot link
```json
{"transport": "i2c" | "mock"}
```
`200 {ok, transport}` - `409` while executing - `503` when the link cannot be
opened (no bus/smbus2/board) - the current mode keeps running.

### `GET /api/field` - `POST /api/field`
```json
{"width_mm": 3000, "height_mm": 2000,
 "defaults": {"speed_mm_s": 200, "robot_width_mm": 300, ...}}
```
`POST` takes `{width_mm, height_mm}` (min 500 each). The size is stored in the
map document (below), so it survives a restart.

### Game/map configuration - `GET /api/map`, `PUT /api/map`

The document the console's **Game Configuration** tab edits:
table size, background artwork, the match clock + timed actions, one colour
per team and the starting zones (with the robot spawn pose of a primary zone).

```json
GET /api/map
{
  "version": 1, "updated_at": 1792737887.3,
  "field": {"width_mm": 3000.0, "height_mm": 2000.0},
  "background": "field/table.png",              // "" = automatic artwork
  "colors": {"yellow": "#efc13c", "blue": "#4b7bec"},
  "zones": [
    {"id": "zone-1", "team": "yellow", "type": "primary",
     "x": 0.0, "y": 1400.0, "w": 700.0, "h": 600.0,
     "spawn": {"x": 315.0, "y": 1800.0, "theta_deg": -90.0}}
  ],
  "game": {
    "duration_s": 100.0, "warning_s": 10.0,
    "actions": [{"id": "action-1", "at_s": 95.0, "kind": "log",
                 "enabled": true, "params": {"message": "release"}}]
  },
  "backgrounds": [                              // images and PDFs on disk
    {"name": "table.png", "value": "field/table.png", "kind": "image",
     "url": "field/table.png?v=1792737887"},
    {"name": "table_FINALE_1.0_2027.pdf", "value": "field/table_FINALE_1.0_2027.pdf",
     "kind": "pdf", "url": "/api/map/background/table_FINALE_1.0_2027.pdf"}
  ]
}
```

`PUT /api/map` takes ANY subset - omitted fields keep their value:

| Field | Meaning |
|---|---|
| `field` | `{width_mm, height_mm}` (min 500 each); the zones are re-clamped inside it |
| `background` | a file name from `backgrounds` - an image (`table.png` or `field/table.png`) or a PDF (`table_FINALE_1.0_2027.pdf`); `""` = automatic; a file that is not in the field folder is ignored |
| `colors` | `{yellow, blue}`, `#rrggbb` each (an invalid colour falls back to the default) |
| `zones` | the COMPLETE list of starting zones (replaces the stored list) |
| `game` | match clock: `{duration_s, warning_s, actions[]}` (duration 0 disables it; `at_s` is elapsed time, action kind `log` or the reserved `rest`) |

Zone rules: `team` is `yellow`/`blue`; `type` is `primary` (main robot) or
`secondary` (no spawn); the rectangle is clamped to the play area (minimum
50 x 50 mm); a `spawn` only exists on a primary zone and is clamped INSIDE
its rectangle (`theta_deg` is normalised to (-180, 180]). A primary zone's
spawn becomes that team's match start (`GET /api/team`).

```json
PUT /api/map   {"zones": [{"team":"blue","type":"secondary","x":2400,"y":1400,"w":600,"h":500}]}
DELETE /api/map/zones/zone-1      // remove one zone by id
```

`GET /api/map/backgrounds` lists the same `backgrounds` without the document.

`GET /api/map/background/{name}` serves one artwork: an image is returned
as-is, a **PDF** is rasterised on the service (poppler `pdftoppm`, page 1 at
~3000 px wide - the Eurobot tables are published as PDF) and answered as
`image/png`. The raster is cached under `MAP_CACHE_DIR` and keyed on the
file's mtime/size, so a re-exported PDF re-renders by itself. `404` for an
unknown or unsafe name (`..`, a sub-folder, a non-image suffix), `503` when
the PDF cannot be rendered (typically poppler missing from the image).
`PUT` answers `200` with the full document, `422` for a field below
500 x 500 mm, `503` when the document cannot be written. The document lives in
`MAP_CONFIG_FILE` (`/data/map/map.json` in docker, its own volume).

Saving zones also **puts the robot's pose on the start**: in `mock` the
simulator is teleported to the current team's primary spawn; on the REAL
robot the service issues `SET_POSE` at that spawn (it never drives - it only
aligns the odometry frame, and it is skipped while the robot is moving or a
mission executes). `DELETE /api/map/zones/{id}`, `POST /api/field`, a team
switch and a link switch do the same when the team start applies;
`POST /api/pose/start` does it on demand (the **Reset robot to start**
button of the Game tab).

### Game clock - `GET /api/game`, `POST /api/game/start`, `POST /api/game/reset`

```json
GET /api/game
{
  "configured": true, "duration_s": 100.0, "warning_s": 10.0,
  "running": true, "expired": false,
  "elapsed_s": 42.5, "remaining_s": 57.5, "warning": false,
  "mock_override": false, "transport": "i2c",
  "actions": [{"id": "action-1", "at_s": 95.0, "kind": "log",
                "enabled": true, "fired": false,
                "params": {"message": "release"}}]
}
```

The same `game` block is embedded in every `GET /api/state` snapshot.
`POST /api/game/start` starts the configured clock (no body, idempotent);
`POST /api/game/reset` rewinds it and clears the fired actions. Execute
starts the clock automatically.

At the limit, a **REAL** transport stops the run immediately and blocks a
new `POST /api/execute` / `POST /api/resume` until Reset; **MOCK** keeps
running and reports `expired: true`, `mock_override: true` plus the overtime
`remaining_s` (negative) for the header warning. Timed actions fire once per
clock run through the pluggable handler registry (Docs/11).

---

## 2. Manual motion (refused while executing)

| Endpoint | Body | Effect |
|---|---|---|
| `POST /api/pose` | `{x_mm, y_mm, theta_deg?}` | `SET_POSE`: the board *adopts* these coordinates (the robot is placed by hand; nothing moves) |
| `POST /api/pose/start` | - | put the robot back on OUR match start (the primary start-zone spawn of the side we play): MOCK teleports, REAL only logs the pose to adopt with Set Pose. Answers the team document; `409`-style refusal comes as `422`/`503` from the guards (refused while a mission executes) |
| `POST /api/odometry/reset` | - | zero the odometry |
| `POST /api/move/drive` | `{distance_mm, speed_mm_s?}` | drive forward (+) / backward (-) |
| `POST /api/move/turn` | `{angle_deg, speed_mm_s?}` | in-place turn |
| `POST /api/move/strafe` | `{distance_mm, speed_mm_s?}` | sideways move (dead-reckoned) |
| `POST /api/move/goto` | `{x_mm, y_mm, theta_deg?, speed_mm_s?}` | one absolute move |
| `POST /api/move/arc` | `{radius_mm, sweep_deg, speed_mm_s?}` | one circular arc: +radius / +sweep = left (CCW); the signs must agree (the board caps the cruise speed on small radii) |

All answer `{ok: true}`; `409` on robot refusal (BUSY/fault), `503` on link
errors.

---

## 3. Planning

### `POST /api/plan`
```json
{
  "elements": [
    {"kind": "erase", "name": "A"},
    {"kind": "stroke", "points": [[500,500], [1500,500]]},
    {"kind": "waypoint", "x": 1500, "y": 900, "theta_deg": -90, "combine": true},
    {"kind": "rotate", "angle_deg": 90},
    {"kind": "pause", "seconds": 2},
    {"kind": "spawn", "name": "A",
     "obstacle": {"kind": "rect", "x": 1800, "y": 900, "w": 200, "h": 200}}
  ],
  "goal": {"x": 1800, "y": 900, "theta_deg": 90},
  "obstacles": [
    {"kind": "rect", "type": "permanent", "x": 1000, "y": 600, "w": 200, "h": 200},
    {"kind": "circle", "type": "permanent", "x": 2400, "y": 400, "r": 150},
    {"kind": "polygon", "type": "game_element", "name": "A",
     "points": [[1000, 1000], [1400, 1000], [1000, 1400]]}
  ],
  "options": {"speed_mm_s": 900, "avoid_obstacles": true},
  "deadman_ms": 1500
}
```
* `elements` - the ordered mission (see [03 section 1](03-planner-algorithms.md));
  legacy shorthand: `"strokes": [[[x,y],...], ...]`.
  `{"kind":"erase","name":"A"}` removes the named game element from the
  routing grid for the legs after it; `{"kind":"spawn","name":"A",
  "obstacle":{...}}` places it back (a different position/size/shape is
  allowed - the geometry travels with the action). Both are idempotent, so a
  strategy replays from any current map state. A permanent table fixture can
  never be the target of either action.
* `waypoint.theta_deg` - the arrival angle the robot has to HOLD on the point
  (performed before any following `pause`); `waypoint.combine` - plan the
  shortest legal arrival ARC (intermediate point + exact exit tangent)
  instead of turning in place ([03 section 6](03-planner-algorithms.md)).
  A waypoint is a **stop point** with its own clearance
  (`waypoint_margin_mm`, default 0 = half the robot body side): it is
  projected out of the field border and every obstacle's waypoint clearance
  before planning. A point inside the routing margin is reached by a straight
  final leg; an arrival angle is performed at a turn-safe stand-off before
  that leg when turning on the point itself would touch.
* `obstacles` - console-drawn obstacles; the **camera snapshot is merged in by
  the service**, it does not need to be repeated here. Three shapes:
  `{"kind":"rect","x","y","w","h"}` (corner + size), `{"kind":"circle","x","y","r"}`
  (centre + radius) and `{"kind":"polygon","points":[[x,y], …]}` - a polygon
  shape of **at least 3 vertices**, closed automatically (the last point joins
  the first); its points are clamped to the table and the whole obstacle is
  dropped when fewer than 3 usable vertices remain. All three are honoured by
  the routing grid, the stop clearances, the rotated-body check and the
  mid-run obstacle reactions.
  `type` is `permanent` (default, legacy documents included) for fixed table
  fixtures or `game_element` for a movable piece; a game element **requires a
  unique `name`** (letters/digits/`.`/`_`/`-`/spaces, max 32 chars) because
  strategy actions reference it. Permanents are anonymous.
  Permanent fixtures are **locked while a mission executes**: the fixtures of
  the plan that started the run must still be present and unchanged in every
  mid-run re-plan. New anonymous obstacles in a re-plan request are treated
  like camera reports (a sensor may see something new); game elements change
  only through `erase` / `spawn` actions.
* `options` - any `PlanOptions` field ([03 section 1](03-planner-algorithms.md)).
* `deadman_ms` - used for the (re)started execution when the robot is running.

**Behaviour while a mission executes** - the request is a *dynamic re-plan*:
the remainder is candidate-planned on the new obstacle map and compared with
the route the robot is following. **If it is not affected the robot is not
stopped at all** and the current plan is returned with `"unchanged": true`.
If it is affected, the robot stops and the remainder is re-planned from its
position - normally **continuing on the current route** (detours included);
if the *elements* changed (an edited mission) the new drawing is sliced at the
robot's position instead. A paused run restarts paused.

Response - the plan document:

```json
{
  "primitives": [ {"type": "line", "a": [...], "b": [...]},
                  {"type": "arc", "center": [...], "r": 350.0, "sweep_deg": 84.2} ],
  "reference": [[x, y], ...],        // fitted/rounded reference geometry
  "path": [[x, y], ...],             // the final path (preview polyline)
  "segments": [ {"kind":"goto","x":...,"y":...,"th":...,"v":900},
                {"kind":"turn","angle_deg":90.0,"v":150},
                {"kind":"pause","seconds":2.0},
                {"kind":"game_element","action":"erase","name":"A"},
                {"kind":"game_element","action":"spawn","name":"A",
                 "obstacle":{"kind":"rect","x":1800,"y":900,"w":200,"h":200}} ],
  "stats": {"length_mm": 4912.3, "est_s": 27.1, "n_segments": 162,
            "n_gotos": 140, "n_arcs": 19, "n_actions": 3, "n_primitives": 6},
  "warnings": ["leg 1 re-routed to stay clear (obstacle or field border)"],
  "resolved_waypoints": [[1498.2, 901.5]]   // effective waypoint positions
}
```
`resolved_waypoints` has one `[x, y]` per waypoint element, in element order:
a waypoint that stood inside the field border or an obstacle's **waypoint
clearance** (or whose rotated body did not fit at the arrival heading) is
projected to the closest legal point, and these are the points the route
really visits (the console moves its markers there and the mission cursor
uses them).
`422 {"detail": "no usable stroke - draw a path first"}` when nothing can be
planned; `503` when no odometry is available yet.

An `arc` segment is executed as **one firmware ARC command**: `x0, y0` and
`x1, y1` are the entry/exit points, `radius_mm` is signed (+ = centre on the
left of the travel direction), `sweep_deg` is signed (+ = CCW, same sign as
the radius), and `th0_deg` is the entry tangent the preceding node is aligned
with; `points` only feeds the console preview. A half-driven arc contributes
its exit point (never its start) to a dynamic re-plan. In `stats`, `n_arcs`
counts arc segments while `n_actions` covers turns and pauses.

### `GET /api/plan`
Returns the last computed plan document (`404` if none). Useful for
verification and tests.

### Strategies - save / load the mission

A **strategy** is the saved mission: the drawing (`elements`), the plan
`options`, an optional `goal`, the table `obstacles` and free-form `notes`.
Storage: one JSON file per strategy in `STRATEGY_DIR` (default
`strategies/`). The console saves/loads them; the brain plays exactly these
files ([08 - Brain & strategies](08-brain-and-strategies.md) has the format
and the runner).

| Endpoint | Body | Effect |
|---|---|---|
| `GET /api/strategies` | - | `{strategies: [{name, updated_at, n_elements, kind, team, notes}, ...], dir}`, newest first |
| `GET /api/strategies/{name}` | - | full document (`elements`, `options`, `goal`, `obstacles`, `field`, `notes`, `version`) plus `authored_for`, `team`, `mirrored` and `warnings`; `404` if unknown |
| `POST /api/strategies/{name}/load` | - | make it the current mission (teleports the mock robot to its recorded start pose, `teleported` tells you), log it and return the document plus `warnings`; `404` if unknown |
| `PUT /api/strategies/{name}` | `{elements, options?, goal?, obstacles?, notes?, kind?, tree?}` | create or replace; the current field size is recorded; `422` on an invalid name or body. `kind` is `"elements"` (v1 drawing, default) or `"bt"` (v2 behavior tree - then `tree` is required; see [08](08-brain-and-strategies.md) section 8) |
| `DELETE /api/strategies/{name}` | - | remove it; `404` if unknown |

`warnings` is the load-time geometry report (issue #10): one line per
motion **leg** whose point leaves the playfield, whose body crosses the field
border at the current table size, or whose recorded start pose does not fit.
Legs are the motion elements in order (a stroke or a waypoint/goal is one leg;
actions do not count), so `"leg 2 (stroke): … outside the playfield"` is the
second drawing step. The list never rejects a document - an old strategy stays
loadable - and `POST …/load` also logs every line as a `warn` event; the brain
prints the same lines to stderr at startup.

```bash
curl -s -X PUT localhost:8090/api/strategies/main -H "Content-Type: application/json" \
     -d '{"elements":[{"kind":"stroke","points":[[500,500],[1500,500]]}],
          "options":{"speed_mm_s":700}, "notes":"qualification"}'
curl -s localhost:8090/api/strategies
```

---

## 4. Execution

| Endpoint | Body | Effect |
|---|---|---|
| `POST /api/execute` | `{segments?, deadman_ms?, follow_path?}` | run the given segments (or the last plan); arms the hardware deadman. `follow_path` (the plan polyline, with `STREAMING_EXEC=1`) runs the streaming follower - refused when the board cannot stream |
| `POST /api/pause` | - | stop in place and **freeze the match clock**; RESUME finishes the interrupted node (the remaining sweep/turn/distance is re-sent from the live pose) |
| `POST /api/resume` | - | continue after pause (the frozen match clock continues too) |
| `POST /api/mission/redo` | - | put the robot back at the pose the last run started from (the console's **Redo**) and **rewind the match clock**. While a run is **paused** it first ends that run, so Execute replays the whole path; while a run is **running** it answers 409 (pause first) |
| `POST /api/mission/rewind` | - | REMOVED - REDO now returns to the start of the sequence |
| `POST /api/stop` | - | abort the run (robot holds position) |
| `POST /api/estop` | - | latched E-STOP (clear faults afterwards) |
| `POST /api/release_motors` | - | release the stepper drivers (idle only; **no** E-STOP latch; next move re-energizes). `409` while a run executes, a fault is latched, or the board reports a moving state |
| `POST /api/clear_faults` | - | `CLEAR_FAULTS` |

`POST /api/execute` answers `{ok: true, n_segments: N, streaming: bool}`;
`409` when an execution is already running, there is no plan, or a requested
stream is REFUSED (`STREAMING_EXEC` off or firmware below 3.4 - never a
silent fallback). With `STREAMING_EXEC=1` and a `follow_path`, the run
drives the polyline with continuous velocity frames; the `exec` events then
carry `mode: "follow"` plus `remaining_mm`/`path_mm` for the streaming
progress line. Progress arrives as `exec` events on `/ws` (see section 6).

---

## 5. Board configuration, display, trace

| Endpoint | Body | Effect |
|---|---|---|
| `POST /api/config/limits` | `{max_speed_mm_s, accel_mm_s2, decel_mm_s2}` | board motion limits (max speed 50..900 mm/s, ramps 50..2000 mm/s2); written into the settings store - `SAVE_CONFIG` to keep across resets |
| `POST /api/config/deadman` | `{timeout_ms}` | board watchdog (0 = off) |
| `POST /api/display/text` | `{line?, offset?, text}` | OLED text (auto/manual mode) |
| `POST /api/display/mode` | `{mode}` | 0 = off, 1 = auto (show text, then logo), 2 = manual |
| `GET /api/trace` | `?after=<id>&limit=<n>` | I2C monitor entries, incremental |
| `GET /api/trace/download` | - | plain-text log as attachment |
| `POST /api/trace/clear` | - | empty the buffer |

### Robot settings (calibration)

| Endpoint | Body | Effect |
|---|---|---|
| `GET /api/config/robot` | - | full settings image from the board: `{map_version, speed_limit_mm_s, dirty, registers:[{index, key, label, unit, group, min, max, step, raw, value, read_only, bits}, ...]}` |
| `POST /api/config/robot` | `{"values": {<key>: <engineering value>, ...}}` | write registers (validated, idle only, all-or-nothing); answers `{ok, applied}`; `422` on a range or unknown-key error |
| `POST /api/config/robot/save` | - | persist to the board flash (`SAVE_CONFIG`) |
| `POST /api/config/robot/load` | - | reload from flash, discarding RAM edits (`LOAD_CONFIG`) |
| `POST /api/config/robot/defaults` | - | restore compile-time defaults in RAM, not saved (`RESET_CONFIG`) |
| `GET /api/errors` | `?after=<index>` | board error log: `{latest, faults, events:[{index, code, aux, name, severity}, ...]}` (up to 5 per call, oldest first) |

Register keys: `wheel_diameter_mm`, `stepper_microsteps`, `robot_track_mm`,
`wheelbase_mm`, `odom_wheel_circumference_mm`, `odom_ticks_per_rev`,
`odom_track_mm`, `max_speed_mm_s`, `min_speed_mm_s`, `accel_mm_s2`,
`decel_mm_s2`, `heading_gain`, `turn_gain`, `strafe_pattern`,
`steer_invert_mask`, `encoder_invert_mask` (plus read-only `map_version`).
Values are in engineering units (mm, mm/s, 1/s, bit masks); the API converts
them to the raw register scale of the firmware.

Mask registers additionally carry a `bits` list
(`[{bit, label, hint, tooltip}, ...]`) so one control exists per bit:
`steer_invert_mask` is a bit per **motor ID** (FL = M2, FR = M1, RL = M3,
RR = M0; 1 inverts that wheel's drive direction), `encoder_invert_mask` is
bit0 = left / bit1 = right odometry encoder (1 inverts its counting
direction), and `strafe_pattern` is bit0 FL, bit1 FR, bit2 RL, bit3 RR
(1 = that wheel is driven forward for a +Y strafe). Invert one wheel or
encoder by writing its bit, e.g. `{"values": {"steer_invert_mask": 4}}`
inverts FL only; `"encoder_invert_mask": 1` inverts the left encoder only.

Example:

```bash
curl -s -X POST localhost:8090/api/config/robot -H "Content-Type: application/json" \
     -d '{"values": {"wheel_diameter_mm": 60.5, "max_speed_mm_s": 400}}'
curl -s -X POST localhost:8090/api/config/robot/save
```

Trace entry:
```json
{"id": 123, "t": 1738073000123.4, "dir": "tx" | "rx" | "err",
 "seq": 42, "cmd": 1, "cmd_name": "GOTO", "status": 0, "status_name": "OK",
 "len": 8, "payload": "...", "frame": "..."}
```

### IMU / gyro (calibration, Docs/09 section 9.3)

The MPU6050 is on the robot's own I2C bus (not the board). These endpoints
apply LIVE (no restart); the response echoes the `docker/.env` line to keep
the value across restarts.

| Endpoint | Body | Effect |
|---|---|---|
| `GET /api/sensors/imu` | - | `{status, stream, reanchor, available}` - full IMU health, gyro stream frames and re-anchor state |
| `POST /api/sensors/imu/reset` | - | zero the spin-test yaw integral (`imu.integrated_deg`) |
| `POST /api/sensors/imu/stop` | - | stop the sampler and the gyro stream ON PURPOSE (bench fallback test, allowed while moving): the board returns to wheel heading within ~200 ms, never a fault |
| `POST /api/sensors/imu/start` | - | re-probe the IMU after a manual stop (idle only: the bias is re-averaged); `409` when no IMU answers |
| `POST /api/sensors/imu/scale` | `{turns_done}` or `{scale}` | spin workflow or manual value: new `GYRO_SCALE` = current x 360 x turns / integrated; answers `{previous, applied, env_line}`; `422` outside the 0.5..2.0 window or on a degenerate spin |
| `POST /api/sensors/imu/bias` | - | re-average the gyro bias while the robot stands still; answers `{bias_deg_s}` |
| `POST /api/sensors/imu/noise` | `{seconds?}` (0.5..5 s) | rest noise measurement: `{samples, mean_deg_s, sigma_deg_s, sigma_suggested, env_line}` |

### Self-tests (acceptance, readiness, drift)

One-click tests behind the console's **Acceptance & readiness** card. Motion
tests refuse while a mission executes or with latched faults (`409`), arm the
deadman while they poll and return `{ok, test, passed, reason, transport,
duration_s, thresholds, metrics}`. They also run on MOCK (simulated).

| Endpoint | Body | Effect |
|---|---|---|
| `POST /api/selftest/readiness` | - | pre-match checklist: `{passed, summary, t, checks:[{name, status: pass/warn/fail, detail}]}` - REAL link, firmware, faults, recent board log, saved settings, IMU bias/stream, the **start zones** (both teams configured) and the robot standing in its own start zone |
| `POST /api/selftest/square` | `{side_mm?, speed_mm_s?}` | GOTO square from the current pose; `metrics` carry position/heading closing errors plus the board-odometry copy (pass: 20 mm / 2 deg) |
| `POST /api/selftest/arc` | `{radius_mm?, speed_mm_s?}` | left + right quarter arcs, each judged from its own start pose; `metrics.arcs[]` with per-arc errors (pass: 20 mm / 2 deg) |
| `POST /api/selftest/chain` | `{length_mm?, radius_mm?, speed_mm_s?}` | flat-arc -> quarter-arc -> flat-arc run through the executor; `metrics.stop_count` (pass: 0 junction stops), `closing_error_mm`, `max_speed_mm_s` |
| `POST /api/selftest/rest-drift` | `{seconds?}` (3..120 s) | standing-still audit: board/fused max drift, encoder `tick_noise_counts`, gyro mean/sigma (pass: 2 mm / 0.5 deg, no motion seen) |
| `POST /api/selftest/turn-repeat` | `{count_per_dir?, angle_deg?, speed_mm_s?}` | N turns CCW then N CW, each judged from its own start; `metrics.directions[]` with fused mean/sigma/max, board copy and the per-turn scrub (pass: \|mean\| <= 3 deg, sigma <= 1.5 deg both directions) |
| `POST /api/selftest/loop-drift` | `{loops?, side_mm?, speed_mm_s?}` | closed square loops without corrections; per loop: fused and board drift vs the expected end pose, tick-implied distance and `tick_ratio` (pass: last-loop fused <= 60 mm / 4 deg) |
| `POST /api/selftest/speed-ramp` | `{distance_mm?, speed_mm_s?}` | one straight, sampled live: `peak_speed_mm_s`, `cruise_speed_mm_s`, `time_to_95pct_s`, `distance_error_mm` (pass: peak +/-15 %, distance 60 mm) |
| `POST /api/selftest/encoders` | `{distance_mm?, speed_mm_s?}` | +d, -d, +90 deg turn with raw tick reads: scale ratios, `checks{}` map (pass: every check - scale 0.7..1.3, backward opposite, turn L<0<R with 30 % magnitude match) |
| `POST /api/selftest/fault-drill` | `{kind: "estop"\|"deadman", distance_mm?, speed_mm_s?}` | safety rehearsals. `estop`: latch, verify, clear, verify. `deadman`: a slow creep while telemetry AND the gyro stream are PAUSED - any frame would refresh the board watchdog - so the board must stop itself at the 1.5 s watchdog and latch DEADMAN; MOVES the robot; clears the fault before returning |

---

## 6. WebSocket `/ws` - service -> console

One bidirectional-connectable channel; the server only **sends**, incoming
text is ignored (keep-alive). Message envelope: `{"type": ..., "data": ...}`.

| `type` | `data` | When |
|---|---|---|
| `telemetry` | the `GET /api/state` snapshot (vision included) | every 100 ms (10 Hz) |
| `exec` | `{status, index, total, message, segment?}` | per segment and on state changes |
| `log` | `{level: "info"\|"warn"\|"error", message}` | every service log line |
| `trace` | a trace entry (see section 5) | per I2C transaction |
| `vision` | camera snapshot: `{count, obstacles, ttl_ms, source, t, t_server, reason}` | on every effective camera change / expiry |
| `map` | the map document (same shape as `GET /api/map`) | after every `PUT /api/map` (keeps other consoles in sync) |

Log messages are the human-readable narration of everything the service does -
re-plans, swaps, warnings, mode switches, board operations.

---

## 7. WebSocket `/ws/vision` - camera -> service

Full protocol, semantics and examples: **[06 - Vision integration](06-vision-camera-integration.md)**.

Quick reference:

| Client sends | Server answers |
|---|---|
| *(on connect)* | `{"type":"hello","protocol":1,"peer":"vision:<host>:<port>","ttl_ms":2000.0,"hint":"..."}` |
| `{"type":"obstacles","obstacles":[...],"ttl_ms":2000,"t":<epoch ms>,"client":"camera"}` (aliases `update`, `report`) | `{"type":"ack","count":N,"changed":bool,"ttl_ms":...,"t":<echo>,"t_server":<epoch ms>}` |
| `{"type":"clear"}` | `{"type":"ack","count":0,"changed":bool}` |
| `{"type":"ping"}` | `{"type":"pong"}` |
| anything else | `{"type":"error","detail":"..."}` |

* Obstacle form: `{"kind":"rect","x","y","w","h"}` or `{"kind":"circle","x","y","r"}` (mm).
  (The camera channel takes rectangles and circles; the `polygon` shape
  exists on the console's static obstacle list, `/api/plan`.)
* The list **replaces** the snapshot; it expires after `ttl_ms` (default 2000,
  min 100) without refresh; coordinates are quantised to 10 mm.
* Sender identity: `peer` (hello) and `client` (per report, e.g. `"console"`
  for the console's mock camera role) surface as `vision.source` /
  `vision.client` in `GET /api/state` - the console yields to a non-console
  stream automatically.
* On disconnect the owner's obstacles **and ownership** are dropped (the
  console resumes reporting when a real camera leaves).
* Correlation is by ordering (one reply per message); `t`/`t_server` give you
  clock/latency information instead of ids.

---

## 8. End-to-end example (curl / Python)

```bash
# health + current state
curl -s localhost:8090/api/health
curl -s localhost:8090/api/state

# place the robot, plan a small mission, run it
curl -s -X POST localhost:8090/api/pose -H "Content-Type: application/json" \
     -d '{"x_mm":500,"y_mm":500,"theta_deg":0}'
curl -s -X POST localhost:8090/api/plan -H "Content-Type: application/json" \
     -d '{"elements":[{"kind":"stroke","points":[[500,500],[2500,500]]}],
          "options":{"speed_mm_s":900}}'
curl -s -X POST localhost:8090/api/execute -d '{}' -H "Content-Type: application/json"

# follow progress (or just open the console)
watch -n0.2 'curl -s localhost:8090/api/state | jq .exec'
```

```bash
# drop a camera obstacle mid-run (see 06 for a full client)
python - <<'PY'
import asyncio, json, websockets
async def main():
    async with websockets.connect("ws://127.0.0.1:8090/ws/vision") as ws:
        print(await ws.recv())
        await ws.send(json.dumps({"type":"obstacles",
            "obstacles":[{"kind":"rect","x":1900,"y":380,"w":200,"h":240}],
            "ttl_ms":60000, "t": 0}))
        print(await ws.recv())
asyncio.run(main())
PY
```
