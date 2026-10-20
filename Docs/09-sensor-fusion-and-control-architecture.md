# 09 - Sensor fusion and control architecture (ArUco camera + MPU6050)

*Author: Mouadh Jeridi <jeridimouadh@gmail.com>*

*Status: implemented (2026-10-05). RPi side: shared session, camera-pose
fusion, MPU6050 sampler, silent gyro stream gated on firmware 3.3,
idle-only re-anchor. Firmware side: the SET_GYRO stream and the gyro-aided
control heading are merged in source; the IAR build and bench validation on
the real board are still pending (checklist: `STM_docs/06` section 8).
Everything stays optional: with no IMU and no camera the system behaves
exactly as before.*

*Date: 2026-10-05*

## 1. Why this document exists

The robot gets two new external sensors that make it substantially more
precise, responsive and smart:

* an **MPU6050 IMU** mounted at the centre of the robot (gyroscope: yaw rate),
* a **global-view camera system** (a second Raspberry Pi 4) that tracks an
  **ArUco tag on our robot and on the opponent robot** and reports both poses
  in the field frame, periodically, with a timestamp.

This document freezes the architecture before implementation:

* what each hardware element is and where it is mounted,
* who computes what (STM32 servo layer, RPi fusion/decision layer, camera
  perception layer),
* the sensor-fusion design and correction policy,
* the **camera system interface contract** (this is the reference for the
  camera team),
* the **degraded modes**: the system must work with no IMU, with no or
  unreliable camera input, and in the UI mock mode with neither; nothing may
  require the new sensors to boot, plan, execute or stop the robot.

Non-goals: no servo loop over the network; no fusion on the camera side; no
changes to the existing fail-safe semantics.

## 2. Design principles

1. **The STM32 remains the only servo layer.** All fast control (ramps,
   heading, step timing, supervision) stays on the board; the new sensors
   improve the *inputs* and the *corrections*, they never enter the fast loop.
2. **Every sensor is optional.** Wheel odometry alone is a valid operating
   state (today's behaviour). IMU and camera are upgrades, not dependencies.
3. **One world frame.** Field coordinates as used by the planner and the
   console; explicit transforms for the board frame and the camera.
4. **Trust by quality.** Every measurement is fused with a realistic sigma,
   outlier gating and a freshness rule; no sensor is trusted blindly.
5. **Degradation is automatic, visible and non-fatal.** Modes downgrade and
   recover by themselves; the console shows the active sources; faults are
   logged once per transition, never spammed.
6. **Mock mode is first-class.** The whole UI + planner + fusion stack runs
   without any hardware, and without IMU/camera code paths being mandatory.

## 3. Hardware

### 3.1 Current (unchanged)

| Element | Role | Interface |
|---|---|---|
| STM32F722 board ("carte_deplacement", firmware v3.3 source - IAR build/bench pending) | Motion engine, odometry, safety, OLED | I2C1 slave 0x0A + OLED master windows |
| 4 stepper motors (2 per side), 2 wheel encoders (0.05 mm/tick) | Drive + feedback | Timers / quadrature |
| Limit switches x3, RGB LED, buttons | Safety/status | EXTI / GPIO |
| Robot Raspberry Pi 4 | Console service + brain, planner, executor, fusion | I2C-1 master to the board |
| OLED SSD1306 0x3C | Messages | On the board's bus |

The wheels are mecanum hardware but the robot is **driven and planned as a
differential-drive robot**: strafe is never used (`STRAFE` stays in the
protocol but is not emitted by the planner). The mecanum rollers still scrub
during rotation; that is handled by the gyro/camera corrections (section 7).

### 3.2 Planned additions

| Element | Placement | Purpose | Interface |
|---|---|---|---|
| MPU6050 | **Centre of the robot**, rigid mount, axis-aligned | Yaw rate for heading | **I2C6** on the robot RPi (GPIO22/23, `dtoverlay=i2c6`, 3.3 V, 3.3 kOhm pull-ups, address 0x68) |
| ArUco tag (ours) | On top of the robot, known height, ID "us" | Absolute pose | Camera system |
| ArUco tag (opponent) | Same setup, different ID | Opponent pose | Camera system |
| Camera system (Raspberry Pi 4 + camera above the field) | Static, full field in view | Detect both tags, report poses | WiFi client of the robot's hotspot, `/ws/vision` |

Placement rules:

* The IMU sits at the **centre of rotation**: yaw rate is measured directly,
  with no lever-arm correction, and translational accelerations do not leak
  into the gyro signal.
* The tag is ideally also centred above the rotation centre (zero offset
  between tag and body centre). If not, the tag offset is a documented
  constant handled in one place only (section 4).
* IMU mounting: rigid, sensor +x forward / +z up, away from stepper drivers
  and motor cables; light vibration damping is acceptable but the mount must
  not wobble.

## 4. Coordinate frames, units and conventions

| Frame | Definition | Notes |
|---|---|---|
| Field frame | Origin = bottom-left corner of the play field; +x right, +y up; theta CCW positive, degrees | Same frame as the planner, console and `SET_POSE`; `_mm` / `_deg` suffixes in messages |
| Board frame | Board odometry pose (0,0,0 at boot or after `RESET_ODOMETRY`) | Related to the field frame by the transform maintained through `SET_POSE` re-anchoring |
| Camera frame | Internal to the camera system | The camera system converts to the field frame before sending |
| Robot body pose | (x, y) of the body centre, theta = robot heading | The pose contract used everywhere |

* Distances mm, speeds mm/s, angles degrees (see [01] section 3).
* `theta` wraps to [-180, +180).
* The **tag-to-body-centre offset** (translation and rotation) is a calibration
  constant. Decision to confirm (section 14): the camera reports the **body
  centre** pose (camera applies the offset), or the raw tag pose and the RPi
  applies it. Only one of the two, never both.

## 5. Target architecture

```
   +---------------------------+
   |  Camera system (RPi 4)    |      ArUco tags: us + opponent
   |  global camera view       |      (pose, timestamp, confidence)
   +------------+--------------+                  |
                | WiFi (robot hotspot)            |
                v                                 |
   +------------------------------------------+  |
   |  Robot Raspberry Pi 4                    |  |
   |                                          |  |
   |   /ws/vision -----> pose store -------+  |  |
   |                                       v  |  |
   |   MPU6050 (I2C, centre) ---->  EKF: x,y,theta,v,omega ----+
   |                                (wheels + gyro + camera)   |
   |                                      |                    |
   |   planner / executor / console <-----+                    |
   |        |                                                  |
   |        | segments, SET_GYRO (later), SET_POSE re-anchor   |
   |        v                                                  |
   |   +--------------------------------------------+          |
   |   | STM32F722: segment loops + heading + step  |          |
   |   | timing + ramps + stall/timeout/limits      |          |
   |   +--------------------------------------------+          |
   +-----------------------------------------------------------+
```

Layers and authority:

| Layer | Owns | Never does |
|---|---|---|
| Camera system | Perception: tag -> pose in field frame, timestamps, confidence | Fuse, command the robot, quantize the pose to 10 mm |
| RPi (robot) | World model (EKF), opponent track, planner/executor, correction policy | Per-step servo control |
| STM32 | Actuation, in-segment control, supervision, safety | Camera/IMU fusion, world coordinates |

### 5.1 RPi software composition (single shared runtime)

The console (`../software/console/runtime/state.py`, class `AppState`) and the
brain (`../software/brain/runner.py`) build the same runtime today (link,
client, executor, localization, telemetry). Adding the sensor lifecycle (IMU
reader, gyro stream, camera-pose ingestion, corrections) twice would let the
two hosts diverge and break the mock/degraded guarantees in one host only.

Decision: extract ONE shared composition - `RobotSession` in a new
`../software/session/` package - that owns:

* link/client/executor construction and transport switching (i2c/mock),
* the `LocalizationService` + odometry observer wiring,
* sensor task lifecycle: IMU reader, gyro forward stream, camera-pose
  ingestion hook; all optional and independently failable,
* the corrections policy task (section 7.2),
* the telemetry core loop and the aggregate status (`mode`, freshness,
  counters) consumed by both hosts.

The console `AppState` and the brain wrap the session and keep only their
extras (web emit/trace, plans, teams, watcher loops vs headless logging).

Target module layout:

```
software/
  robot/            STM32 link/protocol/client   (add CMD_SET_GYRO + stream path)
  sensors/     NEW  MPU6050 driver + sampling     (own I2C bus; optional)
  session/     NEW  RobotSession: shared composition (console + brain)
  core/
    localization/   EKF + gating + latency + source health
    world/          vision store + NEW poses store
    mission/        unchanged (RobotIO port stays narrow)
    navigation/     unchanged
  console/          FastAPI/WS; AppState wraps RobotSession
  brain/            headless; wraps RobotSession
```

Rules:

* The mission `RobotIO` port (executor/orchestrator) does NOT gain
  `SET_GYRO` or corrections - those are session-level, not mission-level.
* Mock transport -> no sensor tasks at all; the session runs exactly the
  current wheel-only chain.
* Source health/freshness has ONE home (the localization service, section
  6.5); no scattered flags.

## 6. Sensor fusion on the RPi

### 6.1 Source roles

| Source | Type | Nominal rate | Primary role | Degraded role |
|---|---|---|---|---|
| Wheel encoders (board) | Relative | 100 Hz on board / 10 Hz telemetry | x, y dead reckoning; forward speed v | Always available baseline |
| MPU6050 gyro (RPi) | Relative | 100-200 Hz read | **theta rate** (in-segment heading truth) | Absent -> wheel-derived heading |
| ArUco camera (RPi 4) | **Absolute** | 10-30 Hz | **x, y** (strong); theta (slow correction only) | Absent -> relative pose only |
| Board dead-reckoned pose | Relative | 10 Hz | Weak cross-check / sanity | Not used as an absolute reference |

### 6.2 Filter state and model

* State: `[x_mm, y_mm, theta_deg, omega_deg_s]` (as built) - the forward
  speed v enters the midpoint prediction step from the wheel twist, and the
  gyro bias is estimated at rest inside the service (boot calibration plus
  slow refinement while stationary).
* Prediction: unicycle midpoint step with process noise from speed and yaw
  rate uncertainty (slip-inflated `sigma_speed_mm_s`).
* The platform is modelled as differential drive (no strafe), which makes the
  full pose observable from the encoders; the absolute sources then only have
  to bound the drift.
* Wheel track: the localizer converts the board's wheel speeds with the
  board's own odometry track (settings register 7), adopted at every connect
  and after every settings write. `ODOM_TRACK_MM` is only the fallback for an
  unreachable board, so a calibrated robot never diverges from the estimator.

### 6.3 Measurements and trust (initial values - to be measured and tuned)

| Measurement | Sigma (initial) | Notes |
|---|---|---|
| Gyro yaw rate | 0.5-1.5 deg/s | Primary theta dynamics; bias-corrected before fusion |
| Wheel twist (prediction input) | 20-40 mm/s slip-inflated | Drives the predict step; never double counted as an independent position measurement |
| Wheel-derived omega | High (does not fight the gyro) | Fused with a large sigma; mecanum scrub makes it an underestimate |
| Camera x, y | 5-15 mm, scaled by confidence | Primary absolute anchor; measure the real noise with known placements |
| Camera theta | 2-5 deg, or bias-path only | Fused as a slow absolute heading correction; planar-target flips are rejected via gyro consistency |
| Board pose | Weak, or strong with no independent source | While a gyro or camera source is fresh only its POSITION is fused (weak): the heading integrates the SAME encoders and is therefore not independent information - applying it would undo every rotation the wheels missed (hand placement, mecanum scrub, slip) and defeat the slip immunity the gyro is there for. With no independent source it stays the full strong reference, preserving the pre-fusion behaviour exactly |

Direct answer to "trust x,y more than theta": **fuse all three sources, with
asymmetric trust exactly as above** - camera x,y is the absolute anchor, the
gyro owns fast theta, camera theta only corrects slow drift.

### 6.4 Gating, latency and confidence

* **Gating** (every camera update): Mahalanobis distance against the current
  covariance (accept <= 3 sigma initially) and a **confirmation rule**: any
  gated-out or teleport-sized sample is only adopted when a consistent
  follow-up repeats it (position within 60 mm, heading within 15 deg) - one
  bad detection never moves the estimate, a sustained correction is never
  rejected forever. A theta-only outlier (planar-tag flip) is retried
  position-only against the gyro/odometry heading.
* **Latency**: the camera message carries its own timestamp; the RPi records
  `t_server - t` as the pipeline latency. Apply delayed updates at their
  timestamp (short state history buffer) or extrapolate the measurement with
  the filter's `v, omega`. At 300 mm/s every 100 ms of latency is 30 mm -
  low-speed precision phases (docking) also keep this small.
* **Confidence**: the camera may send a per-robot confidence (section 8);
  the RPi scales the measurement sigma with it (absent = conservative
  default). Rejected updates are counted and exposed in `status()`.

### 6.5 Status reporting (target contract)

`LocalizationService.status()` keeps its current keys and adds source flags,
for example: `mode` ("wheel", "wheel+gyro", "wheel+camera",
"wheel+gyro+camera"), `gyro_age_s`, `camera_age_s`, `camera_rejected`,
`camera_confidence`, plus the existing sigmas/sample counters. The console
shows the mode; the planner reads the flags for its policies (section 10.4).

## 7. Control architecture and corrections

### 7.1 What stays exactly as it is

* The segment model (`DRIVE`, `TURN`, `ARC`, `GOTO`; `STRAFE` unused),
  ramps/brake profile, heading hold, arc curvature adaptation (kept as
  fallback), stall/timeout/deadman/limit supervision, event log, protocol
  semantics, `BUSY`/fault rules, single-writer motion state.
* `SET_POSE` remains **idle-only**.

### 7.2 Corrections from the RPi to the board

| Mechanism | Rate | Purpose | Status |
|---|---|---|---|
| **Match-start pose** (`SET_POSE`) | once, right after the first connect | A board still reporting (0,0,0) has never been placed: adopt the configured team start so board, fusion and console start inside the start rectangle instead of the field corner. An already-set pose is never overwritten; `START_POSE=0` disables it | Done |
| **Gyro yaw-rate forwarding** (`SET_GYRO`, 0x18) | 50-100 Hz while moving | Slip-immune heading inside segments (turns/arcs/heading hold); board falls back to wheels if the stream is stale (> 200 ms, initial) | Phase 3 (done) |
| **Boundary re-anchor** (`SET_POSE`) | At idle boundaries | Board pose := fused pose when the difference exceeds a threshold (initial 8 mm / 0.7 deg), so every segment starts at camera-grade accuracy | Phase 4 |
| In-motion pose offset injection | - | Smooth frame correction during long moves | Future/optional |
| **Velocity streaming** (`SET_VELOCITY`, 0x19) | 20-50 Hz while following a path | Continuous path following without junction stops (Track B, Docs/10): board ramps to the targets, controlled stale-stop at 200 ms, silent latest-wins frames | Software done (pursuit + gated follower); firmware 3.4 source merged - bench pending |
| Fault drills / audits | on demand | Deadman drill rehearses the link-watchdog safe stop by pausing telemetry on purpose (docs: console guide; endpoint `/api/selftest/fault-drill`) | Done (mock-tested; robot validation pending) |

Policy rules: corrections only when the corresponding source is **fresh and
accepted**; never inject theta jumps while a segment runs; a missing sensor
only disables its correction, never the mission. The re-anchor is a low-rate
task in the shared runtime (section 5.1): it runs only when the board is idle
(never during an execution) and never from the odometry observer callback
(that callback is synchronous and its errors are swallowed by design).

### 7.3 Board-side additions (design level)

* `SET_GYRO` request: `i16 yaw_rate [0.1 deg/s]`, `u8 age [10 ms units]`,
  latest-wins, no retry needed (a lost frame is replaced by the next one).
  Stale handling is silent (wheels-only), documented in the firmware docs set
  (`STM_docs/`) when implemented.
* **Streaming semantics (decided): the command executes silently.** The
  firmware does NOT publish a response for `SET_GYRO`, and the RPi sends the
  stream write-only (no read, no retry, a fresh sequence number per frame).
  Rationale: a request/response frame costs ~5-7 ms inside the client lock
  (write + 2 ms STOP gap + read); at 50-100 Hz the stream would occupy
  35-70% of the lock and of the bus, and every frame would overwrite the
  board's "latest response" slot used by command responses, retries and
  `QUERY_STATUS` diagnostics. The silent write-only path keeps the
  command/response channel clean at ~2 ms per frame. This is the one
  firmware/RPi protocol co-design item of Phase 3.
* Stream lifecycle: owned by the shared runtime (section 5.1), active while
  the board executes motion, stopped on transport switch; frames stop by
  themselves when the IMU stops (the stream task idles on the freshness
  check), and the board falls back to wheel heading on its own.
* Note: the deadman stays a link watchdog (any received frame refreshes it,
  as today - the console telemetry polling already does this); the gyro
  stream does not change the deadman semantics.

## 8. Camera system interface contract (reference for the camera Pi 4)

### 8.1 Connection

* The camera system is a **WebSocket client** of the robot RPi:
  `ws://<robot-pi-address>:8080/ws/vision` (docker/nginx) or `:8090` (dev).
  It connects to the robot's hotspot; reconnect automatically on loss.
* On connect the service sends `hello` (protocol/hints). Messages are
  acknowledged, in order (TCP). Ownership rules are the same as for
  obstacles: the newest snapshot wins; a non-console client owns the channel
  while connected. The camera may send **obstacles and poses** on the same
  connection.

### 8.2 Messages

Obstacles: unchanged, see [06](06-vision-camera-integration.md).

New pose message (name `poses`, final name confirmed at implementation):

```json
{"type": "poses",
 "us":       {"x_mm": 1245.2, "y_mm": 803.4, "theta_deg": 87.5, "conf": 0.90},
 "opponent": {"x_mm": 2450.0, "y_mm": 1210.5, "theta_deg": -95.2, "conf": 0.80},
 "t": 1759680000123.4,
 "ttl_ms": 400,
 "client": "camera"}
```

* `us` / `opponent`: present only when that robot is **currently visible**;
  omit (do not resend stale values). A missing object means "no fresh
  measurement now".
* `x_mm`, `y_mm`, `theta_deg`: body-centre pose in the field frame (or tag
  pose + agreed offset - section 14), theta CCW positive wrapped to
  [-180, +180).
* `conf`: optional 0..1 quality (e.g. from tag pixel size / reprojection
  error). Pi treats absent as a conservative default.
* `t`: sender timestamp, epoch milliseconds (like obstacles; the Pi measures
  `t_server - t` as pipeline latency).
* `ttl_ms`: freshness lifetime of the measurements (default 400, minimum
  100); the Pi marks the source stale after the TTL.
* Ack (shape tentative): `{"type":"ack","seen":{"us":true,"opponent":false},
  "t":..., "t_server":...}`.

### 8.3 Timing, precision and semantics

* Rate: 10-30 Hz, fixed period recommended (20 Hz / 50 ms typical) while at
  least one robot is visible; stop sending when nothing is visible.
* Precision: 1 mm / 0.1 deg on the wire. **Do not reuse the obstacle 10 mm
  quantization** - that path exists to suppress obstacle jitter and would
  inject steps into the Kalman filter. Pose jitter is handled by the fusion.
* Coordinate calibration is the camera system's responsibility: same field
  origin/orientation as the console/planner; verified by placing the robot at
  known poses. Report any residual systematic error to the RPi team so the
  sigmas can reflect it.
* Timestamps must be monotonic within a connection.

### 8.4 Camera duties

* Detect both ArUco IDs; report each robot's pose independently.
* Never mix up robots (unique IDs are the guard).
* Reject unusable detections instead of sending them (better no update than a
  wrong one); when in doubt, send the pose with a low `conf`.
* Keep the pipeline latency small and measured; include `t` always.

### 8.5 What the Pi does with them

* Fuses `us` into the pose filter (section 6); the opponent pose is stored
  for the console/status and reserved for the future predictive avoidance
  (phase 5).
* Gates outliers, scales sigmas by `conf`, compensates latency.
* Uses camera freshness for correction policies (section 10).
* Keeps the existing obstacle flow completely unchanged.

## 9. IMU integration (MPU6050)

### 9.1 Placement (as mounted: sensor facing down)

Centre of the robot (rotation centre), rigid mount, axis aligned with the
robot - on this robot the module is mounted **facing down**, so its +z axis
points at the floor and the yaw-rate sign is inverted.

That fact is detected, not configured: at boot the session reads the gravity
vector once (`MPU6050.detect_orientation`, 8 samples) and uses the sensor
axis closest to the vertical as the yaw axis, with its sign - facing down
becomes `yaw_axis = "-z"`. `IMU_YAW_AXIS` can force an axis
(`x|y|z|-x|-y|-z`); the default `auto` detects it. The detection also reports
the mounting **tilt** (angle between the measured gravity and that axis) in
`GET /api/state` -> `imu.mount` for the health panel.

A degree of mounting tilt needs no correction: it scales the yaw rate by
cos(1 deg) = 0.9998 and leaks at most sin(1 deg) of the other axes' rates
(~0.02 deg/s at 1 deg/s) - below the gyro noise floor, so the tilt is
measured and reported, never compensated. What IS corrected automatically:
the **sign** (above), the **bias** (boot average) and the **scale**
(section 9.3). Because it sits at the centre, no lever-arm corrections are
needed and the yaw rate is measured directly.

### 9.2 Bus decision (as built)

**A dedicated hardware I2C line: I2C6 on GPIO22/23** (physical pins 15/16)
of the robot Raspberry Pi 4 (BCM2711). Enabled in `/boot/firmware/config.txt`:

```
# I2C1 on GPIO2/3   - STM32 board link (/dev/i2c-1, 100 kHz, slave 0x0A)
dtparam=i2c_arm=on
# I2C6 on GPIO22/23 - MPU6050 IMU (/dev/i2c-6, address 0x68, 3.3k pull-ups)
dtoverlay=i2c6,pins_22_23
```

* `deploy.ps1` writes these lines and reboots the robot once when the
  running kernel does not expose `/dev/i2c-1` or `/dev/i2c-6` yet.
  `raspi-config` -> Interface Options -> I2C only enables I2C1, never I2C6,
  and the `i2c6` overlay exists on BCM2711 only (`pins_22_23` is its
  default, stated explicitly here).
* Wiring: SDA = GPIO22 (pin 15), SCL = GPIO23 (pin 16), 3.3 V logic, with
  **external 3.3 kOhm pull-ups** on both lines. The MPU6050 breakout's own
  pull-ups (typically 4.7-10 kOhm) already work for a short single-device
  bus; the 3.3 kOhm pair keeps the edges clean on the longer robot wiring.
* Bus speed: the overlay default, 100 kHz, is used. At 100 kHz a 100 Hz
  6-byte gyro read costs well under 1 ms per sample. Add
  `baudrate=400000` to the overlay if 400 kHz is ever wanted.
* `IMU_BUS=6` and `IMU_ADDRESS=0x68` (AD0 tied low); both `/dev/i2c-1` and
  `/dev/i2c-6` are mapped into the `api`/`brain` containers.

Reasons: fault isolation from the safety-critical board link; no contention
with the board's 100 kHz slave traffic or its OLED master windows; no shared
pull-up duplication; simpler bring-up and recovery.

Same-bus alternative: electrically workable (addresses 0x68/0x69 do not
clash with 0x0A/0x3C) but **not recommended** - a sick IMU then looks like a
bus fault to the board link and eats bandwidth margin.

### 9.3 Configuration and calibration

* ODR 100-200 Hz; DLPF enabled (~20-40 Hz for stepper vibration); gyro range
  +/-1000 deg/s (or +/-500 if noise matters more than headroom).
* No DMP: plain gyro reads + the RPi filter (transparent, debuggable).
* Bias calibration at boot while stationary (driver average), then refined
  by the RPi session itself whenever the robot is provably at rest: board
  idle, wheels stopped, board pose unchanged for 5 s and the RAW yaw rate
  within +/-10 deg/s. The low-pass has a 30 s time constant, so rest noise
  averages out and a single stop is imperceptible, while a long idle
  re-centres the value (a real robot's boot bias read 1.5 deg/s off after
  4 h - the stream to the board would have carried that error through every
  turn). Any motion - a run, a hand push, a carry - freezes it; the fusion
  input AND the board stream use the same refined value. The bias lives in
  RAM and is re-learned at every boot.
* Scale calibration (once per robot, ~2 min): the IMU reports the
  bias-corrected yaw integral as `imu.integrated_deg` in `/api/state`;
  rotate the robot slowly by hand in place over N complete turns, then set
  `GYRO_SCALE = 360*N / integrated_deg` in the environment and restart the
  console. The value stays in `docker/.env.example`-style configuration;
  `1.0` is the factory trim. The residual error left after this check is
  what `GYRO_SIGMA_DEG_S` should honestly describe. The console Calibration
  tab's **IMU / gyro** block runs this workflow interactively (Reset counter
  -> hand turns -> Compute & apply scale; the new scale applies live and the
  block shows the `GYRO_SCALE` line for `docker/.env`), and adds bias
  re-averaging at rest plus a rest-noise measurement that suggests
  `GYRO_SIGMA_DEG_S` (`/api/sensors/imu/*`).
* Bring-up sanity check: rotate the robot 360 deg by hand; with the scale
  applied, `integrated_deg` must land within a few degrees of 360. Anything
  else means a loose or tilted mount, the wrong yaw axis (`imu.yaw_axis`,
  section 9.1), or a stale scale.
* Bus bring-up: after the reboot both buses exist (`ls /dev/i2c-*` shows
  `i2c-1` and `i2c-6`). A detected MPU6050 reports `imu.state: ready` with a
  growing `imu.samples` in `/api/state` (and `i2cdetect -y 6` lists `0x68`
  once `i2c-tools` is installed). `i2cdetect -y 1` must still show `0x0a`
  (the STM32 board).
* The mount orientation is detected from gravity at boot (`IMU_YAW_AXIS=auto`,
  the default): an upside-down mount becomes `-z`, any other bearing picks the
  closest axis. Force it with `IMU_YAW_AXIS=x|y|z|-x|-y|-z` when the detection
  cannot run (e.g. the robot is being handled during boot) - the last
  `imu.mount` reading in `/api/state` shows what was detected. Mounting tilt
  up to a few degrees needs no correction (section 9.1).
* Accelerometer: unused for pose; reserved for optional bump/collision
  detection (future).

### 9.4 Reading and forwarding

* Dedicated task/thread, every sample timestamped; jitter is acceptable
  because samples are rates and the fusion uses their timestamps.
* Once `SET_GYRO` exists (Phase 3): forward the bias-corrected yaw rate at
  50-100 Hz while segments run (latest-wins). Stale stream -> board falls
  back to wheel heading silently.
* The console's IMU / gyro block can rehearse the fallback without touching
  the wiring: **Stop IMU** (`POST /api/sensors/imu/stop`, allowed while
  driving) stops the sampler and the stream, and **Start IMU**
  (`POST /api/sensors/imu/start`, idle only) re-probes the sensor and
  re-averages the bias.
* Failure detection: probe at boot, stale/saturated/NaN checks; any failure
  is equivalent to "IMU not mounted".

## 10. Degraded modes (required behaviour)

### 10.1 Rules

1. No new sensor is required to boot, plan, execute or stop.
2. Fusion updates are optional; the filter consumes whatever is fresh.
3. Freshness per source; stale sources are excluded automatically.
4. Absolute corrections (re-anchor, precision policies) require a fresh
   accepted camera; otherwise they are skipped.
5. Transition logging: one warn per state change, never per sample.
6. The console shows the active mode at all times.
7. Version compatibility is one-way safe: a new RPi with older firmware
   never sends stream frames (the gate is the firmware version reported by
   `QUERY_STATUS`); an older RPi with newer firmware is unaffected. Zero
   stream frames equals the pre-fusion behaviour exactly.
8. The host never fails to start because of the link: a board absent at
   boot (no bus, no smbus2) is a degraded state - the session logs once,
   the routes that need the board answer 503 and the link is retried in
   the background (`CONNECT_RETRY_S`, 5 s) until it opens.

### 10.2 Operating matrix

| Configuration | Pose source | In-segment heading | Absolute corrections | Effect |
|---|---|---|---|---|
| IMU + camera | wheels + gyro + camera | Gyro forwarded | Camera (gated) | Best precision/responsiveness |
| Camera only | wheels + camera | Wheels | Camera | Absolute pose accurate; turns/arcs keep scrub adaptation |
| IMU only | wheels + gyro | Gyro forwarded | None | Fast heading; x,y drifts (relative only) |
| Neither | wheels (today) | Wheels | None | Exactly current behaviour |
| Camera stale/gated | as "IMU only" | unchanged | Paused until fresh | Precision features off, mission continues |
| IMU stale/absent | as "Neither/camera only" | Wheels | unchanged | Board falls back silently |

### 10.3 Mock mode (UI and brain), no IMU, no camera

* Console: `ROBOT_TRANSPORT=mock` (or the UI link switch); brain:
  `--transport mock`. `MockLink` simulates the board (same commands,
  faults, settings, event log; starts at the match pose).
* The localization service runs wheel-only from simulated telemetry; the
  EKF is identical - the camera/gyro updates simply never arrive.
* **Acceptance requirement**: full console workflow (plan, execute, manual
  moves, rehearsal, telemetry) works in mock mode with no IMU and no camera
  process running, and no feature is blocked by their absence.

### 10.4 Planner/correction policies under degradation

* Only use precision behaviours (re-anchor, low-speed fine positioning)
  when camera is fresh and accepted.
* When camera is stale, keep normal waypoint driving (current behaviour),
  log the mode, and optionally lower aggressiveness; never stall the mission.
* When camera returns, re-acquire with N-of-M confirmation before resuming
  corrections (avoid a single bad re-acquisition move).

### 10.5 Status and UI

* `LocalizationService.status()` exposes mode + freshness + rejected counts
  (section 6.5); telemetry carries them to the console; the UI shows the
  active source mix (e.g. "wheel+gyro", "wheel+gyro+camera").
* Tests must cover the four matrix rows, not only the full configuration.

## 11. Failure handling summary

| Failure | Detection | Reaction |
|---|---|---|
| Camera disconnected / silent | No message within `ttl_ms` + grace | Exclude camera; relative mode; precision off |
| Single camera outlier | Mahalanobis / bounds / speed gate | Reject, count, keep pose |
| Persistent camera disagreement | Repeated rejections / large innovations | Reject source, warn, require N-of-M re-acquisition |
| Theta flip (planar tag ambiguity) | Disagreement with gyro heading | Reject theta, keep x,y if plausible |
| WiFi latency spike | `t_server - t` grows | Latency compensation; if too large, treat as stale |
| IMU absent at boot | I2C probe fails | Normal wheels-only operation, log once |
| IMU stale/saturated/bias jump | Sample checks | Exclude gyro; board falls back to wheels |
| Board link down | Existing `LinkError` handling | Unchanged (console/brain behaviour stays) |

## 12. Delivery notes and future work

The delivery order that worked (each step kept "normal + degraded + mock"
tests green for both hosts): shared session -> camera-pose fusion -> IMU ->
silent gyro stream -> corrections. The per-piece implementation status, the
feature gates and the test coverage are in section 15; the firmware bench
validation checklist for the gyro stream is in `STM_docs/06` section 8.

Future work (optional, not required for the current feature set):

* opponent tracker + predictive avoidance (the opponent pose is already
  collected and stored);
* wheel-speed PI on the board (encoder velocity loop under the step timing);
* in-motion pose offset injection (smooth frame correction during long moves);
* velocity streaming mode (`SET_VELOCITY`) is no longer future work: the RPi
  side is implemented (Track B in Docs/10) and the firmware command is
  merged in 3.4 source (bench validation per `STM_docs/06` section 9).

## 13. Safety invariants (never violated)

* STM32 single writer of motion state; limit switches highest priority;
  E-STOP latches; deadman unchanged.
* Corrections use existing commands only (`SET_POSE` idle-only; future
  commands are additive and latest-wins).
* No servo loop over WiFi/I2C; no in-motion theta jumps.
* The gyro stream is advisory: a stale or missing stream only reverts the
  board to wheel heading; it can never block STOP/E-STOP or a fault.
* Absence or failure of IMU/camera can only remove corrections - it can
  never block STOP/E-STOP or normal driving.
* Software must remain fully functional as of today's behaviour when both
  new sensors are missing.

## 14. Open decisions to confirm

1. Camera contract: report body-centre pose (recommended) or raw tag pose
   with an RPi-side offset constant; tag size/height/IDs; theta convention
   verification.
2. Exact `poses` message name/fields and ack shape; confidence definition;
   rate and TTL (400 ms default) confirmed with the camera team.
3. Camera field-calibration ownership and the verification procedure
   (known-pose test).
4. IMU bus: **RESOLVED** - I2C6 on GPIO22/23 (`dtoverlay=i2c6,pins_22_23`),
   `IMU_BUS=6`, `/dev/i2c-6` mapped into the containers; `deploy.ps1`
   provisions it (section 9.2). RESOLVED in code: `SET_GYRO = 0x18`, silent
   semantics, firmware gate 3.3 (IAR build + bench validation still pending).
5. Initial thresholds: re-anchor (8 mm / 0.7 deg), gyro stale (200 ms),
   camera TTL (400 ms), gate (3 sigma); camera and gyro noise measured on
   the real setup before tuning.
6. Whether the console's mock camera (console-as-camera, used for obstacles
   today) should also emit `poses` for UI testing (recommended: later).
7. RESOLVED: `../software/session/` `RobotSession` shipped as phase 0 and
   both hosts wrap it.

## 15. Implementation status (2026-10-05)

| Piece | Where | Tests |
|---|---|---|
| Shared session (phase 0) | `../software/session/`; console `AppState` and `Brain` wrap it | existing suite re-based on it |
| Camera pose store + fusion (phase 1) | `../software/core/world/poses.py`, `update_camera_pose` in `../software/core/localization/service.py`, `poses` message in `../software/console/api/ws.py` | `tests/test_poses.py`, `tests/test_fusion_camera.py`, `tests/test_poses_api.py` |
| MPU6050 driver + sampler (phase 2) | `../software/sensors/mpu6050.py`; lifecycle in the session | `tests/test_imu.py` |
| Silent gyro stream (phase 3) | `../software/robot/protocol.py` / `link.py` / `client.py` + session task; firmware `SET_GYRO` (fw 3.3) | `tests/test_stream.py` (RPi side); firmware bench pending |
| Re-anchor corrections (phase 4) | `RobotSession.maybe_reanchor` (idle-only, camera-gated, first anchor = match start) | `tests/test_corrections.py` |

Feature gates (all optional, default auto): `CAMERA_POSES`, `IMU`,
`IMU_YAW_AXIS` (auto), `GYRO_STREAM`, `REANCHOR`, `START_POSE` (plus the
existing link/geometry variables). The
degraded guarantees are covered by tests: no IMU, no camera, mock transport,
firmware < 3.3, stale stream, lost IMU, stale/gated camera.

Pending validation: the firmware IAR build + the bench checklist
(`STM_docs/06` section 8); real-hardware noise numbers for the sigmas
(section 6.3) and the tuning thresholds (section 14 item 5).

## 16. References

* [01 - System overview](01-system-overview.md), [06 - Vision integration](06-vision-camera-integration.md),
  [07 - Development guide](07-development-guide.md)
* Firmware set: `STM_docs/02-firmware-architecture.md`,
  `STM_docs/03-i2c-protocol-spec.md`, `STM_docs/04-motion-odometry-and-safety.md`
* Code: `../software/core/localization/` (EKF + service),
  `../software/core/world/` (vision store),
  `../software/console/api/ws.py` (`/ws/vision`),
  `../software/robot/` (link, protocol, client),
  `../software/robot/link.py` (`MockLink`).
