# 03 - I2C protocol specification (STM32 <-> Raspberry Pi), protocol v3

*Author: Mouadh Jeridi - jeridimouadh@gmail.com*

This is the complete wire contract. Everything an RPi developer needs is in
this document; a ready-to-use Python class is at the end.

**v3 at a glance:** speeds are 16-bit mm/s (absolute ceiling 900 mm/s), the
board keeps a full error/event log (`QUERY_EVENTS`), and its calibrated
settings (geometry, wheels, encoders, limits, gains) can be read, written,
saved to flash and restored over `GET/SET/SAVE/LOAD/RESET_CONFIG`. Firmware
3.3 adds the **silent `SET_GYRO` stream** (0x18): optional gyro-aided control
heading with a smooth wheel-only fallback when the stream is absent or stale.
Firmware 3.4 adds the **silent `SET_VELOCITY` stream** (0x19): continuous
velocity targets (v, omega) for smooth path following, with a controlled
stale-stop when frames stop - see section 4. Firmware 3.5 adds
**`RELEASE_MOTORS`** (0x06): an idle-only, non-latching driver release.

## 1. Physical layer

| Item | Value |
|---|---|
| Bus | I2C1, 100 kHz standard mode, 3.3 V, PB7 = SDA, PB8 = SCL |
| Raspberry Pi role | **Master** (all transactions start here) |
| STM32 role | **Slave** address **0x0A**, plus short **master** windows for the OLED |
| Other slave | SSD1306 OLED, 0x3C (owned by the STM32, see section 8) |
| Frame sizes | request = **exactly 16 B**, response = **exactly 32 B** |
| Transfers | write .. STOP, then read .. STOP (**two separate transactions**) |

> **Repeated-START note:** do not chain write+read in a single transfer
> session (e.g. one `i2c_rdwr(write, read)` call). The slave's HAL driver
> expects a STOP to close the 16-byte receive. Use two transactions.

## 2. Frame formats

```
Request  (16 B): | A5 | seq | cmd    | len | payload[11] | CRC8 |
Response (32 B): | 5A | seq | status | len | payload[27] | CRC8 |
byte index:         0    1      2       3    4..14/4..30   15/31
```

### Request (RPi -> STM32) - 16 bytes

| Offset | Field | Size | Description |
|---|---|---|---|
| 0 | `SOF` | 1 | `0xA5` |
| 1 | `seq` | 1 | 1..255, incremented per **new** command; reused verbatim on retries |
| 2 | `cmd` | 1 | command code (see section 4) |
| 3 | `len` | 1 | payload length 0..11 (informational; fixed frame) |
| 4..14 | `payload` | 11 | little-endian fields, zero-padded |
| 15 | `crc` | 1 | CRC-8 over bytes 0..14 |

### Response (STM32 -> RPi) - 32 bytes

| Offset | Field | Size | Description |
|---|---|---|---|
| 0 | `SOF` | 1 | `0x5A` |
| 1 | `seq` | 1 | seq of the request that produced this response |
| 2 | `status` | 1 | status code (see section 5) |
| 3 | `len` | 1 | payload length 0..27 |
| 4..30 | `payload` | 27 | little-endian, command dependent |
| 31 | `crc` | 1 | CRC-8 over bytes 0..30 |

Every read returns the **latest** response. Before the first request the
board answers a placeholder (`seq=0`, `status=0x7F NODATA`).

## 3. CRC-8 and retry rules

* CRC-8/ATM: polynomial `0x07`, init `0x00`, no reflection, no final XOR.
* Test vector: `CRC8("123456789") = 0xF4`.
* **Retries:** if a write is lost or the following read is NACKed (bus busy
  with an OLED window), resend the *same* frame with the *same* `seq`.
  The STM32 remembers the last four executed sequence numbers (retry window),
  so a command can never be executed twice by a retry. Invalid frames (bad
  CRC/SOF) are answered with an error status but are **not** remembered, so a
  corrected retry with the same seq is executed normally.
* **Verify the echoed `seq`:** a read that arrives before the board executed
  the request still returns the PREVIOUS response (carrying its own `seq`).
  Accept a response only when byte 1 matches the request's `seq`; otherwise
  retry the same frame - the duplicate rule above makes the retry safe. (The
  boot placeholder carries `seq=0` and is answered with `NODATA` until the
  first request has run.)
* Only after a response with `status == OK` (or a command-specific status)
  should the Pi increment `seq` - always finish the retry of the current
  command before sending the next one.

## 4. Commands

Motion commands are **asynchronous**: the response immediately reports
acceptance (`OK` / `BUSY` / `RANGE` / `FAULT`), and the Pi then polls
`QUERY_STATUS`/`QUERY_ODOMETRY` until the robot is `IDLE` again.

| Code | Name | Direction | Payload (request) |
|---|---|---|---|
| 0x01 | `STOP` | - | controlled stop (keeps odometry, drivers stay enabled) |
| 0x02 | `ESTOP` | - | immediate stop, latched fault `ESTOP`, drivers disabled |
| 0x03 | `CLEAR_FAULTS` | -| clear all latched faults (drivers stay off until the next move) |
| 0x04 | `SET_POSE` | - | `i16 x[0.1 mm]`, `i16 y[0.1 mm]`, `i16 theta[0.1 deg]` (offsets 0/2/4) |
| 0x05 | `RESET_ODOMETRY` | - | pose := (0, 0, 0); also re-zeros encoder counters |
| 0x06 | `RELEASE_MOTORS` | - | no payload; drivers off **only while idle**, no fault latched; the next motion command re-energizes |
| 0x10 | `DRIVE` | -> | `i16 distance[mm]`, `u16 speed[mm/s]` (offsets 0/2) |
| 0x11 | `TURN` | -> | `i16 angle[0.1 deg]` (+ = CCW), `u16 speed` (offsets 0/2) |
| 0x12 | `GOTO` | -> | `i16 x[0.1 mm]`, `i16 y[0.1 mm]`, `i16 theta[0.1 deg]`, `u16 speed` (0/2/4/6) |
| 0x13 | `STRAFE` | -> | `i16 distance[mm]` (+ = left), `u16 speed` (offsets 0/2) |
| 0x14 | `SET_MOTION_LIMITS` | - | `u16 vmax[mm/s]`, `u16 accel[mm/s2]`, `u16 decel[mm/s2]` (0/2/4) |
| 0x15 | `SET_DEADMAN` | - | `u16 timeout[ms]`, 0 = off (offset 0) |
| 0x16 | `ARC` | -> | `i16 radius[mm]` (sign = arc side), `i16 sweep[0.1 deg]` (sign = rotation), `u16 speed` (offsets 0/2/4) |
| 0x17 | `QUEUE_ARC` | -> | same payload as `ARC`; while a segment runs the arc is queued and blended into it without stopping |
| 0x18 | `SET_GYRO` | - | `i16 rate [0.1 deg/s]`, `u8 age [10 ms]` - **silent**: executes without a published response and never enters the retry window |
| 0x19 | `SET_VELOCITY` | - | `i16 v [mm/s]`, `i16 omega [0.1 deg/s]` - **silent** (fw >= 3.4): latest-wins velocity stream; controlled stop when frames stop |
| 0x20 | `QUERY_ODOMETRY` | <- | - |
| 0x21 | `QUERY_STATUS` | <- | - |
| 0x22 | `QUERY_TICKS` | <- | - |
| 0x23 | `QUERY_EVENTS` | <- | `u16 after_index` (offset 0; 0 = all retained) |
| 0x30 | `DISPLAY_MODE` | - | `u8 mode`: 0 = off, 1 = auto (logo home + temporary text), 2 = manual (persistent text) |
| 0x31 | `DISPLAY_TEXT` | - | `u8 line`(0-2), `u8 offset`(0-20), up to 9 ASCII chars |
| 0x40 | `GET_CONFIG` | <- | `u8 offset`, `u8 count` (1..12) -> register values (below) |
| 0x41 | `SET_CONFIG` | - | `u8 offset`, `u8 count` (1..4), `u16 values[count]` (below) |
| 0x42 | `SAVE_CONFIG` | - | persist the RAM settings image to flash |
| 0x43 | `LOAD_CONFIG` | - | reload the settings from flash (discard RAM edits) |
| 0x44 | `RESET_CONFIG` | - | restore the compile-time defaults in RAM (not saved) |
| 0x7F | `PING` | <-> | `u32 echo` - response echoes it (also refreshes the deadman) |

Payloads shorter than a command requires are rejected with `BAD_LEN` and an
event is recorded - the board never executes on stale or zero-padded bytes.

### Command details

**STOP (0x01)** - decelerated stop of the active segment; pose retained;
drivers keep holding torque. Use it for "wait/replan", not for emergencies.

**ESTOP (0x02)** - immediate pulse stop, drivers disabled, fault `ESTOP`
latched. Requires `CLEAR_FAULTS` before the next motion command.

**RELEASE_MOTORS (0x06)** - normal, non-latching driver release for saving
battery at standstill. Accepted only when no segment is active; an active
segment answers `BUSY`, a latched fault answers `FAULT`. It stops the pulses,
disables both driver groups, keeps the pose/odometry and does **not** latch a
fault, so the next `DRIVE`/`TURN`/`GOTO`/`STRAFE`/`ARC`/stream command
re-energizes the drivers automatically. Firmware >= 3.5.

**SET_POSE (0x04)** - allowed only when idle (else `BUSY`); tells the board
its true pose after RPi-side re-localization. Range: +/-3276.7 mm, +/-327.67 deg.

**DRIVE (0x10)** - straight-line move along the current heading; internal
closed loop on the encoder odometry with a heading-hold controller.
Accepted range: `|distance| <= 3000 mm`, `speed` 5..900 mm/s (clamped to
the configured min..max limits).
Response is the acceptance; completion is signalled by state returning
`IDLE` (and the pose reflecting the move).

**TURN (0x11)** - in-place rotation, closed loop on the odometry heading,
tolerance +/-0.8 deg. `angle` in 0.1 deg units; +/-1800 = 180 deg. Use it for fine
heading alignment; DRIVE and ARC cover the path itself.

**ARC (0x16)** - circular arc: forward motion and rotation at the same
time, so the robot centre follows a circle while the heading changes.
Payload: `radius` [mm, signed], `sweep` [0.1 deg, signed], `speed` [mm/s].
Signs: **positive = CCW (arc towards the robot's left), negative = CW**;
`radius` and `sweep` must have the same sign (mismatch -> `RANGE`).
Closed loop on odometry: the heading tracks the arc-length progress with a
slip-tolerant curvature adaptation, and the move completes when the sweep is
within +-0.8 deg while the arc length is done (or overshot by the fail-safe
slack of 5 mm - an arc can never orbit forever). Accepted range:
`50 <= |radius| <= 32767 mm`, `0 < |sweep| <= 360 deg`, arc length
`<= 3000 mm`. The cruise speed is reduced automatically on small radii so
the outer wheel stays within the configured speed limits (at R = 50 mm,
about 40 % of the maximum). Use ARC to compose smooth paths
(line-arc-line) instead of stop-and-turn sequences; stall and timeout
supervision apply as for every segment.

**QUEUE_ARC (0x17)** - same payload and validation as `ARC`, but it may be
sent while another segment is running: the board stores the arc in a single
pending slot and starts it the moment the current segment completes
**without stopping the wheels and without resetting the speed ramp** - a
tangent arc chain then runs as one continuous move (single acceleration at
the chain start, single deceleration at its end). When the board is idle the
arc simply starts at once; `STOP`, `ESTOP` and faults cancel the pending
arc, and sending a second `QUEUE_ARC` while one waits answers `BUSY`.
Tangent continuity is the sender's contract (the console planner inserts an
explicit `TURN` at every real corner); the board executes the arc from the
CURRENT pose exactly like a normal `ARC`.

**SET_GYRO (0x18)** - the optional gyro forward stream (firmware >= 3.3).
The RPi sends its MPU6050 yaw rate (`i16`, units **0.1 deg/s**, CCW positive)
and the sample age (`u8`, units 10 ms) while the robot moves. The frame is
**silent**: the board applies it without publishing a response, without
touching `s_last_seq`/`s_last_status` and without entering the 4-deep retry
window - a 50-100 Hz stream can therefore never flood the response channel
nor evict command sequence numbers (idempotent retries keep working at full
rate).

* `rate` is clamped on the board (sanity, ~+/-1146 deg/s); `age` moves the
  freshness stamp backwards, so a delayed frame is already stale on arrival;
* the yaw rate feeds a **control heading** complementary-blended with the
  wheel odometry (Docs/04, section "Gyro-aided control heading"): fresh
  stream = gyro-driven heading
  with a slow odometry pull; stale/absent = smooth collapse to the wheel
  heading;
* there is **no failure mode**: an absent or interrupted stream produces no
  fault, no event and leaves the board running exactly like firmware 3.2;
* the RPi only streams when `QUERY_STATUS` reports firmware >= 3.3, so an
  older board never sees the frame (it would answer UNKNOWN_CMD).

Read the stream freshness from `QUERY_STATUS` byte 18 (1 = fresh frames
arriving; 0 = the board is using wheel heading).

**SET_VELOCITY (0x19)** - the optional velocity forward stream (firmware
>= 3.4). The RPi sends velocity TARGETS for the robot centre - `i16 v`
(mm/s, signed, forward positive) and `i16 omega` (0.1 deg/s, CCW positive) -
continuously (20-50 Hz) while it follows a path. The frame is **silent**
exactly like `SET_GYRO`: no published response, no retry-window entry, no
event, ever.

* the FIRST frame while the board is idle enters the streaming mode
  (`QUERY_ODOMETRY`/`QUERY_STATUS` state = **8 STREAM**); further frames are
  latest-wins target updates;
* the board MIXES the targets to wheel speeds (`v_left = v - omega *
  track/2`, `v_right = v + omega * track/2`) and RAMPS towards them with the
  configured accel/decel; a nonzero wheel target keeps the driver's
  per-side floor (30 mm/s), an exact (0, 0) target parks the pulses while
  staying in the mode;
* when NO frame arrives for `ROBOT_STREAM_STALE_MS` (200 ms) the board ramps
  BOTH targets to zero and returns to `IDLE` - a **controlled stale-stop,
  never a fault** (a link failure also cannot run away: the deadman
  independently guards any busy motion);
* `STOP` / `ESTOP` / limit switches / any fault cancel the stream exactly
  like a segment; `CLEAR_FAULTS` returns to `IDLE`;
* the stream is NOT stall-supervised and carries no segment deadline (it is
  open-loop on the wheel speeds): the host supervises through telemetry and
  can `STOP` at any time;
* while any OTHER segment runs (DRIVE/TURN/ARC/GOTO/STRAFE/QUEUE_ARC
  pending) the frame has NO effect - the silent path swallows the BUSY
  status by design (the stream is best-effort); send `STOP` first when
  taking over;
* the RPi gates the mode on `QUERY_STATUS` firmware >= 3.4 (older boards
  answer `UNKNOWN_CMD`, which a direct, non-silent transact still sees - useful
  for test tools).

The streaming controller is open-loop on the wheel speeds - the same
step-timing layer every segment uses; the host closes the path loop with its
fused pose (see the Python `software/core/navigation/pursuit.py` and
`software/core/mission/follower.py`).

**GOTO (0x12)** - autonomous three-phase manoeuvre executed on the STM32:
1) turn to face the target, 2) drive the straight distance (heading hold),
3) turn to the final heading. Each phase is closed-loop, stall- and
timeout-supervised. Prefer GOTO for waypoints; use DRIVE/TURN/ARC when the
Pi wants full control of each nuance (e.g. curved avoidance moves).

**STRAFE (0x13)** - mecanum lateral translation, **dead-reckoned, no
feedback** (hardware has no lateral odometry). +/- left/right only. Keep
distances short (< ~150 mm) and verify y drift visually. The wheel mapping
pattern must be bench-validated once (`Docs/06`).

**SET_MOTION_LIMITS (0x14)** - updates the live motion limits through the
settings store (same ranges as registers 8/10/11 below: vmax 50..900,
accel/decel 50..2000). Safe while a segment runs; a following `SAVE_CONFIG`
persists the new limits to flash.

**SET_DEADMAN (0x15)** - communication watchdog on the *command* channel.
When enabled and a segment is running, loss of valid requests for the
timeout triggers fault `DEADMAN` + stop. Recommended: 500-1000 ms with a
20-50 Hz polling loop.

**QUERY_EVENTS (0x23)** - the board error/event log. Request carries
after_index (0 = everything still retained); the response returns the newest
index, the current fault mask and up to 5 events with index > after_index
(codes in section 6). Poll it after `QUERY_STATUS` shows a new last-event
code, or periodically, to drain the log. The RAM ring keeps the newest 16
events; every event is also visible in the console Calibration tab.

**CONFIG commands (0x40..0x44)** - the calibrated robot settings. The board
keeps them as a packed image of 17 u16 registers (section 6, register map)
in RAM, loaded from the last flash sector at every boot and saved on
demand:

* `GET_CONFIG`: request `u8 offset`, `u8 count` (1..12); response
  `u8 offset`, `u8 count`, `u16 values[count]`. Read the full image with two
  frames (e.g. 0..11 then 12..16).
* `SET_CONFIG`: request `u8 offset`, `u8 count` (1..4), `u16 values[count]`.
  All-or-nothing: any unknown/read-only/out-of-range register rejects the
  whole frame with `RANGE` (nothing applied). **Refused with `BUSY` while a
  segment runs** - reconfiguring the wheel geometry reconfigures the drivers
  and must happen at standstill. On success the change is applied to the
  drivers and the motion limits immediately and recorded as
  `CONFIG_APPLIED`; it is RAM-only until `SAVE_CONFIG`.
* `SAVE_CONFIG`: appends the current image to the settings sector (one small
  record; the sector is erased only when full). Response `STORAGE` when the
  flash operation fails (an `FLASH_ERROR` event carries the reason).
* `LOAD_CONFIG`: replaces RAM with the newest valid flash record (missing or
  invalid record -> defaults + a `CONFIG_FALLBACK` event).
* `RESET_CONFIG`: compile-time defaults in RAM (recorded as
  `CONFIG_DEFAULTS`; use `SAVE_CONFIG` to persist).

Typical calibration flow (RPi): `GET_CONFIG` -> compute new values ->
`SET_CONFIG` frames -> verify with a test move -> `SAVE_CONFIG`.

**DISPLAY / PING** - the screen shows the Charara logo by default.
`DISPLAY_TEXT` prints up to three 21-character lines: in `auto` mode the text
is displayed for 10 s (`ROBOT_OLED_MESSAGE_HOLD_MS`) and the logo then comes
back automatically; in `manual` mode the text stays on screen. `PING` is
ideal for keep-alive and latency measurement.

## 5. Status codes

| Value | Name | Meaning |
|---|---|---|
| 0x00 | `OK` | executed / accepted |
| 0x01 | `BAD_CRC` | CRC mismatch (frame dropped, not executed) |
| 0x02 | `BAD_FRAME` | SOF wrong / framing error |
| 0x03 | `UNKNOWN_CMD` | command code not implemented |
| 0x04 | `BUSY` | a segment is active (motion commands, RELEASE_MOTORS, SET_POSE, CONFIG writes, ..) |
| 0x05 | `RANGE` | parameter out of allowed range |
| 0x06 | `FAULT` | latched fault present -> `CLEAR_FAULTS` first |
| 0x07 | `BAD_LEN` | payload shorter than the command requires |
| 0x08 | `STORAGE` | flash storage operation failed (SAVE_CONFIG) |
| 0x7F | `NODATA` | no request processed yet (placeholder) |

Every rejection (bad CRC/len, unknown command, bad parameter, busy, range,
flash failure, faults) is also recorded in the board event log with its
context byte, so the RPi can report *why*, not just *that* something failed.

## 6. Response payloads

**QUERY_ODOMETRY (0x20), 21 bytes**

| Offset | Type | Unit | Meaning |
|---|---|---|---|
| 0..3 | i32 | um | X (forward at boot) |
| 4..7 | i32 | um | Y (left at boot) |
| 8..11 | i32 | mdeg | heading, CCW positive, +/-180000 |
| 12..13 | i16 | mm/s | left encoder speed |
| 14..15 | i16 | mm/s | right encoder speed |
| 16 | u8 | - | motion state (below) |
| 17..18 | u16 | - | fault bits (below) |
| 19..20 | u16 | - | motion segments completed by the board (wraps at 65536 - paces queued arc chains) |

**States:** 0 IDLE, 1 DRIVE, 2 TURN, 3 STRAFE, 4 GOTO, 5 FAULT, 6 ESTOP,
7 ARC.

**Fault bits (latched, 16-bit mask):** 0x0001 ESTOP, 0x0002 LIMIT (wall
switch), 0x0004 STALL, 0x0008 TIMEOUT, 0x0010 DEADMAN.

**QUERY_STATUS (0x21), 19 bytes**

| Offset | Type | Meaning |
|---|---|---|
| 0..1 | u16 | FW version `major<<8 \| minor` (v3.3 -> 0x0303; 3.2 added the ARC/QUEUE_ARC controller) |
| 2..5 | u32 | uptime ms |
| 6 | u8 | motion state |
| 7..8 | u16 | fault bits |
| 9 | u8 | limit-switch bits: 0x01 SW1, 0x02 SW3, 0x04 SW4 (latched by trigger) |
| 10 | u8 | request queue depth (0..15) |
| 11 | u8 | last executed seq |
| 12 | u8 | status of the last executed request |
| 13 | u8 | last event code (below; 0 = none) |
| 14 | u8 | last event context byte |
| 15 | u8 | boot cause (a RESET_* event code) |
| 16 | u8 | settings register-map version |
| 17 | u8 | 1 when the RAM settings differ from the saved flash record |
| 18 | u8 | 1 while recent `SET_GYRO` frames arrive (firmware >= 3.3; 0 = wheel-only heading) |

**QUERY_TICKS (0x22), 8 bytes** - raw encoder accumulators:
`i32 left_ticks`, `i32 right_ticks` (0.05 mm/tick with the default odometry
wheel; use for calibration).

**QUERY_EVENTS (0x23), 5 + 4*count bytes**

| Offset | Type | Meaning |
|---|---|---|
| 0..1 | u16 | newest event index (monotonic; 0 = log empty) |
| 2..3 | u16 | current fault bits |
| 4 | u8 | event count in this frame (0..5) |
| 5+ | 4*count | per event: `u16 index`, `u8 code`, `u8 context` (oldest first) |

**Event codes (code / context meaning):**

| Code | Name | Context | Level |
|---|---|---|---|
| 0x01 | `ESTOP` | 0 | error |
| 0x02 | `LIMIT_SWITCH` | switch bit mask | error |
| 0x03 | `STALL` | motion state that stalled | error |
| 0x04 | `TIMEOUT` | motion state that timed out | error |
| 0x05 | `DEADMAN` | 0 | error |
| 0x06 | `BAD_CRC` | 0 | warn |
| 0x07 | `BAD_FRAME` | offending byte | warn |
| 0x08 | `UNKNOWN_CMD` | command code | warn |
| 0x09 | `BAD_PARAM` | command code | warn |
| 0x0A | `BUSY_REJECT` | command code | warn |
| 0x0B | `I2C_ERROR` | HAL I2C error code | warn |
| 0x0C | `FLASH_ERROR` | 1 erase, 2 program, 3 verify | error |
| 0x0D | `CONFIG_FALLBACK` | 1 magic, 2 version/count, 3 hash, 4 value range | warn |
| 0x0E | `CONFIG_SAVED` | 0 | info |
| 0x0F | `CONFIG_APPLIED` | first register index of the frame | info |
| 0x10 | `CONFIG_DEFAULTS` | 0 | info |
| 0x11 | `RESET_WATCHDOG` | 0 (boot cause) | warn |
| 0x12 | `RESET_POWER_ON` | 0 (boot cause) | info |
| 0x13 | `RESET_SOFTWARE` | 0 (boot cause) | info |
| 0x14 | `RESET_OTHER` | 0 (brown-out/other reset) | info |
| 0x15 | `BAD_LEN` | command code | warn |
| 0x16 | `I2C_RECOVER` | 1 = stuck bus was bit-banged clear, 0 = plain reset | warn |

**GET_CONFIG (0x40) response, 2 + 2*count bytes**

| Offset | Type | Meaning |
|---|---|---|
| 0 | u8 | offset (echoed) |
| 1 | u8 | count (echoed) |
| 2+ | 2*count | register values, little-endian u16 |

**Settings register map** (u16 values; `x10` means the engineering value is
raw/10, `x100` raw/100):

| # | Register | Unit | Raw range | Default |
|---|---|---|---|---|
| 0 | map version (read-only) | - | 1 | 1 |
| 1 | stepper wheel diameter | mm x10 | 100..3000 | 600 (60.0 mm) |
| 2 | stepper microsteps | steps | 1..64 | 32 |
| 3 | drive wheel track (left-right) | mm x10 | 100..4000 | 1480 (148.0 mm) |
| 4 | wheelbase (front-back) | mm x10 | 100..4000 | 1480 (148.0 mm) |
| 5 | odometry wheel circumference | mm x10 | 200..10000 | 2000 (200.0 mm) |
| 6 | odometry ticks per revolution | ticks | 100..20000 | 4000 |
| 7 | odometry wheel track | mm x10 | 100..4000 | 1700 (170.0 mm) |
| 8 | max speed | mm/s | 50..900 | 800 |
| 9 | min speed (creep) | mm/s | 20..150 | 100 |
| 10 | acceleration | mm/s2 | 50..2000 | 600 |
| 11 | deceleration | mm/s2 | 50..2000 | 500 |
| 12 | heading-hold gain | 1/s x100 | 0..500 | 120 (1.20) |
| 13 | turn-controller gain | 1/s x100 | 0..500 | 150 (1.50) |
| 14 | strafe pattern | bits FL,FR,RL,RR | 0..15 | 9 |
| 15 | drive direction inversions | bit per motor 0..3 | 0..15 | 0 |
| 16 | odometry directions | bit0 L, bit1 R | 0..3 | 0 |

Derived: stepper ticks/rev = 200 * microsteps; stepper circumference =
PI * diameter; odometry mm/tick = circumference / ticks.

## 7. Worked examples

**DRIVE +200 mm at 100 mm/s**, seq = 7:

```
req: A5 07 10 04  C8 00 64 00  00 00 00 00 00 00 00  CRC
     |  |  |  |   |_____|  |___|  \_________ zero pad
     |  |  |  len=4  i16=200   u16=100
     |  |  cmd=DRIVE
     |  seq
     SOF
resp: 5A 07 00 00 | 27 zero bytes | CRC     (status OK, accepted)
```
Then poll `QUERY_ODOMETRY` until `state == 0 (IDLE)`.

**GOTO (300 mm, -150 mm, 90 deg) at 120 mm/s**, seq = 8:

```
req: A5 08 12 08 2C 01 6A FF 84 03 78 00 00 00 00 CRC
     offsets: x=0x012C=300 (0.1 mm), y=0xFF6A=-150 (LE!), th=0x0384=900 (90.0 deg), v=0x0078=120 (u16)
```

**ARC radius -300 mm, sweep -90 deg (a right quarter circle), 120 mm/s**,
seq = 9:

```
req: A5 09 16 06 D4 FE 7C FC 78 00 00 00 00 00 00 CRC
     offsets: radius=0xFED4=-300 (LE), sweep=0xFC7C=-900 (0.1 deg), v=0x0078=120 (u16)
```

**SET_CONFIG: wheel diameter := 61.0 mm (raw 610 = 0x0262)**, seq = 10:

```
req: A5 0A 41 04 01 01 62 02 00 00 00 00 00 00 00 CRC
     offsets: register=1, count=1, value=0x0262
```

**GET_CONFIG: read registers 0..16** - two frames, seq = 11 and 12:

```
req: A5 0B 40 02 00 0C 00 00 00 00 00 00 00 00 00 CRC   (offset 0, count 12)
resp: 5A 0B 00 1A | 00 0C <12 x u16> | 00 | CRC        (26-byte payload)
req: A5 0C 40 02 0C 05 00 00 00 00 00 00 00 00 00 CRC   (offset 12, count 5)
```

**QUERY_EVENTS: drain everything**, seq = 13:

```
req: A5 0D 23 02 00 00 00 00 00 00 00 00 00 00 00 CRC
resp: 5A 0D 00 11 | 07 00 00 00 03 | <3 x {index u16, code u8, ctx u8}> | CRC
```

## 8. Bus sharing with the OLED (important)

The STM32 is master for the OLED during short windows (only when the screen
content changes - boot, `DISPLAY_TEXT`, mode change, message-hold expiry -
with no request pending, no motion segment active and after 100 ms of bus
silence; each window ~ 100 ms). During such a window the slave hardware does
not respond and the Pi will see a NACK/timeout.

The STM32 side is designed so this can never cost the Pi control of the
board:

* the master window only starts when **no request is queued** and both bus
  lines are **idle** (a stuck bus triggers a recovery instead);
* any bus error (master timeout, arbitration lost, address/timeout on the
  slave side, ...) schedules a **full I2C reset** (`DeInit` -> bus clear ->
  `Init`) followed by **slave listen mode**; recovery is throttled so the main
  loop and the watchdog keep running;
* a bus held low by a slave is freed with the standard **9-clock bit-bang +
  STOP** sequence (recorded as an `I2C_RECOVER` event, context = 1);
* on a failed OLED transfer the peripheral is **reset** before listening
  again, so the board never stays stuck in master/busy state.

RPi rules:
1. Wrap every transaction in a retry loop (5-8 attempts).
2. Retry with the **same seq** (safe by design).
3. Treat `BAD_CRC`/timeout as "retry", not as a protocol error.
4. Never assume the bus is free - the retry loop is the arbitration.

## 9. Reference Python client (copy-ready)

```python
import time
from smbus2 import SMBus, i2c_msg

ADDR = 0x0A
REQ_SOF, RSP_SOF = 0xA5, 0x5A

def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

class Robot:
    def __init__(self, bus=1):
        self.bus = SMBus(bus)
        self.seq = 0

    def _next_seq(self):
        self.seq = 1 if self.seq >= 255 else self.seq + 1
        return self.seq

    def transact(self, cmd, payload=b"", retries=8, seq=None):
        """Send one request, return (status, payload bytes). Retries reuse the seq."""
        if seq is None:
            seq = self._next_seq()
        body = bytes([REQ_SOF, seq, cmd, len(payload)]) + payload.ljust(11, b"\x00")
        frame = body + bytes([crc8(body)])
        assert len(frame) == 16
        for attempt in range(retries):
            try:
                self.bus.i2c_rdwr(i2c_msg.write(ADDR, list(frame)))
                time.sleep(0.002)                      # small gap (STOP -> START)
                rsp = i2c_msg.read(ADDR, 32)
                self.bus.i2c_rdwr(rsp)
                data = bytes(rsp)
                if data[0] == RSP_SOF and crc8(data[:31]) == data[31] and data[1] == seq:
                    return data[2], data[4:4 + data[3]]
                # stale (not executed yet) or foreign frame -> retry the same seq
            except OSError:
                pass                                    # NACKed (OLED window) -> retry
            time.sleep(0.005)
        return None, b""

    # ---- helpers (little-endian encode) ----
    def ping(self):          return self.transact(0x7F, (123).to_bytes(4, "little"))
    def stop(self):          return self.transact(0x01)
    def estop(self):         return self.transact(0x02)
    def clear_faults(self):  return self.transact(0x03)
    def release_motors(self): return self.transact(0x06)  # idle only, no latch
    def set_pose(self, x_mm, y_mm, th_deg):
        p = (int(round(x_mm * 10)).to_bytes(2, "little", signed=True)
             + int(round(y_mm * 10)).to_bytes(2, "little", signed=True)
             + int(round(th_deg * 10)).to_bytes(2, "little", signed=True))
        return self.transact(0x04, p)
    def drive(self, dist_mm, speed_mm_s):
        return self.transact(0x10, int(round(dist_mm)).to_bytes(2, "little", signed=True)
                             + int(speed_mm_s).to_bytes(2, "little"))
    def turn(self, angle_deg, speed_mm_s):
        return self.transact(0x11, int(round(angle_deg * 10)).to_bytes(2, "little", signed=True)
                             + int(speed_mm_s).to_bytes(2, "little"))
    def goto(self, x_mm, y_mm, th_deg, speed_mm_s):
        p = (int(round(x_mm * 10)).to_bytes(2, "little", signed=True)
             + int(round(y_mm * 10)).to_bytes(2, "little", signed=True)
             + int(round(th_deg * 10)).to_bytes(2, "little", signed=True)
             + int(speed_mm_s).to_bytes(2, "little"))
        return self.transact(0x12, p)
    def strafe(self, dist_mm, speed_mm_s):
        return self.transact(0x13, int(round(dist_mm)).to_bytes(2, "little", signed=True)
                             + int(speed_mm_s).to_bytes(2, "little"))
    def arc(self, radius_mm, sweep_deg, speed_mm_s):
        return self.transact(0x16, int(round(radius_mm)).to_bytes(2, "little", signed=True)
                             + int(round(sweep_deg * 10)).to_bytes(2, "little", signed=True)
                             + int(speed_mm_s).to_bytes(2, "little"))

    def odometry(self):
        st, p = self.transact(0x20)
        if st != 0 or len(p) < 19:
            return None
        return {
            "x_mm":   int.from_bytes(p[0:4],  "little", signed=True) / 1000.0,
            "y_mm":   int.from_bytes(p[4:8],  "little", signed=True) / 1000.0,
            "theta":  int.from_bytes(p[8:12], "little", signed=True) / 1000.0,  # deg
            "v_left": int.from_bytes(p[12:14], "little", signed=True),
            "v_right": int.from_bytes(p[14:16], "little", signed=True),
            "state":  p[16], "faults": int.from_bytes(p[17:19], "little"),
        }

    def status(self):
        st, p = self.transact(0x21)
        if st != 0 or len(p) < 18:
            return None
        return {
            "fw":      f"{p[0]}.{p[1]}", "uptime_ms": int.from_bytes(p[2:6], "little"),
            "state":   p[6], "faults": int.from_bytes(p[7:9], "little"),
            "limits":  p[9], "queue": p[10], "last_seq": p[11],
            "last_status": p[12], "last_event": p[13], "last_event_ctx": p[14],
            "reset_cause": p[15], "cfg_map_version": p[16], "cfg_dirty": p[17],
        }

    def events(self, after=0):
        st, p = self.transact(0x23, int(after).to_bytes(2, "little"))
        if st != 0 or len(p) < 5:
            return None
        out = []
        for slot in range(p[4]):
            base = 5 + 4 * slot
            out.append({"index": int.from_bytes(p[base:base+2], "little"),
                        "code": p[base+2], "context": p[base+3]})
        return {"latest": int.from_bytes(p[0:2], "little"),
                "faults": int.from_bytes(p[2:4], "little"), "events": out}

    def get_config(self, offset=0, count=12):
        st, p = self.transact(0x40, bytes([offset, count]))
        if st != 0:
            return None
        return [int.from_bytes(p[2+2*i:4+2*i], "little") for i in range(p[1])]

    def set_config(self, offset, values):
        p = bytes([offset, len(values)]) + b"".join(int(v).to_bytes(2, "little") for v in values)
        return self.transact(0x41, p)

    def save_config(self):   return self.transact(0x42)
    def load_config(self):   return self.transact(0x43)
    def reset_config(self):  return self.transact(0x44)

    def wait_idle(self, timeout_s=30.0, poll_s=0.02):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            st = self.status()
            if st and st["state"] == 0:
                return True
            if st and (st["state"] in (5, 6)):
                return False
            time.sleep(poll_s)
        return False
```

Example session:

```python
r = Robot()
print(r.status())                       # {'fw': '3.0', .., 'state': 0}
r.clear_faults()                        # after any latched fault
print(r.goto(300, -150, 90, 120))       # (0, b'')  - speeds now u16 mm/s
print(r.wait_idle())                    # True when IDLE again
print(r.odometry())                     # x~300, y~-150, theta~90
print(r.arc(-300, -90, 120))            # right quarter circle at 120 mm/s
print(r.wait_idle())
print(r.get_config(0, 12))              # registers 0..11 (raw u16 values)
print(r.set_config(1, [610]))           # wheel diameter := 61.0 mm
print(r.save_config())                  # persist to flash
print(r.events(0))                      # drain the error/event log
print(r.status()["cfg_dirty"])         # 0 now that it is saved
```

## 10. Versioning

* `QUERY_STATUS.fw` returns 3.0 for this protocol generation.
* v3 changes: motion speeds are **u16 mm/s** (previously u8; the absolute
  ceiling is 900 mm/s), the new `QUERY_EVENTS` log reports every failure
  with a code and context, and the calibrated settings are accessible
  through `GET/SET/SAVE/LOAD/RESET_CONFIG` (persisted in the last flash
  sector). The response payloads of `QUERY_ODOMETRY` (fault mask widened to
  u16) and `QUERY_STATUS` (fault mask u16 + event/boot/config fields) were
extended; the frame sizes and CRC rules are unchanged.
* Legacy 7-byte codes (0x00/0x01/0x02/0x03/0xFF of FW v1) are **no longer
  supported**; see `Docs/07-changelog-v2.md` for the migration table.
