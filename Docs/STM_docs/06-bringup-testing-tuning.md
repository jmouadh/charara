# 06 - Bring-up, calibration, tests and tuning

*Author: Mouadh Jeridi - jeridimouadh@gmail.com*

Do everything in order. **Wheels off the ground for the first three tests.**
Keep a hand near the power switch; `ESTOP` should always be reachable from
the Pi console.

## 1. Bring-up checklist

| # | Step | Expected result |
|---|---|---|
| 1 | Power on (wheels raised) | RGB boot colour walk, then LED blue; OLED shows the Charara logo |
| 2 | `PING` from the Pi | echo + `OK`; if not, see section 5 |
| 3 | `QUERY_STATUS` | `fw = 3.5`, state 0, faults 0, `cfg_map_version = 1` (byte 18 = 0 without a gyro stream) |
| 3b | `GET_CONFIG` (0..16) | readable register image; all values inside their ranges (defaults on a fresh board) |
| 4 | `QUERY_TICKS` | small noise-free values; pushing a wheel by hand changes them |
| 5 | `DRIVE 200 @ 100` (wheels raised) | wheels spin forward, odometry x ~ +200 mm |
| 6 | `TURN +90 @ 100` | wheels rotate opposite pairs, theta ~ +90 deg |
| 7 | `ARC -300 -90 @ 100` | wheels roll at different speeds, theta ~ -90 deg, x,y ~ (300, -300) mm |
| 8 | Enable deadman 750 ms, `DRIVE 2000 @ 100`, stop polling | robot stops after ~0.75 s, fault `DEADMAN` |
| 9 | Press each wall switch during a move | instant stop, fault `LIMIT`, LED red |
| 10 | `ESTOP` + `CLEAR_FAULTS` | stop + drivers off, then cleared |
| 10b | `RELEASE_MOTORS` while idle, then `DRIVE 50 @ 100` | release returns OK with no fault; wheels turn freely by hand; the DRIVE re-energizes and moves. Sending `RELEASE_MOTORS` during a move returns `BUSY` |
| 11 | `GOTO` square test (below) | returns near start |

## 2. Calibration procedures

### 2.1 Encoder sign / left-right check

* Push the robot **forward** by hand -> `x` must **increase**.
* Send `TURN +90` -> theta must be **positive** (CCW).

If forward push decreases x: that encoder counts backwards - set its bit in
register 16 (`encoder_invert_mask`, bit0 left / bit1 right) from the RPi
console (Calibration tab) or with `SET_CONFIG`, then `SAVE_CONFIG`. No
recompilation is needed anymore.

If TURN +90 yields theta ~ -90 even with both signs corrected: the left/right
encoder IDs are swapped - swap `ROBOT_ENCODER_ID_LEFT` /
`ROBOT_ENCODER_ID_RIGHT` in `robot_configs.h` (wiring-level, compile-time).

### 2.2 Strafe mapping (mecanum pattern)

Command `STRAFE +100 @ 100` (wheels on the ground, robot free):

| Observation | Correction |
|---|---|
| Moves left, little rotation | Correct - no action needed |
| Moves right | invert all four bits: write register 14 (`strafe_pattern`) := 15 - current value, from the console or `SET_CONFIG`; `SAVE_CONFIG` to keep |
| Rotates instead of translating | swap FL<->RL and/or FR<->RR IDs (`ROBOT_STEPPER_ID_*`, wiring-level) |
| Translates but with big drift | check wheel order/roller orientation; retest |

Because there is no lateral odometry, this test is manual/visual only.

### 2.3 Distance and speed calibration (console)

Use the **Calibration tab** of the console (or the raw `GET/SET_CONFIG`
commands):

1. Mark a 1000 mm line on the floor (or use any convenient distance).
2. Enter the command distance and speed, press **Drive test**.
3. Measure the real distance with a tape, type it into **Measured**.
4. **Fix wheel size** sets register 1 (stepper wheel diameter) to
   `old * measured / commanded`; **Fix odom wheel** sets register 5
   (odometry wheel circumference) to `old * measured / board-reported`.
5. Press **Apply**, rerun the test, then **Save to flash**.

Slip below 1 % -> nothing to do. A persistent shortfall on BOTH values
points to slip rather than geometry: reduce cruise speed or acceleration
(registers 8/10) before chasing the wheel size.

### 2.4 Turn calibration (console)

1. Enter a command angle (<= 180 deg per TURN command, 90 deg works well)
   and press **Turn test**; the robot rotates in place.
2. Measure the real angle (tape on the floor or a reference edge), type it
   into **Measured**.
3. **Fix drive track** sets register 3 (robot track) to
   `old * commanded / measured`; **Fix odom track** sets register 7
   (odometry wheel track) to `old * board-reported / measured`.
4. Press **Apply**, rerun, then **Save to flash**.

### 2.5 GOTO square test

`GOTO` (500,0,0) -> (500,500,90) -> (0,500,180) -> (0,0,270). Return position
error should be < 20 mm and heading error < 2 deg on a hard floor. Larger errors
point to slip (reduce acceleration), encoder calibration issues, or wheel
scrub during turns (increase turn tolerance only if needed). The console's
**Acceptance & readiness** card runs this test automatically and evaluates the
20 mm / 2 deg windows (odometry-based; keep the tape check as the absolute
reference).

### 2.6 Arc test (smooth curved paths)

From a marked start pose (heading 0), on the ground:

| Command | Expected end pose (approximate) |
|---|---|
| `ARC 300 +90 @ 100` | x ~ +300 mm, y ~ +300 mm, theta ~ +90 deg |
| `ARC -300 -90 @ 100` | x ~ +300 mm, y ~ -300 mm, theta ~ -90 deg |

Tolerance: < 20 mm and < 2 deg, like the square test. Then run the
line-arc-line chain `DRIVE 300` -> `ARC 300 +90 @ 100` -> `DRIVE 300`
(polling at >= 50 Hz): no in-place turn must occur, the second leg must be
straight along the new heading, and the junction pause (creep-speed stop
while the next command is submitted) must be short and smooth. Sending the
following arcs with `QUEUE_ARC` (0x17) removes the junction pause entirely:
the board blends the chain (one acceleration at its start, one deceleration
at its end). A persistent
radius error in one direction points to a wheel-calibration or slip issue;
re-run the distance calibration (section 2.3) first. The console's
**Acceptance & readiness** card runs the two quarter arcs with automatic
20 mm / 2 deg evaluation and the line-arc-line chain as the **Chain audit**
(PASS = zero junction stops).
## 3. Protocol conformance tests (Python, using the class from `Docs/03`)

```python
r = Robot()

# 1. ping / status
assert r.ping()[0] == 0
s = r.status(); assert s["fw"] == "3.5" and s["state"] == 0

# 2. duplicate-seq idempotency: send the same DRIVE seq twice
speed = (100).to_bytes(2, "little")
status, _ = r.transact(0x10, (100).to_bytes(2,'little',signed=True) + speed, seq=42)
status2, _ = r.transact(0x10, (100).to_bytes(2,'little',signed=True) + speed, seq=42)
# second call must NOT move the robot again
r.wait_idle(); odo = r.odometry(); assert abs(odo["x_mm"] - 100) < 5

# 3. CRC rejection: corrupt one payload byte
bad = bytearray(bytes([0xA5,50,0x7F,4]) + (1).to_bytes(4,'little') + b"\x00"*7)
bad[5] ^= 0xFF
bad.append(crc8(bytes(bad)))
# write it, read response -> status must be 0x01 BAD_CRC (nothing executed)

# 4. range checks (v3 speeds are u16 mm/s, ceiling 900)
assert r.drive(99999, 100)[0] == 5          # RANGE
assert r.drive(100, 950)[0] == 5            # above the absolute speed ceiling
assert r.turn(200, 100)[0] == 5             # > 180 deg
assert r.arc(20, 90, 100)[0] == 5           # radius below the 50 mm minimum
assert r.arc(300, -90, 100)[0] == 5         # radius/sweep signs disagree
assert r.arc(300, 0, 100)[0] == 5           # zero sweep

# 5. busy semantics
r.drive(500, 200); assert r.drive(100, 100)[0] == 4   # BUSY
r.wait_idle()

# 6. queries during motion are answered
r.drive(500, 200)
odo = r.odometry(); assert odo is not None and odo["state"] == 1
r.wait_idle()

# 7. settings: read, write, save, restore
values = r.get_config(0, 12)
assert len(values) == 12 and values[0] == 1          # map version 1
assert r.set_config(0, [2])[0] == 5                  # map version is read-only
before = r.get_config(1, 1)[0]
assert r.set_config(1, [before + 10])[0] == 0        # wheel diameter + 1.0 mm
assert r.get_config(1, 1)[0] == before + 10
assert r.save_config()[0] == 0
assert r.reset_config()[0] == 0
assert r.get_config(1, 1)[0] != before + 10          # defaults restored
assert r.load_config()[0] == 0
assert r.get_config(1, 1)[0] == before + 10          # saved record wins
assert r.set_config(1, [before])[0] == 0             # back to the original
assert r.save_config()[0] == 0

# 8. every rejection lands in the event log
r.drive(100, 950)                                     # rejected (RANGE)
log = r.events(0)
assert any(event["code"] == 0x09 for event in log["events"])   # BAD_PARAM
```

Note: test 3 requires building the frame manually (see helper in Docs/03).

## 4. Fault injection tests

| Fault | How to inject | Expected |
|---|---|---|
| LIMIT | press SW during DRIVE | stop, faults=0x0002, limits bit set, `CLEAR_FAULTS` needed |
| STALL | hold a wheel during DRIVE | ~350 ms later fault 0x0004, drivers off |
| TIMEOUT | set `SET_MOTION_LIMITS` with tiny accel and command a long DRIVE into an obstacle.. safer: command `DRIVE 3000` and hold the robot | fault 0x0008 eventually |
| DEADMAN | as section 1 step 8 | fault 0x0010 |
| ESTOP | 0x02 during motion | immediate stop, drivers off, fault 0x0001 |
| OLED absent | unplug OLED | no impact on motion/protocol; OLED task backs off 10 s |
| I2C glitch | briefly short SDA to GND (careful) | error callback schedules a reset + bus clear; board returns to slave listen within ms, protocol retries succeed; `I2C_ERROR` + `I2C_RECOVER` (context 1) events recorded |
| OLED absent | unplug OLED, then send `DISPLAY_TEXT` | no impact on motion/protocol; transfer fails, peripheral is reset and listening resumes immediately; OLED task backs off 10 s |
| Bus left low | hold SDA low during a transaction | next recovery bit-bangs 9 clocks + STOP (`I2C_RECOVER` context 1) and listen resumes |
| Bad frame | send a wrong CRC / short payload | `BAD_CRC`/`BAD_LEN` status, `BAD_CRC`/`BAD_LEN` event, nothing executed |
| Config fallback | corrupt the settings record (debugger) then reset | boot loads defaults, `CONFIG_FALLBACK` event with the reason in the context byte |

All of these appear in `QUERY_EVENTS` (and in the console Calibration tab)
with a code and context byte - check the log after each injection.

## 5. If `PING` fails

1. Check wiring/pull-ups, `i2cdetect -y 1` must show `0a`.
2. Check the Pi's I2C is enabled and the bus speed is 100 kHz.
3. Verify the board is listening: power-cycle, retry after 1 s.
4. Use USB-UART on USART1 (PA9/PA10, 115200) with a future log firmware, or
   the debugger, to confirm the firmware reached the main loop.
5. If `i2cdetect` shows `0a` but transactions fail: check for OLED windows
   (they last ~100 ms); the retry loop in the Python client handles it.

## 6. Tuning reference

The table lists the **compile-time defaults** in `robot_configs.h`. The live
values are the settings registers (Docs/03 section 6): tune them from the RPi
console (Calibration tab) or with `SET_CONFIG`, then `SAVE_CONFIG` to keep
them across resets. `RESET_CONFIG` returns to these defaults.

| Parameter (defaults) | Register | Default | Effect / range |
|---|---|---|---|
| max speed | 8 | 800 mm/s | cap for all segments; absolute ceiling 900 mm/s |
| min speed (creep) | 9 | 100 mm/s | lower -> more precise stop, risk of stall (range 20..150) |
| acceleration / deceleration | 10 / 11 | 600 / 500 mm/s2 | ramps; lower -> less slip (range 50..2000) |
| stepper wheel diameter | 1 | 60 mm | open-loop distance accuracy |
| stepper microsteps | 2 | 32 | steps per revolution = 200 * microsteps |
| drive wheel track | 3 | 148 mm | open-loop turn accuracy |
| odometry wheel circumference | 5 | 200 mm | reported distance scale |
| odometry ticks per rev | 6 | 4000 | encoder counts per wheel turn |
| odometry wheel track | 7 | 170 mm | reported heading scale |
| heading gain | 12 | 1.2 1/s | heading hold aggressiveness during DRIVE |
| turn gain | 13 | 1.5 1/s | turn speed vs. error (oscillation if too high) |
| strafe pattern | 14 | 9 | mecanum +Y wheel directions (bits FL,FR,RL,RR) |
| steer / encoder inversions | 15 / 16 | 0 | direction calibration masks |
| `ROBOT_ARC_MIN_RADIUS_MM` | - | 50 mm | smallest arc radius (compile-time) |
| `ROBOT_TURN_TOL_RAD` | - | 0.014 (0.8 deg) | turn accuracy (compile-time) |
| `ROBOT_DRIVE_TOL_MM` | - | 0.5 | drive accuracy (compile-time) |
| `ROBOT_STALL_WINDOW_MS` | - | 350 | stall detection latency (compile-time) |
| `ROBOT_I2C_BUS_QUIET_MS` | - | 100 | OLED back-off from the bus (compile-time) |
| `ROBOT_I2C_BUS_CLEAR_CLOCKS` | - | 9 | SCL pulses used to free a stuck bus (compile-time) |
| `ROBOT_I2C_RECOVERY_RETRY_MS` | - | 5 | minimum spacing between I2C recovery attempts (compile-time) |
| `ROBOT_OLED_REFRESH_MS` | - | 1000 | minimum spacing between OLED redraws (compile-time) |
| `ROBOT_OLED_MESSAGE_HOLD_MS` | - | 10000 | `DISPLAY_TEXT` display time, auto mode (compile-time) |

`SET_MOTION_LIMITS` updates registers 8/10/11 (safe while driving);
`SET_DEADMAN` stays session-only (0 = off).

## 7. Acceptance criteria (before autonomous operation)

* [ ] All section 1 bring-up steps pass.
* [ ] Encoder signs and strafe pattern validated (section 2.1, 2.2).
* [ ] Distance and turn calibration done from the console; values applied and **saved to flash** (`SAVE_CONFIG`, `cfg_dirty = 0`).
* [ ] Square test < 20 mm / 2 deg error on the operational floor.
* [ ] Arc test: quarter arcs both directions < 20 mm / 2 deg; the line-arc-line chain runs without in-place turns.
* [ ] `QUERY_EVENTS` shows only expected entries (boot cause + config saves); no STALL/TIMEOUT/FLASH errors.
* [ ] Deadman enabled in the Pi software; loss-of-poll test passes.
* [ ] Limit switches tested on all three inputs; wall procedure rehearsed.
* [ ] `PING` latency measured and retry policy tuned (5-8 attempts).
* [ ] E-stop reachable and rehearsed by the operator.
* [ ] Gyro stream validation completed (section 8) - or explicitly waived
      with the RPi streaming disabled.
* [ ] Velocity stream validation completed (section 9) - only needed before
      the first streaming-mode (`PathFollower`) field run.
* The console's **Acceptance & readiness** card automates the square, arc
  and chain evaluations, the pre-run readiness list (mode, firmware, faults,
  board log, saved settings, IMU bias/stream, start pose) and adds the
  rest-drift, turn-repeatability, loop-drift, speed/ramp and encoder audits
  plus the guided fault drills (deadman, e-stop); treat a FAIL there like a
  failed checklist item.

## 8. Gyro stream bring-up and validation (firmware 3.3)

The silent SET_GYRO stream (Docs/03 section 4) is optional in both
directions; this checklist proves the guarantees on the real board.
Prepare: RPi with the IMU mounted at the centre (Docs/09 section 9),
`IMU=1` and `GYRO_STREAM=1`.

| # | Step | Expected result |
|---|---|---|
| 1 | Flash fw 3.3; `QUERY_STATUS` with no RPi stream (console stopped) | `fw = 3.3`, byte 18 = 0; DRIVE/TURN/ARC behave exactly as before |
| 2 | Start the RPi service with a healthy IMU, then `DRIVE 1000 @ 200` | byte 18 = 1 during the move, 0 when idle; heading hold unchanged or better |
| 3 | `TURN +90`, `TURN -90` at 200 mm/s | completes within the usual tolerance; no oscillation |
| 4 | Press **Stop IMU** in the console's IMU / gyro block mid-segment (no wire work; or unplug the IMU) | no fault, no event; byte 18 -> 0 within ~200 ms; the robot continues on wheel heading with no jerk |
| 5 | While streaming, force a DRIVE retry (unread response / API trace) | the move executes exactly once (sequence idempotency intact) |
| 6 | Send short/garbage SET_GYRO frames (test tool) | no response, no event entry, no behaviour change |
| 7 | Old RPi software + new firmware | no stream frames; behaviour identical to step 1 |

Notes:

* the stream must never enter `QUERY_EVENTS` - if it does, capture the aux
  byte and treat it as a firmware bug;
* the console **IMU / gyro** block replaces the wire work: **Stop IMU**
  (`POST /api/sensors/imu/stop`) stops the sampler and the stream even
  while the robot moves, and **Start IMU** (`POST /api/sensors/imu/start`,
  robot still: the bias is re-averaged) probes it again;
* the RPi streams only while the board executes motion and the IMU is
  healthy, so an idle bus stays quiet;
* record the measured gyro noise/bias from the bench run into the RPi
  config (`GYRO_SIGMA_DEG_S`, Docs/09 section 6.3).

## 9. Velocity stream bring-up and validation (firmware 3.4)

The silent SET_VELOCITY stream (Docs/03 section 4) is optional in both
directions; this checklist proves the guarantees on the real board before
the first streaming-mode (`PathFollower`) field run. Prepare: RPi software
with the streaming backend (Track B in Docs/10); robot on blocks for steps
2-3, clear floor for the rest.

| # | Step | Expected result |
|---|---|---|
| 1 | Flash fw 3.4; QUERY_STATUS with no RPi stream | `fw = 3.4`; DRIVE/TURN/ARC/GOTO behave exactly as before |
| 2 | Test tool: one SET_VELOCITY frame v=0, omega=0 while idle (direct transact) | response OK, state 8 (STREAM), no motion; STOP returns IDLE |
| 3 | Stream v=100 for ~1 s at 20 Hz, then STOP | robot creeps straight with a ramp up/down; no events |
| 4 | Stream v=200, omega=30 deg/s for ~2 s | robot follows a left curve; signs correct (+ = CCW) |
| 5 | Stop sending frames while v=150 | robot ramps to rest (~200 ms + decel), state IDLE, NO fault, NO event |
| 6 | While streaming, press E-STOP then CLEAR_FAULTS | motion stops instantly, ESTOP latched, clear returns IDLE |
| 7 | While a DRIVE program runs, send stream frames | frames have no effect; the program completes unchanged |
| 8 | Arm deadman 1500 ms; stream v=150 with a frame gap > 1.5 s | at this speed the 200 ms STALE-STOP ramps to rest and returns IDLE before the 1500 ms watchdog (no DEADMAN fault - the stale-stop IS the safe fallback); at much higher stream speeds the watchdog can fire during the longer ramp-down instead (also a safe stop; CLEAR_FAULTS afterwards) |
| 9 | Stream v=-100, omega=0 | robot reverses; board odometry moves opposite its heading |
| 10 | Old RPi software + new firmware | no stream frames; behaviour identical to step 1 |

Notes:

* stream frames must never enter `QUERY_EVENTS` - if they do, capture the
  aux byte and treat it as a firmware bug;
* the per-side floor is 30 mm/s: at very small omega with v=0 the wheels
  turn slightly faster than commanded - keep v >= 100 for smooth arcs;
* the console's speed/ramp and loop-drift audits give the closed-loop
  numbers once the RPi follower drives the stream.
