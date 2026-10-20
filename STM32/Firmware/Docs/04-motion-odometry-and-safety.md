# 04 - Motion, odometry and safety reference

*Author: Mouadh Jeridi - jeridimouadh@gmail.com*

All constants live in `App/Inc/robot_configs.h` (project layer; the drivers
are project-independent and receive their mechanics at runtime). This document
explains the models and every controller so the behaviour can be predicted
and tuned without reading the code first.

## 1. Conventions and units

* Robot frame at boot: **+x = forward, +y = left, theta = CCW positive**
  (right-handed, radians internally, degrees in the protocol where noted).
* Distances mm, speeds mm/s, accelerations mm/s2.
* Odometry encoder resolution: 0.05 mm/tick; 170 mm between encoder wheels.
* Stepper: 6400 ticks/rev, wheel dia 60 mm -> 33.95 steps/mm, 188.5 mm/rev.

## 2. Step-pulse generation

Step timers run in **output-compare toggle mode** with `CCR = ARR/2`:

$$f_{step} = \frac{f_{APB1tim}}{2 \cdot (ARR+1)} \;\; \Rightarrow \;\; ARR = \frac{f_{APB1tim}}{2 f_{step}} - 1$$

The side target timers (TIM13/14) run at `2 * ARR_step`, which matches the
step rate exactly (toggle mode halves the output frequency). Their update
interrupt is armed only when the driver's open-loop target API is used; the
motion engine closes the loop on the odometry encoders instead. The firmware
converts mm/s -> ARR automatically (`STEPPERS_MJ_set_speed_mm_s`) from the
step timer kernel clock (`f_APB1tim` = 108 MHz, supplied by
`STEPPERS_MJ_set_timer_clock_hz()`) and clamps to 30..900 mm/s (the settings
store - register 8, default 800 - usually limits it further).

| Speed | Step rate | ARR (108 MHz step timer) |
|---|---|---|
| 30 mm/s | 1019 Hz | 53013 |
| 120 mm/s | 4074 Hz | 13252 |
| 300 mm/s | 10186 Hz | 5300 |
| 900 mm/s | 30558 Hz | 1766 |

## 3. Odometry model

Every 10 ms (TIM7, 100 Hz):

$$d_L = N_L \cdot \text{mm\_per\_tick},\quad d_R = N_R \cdot \text{mm\_per\_tick},\quad \text{mm\_per\_tick} = \frac{200\ \text{mm}}{4000} = 0.05\ \text{mm}$$

$$d_{c} = \frac{d_L + d_R}{2},\qquad d_\theta = \frac{d_R - d_L}{170\ \text{mm}}$$

The odometry wheel circumference (200 mm), ticks per revolution (4000) and
wheel track (170 mm) are settings registers 5/6/7 - calibrate them from the
RPi console (Docs/06 section 2.3/2.4). $d_\theta$ is positive for a
counter-clockwise rotation: during CCW motion the
right wheel rolls forward (increasing) and the left one backward. Midpoint
integration (reduces heading error vs. Euler):

$$\theta \mathrel{+}= d_\theta,\qquad x \mathrel{+}= d_c \cos(\theta + d_\theta/2),\qquad y \mathrel{+}= d_c \sin(\theta + d_\theta/2)$$

Encoder counters are 16-bit; the driver polls them at 100 Hz with wrap-safe
signed deltas and low-pass filtered velocity estimates (alpha = 0.3).

### Mecanum limitation (hardware)

There are only **two side encoders** - no lateral measurement. Forward/back
and rotation are observable; **strafe is not**. Therefore:

* `GOTO`/`DRIVE`/`TURN`/`ARC` are fully closed-loop and reliable.
* `STRAFE` is executed open-loop by time; keep it short and slow, verify
  visually, and re-zero with `SET_POSE` after using it.
* During strafe, wheel slip is invisible to odometry: the pose estimate may
  silently drift in Y. The Pi should prefer turn+drive paths and use strafe
  only for final lateral docking with a re-localization afterwards.

**Strafe +Y wheel pattern (robot moves to its left), top view:**

```
             front
       [FL v]      [FR ^]        ^ = wheel driven forward
          |           |          v = wheel driven backward
          |   robot   |
          |           |
       [RL ^]      [RR v]
             rear

  Directions come from the strafe pattern setting (register 14, default
  0b1001 = FL,RR forward); bench-validate once (Docs/06 section 2.2) and
  flip bits or swap wheel IDs if needed.
```

## 4. Speed profile (ramps)

No move starts or stops instantly anymore. The engine ramps the commanded
speed with `accel`/`decel` (defaults 600/500 mm/s2) and applies a brake
profile during DRIVE:

```
speed
  ^            ______________
  |           /              \
  |          /                \     ramp-up : accel limit
  |         /                  \    cruise  : commanded speed
  +--------+--------------------+-> braking : sqrt(2*decel*remaining) + creep floor
       accel       cruise       time
```

$$v_{des} = \min\left(v_{cruise},\ \sqrt{2 \cdot a_{dec} \cdot |remaining|}\right)$$

with a creep speed floor of 100 mm/s. Direction reversals stop the pulses,
re-apply DIR, wait 2 ms (DIR setup), then restart - protected by the engine.

**Chained segments (QUEUE_ARC).** A tangent-continuous chain can run
without intermediate stops: send the next `ARC` with `QUEUE_ARC` (0x17)
while the current segment runs. When the segment completes, the queued arc
is started in the same instant - the ramp is carried over and only
re-targets the new cruise - so the chain has ONE acceleration at its start
and ONE deceleration at its end. `STOP`/`ESTOP`/faults cancel the pending
arc, and the odometry snapshot carries a completed-segment counter so the
sender can keep exactly one arc queued ahead.

## 5. Controllers

**DRIVE** (target distance d, cruise v):
* progress `s` = signed centre travel since segment start (odometry);
* remaining `r = d - s`; done when `|r| <= 0.5 mm` or target overrun;
* heading hold: $e_h = \text{norm}(\theta_{ref} - \theta)$,
  $v_L = v - K_h e_h \cdot (track/2)$, $v_R = v + K_h e_h \cdot (track/2)$
  ($track/2$ = 72 mm by default, register 3),
  ($e_h > 0$ -> the target is to the left -> the right wheel runs faster),
  $K_h$ = 1.2 1/s (register 12), differential clamped to +/-v/2.

**TURN** (target angle, absolute heading target):
* $e = \text{norm}(\theta_{target} - \theta)$, done when `|e| <= 0.8 deg`;
* arc $= e \cdot (track/2)$, speed $v = \mathrm{clamp}(K_t |arc|,\ min\_speed,\ v_{cruise})$,
  $K_t$ = 1.5 1/s (register 13); for $e > 0$ (left turn, CCW) the wheels run at
  $v_L = -v$, $v_R = +v$ (and mirrored for $e < 0$).

**ARC** (signed radius R, signed sweep, cruise v):
* forward motion and rotation at the same time: the centre follows a circle
  of radius `|R|` while the heading changes by the sweep:
  $\omega = v_c / R$, $v_L = v_c - \omega \cdot (track/2)$,
  $v_R = v_c + \omega \cdot (track/2)$ (the sign of R selects the side);
* the speed follows the usual brake profile on the **tighter** of the two
  remaining measures - remaining arc length and remaining sweep (`x |R|`);
* the heading tracks the arc-length progress profile
  $\theta_{ref}(s) = \theta_0 + sweep \cdot s / L$ with a P+I curvature
  adaptation ($K_p = 8\,|1/R|$ per rad, $K_i = 3$ s$^{-1}$, curvature
  clamped to 1/3..3x): on the real floor the body yaws less than the wheel
  differential predicts (mecanum scrub, measured ~58 %), so a
  fixed-curvature controller never satisfied both completion windows and
  the arc orbited until the segment timeout fired;
* done when the sweep is within 0.8 deg and the arc length is done or
  overshot by at most 5 mm (fail-safe: an arc can never orbit forever);
* progress is the *unwrapped* heading change since the segment start, so
  sweeps above 180 deg work;
* at acceptance the cruise is capped so the outer wheel stays within the
  configured speed limit: $v_c \le v_{max} \cdot |R| / (|R| + 72)$.

**GOTO** - three phases: TURN to `atan2(dy, dx)` -> DRIVE distance ->
TURN to final theta (absolute, so mid-course drift is re-corrected). Distance
< 1 mm skips the middle phase.

**STRAFE** - per-wheel direction pattern (register 14), fixed cruise speed,
timed completion `t = |d| / v` + 300 ms margin, timeout 300 ms later.

### Gyro-aided control heading (firmware 3.3)

The RPi may stream its MPU6050 yaw rate while the robot moves (`SET_GYRO`,
0x18, silent frames - Docs/03). The engine keeps a **control heading** that
complementary-blends the streamed gyro with the wheel odometry (100 Hz tick):

```
fresh stream : ctrl = ctrl + rate * 0.01 s + 0.02 * wrap(odom - ctrl)
stale/absent : ctrl = ctrl + 0.20 * wrap(odom - ctrl)
```

* a **fresh** stream (<= 200 ms, sender age included) lets the gyro drive the
  fast yaw - slip can no longer lie to the controller - while the slow pull
  removes the gyro bias over seconds;
* a **stale/absent** stream integrates nothing and a faster pull collapses any
  residual offset back to the wheel odometry: a smooth hand-over with no
  heading step, no fault and no event;
* every steering read uses the control heading: DRIVE heading hold, TURN
  error and completion, ARC curvature tracking and sweep accounting, GOTO
  phases. The REPORTED pose (`QUERY_ODOMETRY`, telemetry) stays the pure
  wheel odometry;
* `SET_POSE` / `RESET_ODOMETRY` re-sync the control heading (and drop any held
  stream state), so a re-localization can never leave a stale offset;
* with no stream at all the behaviour is **identical to firmware 3.2** -
  the stream is an optional quality input, never a dependency (the RPi gates
  it on the firmware version reported by `QUERY_STATUS`; byte 18 reports
  whether the board currently sees fresh frames).

### Velocity stream mode (firmware 3.4)

The RPi may drive the robot with continuous velocity targets instead of
segments (`SET_VELOCITY`, 0x19, silent frames - Docs/03 section 4). The mode
is a pseudo-segment (`SEG_STREAM`, state `STREAM`):

* targets are ramped with the configured accel/decel; wheel mixing is
  `v_left = v - omega * track/2`, `v_right = v + omega * track/2`;
* no frame for 200 ms ramps both targets to zero and returns to `IDLE` -
  a controlled stale-stop with no fault and no event;
* the streaming controller itself runs OPEN-LOOP on the wheel speeds and is
  **not stall-supervised** (no segment deadline, no stall window): a blocked
  wheel simply stops making progress - the HOST watches telemetry (the RPi
  follower aborts on a path timeout) and can STOP at any time;
* every safety layer stays in force: STOP / E-STOP / limit switches cancel
  the stream, the deadman guards busy motion, stall/timeout supervision of
  program segments is untouched (the stream dispatches no segment deadline);
* the mode is entered only from `IDLE` and left by STOP/E-STOP/fault/stale;
  program commands during a stream answer `BUSY` and stream frames during a
  program are silently ignored - the two never mix mid-motion.

## 6. Supervision (all segments)

| Watchdog | Rule | Fault (event code) |
|---|---|---|
| Stall | progress changes < 0.5 mm within 350 ms | `STALL` (bit 0x0004) |
| Timeout | segment exceeds `2.2 * nominal + 3 s` (GOTO: whole manoeuvre) | `TIMEOUT` (bit 0x0008) |
| Deadman (optional) | no valid request during motion within the timeout | `DEADMAN` (bit 0x0010) |
| Limits | any wall switch (EXTI) | `LIMIT` (bit 0x0002), stop + drivers off |
| E-stop | command 0x02 | `ESTOP` (bit 0x0001), stop + drivers off |

Every latch is also recorded in the board event log with its context (which
segment state stalled/timed out, which switch fired): read it with
`QUERY_EVENTS` - codes listed in Docs/03 section 6.

The driver also offers an **open-loop fallback** (side counters that stop the
outputs after the commanded steps without encoder feedback). The closed-loop
engine does not arm it - completion is supervised by stall/timeout - but
`STEPPERS_MJ_left/right_go_to_ticks` can be used standalone when open-loop
motion is wanted.

On any fault: pulses stop, drivers are disabled, the fault latches, the RGB
LED turns red (blinking for ESTOP) and the state reports FAULT/ESTOP until
`CLEAR_FAULTS`.

## 7. Limit switches

Three wall switches (SW1/PB10, SW4/PA11, SW3/PA12) generate EXTI interrupts
at **priority 0** - the highest in the system. At interrupt latency the
handler stops all step pulses, disables both driver groups and records which
switch fired (`QUERY_STATUS.limits` bits); the motion task then latches
`LIMIT` and the FAULT state on its next pass (microseconds later). All engine
state has a single writer (`MOTION_task`), so a limit event that fires in the
middle of a segment update can neither be overwritten nor followed by a
restart of the outputs.

Operating procedure for the RPi:

1. On `faults & 0x0002` (LIMIT) -> stop planning, read `limits` to know where
   you hit.
2. `CLEAR_FAULTS`, back away with a DRIVE opposite the last motion.
3. Re-localize (the wall gives a known constraint) and re-plan.

## 8. Drivers enable policy

| Event | Drivers |
|---|---|
| Idle / after a completed move | **enabled** (holding torque) |
| `STOP` command | enabled |
| `RELEASE_MOTORS` command (idle only) | **disabled** (wheels free), no fault latched |
| `ESTOP`, any fault | **disabled** (wheels free) |
| Next accepted motion | re-enabled automatically |

## 9. Pose management

* `RESET_ODOMETRY` -> pose (0,0,0) and encoder counters re-zeroed.
* `SET_POSE(x, y, theta)` -> keep the motion configuration, translate coordinates
  (idle only). Standard bootstrap at start:
  1. drive the robot against a wall corner or the dock reference,
  2. `SET_POSE(known_x, known_y, known_theta)`,
  3. verify with `QUERY_ODOMETRY`.

## 10. Boot behaviour

1. GPIO/peripheral init, RGB off.
2. Event log started: the RCC reset flags are captured and cleared, so the
   boot cause (watchdog / power-on / software / other) is the first event in
   `QUERY_EVENTS`.
3. Settings store loaded: compile-time defaults, then the newest valid flash
   record; the wheel mechanics, direction inversions and motion limits are
   pushed into the stepper/encoder drivers.
4. OLED splash (logo) drawn; the RGB boot colour walk runs while the logo is
   on screen (~ 1.8 s total; the OLED part is skipped automatically if the
   display is absent). If the first SSD1306 init fails (display still powering
   up, or the RPi holds the shared I2C bus while booting), the display task
   re-runs the FULL controller init every 500 ms for the first 10 attempts,
   then falls back to the normal 10 s back-off; the logo appears as soon as
   the display answers. Colours and timings: Docs/08.
5. IWDG started, slave listen enabled, LED idle blue.
6. Ready to accept commands (protocol placeholder `NODATA` until the first
   request).

## 11. Failure matrix for the RPi

| Symptom | Likely cause | RPi reaction |
|---|---|---|
| No response at all | bus wiring, board unpowered, OLED back-off | retry; check `PING`; after N failures warn operator |
| `NODATA` status | first request not yet processed | resend |
| `BUSY` on motion cmd | previous segment still running | poll status; do not re-send in a loop |
| `RANGE` | parameter out of bounds | fix payload (see section 4 of the protocol doc); the event log names the command |
| `BAD_LEN` / `STORAGE` | short payload / flash failure | fix the frame; `QUERY_EVENTS` carries the reason (BAD_LEN, FLASH_ERROR) |
| `FAULT` | latched fault | read `QUERY_STATUS` faults, decide: `CLEAR_FAULTS` or manual rescue |
| State stuck in FAULT | limit still pressed / no clear issued | `CLEAR_FAULTS` then verify state IDLE |
| Odometry jumps | encoder glitch or slip | validate with `QUERY_TICKS`; if implausible, `SET_POSE` from a landmark |
