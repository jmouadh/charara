# 07 - Changelog (v2.0 and later releases)

*Author: Mouadh Jeridi - jeridimouadh@gmail.com*

This release reworks the firmware after a full review, adds a new robust
protocol, safety supervision and documentation. Below: every finding from the
review -> what was done.

## 0. v3.5 - motor release and reliable OLED boot (2026-10-19)

* **`RELEASE_MOTORS` (0x06)** - idle-only stepper-driver release. It stops
  the pulses, disables both driver groups and keeps the pose/odometry, but
  does **not** latch E-STOP; the next motion command re-energizes the drivers
  automatically. A segment still running answers `BUSY`, so the console/API
  can never drop the drivers while the robot moves. Firmware reports 3.5.
* **OLED boot retry** - the startup splash now retries the complete SSD1306
  initialization (not just the drawing) while the display powers up or the
  Raspberry Pi still holds the shared I2C bus: every 500 ms for the first
  10 attempts, then the normal 10 s back-off. `ssd1306_Init` clears the
  sticky error at entry and bails out on the first failed transfer, so a
  retry can genuinely recover an absent/late display without blocking boot.
* **Console** - new Tools > Release action (immediate or sequence step), a
  non-latching Safety > Release motors button, and an idle-only
  `POST /api/release_motors` route. Strategy elements accept `{"kind":
  "release"}` between moves and pauses.

## 1. Priority fixes (P0)

| # | Legacy problem | v2.0 solution |
|---|---|---|
| 1 | Odometry reply overflowed an int16 after 32.7 mm (`x*1000`) | New 32-byte response frames carry `i32` um / `i32` mdeg |
| 2 | Encoder-tick reply truncated `int32` -> `uint16` (wrapped each 3.28 m) | `QUERY_TICKS` returns two full `i32` accumulators |
| 3 | E-stop could not preempt a blocking `0x03` movement (`while(busy)` + queued commands) | Fully non-blocking engine; STOP/ESTOP are decoded in the I2C ISR and honoured on the next loop pass (us), aborting the active segment |
| 4 | `0x01/0x02` distance parsing used a different byte order than x/y/theta | Clean little-endian accessors; payloads now uniform and documented |
| 5 | Queue race (ISR add vs. main O(n) consume) | SPSC ring buffer (producer = ISR, consumer = main loop), no shifting, no lost frames |
| 6 | OLED full refresh on every command with listen disabled -> Pi NACK storms | Throttled OLED task (1 Hz, idle + bus-quiet only, finite timeouts, 10 s back-off); retries are safe due to seq idempotency (documented) |
| 7 | `HAL_I2C_ErrorCallback` did `DeInit/Init` in the ISR; a failure left the slave permanently silent | Error callback only sets a flag; the main loop retries recovery until success, then re-arms listen |
| 8 | Out-of-bounds `list[stepper+1]` read in group init | Fixed to `list[stepper]` |
| 9 | No watchdog; `Error_Handler` = silent IRQ-off hang | IWDG ~ 500 ms + safe-state handler (motors stopped/released); recovery = reset |
| 10 | Fake button debounce; limit switches wired but dead (EXTI15_10 not enabled) | Limit switches fully implemented: EXTI15_10 at highest priority, hard stop + latched fault + status reporting |

## 2. Architecture and performance (P1)

| # | Legacy problem | v2.0 solution |
|---|---|---|
| 11 | Dead step-timer update interrupts (up to ~25 kHz) + encoder CC interrupts (per edge) | All removed: step timers run interrupt-free; encoders are polled at 100 Hz with wrap-safe deltas |
| 12 | Encoder folding raced between TIM7 and update ISRs | Single writer: only the TIM7 ISR polls/accumulates; getters return cached values |
| 13 | Instant start/stop at full rate (step loss, slip) | Trapezoidal ramps + braking profile + creep speed; direction changes with 2 ms DIR-settle |
| 14 | Open-loop movement (commanded steps only) | Closed-loop on odometry encoders for DRIVE/TURN/GOTO, with stall and timeout supervision |
| 15 | `HAL_Delay(500)` pacing and `while(busy)` blocking | Event-driven main loop; command latency ~ I2C transfer time |
| 16 | Clock 25 MHz HSE (defective quartz) hard-coded constants | HSI 16 MHz, PLL off; I2C/TIM7/PWM/ARR constants derived from `SystemCoreClock` |
| 17 | No strafing support for the mecanum base | `STRAFE` command (dead-reckoned, configurable wheel pattern, documented limitation) |
| 18 | No comms supervision | Optional deadman watchdog (`SET_DEADMAN`) + `PING` keep-alive |
| 19 | OLED could block the bus unbounded (`HAL_MAX_DELAY`) | Finite 50 ms timeouts in the SSD1306 driver + sticky error flag + early bail-out |

## 3. Protocol v2 summary

* Fixed frames (16 B request / 32 B response), CRC-8, status codes,
  sequence-numbered idempotent retries, little-endian, explicit units.
* Commands: STOP, ESTOP, CLEAR_FAULTS, SET_POSE, RESET_ODOMETRY, DRIVE,
  TURN, GOTO, STRAFE, SET_MOTION_LIMITS, SET_DEADMAN, QUERY_ODOMETRY,
  QUERY_STATUS, QUERY_TICKS, DISPLAY_MODE, DISPLAY_TEXT, PING.
* Full spec + Python client: `Docs/03-i2c-protocol-spec.md`.

### Migration table for the Raspberry Pi code

| Legacy v1 (7-byte frames) | v2.0 equivalent |
|---|---|
| `0x00` stop **+ odometry reset** | `STOP` (0x01) - use `RESET_ODOMETRY` (0x05) explicitly if wanted |
| `0x01` forward / `0x02` backward (0.1 mm, buggy parse) | `DRIVE` (0x10) with signed mm |
| `0x03` turn-drive-turn to pose (blocking) | `GOTO` (0x12), non-blocking, poll until IDLE |
| `0xFF` encoder ticks (truncated) | `QUERY_TICKS` (0x22), full i32 |
| plain read = odometry (overflowing int16) | `QUERY_ODOMETRY` (0x20), um/mdeg |
| - | new: `TURN`, `SET_POSE`, `SET_DEADMAN`, `SET_MOTION_LIMITS`, `DISPLAY_*`, `PING` |

## 4. Behaviour changes to be aware of

1. **Clock** 25 MHz -> 16 MHz (HSI). Motion/odometry quality is preserved by
   the derived constants; measured speeds are unchanged (mm/s API).
2. **Drivers stay enabled after a completed move and after `STOP`** (holding
   torque); they are disabled only by `ESTOP`, faults or limit contact. Use
   `ESTOP` if you need the wheels free.
3. **STOP no longer resets odometry** (split into `RESET_ODOMETRY`).
4. **Commands are rejected with BUSY** while a segment runs (no implicit
   queueing) - the Pi polls and serializes.
5. **Limit contact is latched**: motion stays blocked until `CLEAR_FAULTS`.
6. **OLED is now optional**: an absent/broken OLED cannot block the robot.
7. IWDG is active: a firmware crash causes a reset (~0.5 s), not a hang.

## 5. Build-system changes (IAR)

Add to the project:
* `Core/Src/motion.c`, `Core/Src/robot_protocol.c`
* `Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_iwdg.c`
  (`HAL_IWDG_MODULE_ENABLED` is now enabled in `stm32f7xx_hal_conf.h`)

Removed/renamed APIs (no other caller should exist):
* `STEPPERS_MJ_set_speed/set_left_speed/set_right_speed` (percent based)
  -> `STEPPERS_MJ_set_*_speed_mm_s`
* `ENCODERS_MJ_update_global_values_after_overflow`
  -> `ENCODERS_MJ_poll` (+ `ENCODERS_MJ_get_velocity_mm_s`)

## 6. Documentation added

`README.md` + `Docs/01..07`:
hardware reference, firmware architecture, **protocol spec with Python
client**, motion/odometry/safety reference, Raspberry Pi integration guide,
bring-up/calibration/test/tuning guide, and this changelog.

## 7. v2.2 - Layer refactor and RTOS readiness (2026-09-25)

* `Core/Src/main.c` reduced to `main()` only: `ROBOT_APP_init()` +
  `ROBOT_APP_run()`. All functions, state and pin usage moved out.
* New `App/` layer (owned application code):
  - `robot_app.c/h` - init, four task steps, bare-metal scheduler, HAL timer
    and EXTI callbacks, error handler;
  - `robot_board.c/h` - MPU, clock and peripheral bring-up under `BSP_*`
    names plus peripheral handle ownership;
  - `robot_i2c.c/h` - slave listen mode, OLED master windows, bus recovery,
    HAL I2C callbacks;
  - `robot_display.c/h` - boot splash and throttled status screen;
  - `robot_status.c/h` - RGB LED indication and button events;
  - `robot_port.h`, `robot_port_baremetal.c` - execution environment port
    (critical sections, tick, delay, idle) that is the single file to replace
    for a FreeRTOS build (full guide in Docs/02 section 9).
* `motion.c` and `robot_protocol.c` now use only the port layer
  (`ROBOT_PORT_tick_ms`, `ROBOT_PORT_critical_*`), so they run unchanged on
  bare metal or under an RTOS.
* Board/build notes: add `App/Src/*.c` to the IAR project and `App/Inc` to the
  include paths; the original generated `main.c` is archived in
  `Docs/reference/main_cubemx_reference.txt` for CubeMX re-generation
  (workflow in Docs/02 section 10).

## 8. v2.3 - Configuration ownership and driver reuse (2026-09-25)

* `ROBOT_CONFIGS.h` moved from `Drivers/` to `App/Inc/robot_configs.h`: the
  Drivers folder now contains project-independent drivers only.
* Wheel mechanics are no longer compiled into the drivers: new runtime APIs
  `STEPPERS_MJ_configure(ticks_per_rev, wheel_circumference_mm)` and
  `ENCODERS_MJ_configure(ticks_per_rev, wheel_circumference_mm)` are called
  once by `ROBOT_APP_init()` with values from `robot_configs.h`. The drivers
  keep safe defaults and reject invalid values.
* Both drivers are now reusable as-is in other projects; only the application
  passes different mechanics. Include paths and docs were updated accordingly.

## 9. v2.3.1 - Post-review fixes (2026-09-25)

Full re-review of the firmware against its purpose (4 steppers, 2 odometry
encoders, 3 wall switches, Raspberry Pi master over I2C, autonomous use).

* **Kinematics sign convention corrected** to the documented standard
  (+x forward, +y left, theta CCW positive, right-handed):
  - odometry heading increment is now `dtheta = (d_R - d_L) / track`;
  - `TURN +angle` (CCW) drives `v_L = -v`, `v_R = +v`;
  - the DRIVE heading differential is applied to the correct side.
  Previously the engine was internally consistent but mirrored (theta actually
  clockwise-positive, y to the right), so a documented "TURN +90 (CCW)"
  produced a right turn and `GOTO` headings were mirrored - an integration
  trap for the Raspberry Pi side.
* **Limit-switch ISR sanitized (safety race)**: the EXTI handler now only
  performs hardware actions (stop pulses, drivers off), records the switch bit
  and flags the request. `MOTION_task()` latches `LIMIT`/FAULT on its next
  pass. Before, the ISR wrote engine state directly, so a switch hit during a
  segment update could be overwritten by the engine (segment effectively
  continuing) with a small but real window.
* **Output-start guard**: the speed/start path (`motion_apply_speeds`, STRAFE
  start) refuses to (re)start the pulses while a stop/e-stop/limit request is
  pending, closing the preemption window where the interrupted code could
  re-enable the drivers after the switch had already triggered.
* **`RESET_ODOMETRY`, `SET_POSE` atomic** with respect to the 100 Hz odometry
  ISR (single critical section). The encoder driver critical sections are now
  PRIMASK save/restore, so nested protection never re-enables interrupts too
  early. Previously a TIM7 tick inside the reset sequence could integrate one
  stale delta and jump the pose.
* **Side-counter interrupt load removed**: TIM13/14 update interrupts are
  armed only while the driver's open-loop target API is in use; closed-loop
  operation no longer keeps a step-rate ISR (up to ~10 kHz) running while the
  robot moves.
* **Buttons toggle LED1/LED2** (PC3/PA8) instead of writing RGB channels that
  the status task overwrites each pass; the RGB LED now always shows the
  robot state.
* Documentation aligned with the above (Docs/02 interrupt and fault tables,
  Docs/04 odometry/controller equations and limit-switch description).

## 10. v2.3.2 - RGB boot animation and LED documentation (2026-09-25)

* **Boot indication**: the RGB LED now walks through the full colour set at
  power-on - red, green, blue (primaries), then yellow, turquoise, purple
  (their pairwise mixes), then white - 80 ms per colour, while the OLED
  shows the boot logo; the loop then takes over with the state colours. The
  OLED splash no longer holds a fixed 400 ms delay - the animation provides
  the visible hold, so the logo and the colour walk happen together.
* **Status colours**: idle blue, closed-loop move green, dead-reckoned
  strafe yellow, fault red, e-stop blinking red at 2 Hz.
* **New document** `Docs/08-rgb-led-status.md`: complete colour code,
  startup sequence, brightness values, quick diagnosis table and the
  implementation map.
* RGB duty values and the blink period moved to `App/Inc/robot_configs.h`
  (`ROBOT_RGB_*` macros) so colours can be tuned without touching the code -
  keep Docs/08 in sync when changing them.

## 11. v2.3.3 - Display home screen and boot timing (2026-09-25)

* **RGB boot walk slowed to 250 ms per colour** (~ 1.75 s total,
  `ROBOT_RGB_BOOT_STEP_MS`): the walk covers the OLED logo display for its
  whole duration instead of finishing almost unnoticed.
* **OLED display policy reworked**:
  - the Charara logo is now the **home screen**: drawn at boot, it stays on
    screen during the colour walk and is the default content at runtime;
  - `DISPLAY_TEXT` shows the RPi text lines: in `auto` mode the text is
    displayed for `ROBOT_OLED_MESSAGE_HOLD_MS` (10 s) and the logo then
    returns automatically; in `manual` mode the text persists;
  - `DISPLAY_MODE = off` blanks the screen once;
  - the former periodic status screen (state/pose at 1 Hz) is **removed**:
    the screen is redrawn only when its content changes, which eliminates
    the periodic bus occupation (state monitoring stays available on the
    RGB LED and via `QUERY_STATUS`);
  - redraws no longer wait for the robot to be idle - messages are shown
    even during motion, and the ~100 ms bus window remains absorbed by the
    protocol retry loop as before.

## 12. v2.4.0 - Clock tree and timer kernel clocks (2026-09-28)

* **Clock tree:** 25 MHz HSE quartz through the PLL: M=25 (VCO input
  1 MHz), N=432 (VCO 432 MHz), P=2 -> SYSCLK 216 MHz (PLLQ=9 -> 48 MHz
  spare); HCLK 216, PCLK1 54 (APB1 /4), PCLK2 108 MHz (APB2 /2); VOS1 +
  over-drive, flash 7 wait states (over-drive requires VDD >= 2.7 V).
* **HAL time base:** the TIM6 prescaler follows the APB1 timer rule:
  PCLK1 with the APB1 prescaler at 1, 2 x PCLK1 otherwise (108 MHz at
  APB1 /4) - 1 kHz tick.
* **Timer kernel clocks:** the TIM7 odometry prescaler and the stepper
  mm/s -> ARR conversion use the APB1 timer clock (108 MHz) through
  `BSP_apb1_timer_clk_hz()` / `STEPPERS_MJ_set_timer_clock_hz()`; the RGB
  PWM prescalers use the APB2 timer clock (216 MHz). Periods: 100 Hz
  odometry, ~100 Hz PWM, mm/s speed reference.
* **I2C1:** kernel clock = PCLK1; `ROBOT_I2C_TIMING_100K = 0x17028686`
  (~100 kHz at I2CCLK = 54 MHz) - corrected to `0x10728686` in v3.1.0
  (section 15).
* Flash prefetch enabled; M7 I/D caches enabled.
* Bench checks: DRIVE / TURN / GOTO at commanded mm/s, I2C at 100 kHz with
  the Pi, RGB PWM, UART 115200 when used.

## 13. v2.5.0 - Arc move primitive (2026-09-28)

* **`ARC` command (0x16):** forward motion and rotation at the same time -
  the robot centre follows a circle while the heading changes, so paths
  compose as tangent-continuous line-arc-line chains instead of
  stop-and-turn sequences. Payload: `i16 radius[mm]`, `i16 sweep[0.1 deg]`,
  `u8 speed`; positive = CCW (left arc), negative = CW; the two signs must
  agree.
* **Closed-loop arc controller:** unwrapped heading progress (sweeps above
  180 deg work), brake profile on the tighter of the remaining sweep and
  arc length, completion within 0.5 mm arc and 0.8 deg sweep; stall and
  timeout supervision as for every segment. The cruise is capped so the
  outer wheel stays within the speed limits (small radii run slower).
* **Limits:** `|radius| >= 50 mm`, `0 < |sweep| <= 360 deg`, arc length
  `<= 3000 mm`.
* **State 7 (`ARC`)** in QUERY_ODOMETRY / QUERY_STATUS; the RGB LED shows
  the moving colour during arcs.
* Protocol spec, motion math, Pi integration guide and bring-up tests
  updated; the reference Python client gains `arc()`.

## 14. v3.0.0 - Error reporting, flash settings and calibration (2026-09-29)

* **Board event log (`QUERY_EVENTS` 0x23):** every abnormal condition is
  recorded with a code and a context byte - faults (E-stop, limit, stall,
  timeout, deadman), protocol errors (bad CRC, bad frame, unknown command,
  out-of-range parameter, busy rejection), I2C and flash errors, config
  fallback/apply/save events and the boot reset cause. Read up to 5 events
  per query, oldest first, `after` cursor for follow-up reads; the ring
  holds the most recent 16.
* **Fault reporting widened to 16 bits** in `QUERY_STATUS` and
  `QUERY_ODOMETRY` (ESTOP 0x0001, LIMIT 0x0002, STALL 0x0004, TIMEOUT
  0x0008, DEADMAN 0x0010). `QUERY_STATUS` also reports the last event code
  and context, the reset cause and the settings map version / dirty flag.
* **Persistent settings store:** 17 registers (0 map version, 1..16
  parameters) covering wheel geometry, microsteps, robot/odometry tracks,
  speed limits, acceleration, heading/turn gains, strafe pattern and the
  direction inversion masks. New commands `GET_CONFIG` 0x40, `SET_CONFIG`
  0x41, `SAVE_CONFIG` 0x42, `LOAD_CONFIG` 0x43, `RESET_CONFIG` 0x44;
  writes are validated (all-or-nothing, idle only), saved as append-only
  records in flash sector 7 and reloaded at boot (defaults on invalid
  record, `CONFIG_FALLBACK` event). `SET_MOTION_LIMITS` now updates the
  stored limits.
* **Speeds are u16 mm/s** in every motion command; the absolute ceiling
  rises to **900 mm/s** (3x the former 300); the stored speed limit
  (register 8, default 800) is the operational cap.
* **Protocol v3 framing details:** new statuses `BAD_LEN` 0x07 and
  `STORAGE` 0x08; `QUERY_STATUS` payload is 18 B and `QUERY_ODOMETRY`
  19 B (faults u16).
* **Motion engine reads the settings:** track, odometry scale, gains,
  minimum speed and strafe pattern come from the store, so calibration
  applies immediately after `SET_CONFIG`.
* **Pi side:** config API (`GET/POST /config/robot`, save/load/defaults),
  event decoding and a Calibration tab in the web console (drive/turn
  tests, measured-value fix buttons, board error panel); mock link mirrors
  v3 including flash simulation; test suite extended.
* Docs updated: protocol spec (v3 rewrite), firmware architecture, bring-up
  and tuning (console calibration), motion/odometry and RPi integration
  guide (event runbook).

## 15. v3.1.0 - Bullet-proof I2C error handling and safe OLED sharing (2026-09-30)

* **I2C TIMINGR bug fixed:** `ROBOT_I2C_TIMING_100K` wrote the SCLDEL field
  into the reserved bits 27:24 instead of [23:20], which left a 37 ns
  data-setup time that violates the standard-mode 250 ns minimum - a source of
  intermittent bus errors. The constant is corrected to `0x10728686` and the
  I2C1 kernel clock is pinned to PCLK1 (54 MHz, matching the timing) from the
  MSP and again in `BSP_i2c1_init()` (regeneration-safe).
* **Full bus recovery:** any error now triggers `DeInit` -> bus-clear ->
  `Init` -> listen. `ROBOT_I2C_service_recovery()` bit-bangs 9 SCL pulses +
  STOP when a slave holds the bus low, restores the I2C pins and re-arms
  listen mode; attempts are throttled (`ROBOT_I2C_RECOVERY_RETRY_MS`) so the
  main loop and the IWDG keep running while the peripheral is down.
* **Failed OLED transfer:** the peripheral is reset (not just re-armed) so it
  can never stay stuck in master/busy state - the RPi never loses control.
* **Safer OLED sharing:** the master window only starts when no request is
  pending, no motion segment is active and both lines are idle; a pending
  request or a busy line aborts the refresh instead of racing the RPi.
* **New event `I2C_RECOVER` (0x16):** recorded on every recovery, context = 1
  when a stuck bus had to be bit-banged clear.
* The OLED task is re-enabled in the bare-metal scheduler.

## 16. v3.2.0 - Slip-tolerant arc controller and blended arc chaining (2026-10-01)

* **ARC controller reworked (real-robot bug):** on the mecanum floor the body
  yaws only ~58 % of the commanded wheel differential (roller scrub), so a
  commanded `ARC(R=-435, sweep=-65 deg)` executed as ~R=760 / 107 deg: the length
  never converged, the robot kept orbiting past the plan and the segment died
  as `TIMEOUT (aux 7)`. `motion_do_arc()` now tracks the reference heading
  `theta_ref = sweep * s / L` with a P+I curvature correction
  (`ROBOT_ARC_ADAPT_P/I/MAX_SCALE`, clamped to 1/3..3x of `1/R`) instead of
  trusting the commanded wheel speeds; verified by simulation for yaw gains
  from 0.45 to 1.0 and on the bench.
* **Fail-safe arc completion:** the arc also ends when the travelled length
  passes the target by more than `ROBOT_ARC_SLACK_MM` (5 mm), even if the
  curvature loop never converges - a bad arc can no longer become an orbit.
* **New command `QUEUE_ARC` (0x17):** parks the next arc while the current
  one is running. `motion_finish_segment()` hands over to the pending arc
  directly (outputs and ramps kept), so a chain of arcs runs with **one**
  acceleration and **one** deceleration instead of stopping at every
  segment. One pending slot; a second queue while occupied is rejected with
  `BUSY_REJECT (aux 0x17)`. `STOP`, `ESTOP`, faults and `CLEAR_FAULTS`
  discard the pending arc.
* **Odometry report (0x20) grows to 21 bytes:** adds `segments_done` (u16)
  so the Raspberry Pi can release the next arc exactly when the board
  finishes the previous one.
* **Version register:** now reports 3.2 (`0x0302`). v3.1 shipped without
  bumping the define, so from this release the version returned by
  `PING`/`QUERY_STATUS` can be trusted to verify that a flash landed.
* Raspberry Pi planner: consecutive fitted arcs whose tangent breaks by less
  than 55 deg are joined by a small connector arc (trimming both neighbours)
  instead of in-place pivots, so chains are truly tangent-continuous and the
  executor can blend them with `QUEUE_ARC`.

## 17. v3.3.0 - Silent gyro forward stream (2026-10-04)

* **New command `SET_GYRO` (0x18), silent:** the RPi streams its MPU6050
  yaw rate (0.1 deg/s units + sample age); the frame is applied without a
  published response and never enters the retry window. The engine keeps a
  complementary-blended control heading (gyro fast, wheel slow pull) used by
  every steering read; a stale or absent stream collapses smoothly back to
  the wheel heading with no fault and no event (Docs/04, Docs/03 section 4).
* `QUERY_STATUS` byte 18 reports whether fresh stream frames arrive.
* Detailed design and the RPi side: `Docs/09-sensor-fusion-and-control-
  architecture.md` (this repository) and `Docs/06` section 8 for the bench
  checklist.

## 18. v3.4.0 - Silent velocity forward stream (2026-10-09)

* **New command `SET_VELOCITY` (0x19), silent:** the RPi streams velocity
  TARGETS for the robot centre - `i16 v [mm/s]` + `i16 omega [0.1 deg/s]` -
  at 20-50 Hz for continuous path following (no stop at junctions). The
  first frame while idle enters the streaming mode (state 8 = STREAM);
  further frames are latest-wins updates.
* **Streaming controller:** wheel mixing `v_left = v - omega * track/2`,
  `v_right = v + omega * track/2`, per-side ramps inside the configured
  accel/decel, the driver's 30 mm/s per-side floor for nonzero targets, and
  a park path for exact (0, 0) targets that keeps the mode without pulsing.
* **Controlled stale-stop:** no frame for 200 ms ramps both targets to zero
  and returns to IDLE - no fault, no event. STOP / E-STOP / limit switches
  cancel the stream like any segment; the deadman still guards busy motion;
  program commands and stream frames never mix mid-motion (`BUSY`).
* `MOTION_STATE_STREAM = 8` joins the published state values; the mode is
  entered only from IDLE.
* Raspberry Pi side (this repository): `core/navigation/pursuit.py` +
  `tracking.py` + `sim.py`, the gated `core/mission/follower.py` backend and
  the silent `RobotClient.stream_velocity()`; see `Docs/10-roadmap.md`
  Track B and the bring-up checklist in `Docs/06` section 9.

*Bench status: source merged; the IAR build and the section 9 checklist on
 the real board are pending.*
