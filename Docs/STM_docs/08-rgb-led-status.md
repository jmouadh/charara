# 08 - RGB status LED: colours and behaviour

*Author: Mouadh Jeridi - jeridimouadh@gmail.com*

This document is the reference for everything the LEDs show: the RGB LED that
reports the robot state, and the two discrete LEDs driven by the user
buttons. Duty values live in `App/Inc/robot_configs.h` - keep this document
in sync whenever they change.

## 1. Hardware and drivers

| LED | Pins | Driven by | Purpose |
|---|---|---|---|
| RGB red | `RGB_R` PA2 | TIM9 channel 1, ~100 Hz PWM | robot state |
| RGB green | `RGB_G` PA3 | TIM9 channel 2, ~100 Hz PWM | robot state |
| RGB blue | `RGB_B` PB9 | TIM11 channel 1, ~100 Hz PWM | robot state |
| LED1 | PC3 | GPIO output | toggled by BUTTON1 |
| LED2 | PA8 | GPIO output | toggled by BUTTON2 |

* The RGB duty is `0..255` per channel (0 = off, 255 = full brightness).
* The RGB LED is written only by the status task / boot animation
  (`App/Src/robot_status.c`); no other module touches it.
* `LED1`/`LED2` are free indicators: each press of `BUTTON1` / `BUTTON2`
  toggles its LED. They are never written by the firmware logic itself.

## 2. Startup sequence

| # | What happens | OLED | RGB |
|---|---|---|---|
| 1 | Power-on, peripherals init | dark | off |
| 2 | Boot splash drawn | logo (bitmap + "Charara") | blue (idle) |
| 3 | Boot colour walk (see below) | logo stays | cycling (starts red) |
| 4 | Loop starts, status task takes over | logo home screen stays | idle colour |

**Boot colour walk:** red -> green -> blue (the three primaries), then yellow
(R+G) -> turquoise (G+B) -> purple (R+B) (their pairwise mixes), then white
(R+G+B) - 250 ms per colour (~ 1.75 s total) - and back to the idle colour.
The order walks from single channels to all channels on, so it ends exactly
where the status indication begins. It is implemented in
`ROBOT_STATUS_boot_animation()` and runs while the logo is displayed, so the
operator sees "the board is alive" even before the first I2C transaction
(the slave listen mode is armed right after the walk finishes).

## 3. Runtime status colours

| `state` (`QUERY_STATUS`/`QUERY_ODOMETRY`) | State name | RGB | Meaning / operator action |
|---|---|---|---|
| 0 | IDLE | steady **dim blue** | Ready. Waiting for commands |
| 1 | DRIVE | steady **dim green** | Executing a straight move |
| 2 | TURN | steady **dim green** | Executing a rotation |
| 3 | STRAFE | steady **yellow** | Executing a dead-reckoned lateral move (no lateral odometry - expect limited Y accuracy) |
| 4 | GOTO | steady **dim green** | Executing a turn-drive-turn |
| 5 | FAULT | steady **red** | Latched fault (STALL / TIMEOUT / DEADMAN / LIMIT). Motion is blocked until `CLEAR_FAULTS` (0x03) - read `QUERY_STATUS.faults` to know which one |
| 6 | ESTOP | **red, blinking at 2 Hz** | Emergency stop latched, drivers off. `CLEAR_FAULTS` + verify the area before resuming |

Priority when several conditions overlap: `ESTOP` > `FAULT` > `STRAFE`
(yellow) > other moves (green) > idle.

Notes:

* The colour follows the **motion state**, not the individual fault bits;
  faults other than e-stop share one red indication - the fault *code*
  always comes from `QUERY_STATUS`, the LED is a quick visual check.
* The colour is recomputed on every super-loop pass (typically < 1 ms), so
  the LED reacts as fast as the state changes; the blink is derived from the
  millisecond tick (250 ms on / 250 ms off).
* The RGB indication is independent from the OLED and from `DISPLAY_MODE`:
  it works with the display absent, off or owned by the Raspberry Pi.

## 4. Configuration

All values are in `App/Inc/robot_configs.h`, packed as `0xRRGGBB`:

| Macro | Default | Meaning |
|---|---|---|
| `ROBOT_RGB_IDLE_COLOR` | `0x000020` | blue channel 32 (~ 12 % duty) |
| `ROBOT_RGB_BUSY_COLOR` | `0x003000` | green channel 48 (~ 19 % duty) |
| `ROBOT_RGB_STRAFE_COLOR` | `0x303000` | yellow (red + green channels 48) |
| `ROBOT_RGB_FAULT_COLOR` | `0x300000` | red channel 48 (~ 19 % duty) |
| `ROBOT_RGB_ESTOP_COLOR` | `0x300000` | red used during the blink |
| `ROBOT_RGB_ESTOP_BLINK_MS` | `250` | blink half period (250 ms -> 2 Hz) |
| `ROBOT_RGB_BOOT_STEP_MS` | `250` | hold time per colour during the boot walk |

Brightness was chosen low on purpose: the LED sits next to the electronics
and a Eurobot event table; full duty is hard on the eyes and unnecessary at
that distance. To make a state more prominent, increase only its channel
value (e.g. `0x600000` for a bright red fault).

## 5. Quick diagnosis

| What you see | Reading | First action |
|---|---|---|
| Steady blue | Idle, firmware healthy | send commands |
| Steady green but robot not moving | Segment running but blocked (mechanically) | wait for the STALL fault (~ 350 ms) or send `STOP` |
| Steady yellow | Strafe (dead-reckoned lateral move) running | normal - remember to re-localize afterwards |
| Steady red | Fault latched | `QUERY_STATUS` -> `faults`, then `CLEAR_FAULTS` |
| Red blinking | E-stop latched | check the workspace, `CLEAR_FAULTS` |
| Dark for more than one blink period | No colour at all: board off / crashed / in reset | check power, IWDG reset log, active debug session |
| Colours cycling after power-on only | Normal boot animation | none |

## 6. Implementation map

| Item | Where |
|---|---|
| PWM setup (~100 Hz) | `BSP_tim9_init`, `BSP_tim11_init` in `App/Src/robot_board.c` |
| Colour constants | `App/Inc/robot_configs.h`, section "RGB status LED" |
| Boot animation | `ROBOT_STATUS_boot_animation()` in `App/Src/robot_status.c`, called once by `ROBOT_APP_init()` |
| Periodic state colour | `ROBOT_STATUS_task()` in `App/Src/robot_status.c`, called every loop pass by `ROBOT_APP_task_status()` |
| Button events | `ROBOT_STATUS_button_event()` in `App/Src/robot_status.c`, called from the EXTI dispatcher |
