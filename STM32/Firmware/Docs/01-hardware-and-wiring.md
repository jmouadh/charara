# 01 - Hardware and wiring reference

*Author: Mouadh Jeridi - jeridimouadh@gmail.com*

**Robot top view (reference frame at boot: +x forward, +y left, theta CCW):**

```
              +x
               ^
     [FL M2]   |   [FR M1]
               |
   +y <--------o--------> -y
               |
     [RL M3]   |   [RR M0]
               v

  Mx  = stepper motor ID as wired (see robot_configs.h)
  Two passive odometry encoder wheels sit mid-side, 170 mm apart (0.05 mm/tick)
  Three wall limit switches: SW1 / SW4 / SW3 (see section 2)
```

Everything in this document is derived from the schematic, the CubeMX
configuration (`carte_deplacement.ioc`) and the firmware. Hardware is frozen:
the firmware compensates where needed, and the workarounds are listed at the
end.

## 1. MCU and clock tree

| Item | Value |
|---|---|
| MCU | STM32F722RET6 (Cortex-M7, 512 KB flash, 256 KB RAM) |
| Clock source | **HSE 25 MHz quartz + PLL** (`BSP_clock_init`) |
| SYSCLK / HCLK / PCLK1 / PCLK2 | 216 MHz / 216 MHz / 54 MHz / 108 MHz |
| PLL | M=25, N=432, P=2 (VCO 432 MHz), Q=9 -> 48 MHz spare |
| Voltage scaling / flash latency | VOS1 + over-drive / 7 wait states |
| HAL time base | TIM6, 1 kHz, IRQ priority 15 |

> **Clock:** 25 MHz HSE quartz through the PLL at the rated maximum:
> M=25/N=432/P=2 -> 216 MHz (VOS1 + over-drive, 7 flash wait states;
> over-drive requires VDD >= 2.7 V, 3.3 V rail here). When the clock tree
> is changed, re-check `ROBOT_I2C_TIMING_100K` (I2CCLK = PCLK1 = 54 MHz),
> the TIM7 prescaler (`BSP_apb1_timer_clk_hz()`), the RGB PWM prescalers
> (`BSP_apb2_timer_clk_hz()`) and the stepper ARR conversion (step timer
> clock handed over in `ROBOT_APP_init`).

### Derived timing constants (216 MHz core)

| Resource | Formula | Value |
|---|---|---|
| I2C1 | `ROBOT_I2C_TIMING_100K = 0x10728686` (I2CCLK = PCLK1 = 54 MHz) | 100 kHz standard mode |
| TIM7 (odometry) | `f_APB1tim/10000 - 1` presc = 10799, period 100 | **100 Hz** exact |
| TIM9/TIM11 (RGB) | `f_APB2tim/25600 - 1` presc = 8436, period 256 | ~100 Hz PWM |
| Step output | `f_step = f_APB1tim / (2*(ARR+1))` (OC toggle, `f_APB1tim` = 108 MHz) | 30..900 mm/s range |
| Side target timers | `ARR_group = 2 * ARR_step` | same rate as steps |

I2C timing check (`0x10728686`, I2CCLK = PCLK1 = 54 MHz): `PRESC` = 1
(t_PRESC = 37.0 ns), `SCLL` = `SCLH` = 134 -> t_LOW = t_HIGH = 5.0 us, so
t_SCL = 10.0 us = **100.0 kHz** with the standard-mode minimums met (4.7 us
low, 4.0 us high); `SCLDEL` = 7 -> t_SU;DAT = 296 ns (>= 250 ns), `SDADEL` = 2
-> t_HD;DAT = 111 ns. `SCLDEL` lives in bits [23:20]: the pre-v3.1 value
`0x17028686` wrote it into the reserved bits [27:24] (37 ns data setup, 1.5 us
high time) and caused intermittent bus errors - see `07-changelog-v2.md`
section 15.

## 2. Pin map

### Stepper motors (1 driver per NEMA17, EN active-low)

| Motor ID | STEP | DIR | EN | STEP timer | Side group | Notes |
|---|---|---|---|---|---|---|
| 0 | PB14 | PB13 | PB12 | TIM12_CH1 | right | DIR inverted in FW |
| 1 | PB6 | PB5 | PB4 | TIM4_CH1 | right | DIR inverted in FW |
| 2 | PA6 | PA7 | PC4 | TIM3_CH1 | left | |
| 3 | PA15 | PA4 | PB15 | TIM2_CH1 | left | |

Left pair = {2,3}, right pair = {0,1} (see `robot_configs.h`). Which motor of
a pair is physically **front** is only assumed - validate with the strafe
bench test (`Docs/06`) and correct `ROBOT_STEPPER_ID_F*`/`R*` if needed.

### Odometry encoders (quadrature, TIM encoder mode TI12, 16-bit)

| Logical | Pins | Timer | Notes |
|---|---|---|---|
| RIGHT | PA0 / PA1 | TIM5 | id 0; `ROBOT_ENCODER_ID_RIGHT` |
| LEFT | PC6 / PC7 | TIM8 | id 1; `ROBOT_ENCODER_ID_LEFT` |

Passive encoder wheels: 4000 ticks/rev (post 4x decode), 200 mm
circumference -> **0.05 mm/tick**, 170 mm apart.

### I2C1 (shared bus, PB7 = SDA, PB8 = SCL)

| Device | Role | Address |
|---|---|---|
| Raspberry Pi 3 b+ | Master | - |
| **This board**   | **Slave** (listen) + temporary **master** for the OLED | `0x0A` |
| SSD1306 OLED | Slave | `0x3C` |

Pull-ups are on the modules; bus runs at 100 kHz standard mode. See
`Docs/02` section 'I2C bus arbitration' for the dual-role mechanism and its limits.

### Limit switches (walls)

| Signal | Pin | EXTI | Firmware behavior |
|---|---|---|---|
| SW1 | PB10 | EXTI10 | hard stop, latch `MOTION_FAULT_LIMIT`, LED red |
| SW4 | PA11 | EXTI11 | same |
| SW3 | PA12 | EXTI12 | same |

Configured as **rising-edge** interrupts in `BSP_gpio_init`
(`App/Src/robot_board.c`) since the switches
are wired active-high.

Any limit event stops **all** motors immediately (in the EXTI ISR), disables
the drivers and latches a fault. Motion stays blocked until `CLEAR_FAULTS`.
The RPi should treat this as "the robot touched a wall" and re-localize.

### Human interface

| Signal | Pin | Function in FW |
|---|---|---|
| LED1 | PC3 | spare (was boot heartbeat) |
| RGB_R / RGB_G | PA2 / PA3 | TIM9_CH1/CH2 PWM, status color |
| RGB_B | PB9 | TIM11_CH1 PWM, status color |
| BUTTON1 | PC8 | toggles R+B LED |
| BUTTON2 | PC9 | toggles R+G LED |
| USART1 | PA9/PA10 | 115200 8N1, initialized, unused (available for logs/SWO alternative) |

**Status LED colors** (see `ROBOT_STATUS_task`, `App/Src/robot_status.c`): idle = blue, moving = green,
fault = red, e-stop = blinking red.

## 3. Timer allocation

| Timer | Purpose | Interrupt | Rate while active |
|---|---|---|---|
| TIM2/3/4/12 | Step pulse generation (one channel each, OC toggle) | none | step rate |
| TIM13 | Left side target counter (fallback open-loop stop) | TIM8_UP_TIM13, prio 3 | = step rate |
| TIM14 | Right side target counter (fallback) | TIM8_TRG_COM_TIM14, prio 3 | = step rate |
| TIM5 | Right odometry encoder (no interrupts) | - | - |
| TIM8 | Left odometry encoder (no interrupts) | - | - |
| TIM6 | HAL tick | TIM6_DAC, prio 15 | 1 kHz |
| TIM7 | Odometry sampling (poll + integrate) | TIM7, prio 2 | 100 Hz |
| TIM9/11 | RGB LED PWM | none | ~100 Hz |

## 4. Power and drivers

* Drivers are step/dir with active-low enable; the FW keeps them **enabled
  after a completed move** (holding torque) and disables them on STOP/FAULT/
  ESTOP. E-stop therefore leaves the wheels free-running.
* 32 microsteps, 200 full steps/rev -> **6400 ticks/rev**, 33.95 steps/mm.
* Speed range: 30..900 mm/s (absolute ceiling); the stored speed limit
  (settings register 8, default 800 mm/s) usually caps it lower. Default
  commands use 100-200 mm/s.

## 5. Hardware limits the firmware cannot remove

1. **Only two side odometry encoders** -> lateral motion (strafe) has *no*
   feedback; STRAFE is dead-reckoned and documented as such.
2. **Single shared I2C bus** -> during an OLED redraw (~100 ms at 100 kHz)
   the STM32 cannot answer the Pi (it is the bus master). The firmware
   redraws only when the screen content changes (boot, `DISPLAY_TEXT`,
   mode change) and after bus silence; the Pi must retry (the protocol's
   sequence numbers make retries safe).
3. **16-bit step timers** -> lowest step rate at the 108 MHz APB1 timer
   clock is ~ 824 Hz (~ 24 mm/s), fine for the configured minimum of
   30 mm/s.
4. **No motor current sensing** -> stalls are detected by no-progress
   supervision on the encoders.
