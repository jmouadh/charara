/**
 ******************************************************************************
 * @file    robot_configs.h
 * @brief   Robot configuration - single source of truth for this project.
 *
 * @details Everything robot-specific lives here: geometry, motion limits,
 *          gains, protocol and timing settings. The Drivers/ folder contains
 *          project-independent drivers only - the mechanical data they need
 *          is handed over at runtime through STEPPERS_MJ_configure() and
 *          ENCODERS_MJ_configure(), called once by ROBOT_APP_init().
 *          Bench-verify all hardware-related values (Docs/06).
 *
 * MISRA note: this header only defines object-like macros with explicit
 * suffixes and parentheses; no executable code.
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#ifndef ROBOT_CONFIGS_H
#define ROBOT_CONFIGS_H

/* ---------------------------- Firmware identity ----------------------------
 * 3.5 = RELEASE_MOTORS (0x06): idle-only stepper-driver release without a
 * fault latch; the next motion command re-energizes the drivers. Additive -
 * hosts that never send 0x06 behave exactly like 3.4.
 * 3.4 = silent SET_VELOCITY stream (0x19): the RPi streams continuous
 * velocity targets (v, omega); the board ramps towards them and leaves the
 * streaming mode by itself when frames stop (controlled stale-stop, never
 * a fault). Everything is additive - hosts that never send 0x19 behave
 * exactly like 3.3.
 * 3.3 = silent SET_GYRO stream (0x18): gyro-aided control heading with the
 * wheel-only fallback when the stream is absent or stale.
 * 3.2 = slip-tolerant ARC controller (P+I curvature adaptation + fail-safe
 * completion) and QUEUE_ARC (0x17) blended arc chaining. Note: v3.1 (I2C
 * hardening) shipped without bumping this define, so a board reporting 3.0.0
 * is NOT automatically the old firmware. */
#define ROBOT_FW_VERSION_MAJOR        (3U)
#define ROBOT_FW_VERSION_MINOR        (5U)

/* ------------------------------- MCU clock ---------------------------------
 * 25 MHz HSE quartz through the PLL: SYSCLK = 216 MHz.
 * PLL: M=25 (VCO input 1 MHz), N=432 (VCO 432 MHz), P=2 -> 216 MHz,
 * Q=9 -> 48 MHz spare. HCLK 216 / PCLK1 54 (APB1 /4) / PCLK2 108 MHz
 * (APB2 /2); VOS1 + over-drive; flash 7 wait states.
 * Timer kernel clocks: APB1 = 108 MHz, APB2 = 216 MHz (x2 rule).
 * If the clock tree is ever changed, re-check:
 *   - I2C1 TIMINGR (ROBOT_I2C_TIMING_100K; I2CCLK = PCLK1 = 54 MHz)
 *   - TIM7 prescaler (BSP_apb1_timer_clk_hz(), robot_board.c)
 *   - RGB PWM prescalers (BSP_apb2_timer_clk_hz(), robot_board.c)
 *   - stepper ARR conversion (STEPPERS_MJ_set_timer_clock_hz(), robot_app.c)
 */
#define ROBOT_SYSCLK_HZ               (216000000UL)

/* ---------------------------------- Math ----------------------------------- */
#define ROBOT_PI                      (3.14159265358979323846f)
#define ROBOT_TWO_PI                  (2.0f * ROBOT_PI)

/* ----------------------- Robot geometry (stepper wheels) ------------------- */
#define ROBOT_WHEEL_DIAMETER_MM           (60.0f)
#define ROBOT_WHEEL_CIRCUMFERENCE_MM      (ROBOT_PI * ROBOT_WHEEL_DIAMETER_MM) /* 188.4956 mm */
#define ROBOT_TRACK_MM                    (148.0f)  /* left-to-right wheel distance */
#define ROBOT_WHEELBASE_MM                (148.0f)  /* front-to-back wheel distance */
#define ROBOT_STEPPER_FULL_STEPS          (200U)
#define ROBOT_STEPPER_MICROSTEPS          (32U)
#define ROBOT_STEPPER_TICKS_PER_REV       ((float)(ROBOT_STEPPER_FULL_STEPS) * (float)(ROBOT_STEPPER_MICROSTEPS)) /* 6400 */
#define ROBOT_STEPS_PER_MM                (ROBOT_STEPPER_TICKS_PER_REV / ROBOT_WHEEL_CIRCUMFERENCE_MM) /* 33.95 */

/* ------------------------------ Odometry encoders --------------------------- */
#define ROBOT_ODOM_WHEEL_CIRCUMFERENCE_MM (200.0f)  /* passive encoder wheel */
#define ROBOT_ODOM_TICKS_PER_REV          (4000.0f)
#define ROBOT_ODOM_MM_PER_TICK            (ROBOT_ODOM_WHEEL_CIRCUMFERENCE_MM / ROBOT_ODOM_TICKS_PER_REV) /* 0.05 */
#define ROBOT_ODOM_TRACK_MM               (170.0f)  /* distance between encoder wheels */
#define ROBOT_ENCODER_ID_LEFT             (1U)      /* TIM8, PC6/PC7 */
#define ROBOT_ENCODER_ID_RIGHT            (0U)      /* TIM5, PA0/PA1 */

/* Base counting direction of each odometry encoder, handed to
 * ENCODERS_MJ_Init() in ROBOT_APP_init(): 0 keeps the hardware direction,
 * 1 inverts it. The runtime calibration mask encoder_invert_mask (settings
 * register 16, bit 0 = LEFT, bit 1 = RIGHT) is XORed ON TOP of these by
 * ROBOT_SETTINGS_apply(), so the mask toggles relative to the wiring instead
 * of replacing it. */
#define ROBOT_ENCODER_INVERT_COUNT_LEFT   (1U)
#define ROBOT_ENCODER_INVERT_COUNT_RIGHT  (0U)

/* ------------------- Stepper wheel IDs (as wired - verify!) -----------------
 * Left pair = motor IDs {0,1}, right pair = {2,3} (the Init_left_steppers /
 * Init_right_steppers calls in App/Src/robot_app.c). Which motor of a pair is
 * physically front is NOT known from the schematic alone - validate with the
 * strafe bench test and adjust below.
 */
#define ROBOT_STEPPER_ID_FL (2U)
#define ROBOT_STEPPER_ID_FR (1U)
#define ROBOT_STEPPER_ID_RL (3U)
#define ROBOT_STEPPER_ID_RR (0U)

/* Base DIR polarity of each stepper driver, handed to STEPPERS_MJ_Init() in
 * ROBOT_APP_init() (GPIO_PIN_SET inverts the DIR pin for a FORWARD command).
 * The runtime calibration mask steer_invert_mask (settings register 15, bit =
 * motor ID) is XORed ON TOP of these by ROBOT_SETTINGS_apply(), so the mask
 * toggles relative to the wiring instead of replacing it. */
#define ROBOT_STEPPER_INVERT_DIR_0        (GPIO_PIN_RESET)  /* STEPPERS_MJ_Init(0U, ...) */
#define ROBOT_STEPPER_INVERT_DIR_1        (GPIO_PIN_RESET)  /* STEPPERS_MJ_Init(1U, ...) */
#define ROBOT_STEPPER_INVERT_DIR_2        (GPIO_PIN_SET)    /* STEPPERS_MJ_Init(2U, ...) */
#define ROBOT_STEPPER_INVERT_DIR_3        (GPIO_PIN_SET)    /* STEPPERS_MJ_Init(3U, ...) */

/* Mecanum strafe pattern for +Y motion (robot translates to its LEFT).
 * 1 = wheel driven FORWARD, 0 = wheel driven BACKWARD. Order: FL FR RL RR.
 * - If the robot strafes to the wrong side, invert all four bits.
 * - If the robot rotates instead of translating, swap FL<->RL and/or FR<->RR.
 * The protocol documents STRAFE as dead-reckoned only (no lateral odometry). */
#define ROBOT_STRAFE_PLUS_Y_FL (1U)
#define ROBOT_STRAFE_PLUS_Y_FR (0U)
#define ROBOT_STRAFE_PLUS_Y_RL (0U)
#define ROBOT_STRAFE_PLUS_Y_RR (1U)

/* ------------------------- Motion limits and control ------------------------
 * The values below are the compile-time DEFAULTS: the live values come from
 * the flash-backed settings store (robot_settings.h), which is loaded at
 * boot and can be calibrated from the RPi over I2C.
 * ROBOT_MAX_SPEED_LIMIT_MM_S is the ABSOLUTE ceiling accepted by the
 * protocol and the motion engine (3x the previous ceiling). */
#define ROBOT_MAX_SPEED_MM_S            (800.0f) /**< default max speed     */
#define ROBOT_MAX_SPEED_LIMIT_MM_S      (900.0f) /**< absolute max speed    */
#define ROBOT_MIN_SPEED_MM_S            (100.0f) /* creep speed floor        */
#define ROBOT_ARC_MIN_RADIUS_MM         (50.0f)  /* smallest allowed arc radius */
#define ROBOT_DEFAULT_ACCEL_MM_S2       (600.0f)
#define ROBOT_DEFAULT_DECEL_MM_S2       (500.0f)
#define ROBOT_TURN_TOL_RAD              (0.014f) /* about 0.8 deg */
#define ROBOT_DRIVE_TOL_MM              (0.5f)
#define ROBOT_HEADING_GAIN_PER_S        (1.2f)   /* heading hold during DRIVE */
#define ROBOT_TURN_GAIN_PER_S           (1.5f)   /* P gain of TURN controller */
#define ROBOT_MIN_CONTROL_SPEED_MM_S    (30.0f)  /* lower bound per side while regulating
                                                    (matches the driver clamp) */
#define ROBOT_SPEED_EPS_MM_S            (2.0f)   /* do not rewrite ARR for smaller deltas */
#define ROBOT_DIR_SETTLE_MS             (2U)     /* DIR setup delay before pulses */
#define ROBOT_STALL_WINDOW_MS           (350U)
#define ROBOT_STALL_MIN_DELTA_MM        (0.5f)

/* --------------------- Gyro forward stream (SET_GYRO) -----------------------
 * The RPi streams its MPU6050 yaw rate while the robot moves (firmware >= 3.3,
 * silent frames, Docs/STM_docs/03 section 4). Control policy:
 *   fresh stream -> the control heading integrates the gyro (fast, slip-free)
 *   with a slow pull towards the wheel odometry;
 *   absent/stale -> the blend collapses to the wheel odometry with a short
 *   time constant (smooth hand-over, no heading step, no fault).
 * The stream is optional in every way: without it the board behaves exactly
 * like firmware 3.2 until these defines are touched. */
#define ROBOT_GYRO_STALE_MS             (200U)   /* older = wheel heading      */
#define ROBOT_GYRO_BLEND_PULL_ALPHA     (0.02f)  /* per 100 Hz tick, fresh     */
#define ROBOT_GYRO_BLEND_PULL_STALE     (0.20f)  /* per 100 Hz tick, stale     */
#define ROBOT_GYRO_MAX_RATE_RAD_S       (20.0f)  /* sanity clamp (1146 deg/s)  */

/* --------------------- Velocity forward stream (SET_VELOCITY) ---------------
 * The RPi streams velocity targets (v, omega) at 20-50 Hz (firmware >= 3.4,
 * silent frames, Docs/STM_docs/03 section 5). The streaming mode is entered
 * by the first frame while idle and left by STOP / E-STOP / any fault; when
 * no frame arrives for ROBOT_STREAM_STALE_MS the board ramps BOTH targets
 * down with the configured decel and returns to idle - a controlled stop,
 * never a fault (the deadman independently guards any busy motion). The
 * controller is open-loop on the wheel speeds, the same step-timing layer
 * every segment uses; the RPi closes the path loop with its fused pose.
 * Wheel geometry: v_left = v - omega * track / 2, v_right = v + omega *
 * track / 2 (omega CCW positive). */
#define ROBOT_STREAM_STALE_MS          (200U)   /* no frame -> controlled stop */
#define ROBOT_STREAM_MAX_OMEGA_RAD_S   (3.0f)   /* sanity clamp (172 deg/s)    */
#define ROBOT_STREAM_MIN_SIDE_MM_S     (30.0f)  /* per-side floor (driver clamp) */
#define ROBOT_STREAM_END_EPS_MM_S      (5.0f)   /* stale-stop end threshold    */
#define ROBOT_STREAM_END_EPS_RAD_S     (0.05f)  /* stale-stop end threshold    */
#define ROBOT_ODOMETRY_TICK_S           (0.01f)  /* TIM7 period (100 Hz)       */
#define ROBOT_SEGMENT_TIMEOUT_MARGIN_MS (3000U)
#define ROBOT_SEGMENT_NOMINAL_FACTOR    (2.2f)   /* timeout = nominal * factor + margin */

/* ------------------------------ ARC tracking --------------------------------
 * On the real floor the body yaws LESS than the wheel differential predicts
 * (mecanum / skid scrub: measured ~58% of the commanded yaw per distance).
 * The arc controller therefore tracks a heading profile against the measured
 * arc-length progress (th_ref = sweep * s / L) with a P+I curvature
 * adaptation, so the executed sweep and arc length close together whatever
 * the floor slip is (the old fixed-curvature controller could not converge
 * when the two completion windows drifted apart by more than ~2%, which
 * turned every real ARC into an endless circle until the segment timeout
 * fired). */
#define ROBOT_ARC_ADAPT_P                (8.0f)  /* P gain, scales |1/R| per rad  */
#define ROBOT_ARC_ADAPT_I                (3.0f)  /* I gain [1/(rad*s)]            */
#define ROBOT_ARC_ADAPT_MAX_SCALE        (3.0f)  /* curvature may scale 1/3..3x   */
#define ROBOT_ARC_SLACK_MM               (5.0f)  /* length overshoot ends the ARC */

/* ------------------------------ I2C / protocol ------------------------------ */
#define ROBOT_I2C_ADDRESS_7BIT          (0x0AU)
/* TIMINGR for 100 kHz standard mode with I2CCLK = PCLK1 = 54 MHz.
 * Field layout of the STM32F7 I2C_TIMINGR register (see stm32f722xx.h):
 *   PRESC[31:28], SCLDEL[23:20], SDADEL[19:16], SCLH[15:8], SCLL[7:0].
 *   PRESC  = 1   -> t_PRESC = 2 / 54 MHz = 37.0 ns
 *   SCLL   = 134 -> t_LOW   = 135 * 37.0 ns = 5.0 us  (>= 4.7 us)
 *   SCLH   = 134 -> t_HIGH  = 135 * 37.0 ns = 5.0 us  (>= 4.0 us)
 *   SCLDEL = 7   -> t_SU;DAT = 8 * 37.0 ns = 296 ns   (>= 250 ns)
 *   SDADEL = 2   -> t_HD;DAT = 3 * 37.0 ns = 111 ns
 * t_SCL = 10 us -> 100 kHz.
 * NOTE: the previous value (0x17028686) placed SCLDEL in the reserved bits
 * 27:24 instead of [23:20], which left SCLDEL = 0 -> a 37 ns data-setup time
 * that violates the standard-mode 250 ns minimum. That timing violation is a
 * known source of intermittent I2C errors; keep the field layout above in
 * mind whenever this constant is recomputed. */
#define ROBOT_I2C_TIMING_100K           (0x10728686UL)
#define ROBOT_I2C_BUS_QUIET_MS          (100U)   /* OLED redraw only after bus idle */
/* Bus recovery (see ROBOT_I2C_service_recovery):
 * - a bus stuck low by a slave is freed by bit-banging ROBOT_I2C_BUS_CLEAR_CLOCKS
 *   SCL pulses of ROBOT_I2C_BUS_CLEAR_DELAY_MS each, followed by a STOP;
 * - a failed DeInit/Init attempt is retried at most every
 *   ROBOT_I2C_RECOVERY_RETRY_MS so the main loop (and the IWDG refresh) keep
 *   running while the peripheral is momentarily down. */
#define ROBOT_I2C_BUS_CLEAR_CLOCKS      (9U)
#define ROBOT_I2C_BUS_CLEAR_DELAY_MS    (1U)
#define ROBOT_I2C_RECOVERY_RETRY_MS     (5U)
#define ROBOT_OLED_REFRESH_MS           (1000U)  /* minimum spacing between redraws */
#define ROBOT_OLED_BACKOFF_MS           (10000U) /* after an OLED error */
/* Boot retry: the OLED/SSD1306 may still be powering up, or the shared I2C
 * bus may be held by the Raspberry Pi while it starts. Re-run the controller
 * init at a short interval for the first few seconds, then fall back to the
 * normal 10 s back-off (an absent display must not hammer the bus). */
#define ROBOT_OLED_BOOT_RETRY_MS        (500U)
#define ROBOT_OLED_BOOT_RETRY_COUNT     (10U)
#define ROBOT_OLED_MESSAGE_HOLD_MS      (10000U) /* DISPLAY_TEXT display time in AUTO mode */
#define ROBOT_OLED_LINE_COUNT           (3U)
#define ROBOT_OLED_LINE_LEN             (22U)
#define ROBOT_DEADMAN_DEFAULT_MS        (0U)     /* 0 = disabled, enable from RPi */

/* ------------------------------- RGB status LED -----------------------------
 * Packed 0xRRGGBB duty values (0..255 per channel) used by the status task;
 * the colour code and the boot animation are documented in
 * Docs/08-rgb-led-status.md - keep both in sync when changing values.
 */
#define ROBOT_RGB_IDLE_COLOR        (0x000020UL) /* steady dim blue             */
#define ROBOT_RGB_BUSY_COLOR        (0x003000UL) /* steady dim green            */
#define ROBOT_RGB_STRAFE_COLOR      (0x303000UL) /* steady yellow: strafe       */
#define ROBOT_RGB_FAULT_COLOR       (0x300000UL) /* steady dim red              */
#define ROBOT_RGB_ESTOP_COLOR       (0x300000UL) /* red, blinking at 2 Hz       */
#define ROBOT_RGB_ESTOP_BLINK_MS    (250U)       /* blink half period [ms]      */
#define ROBOT_RGB_BOOT_STEP_MS      (250U)       /* boot walk: per-colour hold  */

/* --------------------------- Flash configuration ----------------------------
 * The runtime settings (geometry, wheel/encoder data, motion limits, gains)
 * live in the last flash sector and are loaded at every boot. All values are
 * kept as the packed register image produced by robot_settings.c so a save
 * is a single erase/program cycle of one small record. */
#define ROBOT_FLASH_CONFIG_ADDR          (0x08060000UL) /**< sector 7 base     */
#define ROBOT_FLASH_CONFIG_SECTOR        (7U)           /**< FLASH_SECTOR_7    */
#define ROBOT_FLASH_CONFIG_MAGIC         (0x43485233UL) /**< "CHR3"            */
#define ROBOT_FLASH_CONFIG_WORDS         (16U)          /**< 64-byte record    */

/* ----------------------------- Error event log ------------------------------ */
#define ROBOT_EVENT_RING_SIZE            (16U)  /**< events kept in RAM         */
#define ROBOT_EVENT_READ_MAX             (5U)   /**< events per QUERY_EVENTS    */

/* --------------------------------- Watchdog --------------------------------- */
#define ROBOT_IWDG_RELOAD           (250U)       /* ~500 ms with LSI ~32 kHz, /64 */

#endif /* ROBOT_CONFIGS_H */


