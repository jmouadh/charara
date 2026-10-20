/******************************************************************************
 * @file    motion.h
 * @brief   Non-blocking motion engine: odometry, ramped closed-loop segments,
 *          and safety supervision (stall, timeout, limit switches, e-stop).
 *
 * @details Design rules:
 *          - MOTION_task() and the accept/set functions run in the main loop.
 *          - MOTION_odometry_tick() runs from the TIM7 interrupt (100 Hz).
 *          - MOTION_request_estop() / MOTION_on_limit_switch() are ISR-safe:
 *            they only latch faults and stop the step timers immediately.
 *          - The engine never blocks; every segment is supervised by
 *            closed-loop odometry plus stall and timeout watchdogs.
 *
 * @note    Detailed controller description: Docs/04-motion-odometry-and-safety.md.
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

/** @brief Result codes returned by the MOTION_accept_* functions. */
typedef enum
{
    MOTION_ACCEPT_OK    = 0, /**< Segment accepted, the engine is now busy. */
    MOTION_ACCEPT_BUSY  = 1, /**< A segment is already running. */
    MOTION_ACCEPT_RANGE = 2, /**< Parameter outside the allowed range. */
    MOTION_ACCEPT_FAULT = 3  /**< Latched fault - call MOTION_clear_faults(). */
} MOTION_ACCEPT;

/** @brief Motion states reported to the RPi. Do not reorder (protocol values). */
typedef enum
{
    MOTION_STATE_IDLE   = 0, /**< No segment running. */
    MOTION_STATE_DRIVE  = 1, /**< Straight-line move in progress. */
    MOTION_STATE_TURN   = 2, /**< In-place rotation in progress. */
    MOTION_STATE_STRAFE = 3, /**< Lateral move in progress (dead-reckoned). */
    MOTION_STATE_GOTO   = 4, /**< Turn-drive-turn manoeuvre in progress. */
    MOTION_STATE_FAULT  = 5, /**< Latched fault (any type except e-stop). */
    MOTION_STATE_ESTOP  = 6, /**< Emergency stop latched. */
    MOTION_STATE_ARC    = 7, /**< Circular arc move in progress. */
    MOTION_STATE_STREAM = 8  /**< Velocity-stream mode (fw >= 3.4). */
} MOTION_STATE;

/** @brief Latched fault bits reported to the RPi (QUERY_ODOMETRY/QUERY_STATUS).
 *  @note  16-bit mask; every latch also records a ROBOT_EVENT_* entry with
 *         context, so the service sees WHAT happened, not only "a fault". */
#define MOTION_FAULT_ESTOP   (0x0001U)
#define MOTION_FAULT_LIMIT   (0x0002U)
#define MOTION_FAULT_STALL   (0x0004U)
#define MOTION_FAULT_TIMEOUT (0x0008U)
#define MOTION_FAULT_DEADMAN (0x0010U)

/** @brief Limit switch bits (bit0 = SW1/PB10, bit1 = SW3/PA12, bit2 = SW4/PA11). */
#define MOTION_LIMIT_SW1     (0x01U)
#define MOTION_LIMIT_SW3     (0x02U)
#define MOTION_LIMIT_SW4     (0x04U)

/** @brief Pose and velocity snapshot, already converted to protocol units. */
typedef struct
{
    int32_t x_um;          /**< X position, micrometres (signed).              */
    int32_t y_um;          /**< Y position, micrometres (signed).              */
    int32_t theta_mdeg;    /**< Heading, millidegrees, CCW positive, +-180000. */
    int16_t v_left_mm_s;   /**< Left encoder speed, mm/s.                      */
    int16_t v_right_mm_s;  /**< Right encoder speed, mm/s.                     */
    uint8_t state;         /**< MOTION_STATE value.                            */
    uint16_t faults;       /**< MOTION_FAULT_* bit mask.                       */
    uint16_t segments_done; /**< Motion segments completed (wraps at 16 bits).  */
} MOTION_SNAPSHOT;

/* ------------------------------- Lifecycle --------------------------------- */

/** @brief   Initializes the motion engine state (call once at boot).
 *  @details Zeroes odometry, baselines the encoder counters and leaves the
 *           engine IDLE. Steppers and encoders must be initialized first. */
void MOTION_init(void);

/** @brief   Main-loop worker: executes the active segment, ramps the speeds,
 *           runs the supervision watchdogs and handles stop/e-stop requests.
 *  @note    Must be called as often as possible from the main loop; it never
 *           blocks (microseconds of work per call). */
void MOTION_task(void);

/** @brief   Odometry sampling and integration, called from the TIM7 ISR at
 *           exactly 100 Hz, after ENCODERS_MJ_poll(). */
void MOTION_odometry_tick(void);

/* --------------------------- Segment submission ----------------------------- */

/** @brief  Requests a straight-line move along the current heading.
 *  @param  distance_mm  Signed distance, |d| <= 3000 mm.
 *  @param  speed_mm_s   Cruise speed, clamped to [ROBOT_MIN_SPEED, max limit].
 *  @return MOTION_ACCEPT_OK when accepted (response is asynchronous: poll the
 *          state until IDLE), otherwise the rejection reason. */
MOTION_ACCEPT MOTION_accept_drive(float distance_mm, float speed_mm_s);

/** @brief  Requests an in-place rotation.
 *  @param  angle_rad    Signed rotation, CCW positive, |a| <= PI.
 *  @param  speed_mm_s   Cruise speed at full error.
 *  @return Acceptance result (see MOTION_accept_drive). */
MOTION_ACCEPT MOTION_accept_turn(float angle_rad, float speed_mm_s);

/** @brief  Requests a circular arc: forward motion and rotation together.
 *  @param  radius_mm    Radius of the robot-centre path; the sign selects
 *                       the arc side (+ = left/CCW, - = right/CW). Valid
 *                       range: ROBOT_ARC_MIN_RADIUS_MM .. 32767 mm.
 *  @param  sweep_rad    Signed heading change of the move; must have the
 *                       same sign as radius_mm. |sweep| <= 2*PI rad and the
 *                       resulting arc length must be 1..3000 mm.
 *  @param  speed_mm_s   Cruise speed of the robot centre; clamped to the
 *                       configured limits and reduced automatically on
 *                       small radii so the outer wheel stays within them.
 *  @return Acceptance result (see MOTION_accept_drive).
 *  @note   Closed loop on the odometry: the heading tracks the arc-length
 *          progress (slip-tolerant curvature adaptation) and the move
 *          completes when the swept angle is within +-0.8 deg while the arc
 *          length is done or overshot by at most the fail-safe slack. */
MOTION_ACCEPT MOTION_accept_arc(float radius_mm, float sweep_rad, float speed_mm_s);

/** @brief  Queues a continuation ARC for seamless chaining.
 *
 *  Same parameters and validation as MOTION_accept_arc, but tolerant of a
 *  running segment: when one is active, the arc is stored in a single
 *  pending slot and started the moment the active segment completes -
 *  WITHOUT stopping the wheels or resetting the speed ramp. The result is a
 *  tangent arc chain executed as one continuous move (single acceleration
 *  at the chain start, single deceleration at its end). When the engine is
 *  idle the arc is simply started at once; STOP / E-STOP / faults cancel
 *  the pending arc.
 *
 *  The caller must guarantee tangent continuity between the active segment
 *  and the queued arc (the RPi planner does); the board executes the arc
 *  from its CURRENT pose/heading, exactly like a normal ARC.
 *
 *  @return Acceptance result (see MOTION_accept_drive); BUSY means a
 *          pending arc already exists. */
MOTION_ACCEPT MOTION_queue_arc(float radius_mm, float sweep_rad, float speed_mm_s);

/** @brief  Requests a turn-drive-turn manoeuvre to an absolute pose.
 *  @param  x_mm       Target X, -3276.0 .. +3276.0 mm.
 *  @param  y_mm       Target Y, -3276.0 .. +3276.0 mm.
 *  @param  theta_rad  Final heading, normalized internally.
 *  @param  speed_mm_s Cruise speed of every phase.
 *  @return Acceptance result (see MOTION_accept_drive). */
MOTION_ACCEPT MOTION_accept_goto(float x_mm, float y_mm, float theta_rad, float speed_mm_s);

/** @brief  Requests a lateral (mecanum) translation, dead-reckoned.
 *  @param  distance_mm  Positive = to the robot's left, |d| <= 2000 mm.
 *  @param  speed_mm_s   Cruise speed.
 *  @return Acceptance result (see MOTION_accept_drive).
 *  @note   No lateral odometry exists (hardware); completion is timed. Keep
 *          the move short and re-localize afterwards. */
MOTION_ACCEPT MOTION_accept_strafe(float distance_mm, float speed_mm_s);

/** @brief  Applies one silent SET_VELOCITY stream frame (firmware >= 3.4).
 *
 *  The RPi streams velocity targets (``v_mm_s`` robot-centre speed, signed;
 *  ``omega_rad_s`` yaw rate, CCW positive) at 20-50 Hz. The first frame
 *  while IDLE enters the streaming mode (state STREAM); further frames
 *  replace the targets (latest-wins). While any OTHER segment runs the
 *  frame has NO effect (BUSY) and no response is published - stream frames
 *  are silent everywhere. The board ramps towards the targets with the
 *  configured accel/decel; when no frame arrives for
 *  ROBOT_STREAM_STALE_MS it ramps both targets to zero and returns to IDLE
 *  (controlled stop, never a fault/event). STOP / E-STOP / limit switches
 *  cancel the stream exactly like any segment.
 *
 *  @param  v_mm_s      Signed centre speed, clamped to the motion limits.
 *  @param  omega_rad_s Signed yaw rate, clamped to
 *                      +/-ROBOT_STREAM_MAX_OMEGA_RAD_S.
 *  @return MOTION_ACCEPT_OK (streaming started or targets updated),
 *          MOTION_ACCEPT_BUSY (another segment runs), MOTION_ACCEPT_FAULT
 *          (a fault is latched - clear it first). */
MOTION_ACCEPT MOTION_accept_stream(float v_mm_s, float omega_rad_s);

/* ------------------------- Gyro forward stream ------------------------------ */

/** @brief  Applies one silent SET_GYRO stream frame (firmware >= 3.3).
 *
 *  The RPi sends its MPU6050 yaw rate (rate, 0.1 deg/s units, CCW positive)
 *  and the age of the sample (10 ms units). The value drives the control
 *  heading (see the odometry tick) while frames keep arriving; after
 *  ROBOT_GYRO_STALE_MS without a frame the control heading falls back to the
 *  wheel odometry smoothly. The call is intentionally cheap and cannot fail:
 *  any value is clamped and no fault/event is ever recorded for stream
 *  frames - an absent or interrupted stream must leave the board running
 *  exactly like firmware 3.2.
 *
 *  @param  rate_deci_dps  Yaw rate in 0.1 deg/s units (i16), CCW positive.
 *  @param  age_10_ms      Sample age in 10 ms units (0 = just measured). */
void MOTION_set_gyro_rate(int16_t rate_deci_dps, uint8_t age_10_ms);

/** @brief  1 while a fresh gyro stream drives the control heading.
 *  @return 1 = stream fresh (used by QUERY_STATUS diagnostics). */
uint8_t MOTION_gyro_fresh(void);

/* ---------------------------- Pose management ------------------------------- */

/** @brief  Overwrites the odometry pose (robot idle only).
 *  @param  x_mm      New X position in mm.
 *  @param  y_mm      New Y position in mm.
 *  @param  theta_rad New heading, normalized internally. */
void MOTION_set_pose(float x_mm, float y_mm, float theta_rad);

/** @brief  Sets the pose to (0, 0, 0) and re-zeros both encoder counters. */
void MOTION_reset_odometry(void);

/* ------------------------------- Control ------------------------------------ */

/** @brief  Updates the runtime motion limits (RAM only, lost on reset).
 *  @param  max_speed_mm_s  Clamped to [50, ROBOT_MAX_SPEED_LIMIT_MM_S].
 *  @param  accel_mm_s2     Clamped to [50, 2000].
 *  @param  decel_mm_s2     Clamped to [50, 2000].
 *  @note   Normal callers go through the settings store (robot_settings.c),
 *          which validates the same ranges and can persist them to flash. */
void MOTION_set_limits(float max_speed_mm_s, float accel_mm_s2, float decel_mm_s2);

/** @brief  Enables/updates the command-channel deadman watchdog.
 *  @param  timeout_ms  Timeout in ms; 0 disables the watchdog. */
void MOTION_set_deadman_ms(uint16_t timeout_ms);

/** @brief  Refreshes the deadman timer; call for every valid request received. */
void MOTION_notify_request(uint32_t now_ms);

/** @brief  Checks the deadman timer; trips MOTION_FAULT_DEADMAN when expired
 *          while a segment is running. Called from the protocol task. */
void MOTION_check_deadman(uint32_t now_ms);

/* -------------------------------- Safety ------------------------------------ */

/** @brief  ISR-safe request for a controlled stop (pulses stop, odometry and
 *          holding torque are preserved). Executed by MOTION_task(). */
void MOTION_request_stop(void);

/** @brief  ISR-safe request for an emergency stop. Executed by MOTION_task()
 *          on its next pass (microseconds later): pulses stop, drivers are
 *          disabled and MOTION_FAULT_ESTOP is latched. */
void MOTION_request_estop(void);

/** @brief  EXTI ISR entry point for the three wall limit switches: the ISR
 *          stops the pulses and disables the drivers immediately, records the
 *          switch bit and flags the fault; MOTION_task() latches
 *          MOTION_FAULT_LIMIT (single writer of the engine state) on its
 *          next pass, a few microseconds later.
 *  @param  gpio_pin  GPIO_Pin of the triggered switch (SW1/SW3/SW4). */
void MOTION_on_limit_switch(uint16_t gpio_pin);

/** @brief  Releases the stepper drivers (EN high) without latching a fault.
 *
 *          Safety contract: accepted only while no segment is active. The
 *          current pose and odometry are preserved; the next motion command
 *          re-energizes the drivers automatically. An active segment returns
 *          MOTION_ACCEPT_BUSY, a latched fault returns MOTION_ACCEPT_FAULT.
 *
 *  @return Acceptance result (OK when the drivers are now released). */
MOTION_ACCEPT MOTION_release_motors(void);

/** @brief  Clears all latched faults and the limit-switch status; motion is
 *          allowed again. Drivers stay disabled until the next command. */
void MOTION_clear_faults(void);

/** @brief  Latches an arbitrary fault (used by the protocol layer for the
 *          deadman trip and future extensions).
 *  @param  fault_bit  One of the MOTION_FAULT_* bits. */
void MOTION_force_fault(uint16_t fault_bit);

/* --------------------------------- Status ----------------------------------- */

/** @brief  @return 1 when a segment is currently active, otherwise 0. */
uint8_t MOTION_is_busy(void);

/** @brief  @return Current MOTION_STATE value. */
uint8_t MOTION_get_state(void);

/** @brief  @return Current latched MOTION_FAULT_* bit mask. */
uint16_t MOTION_get_faults(void);

/** @brief  @return Limit-switch status bits latched since the last clear. */
uint8_t MOTION_get_limit_status(void);

/** @brief  Copies a coherent pose/velocity/state snapshot.
 *  @param  snapshot  Destination structure (ignored when NULL). */
void MOTION_get_snapshot(MOTION_SNAPSHOT *snapshot);

#endif /* MOTION_H */
