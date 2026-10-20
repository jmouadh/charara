/**
 ******************************************************************************
 * @file    motion.c
 * @brief   Non-blocking motion engine implementation.
 *
 * Sign convention (same as Docs/04): +x forward, +y left, theta CCW
 * positive; a positive TURN angle therefore rotates the robot to its left.
 *
 * Controller summary (all values in mm, mm/s, rad):
 *   DRIVE : closed-loop on odometry distance, trapezoidal ramp, brake profile
 *           v = min(cruise, sqrt(2*decel*remaining)), heading hold with a
 *           proportional left/right differential.
 *   TURN  : closed-loop on odometry heading, P law  v = Kp * |arc|,
 *           arc = err * track/2, finish when |err| <= tolerance.
 *   ARC   : forward motion and rotation together along a signed radius;
 *           closed loop on the odometry sweep and arc length, brake on the
 *           tighter of the two remaining measures, omega = v / radius.
 *   STRAFE: timed open loop (no lateral odometry available - documented).
 *   GOTO  : [turn to heading] -> [drive] -> [turn to final heading], each
 *           phase closed-loop and timeout/stall supervised.
 *
 * MISRA C:2012 orientation: no dynamic memory, no recursion, all loops
 * bounded, explicit casts, initialized objects, single exit style where
 * practical, every switch has a default.
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "motion.h"
#include "main.h"
#include "robot_port.h"
#include "robot_configs.h"
#include "robot_settings.h"
#include "robot_events.h"
#include "STEPPERS_MouadhJeridi.h"
#include "ENCODERS_MouadhJeridi.h"

/* ------------------------------ internal types ------------------------------ */
#define SEG_NONE   (0U)
#define SEG_DRIVE  (1U)
#define SEG_TURN   (2U)
#define SEG_ARC    (3U)
#define SEG_STRAFE (4U)
#define SEG_GOTO   (5U)
#define SEG_STREAM (6U)

#define GOTO_PHASE_TURN_HEADING (0U)
#define GOTO_PHASE_DRIVE        (1U)
#define GOTO_PHASE_TURN_FINAL   (2U)

/* ---------------------------- module state ---------------------------------- */
static volatile uint8_t s_stop_req;
static volatile uint8_t s_estop_req;
static volatile uint8_t s_limit_req;   /* set by the limit-switch EXTI ISR */

static float s_x_mm;
static float s_y_mm;
static float s_theta_rad;
static float s_travel_mm;              /* signed cumulative centre distance */
static int32_t s_prev_ticks_left;
static int32_t s_prev_ticks_right;

static volatile uint8_t s_state;
static volatile uint16_t s_faults;
static volatile uint8_t s_limit_status;

/* Live settings (geometry, gains, limits): loaded from the flash-backed
   settings store; the struct is updated in place by config writes, so a
   pointer is enough - no copies to keep in sync. */
static const ROBOT_SETTINGS *s_cfg;

static volatile uint8_t s_seg;
static uint8_t s_goto_phase;
static uint8_t s_outputs_started;
static float s_seg_dist_mm;
static float s_seg_turn_rad;
static float s_seg_head_rad;
static float s_seg_speed;
static float s_travel_at_start;
static float s_v_ramp;
static float s_arc_radius_mm;           /* signed: + = left (CCW) arc      */
static float s_arc_sweep_rad;           /* signed: + = CCW heading change  */
static float s_arc_len_mm;              /* |sweep| * |radius| [mm]         */
static volatile float s_arc_theta_acc;  /* heading change since arc start  */
static float s_arc_kappa_cmd;           /* signed commanded curvature 1/R  */
static float s_arc_kappa_iterm;         /* integral term of the adaptation */
static uint8_t s_pend_arc_valid;        /* queued continuation ARC waiting */
static float s_pend_arc_radius;
static float s_pend_arc_sweep;
static float s_pend_arc_cruise;
static float s_v_left_applied;
static float s_v_right_applied;
static int8_t s_hw_dir_left;
static int8_t s_hw_dir_right;
static uint32_t s_dir_settle_until_ms;
static uint32_t s_seg_deadline_ms;
static uint32_t s_last_task_ms;
static uint32_t s_dt_ms;

static float s_stall_ref_progress;
static uint32_t s_stall_ref_ms;
static uint16_t s_segments_done;        /* completed motion segments (odometry) */

/* Velocity stream state (SET_VELOCITY, fw 3.4): targets, ramped values and
   the freshness stamp. A stale stamp ramps everything to zero - a
   controlled stop, never a fault (see motion_do_stream). */
static float s_stream_v_target;         /* mm/s, signed, robot centre      */
static float s_stream_omega_target;     /* rad/s, CCW positive             */
static float s_stream_v_ramp;
static float s_stream_omega_ramp;
static uint32_t s_stream_stamp_ms;      /* last accepted frame (tick ms)   */
static uint8_t s_stream_stopping;       /* stale-stop in progress          */

static float s_max_speed = ROBOT_MAX_SPEED_MM_S;
static float s_accel = ROBOT_DEFAULT_ACCEL_MM_S2;
static float s_decel = ROBOT_DEFAULT_DECEL_MM_S2;
static uint16_t s_deadman_ms = ROBOT_DEADMAN_DEFAULT_MS;
static uint32_t s_last_request_ms;

/* Gyro forward stream state (silent SET_GYRO frames): the control heading
   blends the streamed yaw rate with the wheel odometry (see the odometry
   tick); a stale or absent stream collapses to the wheel odometry - exactly
   the firmware 3.2 behaviour - with no fault and no event. */
static volatile float s_gyro_rate_rad_s;
static volatile uint32_t s_gyro_stamp_ms;
static volatile uint8_t s_gyro_valid;
static float s_theta_ctrl_rad;          /* blended control heading         */
static float s_prev_theta_ctrl_rad;     /* previous tick value (arc gain)  */

/* Strafe wheel table: order FL, FR, RL, RR (the +Y pattern itself is a
   calibrated setting: ROBOT_CFG_IDX_STRAFE_PATTERN). */
static const uint8_t s_strafe_ids[4] =
{
    ROBOT_STEPPER_ID_FL, ROBOT_STEPPER_ID_FR, ROBOT_STEPPER_ID_RL, ROBOT_STEPPER_ID_RR
};

/* ------------------------------- helpers ------------------------------------ */
/** @brief Absolute value for float (avoids the fabs() library dependency). */
static float motion_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/** @brief Sign of a float as -1 or +1 (+1 for zero). */
static int8_t motion_signf(float value)
{
    return (value >= 0.0f) ? (int8_t)1 : (int8_t)-1;
}

/** @brief Clamps a float into [lower, upper]. */
static float motion_clampf(float value, float lower, float upper)
{
    float result = value;
    if (result < lower)
    {
        result = lower;
    }
    if (result > upper)
    {
        result = upper;
    }
    return result;
}

/** @brief Normalizes an angle to the range [-PI, +PI). */
static float motion_normalize_angle(float angle)
{
    float wrapped = fmodf(angle + ROBOT_PI, ROBOT_TWO_PI);
    if (wrapped < 0.0f)
    {
        wrapped += ROBOT_TWO_PI;
    }
    return wrapped - ROBOT_PI;
}

/** @brief Progress metric valid for translation and in-place rotation:
 *         centre travel plus heading arc (theta * track/2). */
static float motion_progress(void)
{
    /* Progress metric valid for both translation and in-place rotation. */
    return s_travel_mm + (s_theta_rad * s_cfg->robot_track_mm * 0.5f);
}

/** @brief 1 while the forwarded gyro stream is fresh (never fails).
 *  @note  Stream frames are silent; the freshness window also covers the
 *         sender-reported age, so a delayed frame can already be stale on
 *         arrival. Stale simply means "wheel heading", never a fault. */
static uint8_t motion_gyro_is_fresh(uint32_t now_ms)
{
    return ((s_gyro_valid != 0U) &&
            ((uint32_t)(now_ms - s_gyro_stamp_ms) <= ROBOT_GYRO_STALE_MS)) ? 1U : 0U;
}

static void motion_stop_outputs(void);
static void motion_disable_drivers(void);
static void motion_set_fault(uint16_t fault_bit);
static void motion_finish_segment(void);
static void motion_ramp_to(float target, uint32_t dt_ms);
static void motion_apply_speeds(float v_left, float v_right, uint32_t now_ms);
static uint8_t motion_do_turn(float target_rad, float cruise, uint32_t now_ms);
static uint8_t motion_do_drive(float target_dist_mm, float head_rad, float cruise, uint32_t now_ms);
static uint8_t motion_do_arc(float cruise, uint32_t now_ms);
static void motion_supervise_stall(uint32_t now_ms);
static void motion_begin_segment(uint8_t seg, float dist_mm, float turn_rad, float head_rad,
                                 float cruise, uint32_t now_ms, float nominal_ms);
static void motion_arc_begin(float radius_mm, float sweep_rad, float cruise, uint32_t now_ms,
                             uint8_t continuation);
static void motion_strafe_setup(float distance_mm, float speed_mm_s, uint32_t now_ms);
static void motion_run_strafe(uint32_t now_ms);
static void motion_run_goto(uint32_t now_ms);
static uint8_t motion_do_stream(uint32_t now_ms);

/* ---------------------------- low-level actions ----------------------------- */
/** @brief Stops all step channels and resets the ramp/speed bookkeeping. */
static void motion_stop_outputs(void)
{
    STEPPERS_MJ_stop_all();
    s_v_ramp = 0.0f;
    s_v_left_applied = 0.0f;
    s_v_right_applied = 0.0f;
    s_outputs_started = 0U;
}

/** @brief Disables both motor driver groups (wheels free). */
static void motion_disable_drivers(void)
{
    STEPPERS_MJ_disable_left_steppers();
    STEPPERS_MJ_disable_right_steppers();
}

/** @brief Maps an internal segment code to the published motion state. */
static uint8_t motion_state_for_seg(uint8_t seg)
{
    uint8_t state;

    switch (seg)
    {
        case SEG_DRIVE:
            state = MOTION_STATE_DRIVE;
            break;
        case SEG_TURN:
            state = MOTION_STATE_TURN;
            break;
        case SEG_STRAFE:
            state = MOTION_STATE_STRAFE;
            break;
        case SEG_ARC:
            state = MOTION_STATE_ARC;
            break;
        case SEG_GOTO:
            state = MOTION_STATE_GOTO;
            break;
        case SEG_STREAM:
            state = MOTION_STATE_STREAM;
            break;
        default:
            state = MOTION_STATE_IDLE;
            break;
    }
    return state;
}

/** @brief Latches a fault: stop pulses, disable drivers, publish the state.
 *  @note  Every latch is also recorded in the error event log with context
 *         (limit mask, stalled/timed-out segment state), so the RPi sees
 *         exactly what failed. */
static void motion_set_fault(uint16_t fault_bit)
{
    uint8_t context = motion_state_for_seg(s_seg);

    motion_stop_outputs();
    motion_disable_drivers();
    s_faults = (uint16_t)(s_faults | fault_bit);
    s_state = ((fault_bit & MOTION_FAULT_ESTOP) != 0U) ? MOTION_STATE_ESTOP : MOTION_STATE_FAULT;
    s_seg = SEG_NONE;
    s_pend_arc_valid = 0U;   /* a fault cancels any queued continuation ARC */
    s_stream_stopping = 0U;  /* and any velocity stream */

    switch (fault_bit)
    {
        case MOTION_FAULT_ESTOP:
            ROBOT_EVENTS_record(ROBOT_EVENT_ESTOP, 0U);
            break;
        case MOTION_FAULT_LIMIT:
            ROBOT_EVENTS_record(ROBOT_EVENT_LIMIT, s_limit_status);
            break;
        case MOTION_FAULT_STALL:
            ROBOT_EVENTS_record(ROBOT_EVENT_STALL, context);
            break;
        case MOTION_FAULT_TIMEOUT:
            ROBOT_EVENTS_record(ROBOT_EVENT_TIMEOUT, context);
            break;
        case MOTION_FAULT_DEADMAN:
            ROBOT_EVENTS_record(ROBOT_EVENT_DEADMAN, 0U);
            break;
        default:
            /* a future mask bit: latched without a specific event */
            break;
    }
}

/** @brief Completes the active segment normally and returns to IDLE.
 *         A queued continuation ARC (QUEUE_ARC while busy) is started
 *         instead: the wheels keep rolling, so a tangent arc chain runs as
 *         ONE continuous move with a single acceleration and deceleration. */
static void motion_finish_segment(void)
{
    s_segments_done++;   /* feeds the odometry snapshot's chaining counter */
    if (s_pend_arc_valid != 0U)
    {
        float radius = s_pend_arc_radius;
        float sweep = s_pend_arc_sweep;
        float cruise = s_pend_arc_cruise;
        s_pend_arc_valid = 0U;
        motion_arc_begin(radius, sweep, cruise, ROBOT_PORT_tick_ms(), 1U);
        return;
    }

    STEPPERS_MJ_stop_all();
    s_v_ramp = 0.0f;
    s_v_left_applied = 0.0f;
    s_v_right_applied = 0.0f;
    s_outputs_started = 0U;
    s_seg = SEG_NONE;
    s_state = MOTION_STATE_IDLE;
}

/** @brief Prepares a new segment: resets ramps and directions, computes the
 *         deadline from the nominal duration and publishes the state. */
static void motion_begin_segment(uint8_t seg, float dist_mm, float turn_rad, float head_rad,
                                 float cruise, uint32_t now_ms, float nominal_ms)
{
    STEPPERS_MJ_stop_all();
    s_seg = seg;
    s_goto_phase = GOTO_PHASE_TURN_HEADING;
    s_seg_dist_mm = dist_mm;
    s_seg_turn_rad = turn_rad;
    s_seg_head_rad = head_rad;
    s_seg_speed = cruise;
    s_travel_at_start = s_travel_mm;
    s_v_ramp = 0.0f;
    s_v_left_applied = 0.0f;
    s_v_right_applied = 0.0f;
    s_hw_dir_left = (int8_t)0;
    s_hw_dir_right = (int8_t)0;
    s_outputs_started = 0U;
    s_dir_settle_until_ms = now_ms + ROBOT_DIR_SETTLE_MS;
    s_seg_deadline_ms = now_ms + (uint32_t)(nominal_ms * ROBOT_SEGMENT_NOMINAL_FACTOR)
                        + ROBOT_SEGMENT_TIMEOUT_MARGIN_MS;
    s_stall_ref_progress = motion_progress();
    s_stall_ref_ms = now_ms;
    switch (seg)
    {
        case SEG_DRIVE:
            s_state = MOTION_STATE_DRIVE;
            break;
        case SEG_TURN:
            s_state = MOTION_STATE_TURN;
            break;
        case SEG_STRAFE:
            s_state = MOTION_STATE_STRAFE;
            break;
        case SEG_ARC:
            s_state = MOTION_STATE_ARC;
            break;
        default:
            s_state = MOTION_STATE_GOTO;
            break;
    }
}

/** @brief Prepares/restarts an ARC segment (normal start or seamless
 *         continuation right after the previous segment).
 *
 *  ``continuation`` != 0 is used when a queued ARC is consumed by
 *  motion_finish_segment(): the outputs keep running and the ramp is NOT
 *  reset, so a tangent arc chain is executed as one continuous move (single
 *  acceleration at the chain start, single deceleration at its end). The
 *  ramp only re-targets the new cruise through the normal limits. */
static void motion_arc_begin(float radius_mm, float sweep_rad, float cruise, uint32_t now_ms,
                             uint8_t continuation)
{
    float arc_len = motion_absf(sweep_rad) * motion_absf(radius_mm);

    s_arc_radius_mm = radius_mm;
    s_arc_sweep_rad = sweep_rad;
    s_arc_len_mm = arc_len;
    s_arc_theta_acc = 0.0f;
    s_arc_kappa_cmd = 1.0f / radius_mm;
    s_arc_kappa_iterm = 0.0f;

    if (continuation == 0U)
    {
        motion_begin_segment(SEG_ARC, 0.0f, 0.0f, s_theta_ctrl_rad, cruise, now_ms,
                             (arc_len / cruise) * 1000.0f);
        return;
    }

    s_seg = SEG_ARC;
    s_state = MOTION_STATE_ARC;
    s_seg_dist_mm = 0.0f;
    s_seg_turn_rad = 0.0f;
    s_seg_head_rad = s_theta_ctrl_rad;
    s_seg_speed = cruise;
    s_travel_at_start = s_travel_mm;
    s_seg_deadline_ms = now_ms
                        + (uint32_t)((arc_len / cruise) * 1000.0f * ROBOT_SEGMENT_NOMINAL_FACTOR)
                        + ROBOT_SEGMENT_TIMEOUT_MARGIN_MS;
    s_stall_ref_progress = motion_progress();
    s_stall_ref_ms = now_ms;
}

/* --------------------------------- ramp ------------------------------------- */
/** @brief Ramps the applied speed toward the target within accel/decel limits. */
/** @brief Rate-limits one scalar towards a target [units/s].
 *  @param  value  In/out value to ramp.
 *  @param  target Target value.
 *  @param  dt_ms  Elapsed time since the last call [ms].
 *  @param  up     Acceleration limit used while the sign is preserved.
 *  @param  down   Deceleration limit used when the sign changes. */
static void motion_ramp_scalar(float *value, float target, uint32_t dt_ms, float up, float down)
{
    float dt_s = (float)dt_ms * 0.001f;
    float delta = target - *value;
    float same_sign = (((target >= 0.0f) && (*value >= 0.0f)) || ((target < 0.0f) && (*value < 0.0f))) ? 1.0f : 0.0f;
    float limit = (same_sign > 0.0f) ? (up * dt_s) : (down * dt_s);

    if (delta > limit)
    {
        delta = limit;
    }
    else if (delta < -limit)
    {
        delta = -limit;
    }
    else
    {
        /* ramp step within limits */
    }
    *value += delta;
}

/** @brief Ramps the centre speed of the active segment (trapezoid profile). */
static void motion_ramp_to(float target, uint32_t dt_ms)
{
    motion_ramp_scalar(&s_v_ramp, target, dt_ms, s_accel, s_decel);
}

/* --------------------------- output application ----------------------------- */
/** @brief Applies left/right speeds: DIR handling with settle time, driver
 *         enable/start and rate-limited ARR updates. */
static void motion_apply_speeds(float v_left, float v_right, uint32_t now_ms)
{
    int8_t dir_left = motion_signf(v_left);
    int8_t dir_right = motion_signf(v_right);

    /* A stop / e-stop / limit request may have arrived while this function
       was running (ISR preemption): never (re)start the outputs while one is
       pending - MOTION_task() consumes them first thing. */
    if ((s_faults != 0U) || (s_estop_req != 0U) || (s_stop_req != 0U) || (s_limit_req != 0U))
    {
        return;
    }

    if ((dir_left != s_hw_dir_left) || (dir_right != s_hw_dir_right))
    {
        STEPPERS_MJ_stop_all();
        s_v_left_applied = 0.0f;
        s_v_right_applied = 0.0f;
        (void)STEPPERS_MJ_set_left_direction((dir_left > 0) ? FORWARD : BACKWARD);
        (void)STEPPERS_MJ_set_right_direction((dir_right > 0) ? FORWARD : BACKWARD);
        s_hw_dir_left = dir_left;
        s_hw_dir_right = dir_right;
        s_dir_settle_until_ms = now_ms + ROBOT_DIR_SETTLE_MS;
        s_outputs_started = 0U;
        return;
    }

    if (s_outputs_started == 0U)
    {
        if ((int32_t)(now_ms - s_dir_settle_until_ms) < 0)
        {
            return;
        }
        STEPPERS_MJ_enable_left_steppers();
        STEPPERS_MJ_enable_right_steppers();
        STEPPERS_MJ_start_left();
        STEPPERS_MJ_start_right();
        s_outputs_started = 1U;
    }

    if (motion_absf(motion_absf(v_left) - s_v_left_applied) >= ROBOT_SPEED_EPS_MM_S)
    {
        (void)STEPPERS_MJ_set_left_speed_mm_s(motion_absf(v_left));
        s_v_left_applied = motion_absf(v_left);
    }
    if (motion_absf(motion_absf(v_right) - s_v_right_applied) >= ROBOT_SPEED_EPS_MM_S)
    {
        (void)STEPPERS_MJ_set_right_speed_mm_s(motion_absf(v_right));
        s_v_right_applied = motion_absf(v_right);
    }
}

/* ------------------------------ controllers --------------------------------- */
/** @brief DRIVE controller: brake profile, ramp and heading hold.
 *  @return 1 when the target is reached, 0 while still running. */
static uint8_t motion_do_drive(float target_dist_mm, float head_rad, float cruise, uint32_t now_ms)
{
    float progress = s_travel_mm - s_travel_at_start;
    float remaining = target_dist_mm - progress;
    float v_des;
    float v_brake;
    float eh;
    float dv;
    float v_left;
    float v_right;

    if ((motion_absf(remaining) <= ROBOT_DRIVE_TOL_MM) || ((remaining * target_dist_mm) < 0.0f))
    {
        return 1U;
    }

    v_des = cruise;
    v_brake = sqrtf(2.0f * s_decel * motion_absf(remaining));
    if (v_brake < v_des)
    {
        v_des = v_brake;
    }
    if (v_des < s_cfg->min_speed_mm_s)
    {
        v_des = s_cfg->min_speed_mm_s;
    }
    if (remaining < 0.0f)
    {
        v_des = -v_des;
    }

    motion_ramp_to(v_des, s_dt_ms);

    eh = motion_normalize_angle(head_rad - s_theta_ctrl_rad);
    dv = s_cfg->heading_gain * eh * (s_cfg->robot_track_mm * 0.5f);
    if (dv > (0.5f * cruise))
    {
        dv = 0.5f * cruise;
    }
    else if (dv < (-0.5f * cruise))
    {
        dv = -0.5f * cruise;
    }
    else
    {
        /* within bounds */
    }

    v_left = s_v_ramp - dv;
    v_right = s_v_ramp + dv;

    if (motion_absf(v_left) < ROBOT_MIN_CONTROL_SPEED_MM_S)
    {
        v_left = (s_v_ramp >= 0.0f) ? ROBOT_MIN_CONTROL_SPEED_MM_S : -ROBOT_MIN_CONTROL_SPEED_MM_S;
    }
    if (motion_absf(v_right) < ROBOT_MIN_CONTROL_SPEED_MM_S)
    {
        v_right = (s_v_ramp >= 0.0f) ? ROBOT_MIN_CONTROL_SPEED_MM_S : -ROBOT_MIN_CONTROL_SPEED_MM_S;
    }
    v_left = motion_clampf(v_left, -s_max_speed, s_max_speed);
    v_right = motion_clampf(v_right, -s_max_speed, s_max_speed);

    motion_apply_speeds(v_left, v_right, now_ms);
    motion_supervise_stall(now_ms);
    return 0U;
}

/** @brief TURN controller: P law on the heading arc, opposite side speeds.
 *  @return 1 when the target heading is reached, 0 while still turning. */
static uint8_t motion_do_turn(float target_rad, float cruise, uint32_t now_ms)
{
    float err = motion_normalize_angle(target_rad - s_theta_ctrl_rad);
    float arc;
    float v;
    float v_des;

    if (motion_absf(err) <= ROBOT_TURN_TOL_RAD)
    {
        return 1U;
    }

    arc = err * (s_cfg->robot_track_mm * 0.5f);
    v = s_cfg->turn_gain * motion_absf(arc);
    if (v > cruise)
    {
        v = cruise;
    }
    if (v < s_cfg->min_speed_mm_s)
    {
        v = s_cfg->min_speed_mm_s;
    }
    v_des = (err >= 0.0f) ? v : -v;

    motion_ramp_to(v_des, s_dt_ms);
    /* Positive ramp = CCW turn = left wheel backward, right wheel forward. */
    motion_apply_speeds(-s_v_ramp, s_v_ramp, now_ms);
    motion_supervise_stall(now_ms);
    return 0U;
}

/** @brief ARC controller: forward motion and rotation together along a
 *         signed radius, with slip-tolerant curvature tracking.
 *
 *  The heading must follow th_ref(s) = sweep * s / L (s = measured arc
 *  length) so that on the real floor - where the body yaws less than the
 *  commanded wheel differential (mecanum/skid scrub) - the sweep and the
 *  arc length still close together. A P+I term adapts the curvature 1/R:
 *  the P term reacts within one odometry period, the I term removes the
 *  residual (see the constants in robot_configs.h).
 *
 *  Completion:
 *    - normal: arc length and swept angle both within tolerance;
 *    - fail-safe: the length overshot by ROBOT_ARC_SLACK_MM (the old
 *      dual-window rule could never converge when the yaw/length ratio
 *      differed by more than ~2%, so the arc orbited until the segment
 *      timeout fired - a real-robot ARC must never do that).
 *  @return 1 when the arc is complete, 0 while still running. */
static uint8_t motion_do_arc(float cruise, uint32_t now_ms)
{
    float progress = s_travel_mm - s_travel_at_start;
    float s_rem = s_arc_len_mm - progress;
    float th_rem = s_arc_sweep_rad - s_arc_theta_acc;
    float v_brake_s;
    float v_brake_th;
    float v_des;
    float frac;
    float th_ref;
    float eh;
    float kappa;
    float k_lo;
    float k_hi;
    float omega;
    float v_left;
    float v_right;

    if (((s_rem <= ROBOT_DRIVE_TOL_MM) && (motion_absf(th_rem) <= ROBOT_TURN_TOL_RAD))
        || (s_rem <= -ROBOT_ARC_SLACK_MM))
    {
        return 1U;
    }

    /* Brake on the tighter of the two remaining measures, both expressed in
       mm of arc: the remaining arc length and the remaining sweep. */
    v_brake_s = sqrtf(2.0f * s_decel * motion_absf(s_rem));
    v_brake_th = sqrtf(2.0f * s_decel * motion_absf(th_rem) * motion_absf(s_arc_radius_mm));
    v_des = (v_brake_s < v_brake_th) ? v_brake_s : v_brake_th;
    if (v_des > cruise)
    {
        v_des = cruise;
    }
    if (v_des < s_cfg->min_speed_mm_s)
    {
        v_des = s_cfg->min_speed_mm_s;
    }

    motion_ramp_to(v_des, s_dt_ms);

    /* Slip-tolerant curvature: track the heading profile of the arc. */
    frac = (s_arc_len_mm > 0.001f) ? (progress / s_arc_len_mm) : 1.0f;
    if (frac < 0.0f)
    {
        frac = 0.0f;
    }
    if (frac > 1.0f)
    {
        frac = 1.0f;
    }
    th_ref = s_arc_sweep_rad * frac;
    eh = th_ref - s_arc_theta_acc;
    s_arc_kappa_iterm += ROBOT_ARC_ADAPT_I * eh * ((float)s_dt_ms * 0.001f);
    kappa = s_arc_kappa_cmd
            + (ROBOT_ARC_ADAPT_P * motion_absf(s_arc_kappa_cmd) * eh)
            + s_arc_kappa_iterm;
    k_lo = s_arc_kappa_cmd * ROBOT_ARC_ADAPT_MAX_SCALE;
    k_hi = s_arc_kappa_cmd / ROBOT_ARC_ADAPT_MAX_SCALE;
    kappa = motion_clampf(kappa, (k_lo < k_hi) ? k_lo : k_hi, (k_lo < k_hi) ? k_hi : k_lo);

    /* Differential mixing: omega = v * curvature (the signed curvature
       selects the rotation direction), v_left/right = v -/+ omega * track/2. */
    omega = s_v_ramp * kappa;
    v_left = s_v_ramp - (omega * (s_cfg->robot_track_mm * 0.5f));
    v_right = s_v_ramp + (omega * (s_cfg->robot_track_mm * 0.5f));
    v_left = motion_clampf(v_left, -s_max_speed, s_max_speed);
    v_right = motion_clampf(v_right, -s_max_speed, s_max_speed);

    motion_apply_speeds(v_left, v_right, now_ms);
    motion_supervise_stall(now_ms);
    return 0U;
}

/** @brief Stall watchdog: trips MOTION_FAULT_STALL when no motion progress
 *         is observed for ROBOT_STALL_WINDOW_MS. */
static void motion_supervise_stall(uint32_t now_ms)
{
    float progress = motion_progress();

    if (motion_absf(progress - s_stall_ref_progress) >= ROBOT_STALL_MIN_DELTA_MM)
    {
        s_stall_ref_progress = progress;
        s_stall_ref_ms = now_ms;
    }
    else if ((now_ms - s_stall_ref_ms) > ROBOT_STALL_WINDOW_MS)
    {
        motion_set_fault(MOTION_FAULT_STALL);
    }
    else
    {
        /* still moving (or within window) */
    }
}

/* ------------------------------- strafe ------------------------------------- */
/** @brief Configures the four wheels for a strafe move (directions + speeds). */
static void motion_strafe_setup(float distance_mm, float speed_mm_s, uint32_t now_ms)
{
    uint8_t wheel;
    uint8_t forward_pattern;
    STEPPERS_MJ_DIRECTIONS direction;

    STEPPERS_MJ_stop_all();
    for (wheel = 0U; wheel < 4U; wheel++)
    {
        forward_pattern = (uint8_t)((s_cfg->strafe_pattern >> wheel) & 0x1U);
        if (distance_mm < 0.0f)
        {
            forward_pattern = (forward_pattern == 0U) ? 1U : 0U; /* invert for -Y */
        }
        direction = (forward_pattern != 0U) ? FORWARD : BACKWARD;
        (void)STEPPERS_MJ_set_direction(s_strafe_ids[wheel], direction);
        (void)STEPPERS_MJ_set_speed_mm_s(s_strafe_ids[wheel], speed_mm_s);
    }
    s_hw_dir_left = (int8_t)0;
    s_hw_dir_right = (int8_t)0;
    s_dir_settle_until_ms = now_ms + ROBOT_DIR_SETTLE_MS;
    s_outputs_started = 0U;
}

/** @brief STRAFE run loop: starts the wheels after the settle delay and stops
 *         them at the timed deadline (no lateral feedback exists). */
static void motion_run_strafe(uint32_t now_ms)
{
    if ((int32_t)(now_ms - s_seg_deadline_ms) >= 0)
    {
        uint8_t wheel;
        for (wheel = 0U; wheel < 4U; wheel++)
        {
            STEPPERS_MJ_stop(s_strafe_ids[wheel]);
        }
        motion_finish_segment();
        return;
    }

    if (s_outputs_started == 0U)
    {
        uint8_t wheel;
        if ((int32_t)(now_ms - s_dir_settle_until_ms) < 0)
        {
            return;
        }
        if ((s_faults != 0U) || (s_estop_req != 0U) || (s_stop_req != 0U) || (s_limit_req != 0U))
        {
            return; /* pending stop/fault request: keep the outputs off */
        }
        for (wheel = 0U; wheel < 4U; wheel++)
        {
            STEPPERS_MJ_enable_stepper(s_strafe_ids[wheel]);
            STEPPERS_MJ_start(s_strafe_ids[wheel]);
        }
        s_outputs_started = 1U;
    }
}

/* -------------------------------- public API -------------------------------- */
/** @brief Initializes the motion engine: zeroes odometry and baselines the
 *         encoders (call once after the drivers are initialized). */
void MOTION_init(void)
{
    s_cfg = ROBOT_SETTINGS_get();
    s_stop_req = 0U;
    s_estop_req = 0U;
    s_x_mm = 0.0f;
    s_y_mm = 0.0f;
    s_theta_rad = 0.0f;
    s_theta_ctrl_rad = 0.0f;
    s_prev_theta_ctrl_rad = 0.0f;
    s_gyro_rate_rad_s = 0.0f;
    s_gyro_stamp_ms = 0U;
    s_gyro_valid = 0U;
    s_travel_mm = 0.0f;
    s_prev_ticks_left = ENCODERS_MJ_get_absolut_ticks(ROBOT_ENCODER_ID_LEFT);
    s_prev_ticks_right = ENCODERS_MJ_get_absolut_ticks(ROBOT_ENCODER_ID_RIGHT);
    s_state = MOTION_STATE_IDLE;
    s_faults = 0U;
    s_limit_status = 0U;
    s_seg = SEG_NONE;
    s_goto_phase = GOTO_PHASE_TURN_HEADING;
    s_outputs_started = 0U;
    s_seg_dist_mm = 0.0f;
    s_seg_turn_rad = 0.0f;
    s_seg_head_rad = 0.0f;
    s_seg_speed = 0.0f;
    s_travel_at_start = 0.0f;
    s_v_ramp = 0.0f;
    s_v_left_applied = 0.0f;
    s_v_right_applied = 0.0f;
    s_arc_radius_mm = 0.0f;
    s_arc_sweep_rad = 0.0f;
    s_arc_len_mm = 0.0f;
    s_arc_theta_acc = 0.0f;
    s_arc_kappa_cmd = 0.0f;
    s_arc_kappa_iterm = 0.0f;
    s_pend_arc_valid = 0U;
    s_pend_arc_radius = 0.0f;
    s_pend_arc_sweep = 0.0f;
    s_pend_arc_cruise = 0.0f;
    s_stream_v_target = 0.0f;
    s_stream_omega_target = 0.0f;
    s_stream_v_ramp = 0.0f;
    s_stream_omega_ramp = 0.0f;
    s_stream_stamp_ms = 0U;
    s_stream_stopping = 0U;
    s_hw_dir_left = (int8_t)0;
    s_hw_dir_right = (int8_t)0;
    s_dir_settle_until_ms = 0U;
    s_seg_deadline_ms = 0U;
    s_last_task_ms = ROBOT_PORT_tick_ms();
    s_dt_ms = 1U;
    s_stall_ref_progress = 0.0f;
    s_stall_ref_ms = s_last_task_ms;
    s_last_request_ms = s_last_task_ms;
    s_segments_done = 0U;
    MOTION_set_limits(s_cfg->max_speed_mm_s, s_cfg->accel_mm_s2, s_cfg->decel_mm_s2);
}

/** @brief 100 Hz odometry integration (TIM7 ISR entry point). */
void MOTION_odometry_tick(void)
{
    int32_t ticks_left = ENCODERS_MJ_get_absolut_ticks(ROBOT_ENCODER_ID_LEFT);
    int32_t ticks_right = ENCODERS_MJ_get_absolut_ticks(ROBOT_ENCODER_ID_RIGHT);
    float d_left = (float)(ticks_left - s_prev_ticks_left) * s_cfg->odom_mm_per_tick;
    float d_right = (float)(ticks_right - s_prev_ticks_right) * s_cfg->odom_mm_per_tick;
    float d_center = 0.5f * (d_left + d_right);
    /* CCW positive: a CCW rotation drives the right wheel forward, so the
       heading increment is (right - left) / track. */
    float d_theta = (d_right - d_left) / s_cfg->odom_track_mm;
    float theta_mid = s_theta_rad + (0.5f * d_theta);
    uint8_t gyro_fresh;

    s_prev_ticks_left = ticks_left;
    s_prev_ticks_right = ticks_right;

    s_theta_rad = motion_normalize_angle(s_theta_rad + d_theta);
    s_x_mm += d_center * cosf(theta_mid);
    s_y_mm += d_center * sinf(theta_mid);
    s_travel_mm += d_center;

    /* Control heading: complementary blend of the forwarded gyro (fast,
       slip-free) with the wheel odometry (slow, bias-free).
       Fresh stream : integrate the rate, then pull slowly towards the
                      odometry (removes gyro bias over seconds).
       Stale/absent : no integration; a larger pull collapses any residual
                      offset towards the odometry - a smooth hand-over with
                      no heading step when the stream stops. */
    gyro_fresh = motion_gyro_is_fresh(ROBOT_PORT_tick_ms());
    if (gyro_fresh != 0U)
    {
        s_theta_ctrl_rad = motion_normalize_angle(
            s_theta_ctrl_rad + (s_gyro_rate_rad_s * ROBOT_ODOMETRY_TICK_S)
            + (ROBOT_GYRO_BLEND_PULL_ALPHA * motion_normalize_angle(s_theta_rad - s_theta_ctrl_rad)));
    }
    else
    {
        s_theta_ctrl_rad = motion_normalize_angle(
            s_theta_ctrl_rad
            + (ROBOT_GYRO_BLEND_PULL_STALE * motion_normalize_angle(s_theta_rad - s_theta_ctrl_rad)));
    }

    if (s_seg == SEG_ARC)
    {
        /* Unwrapped heading progress from the CONTROL heading: valid for
           sweeps above 180 deg and consistent with the curvature steering. */
        s_arc_theta_acc += motion_normalize_angle(s_theta_ctrl_rad - s_prev_theta_ctrl_rad);
    }
    s_prev_theta_ctrl_rad = s_theta_ctrl_rad;
}

/** @brief Main-loop engine: handles stop/e-stop, runs the active segment,
 *         supervises stall/timeout and updates the ramp. Never blocks. */
void MOTION_task(void)
{
    uint32_t now_ms = ROBOT_PORT_tick_ms();
    uint32_t dt = now_ms - s_last_task_ms;

    if (dt == 0U)
    {
        dt = 1U;
    }
    if (dt > 100U)
    {
        dt = 100U;
    }
    s_dt_ms = dt;
    s_last_task_ms = now_ms;

    if (s_estop_req != 0U)
    {
        s_estop_req = 0U;
        motion_set_fault(MOTION_FAULT_ESTOP);
        return;
    }

    if (s_limit_req != 0U)
    {
        /* The EXTI ISR already stopped the pulses and disabled the drivers;
           finalize the latched fault here (single writer of the state). */
        s_limit_req = 0U;
        motion_set_fault(MOTION_FAULT_LIMIT);
        return;
    }

    if (s_stop_req != 0U)
    {
        s_stop_req = 0U;
        s_pend_arc_valid = 0U;   /* a stop cancels any queued continuation ARC */
        if (s_seg != SEG_NONE)
        {
            motion_stop_outputs();
            s_seg = SEG_NONE;
            s_state = MOTION_STATE_IDLE;
        }
    }

    if ((s_state == MOTION_STATE_FAULT) || (s_state == MOTION_STATE_ESTOP))
    {
        return;
    }

    switch (s_seg)
    {
        case SEG_DRIVE:
            if ((int32_t)(now_ms - s_seg_deadline_ms) >= 0)
            {
                motion_set_fault(MOTION_FAULT_TIMEOUT);
            }
            else if (motion_do_drive(s_seg_dist_mm, s_seg_head_rad, s_seg_speed, now_ms) != 0U)
            {
                motion_finish_segment();
            }
            else
            {
                /* still driving */
            }
            break;

        case SEG_TURN:
            if ((int32_t)(now_ms - s_seg_deadline_ms) >= 0)
            {
                motion_set_fault(MOTION_FAULT_TIMEOUT);
            }
            else if (motion_do_turn(s_seg_turn_rad, s_seg_speed, now_ms) != 0U)
            {
                motion_finish_segment();
            }
            else
            {
                /* still turning */
            }
            break;

        case SEG_ARC:
            if ((int32_t)(now_ms - s_seg_deadline_ms) >= 0)
            {
                motion_set_fault(MOTION_FAULT_TIMEOUT);
            }
            else if (motion_do_arc(s_seg_speed, now_ms) != 0U)
            {
                motion_finish_segment();
            }
            else
            {
                /* still arcing */
            }
            break;

        case SEG_STRAFE:
            motion_run_strafe(now_ms);
            break;

        case SEG_GOTO:
            motion_run_goto(now_ms);
            break;

        case SEG_STREAM:
            if (motion_do_stream(now_ms) != 0U)
            {
                motion_finish_segment();
            }
            else
            {
                /* still streaming */
            }
            break;

        default:
            s_seg = SEG_NONE;
            break;
    }
}

/** @brief Requests a controlled stop (ISR-safe; executed by MOTION_task). */
void MOTION_request_stop(void)
{
    s_stop_req = 1U;
}

/** @brief Requests an emergency stop (ISR-safe): stop, drivers off, latch. */
void MOTION_request_estop(void)
{
    s_estop_req = 1U;
}

/** @brief EXTI entry for wall switches: immediate hard stop; the fault is
 *         finalized by MOTION_task() on its next pass. */
void MOTION_on_limit_switch(uint16_t gpio_pin)
{
    if (gpio_pin == SW1_Pin)
    {
        s_limit_status = (uint8_t)(s_limit_status | MOTION_LIMIT_SW1);
    }
    else if (gpio_pin == SW3_Pin)
    {
        s_limit_status = (uint8_t)(s_limit_status | MOTION_LIMIT_SW3);
    }
    else if (gpio_pin == SW4_Pin)
    {
        s_limit_status = (uint8_t)(s_limit_status | MOTION_LIMIT_SW4);
    }
    else
    {
        /* unrelated EXTI line - ignore */
    }

    /* Hard stop immediately in the ISR (pulses + drivers); the latched FAULT
       state is finalized by MOTION_task() on its next pass, which keeps the
       engine state single-writer even when this ISR preempts a segment. */
    motion_stop_outputs();
    motion_disable_drivers();
    s_limit_req = 1U;
}

/** @brief Releases the stepper drivers while the robot is idle.
 *
 *  This is deliberately NOT an E-STOP: no fault is latched and the next
 *  motion command re-energizes the drivers through motion_apply_speeds()
 *  (or the STRAFE / STREAM paths), so a mission can release, wait and then
 *  continue. A segment still running is rejected with BUSY.
 */
MOTION_ACCEPT MOTION_release_motors(void)
{
    if (s_seg != SEG_NONE)
    {
        return MOTION_ACCEPT_BUSY;
    }
    if ((s_faults != 0U) || (s_limit_req != 0U))
    {
        return MOTION_ACCEPT_FAULT;
    }

    motion_stop_outputs();
    motion_disable_drivers();
    s_pend_arc_valid = 0U;   /* no motion may follow a release implicitly */
    s_state = MOTION_STATE_IDLE;
    return MOTION_ACCEPT_OK;
}

/** @brief Clears all latched faults and the limit status; state returns IDLE. */
void MOTION_clear_faults(void)
{
    motion_stop_outputs();
    s_faults = 0U;
    s_limit_status = 0U;
    s_seg = SEG_NONE;
    s_pend_arc_valid = 0U;
    s_state = MOTION_STATE_IDLE;
}

/** @brief Latches the given fault bit and stops the robot safely. */
void MOTION_force_fault(uint16_t fault_bit)
{
    motion_set_fault(fault_bit);
}

/** @brief Overwrites the pose (used after RPi re-localization, robot idle). */
void MOTION_set_pose(float x_mm, float y_mm, float theta_rad)
{
    ROBOT_PORT_critical_enter();
    s_x_mm = x_mm;
    s_y_mm = y_mm;
    s_theta_rad = motion_normalize_angle(theta_rad);
    s_theta_ctrl_rad = s_theta_rad;
    s_prev_theta_ctrl_rad = s_theta_ctrl_rad;
    s_gyro_valid = 0U;   /* a teleport invalidates any held stream state */
    s_travel_mm = 0.0f;
    s_stall_ref_progress = motion_progress();
    s_stall_ref_ms = ROBOT_PORT_tick_ms();
    ROBOT_PORT_critical_exit();
}

/** @brief Zeroes the pose and both encoder counters. */
void MOTION_reset_odometry(void)
{
    /* One critical section for the whole sequence: a TIM7 tick between the
       counter reset and the baseline update would otherwise integrate one
       bogus delta and jump the pose. */
    ROBOT_PORT_critical_enter();
    ENCODERS_MJ_reset(ROBOT_ENCODER_ID_LEFT);
    ENCODERS_MJ_reset(ROBOT_ENCODER_ID_RIGHT);
    s_prev_ticks_left = 0;
    s_prev_ticks_right = 0;
    s_x_mm = 0.0f;
    s_y_mm = 0.0f;
    s_theta_rad = 0.0f;
    s_theta_ctrl_rad = 0.0f;
    s_prev_theta_ctrl_rad = 0.0f;
    s_gyro_valid = 0U;
    s_travel_mm = 0.0f;
    s_stall_ref_progress = motion_progress();
    s_stall_ref_ms = ROBOT_PORT_tick_ms();
    ROBOT_PORT_critical_exit();
}

/** @brief Updates the runtime motion limits (speed, accel, decel). */
void MOTION_set_limits(float max_speed_mm_s, float accel_mm_s2, float decel_mm_s2)
{
    s_max_speed = motion_clampf(max_speed_mm_s, 50.0f, ROBOT_MAX_SPEED_LIMIT_MM_S);
    s_accel = motion_clampf(accel_mm_s2, 50.0f, 2000.0f);
    s_decel = motion_clampf(decel_mm_s2, 50.0f, 2000.0f);
}

/** @brief Configures the command-channel deadman watchdog timeout. */
void MOTION_set_deadman_ms(uint16_t timeout_ms)
{
    s_deadman_ms = timeout_ms;
}

/** @brief Applies one silent SET_GYRO stream frame (see motion.h).
 *  @note  Cannot fail and never records an event: an invalid, wild or late
 *         value only lowers the gyro's influence until the fallback takes
 *         over - the board keeps running on wheels either way. */
void MOTION_set_gyro_rate(int16_t rate_deci_dps, uint8_t age_10_ms)
{
    float rate_rad_s = ((float)rate_deci_dps * 0.1f) * (ROBOT_PI / 180.0f);
    uint32_t now_ms = ROBOT_PORT_tick_ms();
    uint32_t age_ms = (uint32_t)age_10_ms * 10U;

    if (rate_rad_s > ROBOT_GYRO_MAX_RATE_RAD_S)
    {
        rate_rad_s = ROBOT_GYRO_MAX_RATE_RAD_S;
    }
    else if (rate_rad_s < -ROBOT_GYRO_MAX_RATE_RAD_S)
    {
        rate_rad_s = -ROBOT_GYRO_MAX_RATE_RAD_S;
    }
    else
    {
        /* within bounds */
    }
    s_gyro_rate_rad_s = rate_rad_s;
    /* the sender-reported age moves the stamp backwards, so a delayed frame
       is already (or soon becomes) stale on arrival */
    s_gyro_stamp_ms = (now_ms > age_ms) ? (now_ms - age_ms) : 0U;
    s_gyro_valid = 1U;
}

/** @brief 1 while a fresh gyro stream drives the control heading (motion.h). */
uint8_t MOTION_gyro_fresh(void)
{
    return motion_gyro_is_fresh(ROBOT_PORT_tick_ms());
}

/** @brief Refreshes the deadman reference time (every valid request). */
void MOTION_notify_request(uint32_t now_ms)
{
    s_last_request_ms = now_ms;
}

/** @brief Trips the deadman fault when the timeout expires during motion. */
void MOTION_check_deadman(uint32_t now_ms)
{
    if ((s_deadman_ms != 0U) && (s_seg != SEG_NONE) &&
        ((uint32_t)(now_ms - s_last_request_ms) > (uint32_t)s_deadman_ms))
    {
        motion_set_fault(MOTION_FAULT_DEADMAN);
    }
    else
    {
        /* deadman disabled, idle or still fed */
    }
}

/** @brief Returns 1 while a segment is active. */
uint8_t MOTION_is_busy(void)
{
    return (s_seg != SEG_NONE) ? 1U : 0U;
}

/** @brief Returns the current motion state. */
uint8_t MOTION_get_state(void)
{
    return s_state;
}

/** @brief Returns the latched fault bit mask. */
uint16_t MOTION_get_faults(void)
{
    return s_faults;
}

/** @brief Returns the limit-switch status bits. */
uint8_t MOTION_get_limit_status(void)
{
    return s_limit_status;
}

/** @brief Copies a coherent pose/velocity/state snapshot (critical section). */
void MOTION_get_snapshot(MOTION_SNAPSHOT *snapshot)
{
    float v_left;
    float v_right;

    if (snapshot == NULL)
    {
        return;
    }

    ROBOT_PORT_critical_enter();
    snapshot->x_um = (int32_t)(s_x_mm * 1000.0f);
    snapshot->y_um = (int32_t)(s_y_mm * 1000.0f);
    snapshot->theta_mdeg = (int32_t)(s_theta_rad * (180000.0f / ROBOT_PI));
    snapshot->state = s_state;
    snapshot->faults = s_faults;
    snapshot->segments_done = s_segments_done;
    ROBOT_PORT_critical_exit();

    v_left = ENCODERS_MJ_get_velocity_mm_s(ROBOT_ENCODER_ID_LEFT);
    v_right = ENCODERS_MJ_get_velocity_mm_s(ROBOT_ENCODER_ID_RIGHT);

    if (v_left > 32767.0f)
    {
        v_left = 32767.0f;
    }
    else if (v_left < -32768.0f)
    {
        v_left = -32768.0f;
    }
    else
    {
        /* in range */
    }
    if (v_right > 32767.0f)
    {
        v_right = 32767.0f;
    }
    else if (v_right < -32768.0f)
    {
        v_right = -32768.0f;
    }
    else
    {
        /* in range */
    }

    snapshot->v_left_mm_s = (int16_t)v_left;
    snapshot->v_right_mm_s = (int16_t)v_right;
}

/* ---------------------------- segment builders ------------------------------ */

/** @brief Accepts a straight-line DRIVE segment (see motion.h for limits). */
MOTION_ACCEPT MOTION_accept_drive(float distance_mm, float speed_mm_s)
{
    float cruise;
    float nominal_ms;
    uint32_t now_ms = ROBOT_PORT_tick_ms();

    if (s_seg != SEG_NONE)
    {
        return MOTION_ACCEPT_BUSY;
    }
    if ((s_faults != 0U) || (s_limit_req != 0U))
    {
        return MOTION_ACCEPT_FAULT;
    }
    if ((motion_absf(distance_mm) < ROBOT_DRIVE_TOL_MM) || (motion_absf(distance_mm) > 3000.0f))
    {
        return MOTION_ACCEPT_RANGE;
    }
    if (speed_mm_s < 5.0f)
    {
        return MOTION_ACCEPT_RANGE;
    }

    cruise = motion_clampf(speed_mm_s, s_cfg->min_speed_mm_s, s_max_speed);
    nominal_ms = (motion_absf(distance_mm) / cruise) * 1000.0f;
    motion_begin_segment(SEG_DRIVE, distance_mm, 0.0f, s_theta_ctrl_rad, cruise, now_ms, nominal_ms);
    return MOTION_ACCEPT_OK;
}

/** @brief Accepts an in-place TURN segment (see motion.h for limits). */
MOTION_ACCEPT MOTION_accept_turn(float angle_rad, float speed_mm_s)
{
    float cruise;
    float nominal_ms;
    float target_rad;
    uint32_t now_ms = ROBOT_PORT_tick_ms();

    if (s_seg != SEG_NONE)
    {
        return MOTION_ACCEPT_BUSY;
    }
    if ((s_faults != 0U) || (s_limit_req != 0U))
    {
        return MOTION_ACCEPT_FAULT;
    }
    if ((motion_absf(angle_rad) < 0.005f) || (motion_absf(angle_rad) > ROBOT_PI))
    {
        return MOTION_ACCEPT_RANGE;
    }
    if (speed_mm_s < 5.0f)
    {
        return MOTION_ACCEPT_RANGE;
    }

    cruise = motion_clampf(speed_mm_s, s_cfg->min_speed_mm_s, s_max_speed);
    target_rad = motion_normalize_angle(s_theta_ctrl_rad + angle_rad);
    nominal_ms = ((motion_absf(angle_rad) * s_cfg->robot_track_mm * 0.5f) / cruise) * 1000.0f;
    motion_begin_segment(SEG_TURN, 0.0f, target_rad, s_theta_ctrl_rad, cruise, now_ms, nominal_ms);
    return MOTION_ACCEPT_OK;
}

/** @brief Accepts one silent SET_VELOCITY frame (see motion.h).
 *
 *  The first frame while IDLE enters the streaming mode (state STREAM,
 *  pseudo-segment SEG_STREAM); further frames only replace the targets and
 *  refresh the freshness stamp. While any OTHER segment runs the frame has
 *  no effect (BUSY) - the protocol layer ignores the status silently, so
 *  stream frames never record events nor disturb the retry window. */
MOTION_ACCEPT MOTION_accept_stream(float v_mm_s, float omega_rad_s)
{
    uint32_t now_ms = ROBOT_PORT_tick_ms();
    float v = motion_clampf(v_mm_s, -s_max_speed, s_max_speed);
    float w = motion_clampf(omega_rad_s, -ROBOT_STREAM_MAX_OMEGA_RAD_S, ROBOT_STREAM_MAX_OMEGA_RAD_S);

    if ((s_faults != 0U) || (s_limit_req != 0U))
    {
        return MOTION_ACCEPT_FAULT;
    }
    if (s_seg == SEG_STREAM)
    {
        s_stream_v_target = v;
        s_stream_omega_target = w;
        s_stream_stamp_ms = now_ms;
        s_stream_stopping = 0U;
        return MOTION_ACCEPT_OK;
    }
    if (s_seg != SEG_NONE)
    {
        return MOTION_ACCEPT_BUSY;
    }

    /* fresh streaming session: the wheels restart from rest */
    motion_stop_outputs();
    s_seg = SEG_STREAM;
    s_state = MOTION_STATE_STREAM;
    s_stream_v_target = v;
    s_stream_omega_target = w;
    s_stream_v_ramp = 0.0f;
    s_stream_omega_ramp = 0.0f;
    s_stream_stamp_ms = now_ms;
    s_stream_stopping = 0U;
    s_stall_ref_progress = motion_progress();
    s_stall_ref_ms = now_ms;
    return MOTION_ACCEPT_OK;
}

/** @brief Streaming step: ramps (v, omega) and applies the wheel speeds.
 *
 *  Wheel mixing: v_left = v - omega * track/2, v_right = v + omega *
 *  track/2 (omega CCW positive). Nonzero wheel targets keep the driver's
 *  per-side floor; an exact (0, 0) target parks the pulses WITHOUT leaving
 *  the mode (the host may hold still mid-stream).
 *
 *  @return 1 when the stale-stop finished (the caller ends the segment),
 *          0 while streaming continues. */
static uint8_t motion_do_stream(uint32_t now_ms)
{
    float half_track = s_cfg->robot_track_mm * 0.5f;
    float v_target;
    float w_target;
    float v_left;
    float v_right;

    if ((uint32_t)(now_ms - s_stream_stamp_ms) > ROBOT_STREAM_STALE_MS)
    {
        s_stream_stopping = 1U;   /* controlled stale-stop: never a fault */
    }
    v_target = (s_stream_stopping != 0U) ? 0.0f : s_stream_v_target;
    w_target = (s_stream_stopping != 0U) ? 0.0f : s_stream_omega_target;

    motion_ramp_scalar(&s_stream_v_ramp, v_target, s_dt_ms, s_accel, s_decel);
    motion_ramp_scalar(&s_stream_omega_ramp, w_target, s_dt_ms,
                       s_accel / half_track, s_decel / half_track);

    if ((s_stream_stopping != 0U) &&
        (motion_absf(s_stream_v_ramp) < ROBOT_STREAM_END_EPS_MM_S) &&
        (motion_absf(s_stream_omega_ramp) < ROBOT_STREAM_END_EPS_RAD_S))
    {
        return 1U;
    }

    /* a held (0, 0) target parks the wheels without leaving the mode */
    if ((s_stream_stopping == 0U) &&
        (motion_absf(v_target) < 0.01f) && (motion_absf(w_target) < 0.01f))
    {
        if (s_outputs_started != 0U)
        {
            motion_stop_outputs();   /* once: the pulses are already idle */
        }
        return 0U;
    }

    v_left = s_stream_v_ramp - (s_stream_omega_ramp * half_track);
    v_right = s_stream_v_ramp + (s_stream_omega_ramp * half_track);
    if ((motion_absf(v_left) < ROBOT_STREAM_MIN_SIDE_MM_S) && (motion_absf(v_left) > 0.0001f))
    {
        v_left = (v_left >= 0.0f) ? ROBOT_STREAM_MIN_SIDE_MM_S : -ROBOT_STREAM_MIN_SIDE_MM_S;
    }
    if ((motion_absf(v_right) < ROBOT_STREAM_MIN_SIDE_MM_S) && (motion_absf(v_right) > 0.0001f))
    {
        v_right = (v_right >= 0.0f) ? ROBOT_STREAM_MIN_SIDE_MM_S : -ROBOT_STREAM_MIN_SIDE_MM_S;
    }
    v_left = motion_clampf(v_left, -s_max_speed, s_max_speed);
    v_right = motion_clampf(v_right, -s_max_speed, s_max_speed);
    motion_apply_speeds(v_left, v_right, now_ms);
    return 0U;
}

/** @brief Validates ARC parameters and clamps the cruise speed.
 *  @param  cruise_out Receives the clamped cruise speed on OK.
 *  @return MOTION_ACCEPT_OK or MOTION_ACCEPT_RANGE (no busy/fault checks:
 *          the callers decide when a BUSY/FAULT result applies). */
static MOTION_ACCEPT motion_arc_prepare(float radius_mm, float sweep_rad, float speed_mm_s,
                                        float *cruise_out)
{
    float abs_radius = motion_absf(radius_mm);
    float cruise;
    float arc_len;
    float v_outer_limit;

    if ((abs_radius < ROBOT_ARC_MIN_RADIUS_MM) || (abs_radius > 32767.0f))
    {
        return MOTION_ACCEPT_RANGE;
    }
    if ((motion_absf(sweep_rad) < 0.005f) || (motion_absf(sweep_rad) > ROBOT_TWO_PI))
    {
        return MOTION_ACCEPT_RANGE;
    }
    /* + radius and + sweep = left/CCW arc: the two signs must agree. */
    if (motion_signf(radius_mm) != motion_signf(sweep_rad))
    {
        return MOTION_ACCEPT_RANGE;
    }
    if (speed_mm_s < 5.0f)
    {
        return MOTION_ACCEPT_RANGE;
    }

    arc_len = motion_absf(sweep_rad) * abs_radius;
    if ((arc_len < 1.0f) || (arc_len > 3000.0f))
    {
        return MOTION_ACCEPT_RANGE;
    }

    cruise = motion_clampf(speed_mm_s, s_cfg->min_speed_mm_s, s_max_speed);
    /* Cap the centre speed so the outer wheel stays within the speed limit:
       v_outer = v_center * (1 + (track/2) / |R|). */
    v_outer_limit = (s_max_speed * abs_radius) / (abs_radius + (s_cfg->robot_track_mm * 0.5f));
    if (cruise > v_outer_limit)
    {
        cruise = v_outer_limit;
    }
    if (cruise < s_cfg->min_speed_mm_s)
    {
        return MOTION_ACCEPT_RANGE;
    }

    *cruise_out = cruise;
    return MOTION_ACCEPT_OK;
}

/** @brief Accepts a circular ARC segment (see motion.h for limits and signs). */
MOTION_ACCEPT MOTION_accept_arc(float radius_mm, float sweep_rad, float speed_mm_s)
{
    float cruise = 0.0f;
    MOTION_ACCEPT result;

    if (s_seg != SEG_NONE)
    {
        return MOTION_ACCEPT_BUSY;
    }
    if ((s_faults != 0U) || (s_limit_req != 0U))
    {
        return MOTION_ACCEPT_FAULT;
    }
    result = motion_arc_prepare(radius_mm, sweep_rad, speed_mm_s, &cruise);
    if (result != MOTION_ACCEPT_OK)
    {
        return result;
    }

    motion_arc_begin(radius_mm, sweep_rad, cruise, ROBOT_PORT_tick_ms(), 0U);
    return MOTION_ACCEPT_OK;
}

/** @brief Queues (or starts) a continuation ARC: see the header for the
 *         chaining semantics. Only valid while idle (starts immediately) or
 *         while a segment runs (one pending slot); the pending arc is
 *         consumed by motion_finish_segment() without stopping the wheels. */
MOTION_ACCEPT MOTION_queue_arc(float radius_mm, float sweep_rad, float speed_mm_s)
{
    float cruise = 0.0f;
    MOTION_ACCEPT result;

    if ((s_faults != 0U) || (s_limit_req != 0U))
    {
        return MOTION_ACCEPT_FAULT;
    }
    result = motion_arc_prepare(radius_mm, sweep_rad, speed_mm_s, &cruise);
    if (result != MOTION_ACCEPT_OK)
    {
        return result;
    }

    if (s_seg == SEG_NONE)
    {
        /* Idle: the queued arc IS the next segment - start it immediately.
           The RPi can therefore send every arc of a chain the same way and
           the board decides whether it continues or starts. */
        motion_arc_begin(radius_mm, sweep_rad, cruise, ROBOT_PORT_tick_ms(), 0U);
        return MOTION_ACCEPT_OK;
    }

    if (s_pend_arc_valid != 0U)
    {
        return MOTION_ACCEPT_BUSY;   /* single pending slot */
    }
    s_pend_arc_radius = radius_mm;
    s_pend_arc_sweep = sweep_rad;
    s_pend_arc_cruise = cruise;
    s_pend_arc_valid = 1U;
    return MOTION_ACCEPT_OK;
}

/** @brief Accepts a GOTO manoeuvre (turn, drive, turn; see motion.h). */
MOTION_ACCEPT MOTION_accept_goto(float x_mm, float y_mm, float theta_rad, float speed_mm_s)
{
    float cruise;
    float dx;
    float dy;
    float distance;
    float heading;
    float arc_heading;
    float arc_final;
    float nominal_ms;
    uint32_t now_ms = ROBOT_PORT_tick_ms();

    if (s_seg != SEG_NONE)
    {
        return MOTION_ACCEPT_BUSY;
    }
    if ((s_faults != 0U) || (s_limit_req != 0U))
    {
        return MOTION_ACCEPT_FAULT;
    }
    if ((motion_absf(x_mm) > 3276.0f) || (motion_absf(y_mm) > 3276.0f))
    {
        return MOTION_ACCEPT_RANGE;
    }
    if (speed_mm_s < 5.0f)
    {
        return MOTION_ACCEPT_RANGE;
    }

    cruise = motion_clampf(speed_mm_s, s_cfg->min_speed_mm_s, s_max_speed);
    dx = x_mm - s_x_mm;
    dy = y_mm - s_y_mm;
    distance = sqrtf((dx * dx) + (dy * dy));
    heading = atan2f(dy, dx);
    arc_heading = motion_absf(motion_normalize_angle(heading - s_theta_ctrl_rad)) * s_cfg->robot_track_mm * 0.5f;
    arc_final = motion_absf(motion_normalize_angle(theta_rad - heading)) * s_cfg->robot_track_mm * 0.5f;
    nominal_ms = ((distance + arc_heading + arc_final) / cruise) * 1000.0f;

    motion_begin_segment(SEG_GOTO, distance, motion_normalize_angle(theta_rad), heading,
                         cruise, now_ms, nominal_ms);
    if (distance < 1.0f)
    {
        s_goto_phase = GOTO_PHASE_TURN_FINAL;
    }
    return MOTION_ACCEPT_OK;
}

/** @brief Accepts a timed STRAFE segment (dead-reckoned, see motion.h). */
MOTION_ACCEPT MOTION_accept_strafe(float distance_mm, float speed_mm_s)
{
    float cruise;
    uint32_t now_ms = ROBOT_PORT_tick_ms();

    if (s_seg != SEG_NONE)
    {
        return MOTION_ACCEPT_BUSY;
    }
    if ((s_faults != 0U) || (s_limit_req != 0U))
    {
        return MOTION_ACCEPT_FAULT;
    }
    if ((motion_absf(distance_mm) < 1.0f) || (motion_absf(distance_mm) > 2000.0f))
    {
        return MOTION_ACCEPT_RANGE;
    }
    if (speed_mm_s < 5.0f)
    {
        return MOTION_ACCEPT_RANGE;
    }

    cruise = motion_clampf(speed_mm_s, s_cfg->min_speed_mm_s, s_max_speed);
    motion_begin_segment(SEG_STRAFE, distance_mm, 0.0f, 0.0f, cruise, now_ms,
                         (motion_absf(distance_mm) / cruise) * 1000.0f);
    motion_strafe_setup(distance_mm, cruise, now_ms);
    /* Strafe completes on time (no lateral odometry); keep a short margin. */
    s_seg_deadline_ms = now_ms + (uint32_t)((motion_absf(distance_mm) / cruise) * 1000.0f) + 300U;
    return MOTION_ACCEPT_OK;
}

/* ------------------------------ goto runner --------------------------------- */
/** @brief GOTO run loop: sequences TURN -> DRIVE -> TURN with per-phase
 *         supervision and timeout checking. */
static void motion_run_goto(uint32_t now_ms)
{
    if ((int32_t)(now_ms - s_seg_deadline_ms) >= 0)
    {
        motion_set_fault(MOTION_FAULT_TIMEOUT);
        return;
    }

    switch (s_goto_phase)
    {
        case GOTO_PHASE_TURN_HEADING:
            if (motion_do_turn(s_seg_head_rad, s_seg_speed, now_ms) != 0U)
            {
                s_goto_phase = GOTO_PHASE_DRIVE;
                s_travel_at_start = s_travel_mm;
                s_v_ramp = 0.0f;
                s_stall_ref_progress = motion_progress();
                s_stall_ref_ms = now_ms;
            }
            else
            {
                /* still aligning */
            }
            break;

        case GOTO_PHASE_DRIVE:
            if (motion_do_drive(s_seg_dist_mm, s_seg_head_rad, s_seg_speed, now_ms) != 0U)
            {
                s_goto_phase = GOTO_PHASE_TURN_FINAL;
                s_v_ramp = 0.0f;
                s_stall_ref_progress = motion_progress();
                s_stall_ref_ms = now_ms;
            }
            else
            {
                /* still driving */
            }
            break;

        case GOTO_PHASE_TURN_FINAL:
            if (motion_do_turn(s_seg_turn_rad, s_seg_speed, now_ms) != 0U)
            {
                motion_finish_segment();
            }
            else
            {
                /* still aligning */
            }
            break;

        default:
            motion_finish_segment();
            break;
    }
}
