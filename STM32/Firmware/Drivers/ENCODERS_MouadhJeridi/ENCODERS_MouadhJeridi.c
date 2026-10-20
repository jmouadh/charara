/******************************************************************************
 * @file    ENCODERS_MouadhJeridi.c
 * @brief   Quadrature odometry encoder driver implementation (see the header
 *          for the sampling and wrap-safety rationale).
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 2.3.3
 * @date    2026-09-25
 ******************************************************************************/
#include "ENCODERS_MouadhJeridi.h"


/* Private module state */
static ENCODERS_MJ_t ENCODERS_MJ[ENCODERS_MJ_MAX_ENCODERS_NUMBER];

/* Relative baseline per encoder (so we don’t need to change the public struct) */
static int32_t ENCODERS_MJ_rel_zero_ticks[ENCODERS_MJ_MAX_ENCODERS_NUMBER] = {0};

/* Low-pass filtered speed per encoder, refreshed by ENCODERS_MJ_poll() */
static float ENCODERS_MJ_velocity_mm_s[ENCODERS_MJ_MAX_ENCODERS_NUMBER] = {0.0f};

/** @brief Odometry wheel mechanics (defaults: 4000 ticks/rev, 200 mm wheel
 *         circumference -> 0.05 mm/tick). Set by ENCODERS_MJ_configure(). */
static float ENCODERS_MJ_mm_per_tick = 0.05f;

/** @brief Supplies the mechanical data of the odometry wheels. */
void ENCODERS_MJ_configure(float ticks_per_rev, float wheel_circumference_mm)
{
    if ((ticks_per_rev > 0.0f) && (wheel_circumference_mm > 0.0f))
    {
        ENCODERS_MJ_mm_per_tick = wheel_circumference_mm / ticks_per_rev;
    }
}

/** @brief Updates the counting direction of one encoder (runtime calibration).
 *  @note  A single sign store; ENCODERS_MJ_poll() applies it from the next
 *         sample on, which is what a sign calibration needs. */
void ENCODERS_MJ_set_invert_counting(uint8_t encoder_id, uint8_t invert_counting)
{
    if (encoder_id >= ENCODERS_MJ_MAX_ENCODERS_NUMBER)
    {
        return;
    }
    ENCODERS_MJ[encoder_id].invert_counting = (invert_counting == 0U) ? (int8_t)1 : (int8_t)-1;
}


/* ---------------------------- Critical sections --------------------------- */
/* PRIMASK save/restore instead of plain disable/enable: these helpers can be
   called from inside an outer critical section (e.g. the motion engine reset
   sequence) without re-enabling interrupts too early. */

/** @brief Disables interrupts and returns the previous PRIMASK state. */
static uint32_t ENCODERS_MJ_critical_enter(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/** @brief Restores the PRIMASK state saved by ENCODERS_MJ_critical_enter(). */
static void ENCODERS_MJ_critical_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/* ----------------------------- Initialization ----------------------------- */
/** @brief Initializes one encoder: starts the timer, centers the counter,
 *         sets the counting direction and clears all accumulators. */
void ENCODERS_MJ_Init(uint8_t encoder_id, TIM_HandleTypeDef *htim, uint8_t invert_counting)
{
    if (encoder_id >= ENCODERS_MJ_MAX_ENCODERS_NUMBER) return;

    ENCODERS_MJ_t ENCODER_MJ = {0};
    ENCODER_MJ.htim            = *htim;
    ENCODER_MJ.relatif_mm      = 0.0f;
    ENCODER_MJ.relatif_ticks   = 0;
    ENCODER_MJ.absolut_ticks   = 0;
    ENCODER_MJ.invert_counting = (invert_counting == 0 ? 1 : -1);
    ENCODER_MJ.last_counter    = ENCODER_MJ_INIT_COUNTER;
    ENCODERS_MJ[encoder_id]    = ENCODER_MJ;

    ENCODERS_MJ_rel_zero_ticks[encoder_id] = 0;  // relative baseline at absolute=0

    /* No capture/compare interrupts: the hardware counter is sampled at
       100 Hz by ENCODERS_MJ_poll() using wrap-safe signed deltas. This
       removes up to ~16 k IRQ/s of pure overhead at full speed. */
    (void)HAL_TIM_Encoder_Start(&ENCODERS_MJ[encoder_id].htim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&ENCODERS_MJ[encoder_id].htim, ENCODER_MJ_INIT_COUNTER);
    __HAL_TIM_CLEAR_FLAG(&ENCODERS_MJ[encoder_id].htim, TIM_FLAG_UPDATE);
}

/** @brief Re-centers the hardware counter and the sampling reference point. */
void ENCODERS_MJ_reset_encoder_timer_counter(uint8_t encoder_id)
{
    if (encoder_id >= ENCODERS_MJ_MAX_ENCODERS_NUMBER) return;
    __HAL_TIM_SET_COUNTER(&(ENCODERS_MJ[encoder_id].htim), ENCODER_MJ_INIT_COUNTER);
    ENCODERS_MJ[encoder_id].last_counter = ENCODER_MJ_INIT_COUNTER;
    __HAL_TIM_CLEAR_FLAG(&ENCODERS_MJ[encoder_id].htim, TIM_FLAG_UPDATE);
}

/** @brief Re-zeros one encoder: counter, baseline and accumulators. */
void ENCODERS_MJ_reset(uint8_t encoder_id)
{
    if (encoder_id >= ENCODERS_MJ_MAX_ENCODERS_NUMBER) return;

    uint32_t primask = ENCODERS_MJ_critical_enter();
    /* Reset hardware counter back to center and software accumulators */
    ENCODERS_MJ_reset_encoder_timer_counter(encoder_id);

    ENCODERS_MJ[encoder_id].absolut_ticks = 0;

    /* Make "relative" zero at this moment using the absolute baseline */
    ENCODERS_MJ_rel_zero_ticks[encoder_id] = 0;
    ENCODERS_MJ[encoder_id].relatif_ticks  = 0;
    ENCODERS_MJ[encoder_id].relatif_mm     = 0.0f;
    ENCODERS_MJ_velocity_mm_s[encoder_id]  = 0.0f;
    ENCODERS_MJ_critical_exit(primask);
}

/* ---------------------------- Absolute readings --------------------------- */
/* Fold deltas into the 32-bit accumulator on every read (wrap safe).         */

/** @brief Returns the cached wrap-safe absolute count (refreshed at 100 Hz). */
int32_t ENCODERS_MJ_get_absolut_ticks(uint8_t encoder_id)
{
    if (encoder_id >= ENCODERS_MJ_MAX_ENCODERS_NUMBER) return -1;

    /* Cached value, refreshed by ENCODERS_MJ_poll() from the 100 Hz TIM7 ISR.
       No folding here: single writer keeps the state race-free. */
    return ENCODERS_MJ[encoder_id].absolut_ticks;
}

/** @brief Returns the low-pass filtered encoder speed in mm/s. */
float ENCODERS_MJ_get_velocity_mm_s(uint8_t encoder_id)
{
    if (encoder_id >= ENCODERS_MJ_MAX_ENCODERS_NUMBER) return -1.0f;
    return ENCODERS_MJ_velocity_mm_s[encoder_id];
}

/** @brief Returns the absolute position in mm derived from the absolute count. */
float ENCODERS_MJ_get_absolut_position(uint8_t encoder_id)
{
    if (encoder_id >= ENCODERS_MJ_MAX_ENCODERS_NUMBER) return -1.0f;
    int32_t abs_ticks = ENCODERS_MJ_get_absolut_ticks(encoder_id); /* folds now */
    return (float)abs_ticks * ENCODERS_MJ_mm_per_tick;
}

/* ---------------------------- Relative readings --------------------------- */
/* Derived from absolute − baseline (no ±32767 flip).                         */

/** @brief Returns the count since the last rezero point. */
int32_t ENCODERS_MJ_get_relatif_ticks(uint8_t encoder_id)
{
    if (encoder_id >= ENCODERS_MJ_MAX_ENCODERS_NUMBER) return -1;

    int32_t abs_now = ENCODERS_MJ_get_absolut_ticks(encoder_id); /* folds now */
    int32_t rel     = abs_now - ENCODERS_MJ_rel_zero_ticks[encoder_id];
    ENCODERS_MJ[encoder_id].relatif_ticks = rel;
    return rel;
}

/** @brief Returns the relative position in mm. */
float ENCODERS_MJ_get_relatif_position(uint8_t encoder_id)
{
    if (encoder_id >= ENCODERS_MJ_MAX_ENCODERS_NUMBER) return -1.0f;
    int32_t rel_ticks = ENCODERS_MJ_get_relatif_ticks(encoder_id);
    ENCODERS_MJ[encoder_id].relatif_mm = (float)rel_ticks * ENCODERS_MJ_mm_per_tick;
    return ENCODERS_MJ[encoder_id].relatif_mm;
}

/* ------------------------------ Misc helpers ------------------------------ */

/** @brief Returns the current counting direction of the timer. */
uint8_t ENCODERS_MJ_get_direction(uint8_t encoder_id)
{
    if (encoder_id >= ENCODERS_MJ_MAX_ENCODERS_NUMBER) return 0xFF;
    return __HAL_TIM_IS_TIM_COUNTING_DOWN(&ENCODERS_MJ[encoder_id].htim);
}

/* Polling folder: call at exactly 100 Hz (TIM7 ISR). Replaces the former
   update-event callback; single writer, race-free. */
void ENCODERS_MJ_poll(void)
{
    for (uint8_t encoder_id = 0; encoder_id < ENCODERS_MJ_MAX_ENCODERS_NUMBER; encoder_id++)
    {
        ENCODERS_MJ_t *e = &ENCODERS_MJ[encoder_id];
        uint16_t curr  = __HAL_TIM_GET_COUNTER(&e->htim);
        int16_t  delta = (int16_t)(curr - e->last_counter);   /* signed wrap-safe */
        float    delta_mm;
        float    velocity_raw;

        e->last_counter   = curr;
        e->absolut_ticks += (int32_t)delta * e->invert_counting;

        delta_mm     = ((float)delta * (float)e->invert_counting) * ENCODERS_MJ_mm_per_tick;
        velocity_raw = delta_mm / ENCODERS_MJ_SAMPLE_PERIOD_S;
        ENCODERS_MJ_velocity_mm_s[encoder_id] +=
            0.3f * (velocity_raw - ENCODERS_MJ_velocity_mm_s[encoder_id]);
    }
}

/* -------------------------- Absolute setters (API) ------------------------ */

/** @brief Forces the absolute count of one encoder (counter re-centered). */
void ENCODERS_MJ_set_absolut_ticks(uint8_t encoder_id, int32_t ticks)
{
    if (encoder_id >= ENCODERS_MJ_MAX_ENCODERS_NUMBER) return;

    ENCODERS_MJ_t *enc = &ENCODERS_MJ[encoder_id];
    /* Keep the hardware counter centered at ENCODER_MJ_INIT_COUNTER to minimize
       immediate wrap concerns; last_counter follows it so next delta is 0. */
    uint16_t hw_counter = (uint16_t)(ENCODER_MJ_INIT_COUNTER + (uint16_t)ticks);
    uint32_t primask = ENCODERS_MJ_critical_enter();

    __HAL_TIM_SET_COUNTER(&enc->htim, hw_counter);
    enc->last_counter  = hw_counter;
    enc->absolut_ticks = ticks;
    ENCODERS_MJ_critical_exit(primask);

    __HAL_TIM_CLEAR_FLAG(&enc->htim, TIM_FLAG_UPDATE);
}

/** @brief Forces the absolute position in mm of one encoder. */
void ENCODERS_MJ_set_absolut_position(uint8_t encoder_id, float position_mm)
{
    if (encoder_id >= ENCODERS_MJ_MAX_ENCODERS_NUMBER) return;

    float   ticks_f = position_mm / ENCODERS_MJ_mm_per_tick;
    int32_t ticks   = (int32_t)((ticks_f >= 0.0f) ? (ticks_f + 0.5f) : (ticks_f - 0.5f));

    ENCODERS_MJ_set_absolut_ticks(encoder_id, ticks);
}

/* -------------------------- Optional: re-zero relative --------------------- */
/* Call this if you want to re-zero the relative baseline at any time.        */
/** @brief Re-baselines the relative counter to the current absolute value. */
void ENCODERS_MJ_rezero_relative(uint8_t encoder_id)
{
    if (encoder_id >= ENCODERS_MJ_MAX_ENCODERS_NUMBER) return;
    ENCODERS_MJ_rel_zero_ticks[encoder_id] = ENCODERS_MJ_get_absolut_ticks(encoder_id);
    ENCODERS_MJ[encoder_id].relatif_ticks  = 0;
    ENCODERS_MJ[encoder_id].relatif_mm     = 0.0f;
}
