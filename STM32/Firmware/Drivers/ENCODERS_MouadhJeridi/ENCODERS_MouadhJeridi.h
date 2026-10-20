/******************************************************************************
 * @file    ENCODERS_MouadhJeridi.h
 * @brief   Quadrature odometry encoder driver (TIM encoder mode, 16-bit):
 *          wrap-safe absolute accumulation at 100 Hz plus filtered velocity.
 *
 * @details The hardware counters are sampled by ENCODERS_MJ_poll() from the
 *          TIM7 interrupt at exactly ENCODERS_MJ_SAMPLE_PERIOD_S (10 ms):
 *          signed 16-bit deltas handle counter wrap naturally. Getters return
 *          cached values, which keeps the driver race-free (single writer).
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 2.3.3
 * @date    2026-09-25
 ******************************************************************************
 */
#ifndef __ENCODERS_MOUADHJERIDI_H__
#define __ENCODERS_MOUADHJERIDI_H__

#include "stm32f7xx_hal.h"


#define ENCODERS_MJ_MAX_ENCODERS_NUMBER                      ((uint8_t)      4)
#define ENCODER_MJ_INIT_COUNTER                              ((uint16_t)0x7FFF)
#define ENCODERS_MJ_SAMPLE_PERIOD_S                          (0.01f)

/** @brief Runtime state of one encoder channel. */
typedef struct
{
    TIM_HandleTypeDef  htim;              /**< Encoder timer (TI12 mode).           */
    volatile int32_t   absolut_ticks;     /**< Wrap-safe absolute count.            */
    volatile int32_t   relatif_ticks;     /**< Count relative to the rezero point.  */
    volatile float     relatif_mm;        /**< Relative position in mm.             */
    int8_t             invert_counting;   /**< +1 or -1 counting direction.         */
    int16_t            last_counter;      /**< Last sampled hardware counter.       */
} ENCODERS_MJ_t;

/** @brief  Supplies the mechanical data of the odometry wheels.
 *  @param  ticks_per_rev           Encoder counts per wheel revolution.
 *  @param  wheel_circumference_mm  Encoder wheel circumference in millimetres.
 *  @note   Values <= 0 are rejected; defaults (4000 ticks, 200 mm) apply
 *          until called. */
void ENCODERS_MJ_configure(float ticks_per_rev, float wheel_circumference_mm);

/** @brief  Updates the counting direction of one encoder (runtime calibration).
 *  @param  encoder_id        Encoder index 0..ENCODERS_MJ_MAX_ENCODERS_NUMBER-1.
 *  @param  invert_counting   0 = keep the hardware direction, 1 = invert it. */
void ENCODERS_MJ_set_invert_counting(uint8_t encoder_id, uint8_t invert_counting);

/** @brief  Initializes one encoder: starts the timer in encoder mode, centers
 *          the counter and clears all accumulators.
 *  @param  encoder_id       Encoder index 0..ENCODERS_MJ_MAX_ENCODERS_NUMBER-1.
 *  @param  htim             Encoder timer handle (TI12, 16-bit).
 *  @param  invert_counting  0 = keep counting direction, 1 = invert it (use to
 *                           match the robot sign convention). */
void ENCODERS_MJ_Init(uint8_t encoder_id,
                      TIM_HandleTypeDef *htim,
                      uint8_t invert_counting);

/** @brief  Samples all encoder counters and updates absolute counts, relative
 *          values and the filtered velocities. Must be called at exactly
 *          100 Hz (TIM7 interrupt); it is the only writer of the state. */
void ENCODERS_MJ_poll(void);

/** @brief  Re-zeros one encoder: counter re-centered, accumulators cleared.
 *  @param  encoder_id  Encoder index. */
void ENCODERS_MJ_reset(uint8_t encoder_id);

/** @brief  Re-centers the hardware counter of one encoder without touching the
 *          software accumulators (used by ENCODERS_MJ_reset).
 *  @param  encoder_id  Encoder index. */
void ENCODERS_MJ_reset_encoder_timer_counter(uint8_t encoder_id);

/** @brief  Re-baselines the relative counter to the current absolute value.
 *  @param  encoder_id  Encoder index. */
void ENCODERS_MJ_rezero_relative(uint8_t encoder_id);

/** @brief  @return Wrap-safe total count since reset (-1 for a bad ID). */
int32_t ENCODERS_MJ_get_absolut_ticks(uint8_t encoder_id);

/** @brief  @return Count since the last rezero (-1 for a bad ID). */
int32_t ENCODERS_MJ_get_relatif_ticks(uint8_t encoder_id);

/** @brief  @return Absolute position in mm (-1.0f for a bad ID). */
float ENCODERS_MJ_get_absolut_position(uint8_t encoder_id);

/** @brief  @return Relative position in mm (-1.0f for a bad ID). */
float ENCODERS_MJ_get_relatif_position(uint8_t encoder_id);

/** @brief  @return Low-pass filtered speed in mm/s (-1.0f for a bad ID). */
float ENCODERS_MJ_get_velocity_mm_s(uint8_t encoder_id);

/** @brief  @return 0 while counting up, 1 while counting down
 *          (0xFF for a bad ID). */
uint8_t ENCODERS_MJ_get_direction(uint8_t encoder_id);

/** @brief  Forces the accumulated absolute count of one encoder.
 *  @param  encoder_id  Encoder index.
 *  @param  ticks       New absolute count (counter is re-centered around it). */
void ENCODERS_MJ_set_absolut_ticks(uint8_t encoder_id, int32_t ticks);

/** @brief  Forces the absolute position in mm of one encoder.
 *  @param  encoder_id   Encoder index.
 *  @param  position_mm  Position in mm. */
void ENCODERS_MJ_set_absolut_position(uint8_t encoder_id, float position_mm);

#endif //__ENCODERS_MJ_H__