/******************************************************************************
 * @file    STEPPERS_MouadhJeridi.c
 * @brief   Stepper motor driver implementation (see the header for the
 *          speed/ARR relations and the side group concept).
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 2.4.0
 * @date    2026-09-28
 ******************************************************************************/
#include "STEPPERS_MouadhJeridi.h"

static STEPPERS_MJ_t STEPPERS_MJ[STEPPERS_MJ_MAX_STEPPERS_NUMBER];
static STEPPERS_GROUP_MJ_t LEFT_STEPPERS;
static STEPPERS_GROUP_MJ_t RIGHT_STEPPERS;

/** @brief Wheel mechanics (defaults: 6400 ticks per revolution and a 60 mm
 *         wheel, i.e. ~33.95 steps per mm). Set by STEPPERS_MJ_configure(). */
static float s_steps_per_mm = 33.953f;

/** @brief Step timer kernel clock [Hz]; 0 = use SystemCoreClock (default).
 *         Set by STEPPERS_MJ_set_timer_clock_hz(). */
static uint32_t s_timer_clock_hz = 0U;

/** @brief Supplies the mechanical data used by all position and speed conversions. */
void STEPPERS_MJ_configure(float ticks_per_rev, float wheel_circumference_mm)
{
    if ((ticks_per_rev > 0.0f) && (wheel_circumference_mm > 0.0f))
    {
        s_steps_per_mm = ticks_per_rev / wheel_circumference_mm;
    }
}

/** @brief Supplies the kernel clock of the step timers (mm/s -> ARR conversion).
 *  @note  0 restores the default (SystemCoreClock). */
void STEPPERS_MJ_set_timer_clock_hz(uint32_t timer_clock_hz)
{
    s_timer_clock_hz = timer_clock_hz;
}

/** @brief Updates the DIR polarity of one motor (runtime calibration).
 *  @param  stepper_id        Motor index 0..STEPPERS_MJ_MAX_STEPPERS_NUMBER-1.
 *  @param  invert_direction  GPIO_PIN_SET to invert the DIR pin for FORWARD.
 *  @note   Re-imposes the last commanded direction on the pin, so the
 *          physical state stays consistent with the new polarity. */
void STEPPERS_MJ_set_invert_direction(uint8_t stepper_id, GPIO_PinState invert_direction)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return;
    }
    STEPPERS_MJ[stepper_id].PARAMS.invert_direction = (invert_direction != GPIO_PIN_RESET)
        ? GPIO_PIN_SET : GPIO_PIN_RESET;
    (void)STEPPERS_MJ_set_direction(stepper_id, STEPPERS_MJ[stepper_id].PARAMS.direction);
}

/** @brief Registers one stepper motor (timer channel, EN/DIR pins, direction polarity). */
void STEPPERS_MJ_Init(uint8_t stepper_id,
                      TIM_HandleTypeDef *htim,
                      uint32_t channel,
                      uint16_t enable_pin,
                      GPIO_TypeDef *enable_port,
                      uint16_t direction_pin,
                      GPIO_TypeDef *direction_port,
                      GPIO_PinState invert_direction)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return;
    }
    // Initialize the stepper structure
    STEPPERS_MJ_t STEPPER_MJ           = {0};
    STEPPER_MJ.htim                    = *htim;
    STEPPER_MJ.channel                 = channel;
    STEPPER_MJ.enable_pin              = enable_pin;
    STEPPER_MJ.enable_port             = enable_port;
    STEPPER_MJ.direction_pin           = direction_pin;
    STEPPER_MJ.direction_port          = direction_port;
    STEPPER_MJ.PARAMS.invert_direction = invert_direction ? GPIO_PIN_SET : GPIO_PIN_RESET;
    STEPPER_MJ.PARAMS.direction        = FORWARD;
    STEPPER_MJ.PARAMS.absolut_ticks    = 0;
    STEPPER_MJ.PARAMS.absolut_position = 0;
    STEPPER_MJ.PARAMS.target_ticks     = 0;
    STEPPER_MJ.PARAMS.target_position  = 0;
    STEPPER_MJ.PARAMS.is_there_target  = 0;
    STEPPERS_MJ[stepper_id]            = STEPPER_MJ;
    /* No update interrupt is enabled on the step timers: the motion engine
       generates pulses open-loop and closes the loop on the odometry
       encoders, so per-step interrupts would be pure CPU waste. */
}

/** @brief Creates the left side group bound to a shared side counter timer. */
void STEPPERS_MJ_Init_left_steppers(uint8_t *left_steppers_list,
                                   uint8_t stepper_number,
                                   TIM_HandleTypeDef *left_timer,
                                   void (*reach_target_callback)(void))
{
    if (stepper_number > STEPPERS_MJ_MAX_STEPPERS_NUMBER/2)
    {
        return;
    }
    for (uint8_t stepper = 0; stepper < stepper_number; stepper++)
    {
        if (left_steppers_list[stepper] >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
        {
            return;
        }
    }
    STEPPERS_GROUP_MJ_t left_steppers = {0};
    for (uint8_t stepper = 0; stepper < stepper_number; stepper++)
    {
        left_steppers.steppers_id[stepper] = left_steppers_list[stepper];
    }
    left_steppers.set_target_timer       = *left_timer;
    left_steppers.direction              = FORWARD;
    left_steppers.tick_target            = 0;
    left_steppers.position_target        = 0.0;
    left_steppers.timer_counter          = 0;
    left_steppers.is_there_target        = 0;
    left_steppers.speed                  = 0;
    left_steppers.absolut_ticks          = 0;
    left_steppers.absolut_position       = 0.0;
    left_steppers.reach_target_callback  = *reach_target_callback;
    LEFT_STEPPERS                        = left_steppers;
    /* The side-counter update interrupt stays disabled until an open-loop
       target is actually started (STEPPERS_MJ_start_left): the closed-loop
       motion engine stops on encoder feedback, so running this interrupt at
       step rate would be pure CPU waste. */
}

/** @brief Creates the right side group bound to a shared side counter timer. */
void STEPPERS_MJ_Init_right_steppers(uint8_t *right_steppers_list,
                                    uint8_t stepper_number,
                                    TIM_HandleTypeDef *right_timer,
                                    void (*reach_target_callback)(void))
{
if (stepper_number > STEPPERS_MJ_MAX_STEPPERS_NUMBER/2)
{
return;
}
for (uint8_t stepper = 0; stepper < stepper_number; stepper++)
{
if (right_steppers_list[stepper] >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
{
return;
}
}
STEPPERS_GROUP_MJ_t right_steppers = {0};
for (uint8_t stepper = 0; stepper < stepper_number; stepper++)
{
right_steppers.steppers_id[stepper] = right_steppers_list[stepper];
}
right_steppers.set_target_timer       = *right_timer;
right_steppers.direction              = FORWARD;
right_steppers.tick_target            = 0;
right_steppers.position_target        = 0.0;
right_steppers.timer_counter          = 0;
right_steppers.is_there_target        = 0;
right_steppers.speed                  = 0;
right_steppers.absolut_ticks          = 0;
right_steppers.absolut_position       = 0.0;
right_steppers.reach_target_callback  = *reach_target_callback;
RIGHT_STEPPERS                        = right_steppers;
/* Side-counter interrupt enabled on demand - see STEPPERS_MJ_start_right. */
}

/** @brief Enables one driver by driving its EN pin low. */
void STEPPERS_MJ_enable_stepper(uint8_t stepper_id)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return;
    }
    HAL_GPIO_WritePin(STEPPERS_MJ[stepper_id].enable_port,
                      STEPPERS_MJ[stepper_id].enable_pin,
                      GPIO_PIN_RESET);
}

/** @brief Enables both drivers of the left side group. */
void STEPPERS_MJ_enable_left_steppers(void)
{
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        STEPPERS_MJ_enable_stepper(LEFT_STEPPERS.steppers_id[stepper]);
    }
}

/** @brief Enables both drivers of the right side group. */
void STEPPERS_MJ_enable_right_steppers(void)
{
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        STEPPERS_MJ_enable_stepper(RIGHT_STEPPERS.steppers_id[stepper]);
    }
}

/** @brief Disables one driver by driving its EN pin high. */
void STEPPERS_MJ_disable_stepper(uint8_t stepper_id)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return;
    }
    HAL_GPIO_WritePin(STEPPERS_MJ[stepper_id].enable_port,
                      STEPPERS_MJ[stepper_id].enable_pin,
                      GPIO_PIN_SET);
}

/** @brief Disables both drivers of the left side group. */
void STEPPERS_MJ_disable_left_steppers(void)
{
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        STEPPERS_MJ_disable_stepper(LEFT_STEPPERS.steppers_id[stepper]);
    }
}

/** @brief Disables both drivers of the right side group. */
void STEPPERS_MJ_disable_right_steppers(void)
{
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        STEPPERS_MJ_disable_stepper(RIGHT_STEPPERS.steppers_id[stepper]);
    }
}

/** @brief Sets the DIR pin of one motor, applying its configured inversion. */
STEPPERS_MJ_DIRECTIONS STEPPERS_MJ_set_direction(uint8_t stepper_id, STEPPERS_MJ_DIRECTIONS direction)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return DIR_ERROR;
    }
    STEPPERS_MJ_t *STEPPER_MJ = &STEPPERS_MJ[stepper_id];
    HAL_GPIO_WritePin(STEPPER_MJ->direction_port,
                      STEPPER_MJ->direction_pin,
                      (GPIO_PinState)((GPIO_PinState)direction^STEPPER_MJ->PARAMS.invert_direction));
    STEPPER_MJ->PARAMS.direction = direction;
    return STEPPERS_MJ_get_direction(stepper_id);
}

/** @brief Sets the direction of both left motors. */
STEPPERS_MJ_DIRECTIONS STEPPERS_MJ_set_left_direction(STEPPERS_MJ_DIRECTIONS direction)
{
    STEPPERS_GROUP_MJ_t *left_steppers = &LEFT_STEPPERS;
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        STEPPERS_MJ_set_direction(left_steppers->steppers_id[stepper] ,direction);
    }
    left_steppers->direction = direction;
    return STEPPERS_MJ_get_left_direction();
}

/** @brief Sets the direction of both right motors. */
STEPPERS_MJ_DIRECTIONS STEPPERS_MJ_set_right_direction(STEPPERS_MJ_DIRECTIONS direction)
{
    STEPPERS_GROUP_MJ_t *right_steppers = &RIGHT_STEPPERS;
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        STEPPERS_MJ_set_direction(right_steppers->steppers_id[stepper] ,direction);
    }
    right_steppers->direction = direction;
    return STEPPERS_MJ_get_right_direction();
}

/** @brief Returns the last commanded direction of one motor. */
STEPPERS_MJ_DIRECTIONS STEPPERS_MJ_get_direction(uint8_t stepper_id)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return DIR_ERROR;
    }
    return STEPPERS_MJ[stepper_id].PARAMS.direction;
}

/** @brief Returns the last commanded direction of the left group. */
STEPPERS_MJ_DIRECTIONS STEPPERS_MJ_get_left_direction(void)
{
    return LEFT_STEPPERS.direction;
}

/** @brief Returns the last commanded direction of the right group. */
STEPPERS_MJ_DIRECTIONS STEPPERS_MJ_get_right_direction(void)
{
    return RIGHT_STEPPERS.direction;
}

/** @brief Returns 1 when the motor has an active open-loop target. */
int8_t STEPPERS_MJ_is_stepper_busy(uint8_t stepper_id)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return -1;
    }
    return STEPPERS_MJ[stepper_id].PARAMS.is_there_target;
}

/** @brief Returns 1 when the left group has an active target. */
int8_t STEPPERS_MJ_is_left_busy(void)
{
    return LEFT_STEPPERS.is_there_target;
}

/** @brief Returns 1 when the right group has an active target. */
int8_t STEPPERS_MJ_is_right_busy(void)
{
    return RIGHT_STEPPERS.is_there_target;
}

/** @brief Writes the step ARR value and sets CCR to ARR/2 (50% duty). */
void STEPPERS_MJ_set_ARR(uint8_t stepper_id, uint32_t newARR)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return;
    }
    // Update ARR with the new period
    __HAL_TIM_SET_AUTORELOAD(&STEPPERS_MJ[stepper_id].htim, newARR);
    // Set CCR (Pulse) to half of the new ARR
    __HAL_TIM_SET_COMPARE(&STEPPERS_MJ[stepper_id].htim,
                          STEPPERS_MJ[stepper_id].channel,
                          (uint32_t)(newARR / 2));
}

/** @brief Writes the left step ARRs and matches the side timer (2 x ARR). */
void STEPPERS_MJ_set_left_ARR(uint32_t newARR)
{
    STEPPERS_GROUP_MJ_t *left_steppers = &LEFT_STEPPERS;
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        STEPPERS_MJ_set_ARR(left_steppers->steppers_id[stepper] ,newARR);
    }
    // When applying a new ARR (stepper speed) the side target timer should match this new speed
    __HAL_TIM_SET_AUTORELOAD(&(left_steppers->set_target_timer), newARR*2);
}

/** @brief Writes the right step ARRs and matches the side timer (2 x ARR). */
void STEPPERS_MJ_set_right_ARR(uint32_t newARR)
{
    STEPPERS_GROUP_MJ_t *right_steppers = &RIGHT_STEPPERS;
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        STEPPERS_MJ_set_ARR(right_steppers->steppers_id[stepper] ,newARR);
    }
    __HAL_TIM_SET_AUTORELOAD(&(right_steppers->set_target_timer), newARR*2);
}

/** @brief Returns the current step ARR of one motor. */
uint32_t STEPPERS_MJ_get_ARR(uint8_t stepper_id)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return 0;
    }
    return __HAL_TIM_GET_AUTORELOAD(&STEPPERS_MJ[stepper_id].htim);
}

/** @brief Returns the current step ARR of the left motors. */
uint32_t STEPPERS_MJ_get_left_ARR(void)
{
    return __HAL_TIM_GET_AUTORELOAD(&STEPPERS_MJ[LEFT_STEPPERS.steppers_id[0]].htim);
}

/** @brief Returns the current step ARR of the right motors. */
uint32_t STEPPERS_MJ_get_right_ARR(void)
{
    return __HAL_TIM_GET_AUTORELOAD(&STEPPERS_MJ[RIGHT_STEPPERS.steppers_id[0]].htim);
}

/** @brief Converts a linear speed (mm/s) into the timer ARR used for OC-toggle step
   generation on a 16-bit timer:
       f_step = f_tim / (2 * (ARR + 1))    (toggle halves f_tim)
   where f_tim is the step timer kernel clock supplied by
   STEPPERS_MJ_set_timer_clock_hz() (SystemCoreClock by default). The ARR is
   clamped to fit the 16-bit auto-reload register. */
static uint32_t STEPPERS_MJ_speed_to_arr(float speed_mm_s)
{
    float  f_step = speed_mm_s * s_steps_per_mm;
    float  arr_f;
    uint32_t arr;
    uint32_t timer_clock_hz = (s_timer_clock_hz != 0U) ? s_timer_clock_hz
                                                       : (uint32_t)SystemCoreClock;

    if (f_step < 1.0f)
    {
        f_step = 1.0f;
    }
    arr_f = ((float)timer_clock_hz / (2.0f * f_step)) - 1.0f;
    if (arr_f < 0.0f)
    {
        arr_f = 0.0f;
    }
    arr = (uint32_t)arr_f;
    if (arr < STEPPERS_MJ_MIN_ARR)
    {
        arr = STEPPERS_MJ_MIN_ARR;
    }
    else if (arr > 65534U)
    {
        arr = 65534U;
    }
    else
    {
        /* within range */
    }
    return arr;
}

/** @brief Sets the linear speed of one motor (mm/s, clamped, ARR derived from the clock). */
float STEPPERS_MJ_set_speed_mm_s(uint8_t stepper_id, float speed_mm_s)
{
    float speed = speed_mm_s;

    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return -1.0f;
    }
    if (speed > STEPPERS_MJ_MAX_SPEED_MM_S)
    {
        speed = STEPPERS_MJ_MAX_SPEED_MM_S;
    }
    if (speed <= 0.0f)
    {
        STEPPERS_MJ_stop(stepper_id);
        STEPPERS_MJ[stepper_id].PARAMS.speed = 0.0f;
        return 0.0f;
    }
    if (speed < STEPPERS_MJ_MIN_SPEED_MM_S)
    {
        speed = STEPPERS_MJ_MIN_SPEED_MM_S;
    }
    STEPPERS_MJ_set_ARR(stepper_id, STEPPERS_MJ_speed_to_arr(speed));
    STEPPERS_MJ[stepper_id].PARAMS.speed = speed;
    return speed;
}

/** @brief Sets the speed of both left motors and matches the side timer ARR. */
float STEPPERS_MJ_set_left_speed_mm_s(float speed_mm_s)
{
    float speed = speed_mm_s;
    STEPPERS_GROUP_MJ_t *left_steppers = &LEFT_STEPPERS;
    uint32_t group_arr;

    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        speed = STEPPERS_MJ_set_speed_mm_s(left_steppers->steppers_id[stepper], speed_mm_s);
    }
    /* The side target timer must run at the step rate: in OC toggle mode the
       output frequency is half the timer frequency, so use 2x the step ARR. */
    group_arr = STEPPERS_MJ_get_left_ARR() * 2U;
    if (group_arr > 65534U)
    {
        group_arr = 65534U;
    }
    __HAL_TIM_SET_AUTORELOAD(&(left_steppers->set_target_timer), group_arr);
    left_steppers->speed = speed;
    return speed;
}

/** @brief Sets the speed of both right motors and matches the side timer ARR. */
float STEPPERS_MJ_set_right_speed_mm_s(float speed_mm_s)
{
    float speed = speed_mm_s;
    STEPPERS_GROUP_MJ_t *right_steppers = &RIGHT_STEPPERS;
    uint32_t group_arr;

    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        speed = STEPPERS_MJ_set_speed_mm_s(right_steppers->steppers_id[stepper], speed_mm_s);
    }
    group_arr = STEPPERS_MJ_get_right_ARR() * 2U;
    if (group_arr > 65534U)
    {
        group_arr = 65534U;
    }
    __HAL_TIM_SET_AUTORELOAD(&(right_steppers->set_target_timer), group_arr);
    right_steppers->speed = speed;
    return speed;
}

/** @brief Returns the last commanded speed of one motor. */
float STEPPERS_MJ_get_speed_mm_s(uint8_t stepper_id)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return -1.0f;
    }
    return STEPPERS_MJ[stepper_id].PARAMS.speed;
}

/** @brief Returns the last commanded speed of the left group. */
float STEPPERS_MJ_get_left_speed_mm_s(void)
{
    return LEFT_STEPPERS.speed;
}

/** @brief Returns the last commanded speed of the right group. */
float STEPPERS_MJ_get_right_speed_mm_s(void)
{
    return RIGHT_STEPPERS.speed;
}

/** @brief Sets a step target for one motor and marks it active. */
int32_t STEPPERS_MJ_set_target_ticks(uint8_t stepper_id, int32_t target_ticks)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return -1;
    }
    STEPPERS_MJ_t *STEPPER_MJ          = &STEPPERS_MJ[stepper_id];
    STEPPER_MJ->PARAMS.target_ticks    = target_ticks;
    STEPPER_MJ->PARAMS.target_position = (float)target_ticks / s_steps_per_mm;
    STEPPER_MJ->PARAMS.is_there_target = 1;
    return STEPPERS_MJ_get_target_ticks(stepper_id);
}

/** @brief Sets the same step target for both left motors. */
int32_t STEPPERS_MJ_set_left_target_ticks(int32_t target_ticks)
{
    int32_t target = 0;
    STEPPERS_GROUP_MJ_t *left_steppers = &LEFT_STEPPERS;
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        target = STEPPERS_MJ_set_target_ticks(left_steppers->steppers_id[stepper], target_ticks);
    }
    left_steppers->tick_target     = target;
    left_steppers->position_target = (float)target / s_steps_per_mm;
    left_steppers->is_there_target = 1;
    return target;
}

/** @brief Sets the same step target for both right motors. */
int32_t STEPPERS_MJ_set_right_target_ticks(int32_t target_ticks)
{
    int32_t target = 0;
    STEPPERS_GROUP_MJ_t *right_steppers = &RIGHT_STEPPERS;
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        target = STEPPERS_MJ_set_target_ticks(right_steppers->steppers_id[stepper], target_ticks);
    }
    right_steppers->tick_target     = target;
    right_steppers->position_target = (float)target / s_steps_per_mm;
    right_steppers->is_there_target = 1;
    return target;
}

/** @brief Sets a mm target for one motor and marks it active. */
float STEPPERS_MJ_set_target_position(uint8_t stepper_id, float target_position)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return -1.0f;
    }
    STEPPERS_MJ_t *STEPPER_MJ          = &STEPPERS_MJ[stepper_id];
    STEPPER_MJ->PARAMS.target_position = target_position;
    STEPPER_MJ->PARAMS.target_ticks    = (int32_t)(target_position * s_steps_per_mm);
    STEPPER_MJ->PARAMS.is_there_target = 1;
    return STEPPERS_MJ_get_target_position(stepper_id);
}

/** @brief Sets the same mm target for both left motors. */
float STEPPERS_MJ_set_left_target_position(float target_position)
{
    float target = 0;
    STEPPERS_GROUP_MJ_t *left_steppers = &LEFT_STEPPERS;
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        target = STEPPERS_MJ_set_target_position(left_steppers->steppers_id[stepper], target_position);
    }
    left_steppers->position_target = target;
    left_steppers->tick_target     = (int32_t)(target * s_steps_per_mm);
    left_steppers->is_there_target = 1;
    return target;
}

/** @brief Sets the same mm target for both right motors. */
float STEPPERS_MJ_set_right_target_position(float target_position)
{
    float target = 0;
    STEPPERS_GROUP_MJ_t *right_steppers = &RIGHT_STEPPERS;
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        target = STEPPERS_MJ_set_target_position(right_steppers->steppers_id[stepper], target_position);
    }
    right_steppers->position_target = target;
    right_steppers->tick_target     = (int32_t)(target * s_steps_per_mm);
    right_steppers->is_there_target = 1;
    return target;
}

/** @brief Returns the stored step target of one motor. */
int32_t STEPPERS_MJ_get_target_ticks(uint8_t stepper_id)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return -1;
    }
    return STEPPERS_MJ[stepper_id].PARAMS.target_ticks;
}

/** @brief Returns the stored step target of the left group. */
int32_t STEPPERS_MJ_get_left_target_ticks(void)
{
    return LEFT_STEPPERS.tick_target;
}

/** @brief Returns the stored step target of the right group. */
int32_t STEPPERS_MJ_get_right_target_ticks(void)
{
    return RIGHT_STEPPERS.tick_target;
}

/** @brief Returns the stored mm target of one motor. */
float STEPPERS_MJ_get_target_position(uint8_t stepper_id)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return -1.0f;
    }
    return STEPPERS_MJ[stepper_id].PARAMS.target_position;
}

/** @brief Returns the stored mm target of the left group. */
float STEPPERS_MJ_get_left_target_position(void)
{
    return LEFT_STEPPERS.position_target;
}

/** @brief Returns the stored mm target of the right group. */
float STEPPERS_MJ_get_right_target_position(void)
{
    return RIGHT_STEPPERS.position_target;
}

/** @brief Side timer handler: counts left steps, fires the completion callback. */
void STEPPERS_MJ_set_left_target_callback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == LEFT_STEPPERS.set_target_timer.Instance)
    {
        if (LEFT_STEPPERS.is_there_target)
        {
            STEPPERS_GROUP_MJ_t *left_steppers = &LEFT_STEPPERS;
            left_steppers->timer_counter++;
            left_steppers->absolut_ticks += (left_steppers->direction == FORWARD) ? 1 : -1;
            if(left_steppers->timer_counter >= left_steppers->tick_target)
            {
                left_steppers->timer_counter   = 0;
                left_steppers->is_there_target = 0;
                left_steppers->reach_target_callback();
            }
        }
    }
}

/** @brief Side timer handler: counts right steps, fires the completion callback. */
void STEPPERS_MJ_set_right_target_callback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == RIGHT_STEPPERS.set_target_timer.Instance)
    {
        if (RIGHT_STEPPERS.is_there_target)
        {
            STEPPERS_GROUP_MJ_t *right_steppers = &RIGHT_STEPPERS;
            right_steppers->timer_counter++;
            right_steppers->absolut_ticks += (right_steppers->direction == FORWARD) ? 1 : -1;
            if(right_steppers->timer_counter >= right_steppers->tick_target)
            {
                right_steppers->timer_counter   = 0;
                right_steppers->is_there_target = 0;
                right_steppers->reach_target_callback();
            }
        }
    }
}

/** @brief Returns the open-loop step counter of one motor. */
int32_t STEPPERS_MJ_get_absolut_ticks(uint8_t stepper_id)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return -1;
    }
    return STEPPERS_MJ[stepper_id].PARAMS.absolut_ticks;
}

/** @brief Returns the open-loop step counter of the left group. */
int32_t STEPPERS_MJ_get_left_absolut_ticks(void)
{
    return LEFT_STEPPERS.absolut_ticks;
}

/** @brief Returns the open-loop step counter of the right group. */
int32_t STEPPERS_MJ_get_right_absolut_ticks(void)
{
    return RIGHT_STEPPERS.absolut_ticks;
}

/** @brief Returns the position in mm derived from one motor's step counter. */
float STEPPERS_MJ_get_absolut_position(uint8_t stepper_id)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return -1.0f;
    }
    STEPPERS_MJ_t *STEPPER_MJ           = &STEPPERS_MJ[stepper_id];
    STEPPER_MJ->PARAMS.absolut_position = (float)STEPPER_MJ->PARAMS.absolut_ticks / s_steps_per_mm;
    return STEPPER_MJ->PARAMS.absolut_position;
}

/** @brief Returns the position in mm of the left group. */
float STEPPERS_MJ_get_left_absolut_position(void)
{
    STEPPERS_GROUP_MJ_t *left_steppers = &LEFT_STEPPERS;
    left_steppers->absolut_position = (float)left_steppers->absolut_ticks / s_steps_per_mm;
    return left_steppers->absolut_position;
}

/** @brief Returns the position in mm of the right group. */
float STEPPERS_MJ_get_right_absolut_position(void)
{
    STEPPERS_GROUP_MJ_t *right_steppers = &RIGHT_STEPPERS;
    right_steppers->absolut_position = (float)right_steppers->absolut_ticks / s_steps_per_mm;
    return right_steppers->absolut_position;
}

/** @brief Clears the open-loop counters of one motor. */
void STEPPERS_MJ_reset_absolut_counters(uint8_t stepper_id)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return;
    }
    STEPPERS_MJ_t *STEPPER_MJ           = &STEPPERS_MJ[stepper_id];
    STEPPER_MJ->PARAMS.absolut_position = 0.0;
    STEPPER_MJ->PARAMS.absolut_ticks    = 0;
}

/** @brief Clears the open-loop counters of both left motors. */
void STEPPERS_MJ_reset_left_absolut_counters(void)
{
    STEPPERS_GROUP_MJ_t *left_steppers = &LEFT_STEPPERS;
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        STEPPERS_MJ_reset_absolut_counters(left_steppers->steppers_id[stepper]);
    }
}

/** @brief Clears the open-loop counters of both right motors. */
void STEPPERS_MJ_reset_right_absolut_counters(void)
{
    STEPPERS_GROUP_MJ_t *right_steppers = &RIGHT_STEPPERS;
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        STEPPERS_MJ_reset_absolut_counters(right_steppers->steppers_id[stepper]);
    }
}

/** @brief Enables one driver and starts its step channel. */
void STEPPERS_MJ_start(uint8_t stepper_id)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return;
    }
    STEPPERS_MJ_enable_stepper(stepper_id);
    HAL_TIM_OC_Start(&STEPPERS_MJ[stepper_id].htim, STEPPERS_MJ[stepper_id].channel);
}

/** @brief Restarts the left side timer and starts both left step channels. */
void STEPPERS_MJ_start_left(void)
{
    STEPPERS_GROUP_MJ_t *left_steppers = &LEFT_STEPPERS;
    __HAL_TIM_SET_COUNTER(&(left_steppers->set_target_timer), 0);
    left_steppers->timer_counter = 0;
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        STEPPERS_MJ_start(left_steppers->steppers_id[stepper]);
    }
    if (left_steppers->is_there_target)
    {
        /* Open-loop fallback active: count the steps with the side timer. */
        (void)HAL_TIM_Base_Start_IT(&(left_steppers->set_target_timer));
    }
    else
    {
        (void)HAL_TIM_Base_Stop_IT(&(left_steppers->set_target_timer));
    }
}

/** @brief Restarts the right side timer and starts both right step channels. */
void STEPPERS_MJ_start_right(void)
{
    STEPPERS_GROUP_MJ_t *right_steppers = &RIGHT_STEPPERS;
    __HAL_TIM_SET_COUNTER(&(right_steppers->set_target_timer), 0);
    right_steppers->timer_counter = 0;
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        STEPPERS_MJ_start(right_steppers->steppers_id[stepper]);
    }
    if (right_steppers->is_there_target)
    {
        /* Open-loop fallback active: count the steps with the side timer. */
        (void)HAL_TIM_Base_Start_IT(&(right_steppers->set_target_timer));
    }
    else
    {
        (void)HAL_TIM_Base_Stop_IT(&(right_steppers->set_target_timer));
    }
}

/** @brief Stops the step channel of one motor. */
void STEPPERS_MJ_stop(uint8_t stepper_id)
{
    if (stepper_id >= STEPPERS_MJ_MAX_STEPPERS_NUMBER)
    {
        return;
    }
    HAL_TIM_OC_Stop(&STEPPERS_MJ[stepper_id].htim, STEPPERS_MJ[stepper_id].channel);
}

/** @brief Stops both left channels and clears the group target state. */
void STEPPERS_MJ_stop_left(void)
{
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        STEPPERS_MJ_stop(LEFT_STEPPERS.steppers_id[stepper]);
    }
    LEFT_STEPPERS.is_there_target = 0;
    LEFT_STEPPERS.timer_counter   = 0;
    (void)HAL_TIM_Base_Stop_IT(&LEFT_STEPPERS.set_target_timer);
    __HAL_TIM_CLEAR_FLAG(&LEFT_STEPPERS.set_target_timer, TIM_FLAG_UPDATE);
}

/** @brief Stops both right channels and clears the group target state. */
void STEPPERS_MJ_stop_right(void)
{
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER/2; stepper++)
    {
        STEPPERS_MJ_stop(RIGHT_STEPPERS.steppers_id[stepper]);
    }
    RIGHT_STEPPERS.is_there_target = 0;
    RIGHT_STEPPERS.timer_counter   = 0;
    (void)HAL_TIM_Base_Stop_IT(&RIGHT_STEPPERS.set_target_timer);
    __HAL_TIM_CLEAR_FLAG(&RIGHT_STEPPERS.set_target_timer, TIM_FLAG_UPDATE);
}

/** @brief Stops all four motors and clears both group target states. */
void STEPPERS_MJ_stop_all(void)
{
    for (uint8_t stepper = 0; stepper < STEPPERS_MJ_MAX_STEPPERS_NUMBER; stepper++)
    {
        STEPPERS_MJ_stop(stepper);
    }
    LEFT_STEPPERS.is_there_target  = 0;
    LEFT_STEPPERS.timer_counter    = 0;
    RIGHT_STEPPERS.is_there_target = 0;
    RIGHT_STEPPERS.timer_counter   = 0;
    (void)HAL_TIM_Base_Stop_IT(&LEFT_STEPPERS.set_target_timer);
    (void)HAL_TIM_Base_Stop_IT(&RIGHT_STEPPERS.set_target_timer);
}

/** @brief Sets a step target for the left group and starts the motion. */
void STEPPERS_MJ_left_go_to_ticks(int32_t target)
{
    STEPPERS_MJ_set_left_target_ticks(target);
    STEPPERS_MJ_start_left();
}

/** @brief Sets a step target for the right group and starts the motion. */
void STEPPERS_MJ_right_go_to_ticks(int32_t target)
{
    STEPPERS_MJ_set_right_target_ticks(target);
    STEPPERS_MJ_start_right();
}

/** @brief Sets a mm target for the left group and starts the motion. */
void STEPPERS_MJ_left_go_to_position(float target)
{
    STEPPERS_MJ_set_left_target_position(target);
    STEPPERS_MJ_start_left();
}

/** @brief Sets a mm target for the right group and starts the motion. */
void STEPPERS_MJ_right_go_to_position(float target)
{
    STEPPERS_MJ_set_right_target_position(target);
    STEPPERS_MJ_start_right();
}
