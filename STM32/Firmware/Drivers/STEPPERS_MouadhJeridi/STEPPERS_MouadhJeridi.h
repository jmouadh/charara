/******************************************************************************
 * @file    STEPPERS_MouadhJeridi.h
 * @brief   Stepper motor driver: step pulse generation (output-compare toggle),
 *          mm/s speed control, side target groups with completion callbacks
 *          and enable/direction management for up to four NEMA17 motors.
 *
 * @details Key relations:
 *          - Step rate: f_step = f_tim / (2 * (ARR + 1)), where f_tim is the
 *            step timer kernel clock (SystemCoreClock by default; set it with
 *            STEPPERS_MJ_set_timer_clock_hz(), e.g. the 108 MHz APB1 timer
 *            clock at 216 MHz SYSCLK).
 *          - Side target timers run at twice the step ARR (the OC toggle mode
 *            halves the output frequency, so both rates match). Their update
 *            interrupt is enabled only while an open-loop target is active.
 *          - Speeds are given in mm/s and converted to ARR from f_tim; the
 *            conversion is clock-change safe once the timer clock is set.
 *
 * @note    The wheel mechanics are supplied at runtime by
 *          STEPPERS_MJ_configure(); motion limits live in the application
 *          configuration (App/Inc/robot_configs.h).
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 2.4.0
 * @date    2026-09-28
 ******************************************************************************
 */
#ifndef __STEPPERS_MOUADHJERIDI_H__
#define __STEPPERS_MOUADHJERIDI_H__


/* ****************************************** Includes ****************************************** */
/* ********************************************************************************************** */
#include "stm32f7xx_hal.h"


/* ****************************************** Defines ******************************************* */
/* ********************************************************************************************** */
#define STEPPERS_MJ_MAX_STEPPERS_NUMBER                                                          4
#define STEPPERS_MJ_MIN_SPEED_MM_S                                                           30.0f
#define STEPPERS_MJ_MAX_SPEED_MM_S                                                          900.0f
#define STEPPERS_MJ_MIN_ARR                                                                    48U


/* ****************************************** Typedefs ****************************************** */
/* ********************************************************************************************** */
/** @brief Rotation directions; values map to the DIR pin state after inversion.
 *         DIR_ERROR is returned by the getters for invalid motor IDs. */
typedef enum
{
    FORWARD      = GPIO_PIN_SET,
    BACKWARD     = GPIO_PIN_RESET,
    DIR_ERROR    = -1
} STEPPERS_MJ_DIRECTIONS;

/** @brief Per-motor runtime parameters (position, target and speed state). */
typedef struct
{
    STEPPERS_MJ_DIRECTIONS   direction;          /**< Last commanded direction.       */
    GPIO_PinState            invert_direction;   /**< DIR inversion for this motor.   */
    int32_t                  absolut_ticks;      /**< Open-loop step counter.         */
    float                    absolut_position;   /**< Position derived from ticks.    */
    int32_t                  target_ticks;       /**< Target in steps.                */
    float                    target_position;    /**< Target in mm.                   */
    uint8_t                  is_there_target;    /**< 1 while a target is active.     */
    float                    speed;              /**< Last commanded speed [mm/s].    */
}STEPPERS_MJ_PARAMS;

/** @brief One stepper instance: timer channel, GPIO mapping and parameters. */
typedef struct
{
    TIM_HandleTypeDef   htim;                    /**< Step timer (OC toggle mode).    */
    uint32_t            channel;                 /**< Timer channel of the STEP pin.  */
    uint16_t            enable_pin;              /**< Driver EN pin (active low).     */
    uint16_t            direction_pin;           /**< Driver DIR pin.                 */
    GPIO_TypeDef        *enable_port;            /**< GPIO port of EN.                */
    GPIO_TypeDef        *direction_port;         /**< GPIO port of DIR.               */
    STEPPERS_MJ_PARAMS  PARAMS;                  /**< Runtime parameters.             */
} STEPPERS_MJ_t;

/** @brief One side group (left or right pair) with its target counter timer. */
typedef struct
{
    TIM_HandleTypeDef        set_target_timer;   /**< Side counter timer (100% duty). */
    STEPPERS_MJ_DIRECTIONS   direction;          /**< Group direction.                */
    uint8_t                  steppers_id[STEPPERS_MJ_MAX_STEPPERS_NUMBER/2];
                                                 /**< Motor IDs of this side.         */
    uint32_t                 tick_target;        /**< Open-loop fallback target.      */
    float                    position_target;    /**< Target position in mm.          */
    uint8_t                  is_there_target;    /**< 1 while a target is active.     */
    int32_t                  absolut_ticks;      /**< Group step counter.             */
    float                    absolut_position;   /**< Group position in mm.           */
    uint16_t                 timer_counter;      /**< Steps counted since start.      */
    float                    speed;              /**< Last commanded speed [mm/s].    */
    void                     (*reach_target_callback)(void);
                                                 /**< Called when the target is met.  */
} STEPPERS_GROUP_MJ_t;


/* ************************************* Function prototypes ************************************ */
/* ********************************************************************************************** */

/** @brief  Supplies the mechanical data used by all position and speed
 *          conversions of the driver.
 *  @param  ticks_per_rev           Microsteps per wheel revolution (e.g. 6400).
 *  @param  wheel_circumference_mm  Wheel circumference in millimetres.
 *  @note   Values <= 0 are rejected (the previous configuration is kept).
 *          Defaults (6400 ticks, diameter 60 mm) apply until called. */
void STEPPERS_MJ_configure(float ticks_per_rev, float wheel_circumference_mm);
/** @brief  Sets the kernel clock of the step timers (mm/s -> ARR conversion).
 *  @param  timer_clock_hz  Step timer kernel clock in Hz (e.g. the APB1
 *                          timer clock: 108000000 at 216 MHz SYSCLK with
 *                          APB1 /4). 0 restores the default.
 *  @note   Default is SystemCoreClock. Call it once before the first speed
 *          command when the step timers are clocked differently. */
void STEPPERS_MJ_set_timer_clock_hz(uint32_t timer_clock_hz);
/** @brief  Updates the DIR polarity of one motor (runtime calibration).
 *  @param  stepper_id        Motor index 0..STEPPERS_MJ_MAX_STEPPERS_NUMBER-1.
 *  @param  invert_direction  GPIO_PIN_SET to invert the DIR pin for FORWARD.
 *  @note   Re-imposes the last commanded direction on the pin. */
void STEPPERS_MJ_set_invert_direction(uint8_t stepper_id, GPIO_PinState invert_direction);
/** @brief  Registers one stepper motor (timer channel + enable/direction pins).
 *  @param  stepper_id        Motor index 0..STEPPERS_MJ_MAX_STEPPERS_NUMBER-1.
 *  @param  htim              Step timer handle (already configured in OC toggle).
 *  @param  Channel           Timer channel connected to the STEP pin.
 *  @param  enable_pin        EN GPIO pin of the driver (active low).
 *  @param  enable_port       GPIO port of the EN pin.
 *  @param  direction_pin     DIR GPIO pin of the driver.
 *  @param  direction_port    GPIO port of the DIR pin.
 *  @param  invert_direction  GPIO_PIN_SET to invert the DIR polarity. */
void STEPPERS_MJ_Init(uint8_t stepper_id,
                      TIM_HandleTypeDef *htim,
                      uint32_t Channel,
                      uint16_t enable_pin,
                      GPIO_TypeDef *enable_port,
                      uint16_t direction_pin,
                      GPIO_TypeDef *direction_port,
                      GPIO_PinState invert_direction);

/** @brief  Creates the left side group: binds motor IDs to a shared side timer.
 *  @param  left_steppers_list   Array of motor IDs on the left side.
 *  @param  stepper_number       Number of motors in the array (max 2).
 *  @param  left_timer           Side counter timer (update interrupt enabled
 *                               on demand by STEPPERS_MJ_start_left).
 *  @param  reach_target_callback Function called when the fallback target is met. */
void STEPPERS_MJ_Init_left_steppers(uint8_t *left_steppers_list,
                      uint8_t stepper_number,
                      TIM_HandleTypeDef *left_timer,
                      void (*reach_target_callback)(void));

/** @brief  Creates the right side group (same semantics as the left version).
 *  @param  right_steppers_list  Array of motor IDs on the right side.
 *  @param  stepper_number       Number of motors in the array (max 2).
 *  @param  right_timer          Side counter timer (update interrupt enabled
 *                               on demand by STEPPERS_MJ_start_right).
 *  @param  reach_target_callback Function called when the fallback target is met. */
void STEPPERS_MJ_Init_right_steppers(uint8_t *right_steppers_list,
                      uint8_t stepper_number,
                      TIM_HandleTypeDef *right_timer,
                      void (*reach_target_callback)(void));

/** @brief  Enables one driver (drives its EN pin low).
 *  @param  stepper_id  Motor index. */
void STEPPERS_MJ_enable_stepper(uint8_t stepper_id);

/** @brief  Enables both drivers of the left side group. */
void STEPPERS_MJ_enable_left_steppers(void);

/** @brief  Enables both drivers of the right side group. */
void STEPPERS_MJ_enable_right_steppers(void);

/** @brief  Disables one driver (drives its EN pin high).
 *  @param  stepper_id  Motor index. */
void STEPPERS_MJ_disable_stepper(uint8_t stepper_id);

/** @brief  Disables both drivers of the left side group. */
void STEPPERS_MJ_disable_left_steppers(void);

/** @brief  Disables both drivers of the right side group. */
void STEPPERS_MJ_disable_right_steppers(void);

/** @brief  @return 1 when the motor has an active target (open-loop fallback).
 *  @param  stepper_id  Motor index; -1 for an invalid ID. */
int8_t STEPPERS_MJ_is_stepper_busy(uint8_t stepper_id);

/** @brief  @return 1 when the left group has an active target. */
int8_t STEPPERS_MJ_is_left_busy(void);

/** @brief  @return 1 when the right group has an active target. */
int8_t STEPPERS_MJ_is_right_busy(void);

/** @brief  Sets the DIR pin of one motor, applying its inversion.
 *  @param  stepper_id  Motor index.
 *  @param  direction   FORWARD or BACKWARD.
 *  @return The direction actually latched, or DIR_ERROR for an invalid ID. */
STEPPERS_MJ_DIRECTIONS STEPPERS_MJ_set_direction(uint8_t stepper_id, STEPPERS_MJ_DIRECTIONS direction);

/** @brief  Sets the direction of both left motors.
 *  @param  direction  FORWARD or BACKWARD.
 *  @return The direction actually latched. */
STEPPERS_MJ_DIRECTIONS STEPPERS_MJ_set_left_direction(STEPPERS_MJ_DIRECTIONS direction);

/** @brief  Sets the direction of both right motors.
 *  @param  direction  FORWARD or BACKWARD.
 *  @return The direction actually latched. */
STEPPERS_MJ_DIRECTIONS STEPPERS_MJ_set_right_direction(STEPPERS_MJ_DIRECTIONS direction);

/** @brief  @return The last commanded direction of one motor.
 *  @param  stepper_id  Motor index; DIR_ERROR for an invalid ID. */
STEPPERS_MJ_DIRECTIONS STEPPERS_MJ_get_direction(uint8_t stepper_id);

/** @brief  @return The last commanded direction of the left group. */
STEPPERS_MJ_DIRECTIONS STEPPERS_MJ_get_left_direction(void);

/** @brief  @return The last commanded direction of the right group. */
STEPPERS_MJ_DIRECTIONS STEPPERS_MJ_get_right_direction(void);

/** @brief  Writes the timer ARR (and CCR = ARR/2) of one step timer.
 *  @param  stepper_id  Motor index.
 *  @param  newARR      New auto-reload value (preload enabled). */
void STEPPERS_MJ_set_ARR(uint8_t stepper_id, uint32_t newARR);

/** @brief  Writes the ARR of both left step timers and of the side timer
 *          (2 x ARR so the side counter matches the step rate).
 *  @param  newARR  New step auto-reload value. */
void STEPPERS_MJ_set_left_ARR(uint32_t newARR);

/** @brief  Writes the ARR of both right step timers and of the side timer.
 *  @param  newARR  New step auto-reload value. */
void STEPPERS_MJ_set_right_ARR(uint32_t newARR);

/** @brief  @return Current ARR of one step timer (0 for an invalid ID). */
uint32_t STEPPERS_MJ_get_ARR(uint8_t stepper_id);

/** @brief  @return Current ARR of the left step timers. */
uint32_t STEPPERS_MJ_get_left_ARR(void);

/** @brief  @return Current ARR of the right step timers. */
uint32_t STEPPERS_MJ_get_right_ARR(void);

/** @brief  Sets the linear speed of one motor (clock-derived ARR conversion).
 *  @param  stepper_id   Motor index.
 *  @param  speed_mm_s   Speed in mm/s, clamped to the driver limits.
 *  @return The applied speed in mm/s, or -1 for an invalid ID. 0 stops the motor. */
float STEPPERS_MJ_set_speed_mm_s(uint8_t stepper_id, float speed_mm_s);

/** @brief  Sets the speed of both left motors and matches the side timer ARR.
 *  @param  speed_mm_s  Speed in mm/s.
 *  @return The applied speed in mm/s. */
float STEPPERS_MJ_set_left_speed_mm_s(float speed_mm_s);

/** @brief  Sets the speed of both right motors and matches the side timer ARR.
 *  @param  speed_mm_s  Speed in mm/s.
 *  @return The applied speed in mm/s. */
float STEPPERS_MJ_set_right_speed_mm_s(float speed_mm_s);

/** @brief  @return Last commanded speed of one motor, or -1 for an invalid ID. */
float STEPPERS_MJ_get_speed_mm_s(uint8_t stepper_id);

/** @brief  @return Last commanded speed of the left group. */
float STEPPERS_MJ_get_left_speed_mm_s(void);

/** @brief  @return Last commanded speed of the right group. */
float STEPPERS_MJ_get_right_speed_mm_s(void);

/** @brief  Sets a target in steps for one motor and marks it active.
 *  @param  stepper_id   Motor index.
 *  @param  target_ticks Target step count.
 *  @return The stored target, or -1 for an invalid ID. */
int32_t STEPPERS_MJ_set_target_ticks(uint8_t stepper_id, int32_t target_ticks);

/** @brief  Sets the same step target for both left motors.
 *  @param  target_ticks  Target step count.
 *  @return The stored target. */
int32_t STEPPERS_MJ_set_left_target_ticks(int32_t target_ticks);

/** @brief  Sets the same step target for both right motors.
 *  @param  target_ticks  Target step count.
 *  @return The stored target. */
int32_t STEPPERS_MJ_set_right_target_ticks(int32_t target_ticks);

/** @brief  Sets a target in mm for one motor and marks it active.
 *  @param  stepper_id        Motor index.
 *  @param  target_position   Target position in mm (signed magnitude).
 *  @return The stored target, or -1.0f for an invalid ID. */
float STEPPERS_MJ_set_target_position(uint8_t stepper_id, float target_position);

/** @brief  Sets the same mm target for both left motors.
 *  @param  target_position  Target position in mm.
 *  @return The stored target. */
float STEPPERS_MJ_set_left_target_position(float target_position);

/** @brief  Sets the same mm target for both right motors.
 *  @param  target_position  Target position in mm.
 *  @return The stored target. */
float STEPPERS_MJ_set_right_target_position(float target_position);

/** @brief  @return Target in steps of one motor (-1 for an invalid ID). */
int32_t STEPPERS_MJ_get_target_ticks(uint8_t stepper_id);

/** @brief  @return Target in steps of the left group. */
int32_t STEPPERS_MJ_get_left_target_ticks(void);

/** @brief  @return Target in steps of the right group. */
int32_t STEPPERS_MJ_get_right_target_ticks(void);

/** @brief  @return Target in mm of one motor (-1.0f for an invalid ID). */
float STEPPERS_MJ_get_target_position(uint8_t stepper_id);

/** @brief  @return Target in mm of the left group. */
float STEPPERS_MJ_get_left_target_position(void);

/** @brief  @return Target in mm of the right group. */
float STEPPERS_MJ_get_right_target_position(void);

/** @brief  Side timer update handler for the left group: counts steps and fires
 *          the reach-target callback when the open-loop target is met.
 *  @note   Call from the side timer's update interrupt only.
 *  @param  htim  Timer that generated the update event. */
void STEPPERS_MJ_set_left_target_callback(TIM_HandleTypeDef *htim);

/** @brief  Side timer update handler for the right group (same semantics).
 *  @param  htim  Timer that generated the update event. */
void STEPPERS_MJ_set_right_target_callback(TIM_HandleTypeDef *htim);

/** @brief  @return Open-loop step counter of one motor (-1 for an invalid ID). */
int32_t STEPPERS_MJ_get_absolut_ticks(uint8_t stepper_id);

/** @brief  @return Open-loop step counter of the left group. */
int32_t STEPPERS_MJ_get_left_absolut_ticks(void);

/** @brief  @return Open-loop step counter of the right group. */
int32_t STEPPERS_MJ_get_right_absolut_ticks(void);

/** @brief  @return Position in mm derived from one motor's step counter. */
float STEPPERS_MJ_get_absolut_position(uint8_t stepper_id);

/** @brief  @return Position in mm of the left group. */
float STEPPERS_MJ_get_left_absolut_position(void);

/** @brief  @return Position in mm of the right group. */
float STEPPERS_MJ_get_right_absolut_position(void);

/** @brief  Clears the step counter and position of one motor.
 *  @param  stepper_id  Motor index. */
void STEPPERS_MJ_reset_absolut_counters(uint8_t stepper_id);

/** @brief  Clears the step counters of both left motors. */
void STEPPERS_MJ_reset_left_absolut_counters(void);

/** @brief  Clears the step counters of both right motors. */
void STEPPERS_MJ_reset_right_absolut_counters(void);

/** @brief  Enables one driver and starts its step channel.
 *  @param  stepper_id  Motor index. */
void STEPPERS_MJ_start(uint8_t stepper_id);

/** @brief  Restarts the left side timer, resets the step counter and starts
 *          both left step channels. */
void STEPPERS_MJ_start_left(void);

/** @brief  Restarts the right side timer, resets the step counter and starts
 *          both right step channels. */
void STEPPERS_MJ_start_right(void);

/** @brief  Stops the step channel of one motor (whole timer is disabled).
 *  @param  stepper_id  Motor index. */
void STEPPERS_MJ_stop(uint8_t stepper_id);

/** @brief  Stops both left step channels and clears the group target state. */
void STEPPERS_MJ_stop_left(void);

/** @brief  Stops both right step channels and clears the group target state. */
void STEPPERS_MJ_stop_right(void);

/** @brief  Stops all four motors and clears both group target states. */
void STEPPERS_MJ_stop_all(void);

/** @brief  Sets a step target for the left group and starts the motion.
 *  @param  target  Target step count (unsigned magnitude). */
void STEPPERS_MJ_left_go_to_ticks(int32_t target);

/** @brief  Sets a step target for the right group and starts the motion.
 *  @param  target  Target step count (unsigned magnitude). */
void STEPPERS_MJ_right_go_to_ticks(int32_t target);

/** @brief  Sets a mm target for the left group and starts the motion.
 *  @param  target  Target position in mm (unsigned magnitude). */
void STEPPERS_MJ_left_go_to_position(float target);

/** @brief  Sets a mm target for the right group and starts the motion.
 *  @param  target  Target position in mm (unsigned magnitude). */
void STEPPERS_MJ_right_go_to_position(float target);


#endif //__STEPPERS_MJ_H__