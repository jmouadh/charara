/******************************************************************************
 * @file    robot_app.c
 * @brief   Application layer: initialization, task steps, scheduler and the
 *          application level HAL callbacks (timers, EXTI, error handler).
 *
 * @details Task step contract (identical in bare-metal and RTOS builds):
 *          - ROBOT_APP_task_comms  : executes the protocol ring and I2C
 *            recovery; the only writer of protocol and motion accept APIs.
 *          - ROBOT_APP_task_motion : single writer of the motion engine.
 *          - ROBOT_APP_task_display: takes the I2C bus only when quiet.
 *          - ROBOT_APP_task_status : LED + watchdog refresh (must keep
 *            running - it feeds the IWDG).
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#include "robot_app.h"
#include "robot_board.h"
#include "robot_i2c.h"
#include "robot_display.h"
#include "robot_status.h"
#include "robot_port.h"
#include "robot_configs.h"
#include "robot_events.h"
#include "robot_settings.h"
#include "motion.h"
#include "robot_protocol.h"
#include "STEPPERS_MouadhJeridi.h"
#include "ENCODERS_MouadhJeridi.h"

/** @brief Stepper motor IDs per side (as wired; see robot_configs.h). */
static uint8_t s_right_motor_ids[]  = {2U, 3U};
static uint8_t s_left_motor_ids[] = {0U, 1U};

/** @brief Set once the application init finished (Error_Handler safety gate). */
static volatile uint8_t s_app_ready = 0U;

/** @brief One-time bring-up: board, drivers, protocol, motion, UI, watchdog. */
void ROBOT_APP_init(void)
{
    const ROBOT_SETTINGS *settings;

    BSP_init();

    /* Error log first: it captures the reset cause and is ready for any
       failure during the rest of the bring-up. */
    ROBOT_EVENTS_init();

    /* Calibrated settings: defaults + the newest valid flash record (driver
       independent - the values are handed over on the next lines). */
    ROBOT_SETTINGS_init();
    settings = ROBOT_SETTINGS_get();

    /* Hand the wheel mechanics over to the project-independent drivers ---- */
    STEPPERS_MJ_configure(settings->stepper_ticks_per_rev, settings->stepper_circumference_mm);
    ENCODERS_MJ_configure(settings->odom_ticks_per_rev, settings->odom_wheel_circumference_mm);

    /* Step timers (TIM2/3/4/12) use the APB1 timer kernel clock (108 MHz
       at 216 MHz SYSCLK): hand it over before any speed is computed. */
    STEPPERS_MJ_set_timer_clock_hz(BSP_apb1_timer_clk_hz());

    /* Steppers (ids as wired; left pair {2,3}, right pair {0,1}) ---------- */
    STEPPERS_MJ_Init(0U, &htim12, TIM_CHANNEL_1, EN4_Pin, GPIOB, DIR4_Pin, GPIOB, ROBOT_STEPPER_INVERT_DIR_0);
    STEPPERS_MJ_Init(1U, &htim4,  TIM_CHANNEL_1, EN3_Pin, GPIOB, DIR3_Pin, GPIOB, ROBOT_STEPPER_INVERT_DIR_1);
    STEPPERS_MJ_Init(2U, &htim3,  TIM_CHANNEL_1, EN2_Pin, GPIOC, DIR2_Pin, GPIOA, ROBOT_STEPPER_INVERT_DIR_2);
    STEPPERS_MJ_Init(3U, &htim2,  TIM_CHANNEL_1, EN1_Pin, GPIOB, DIR1_Pin, GPIOA, ROBOT_STEPPER_INVERT_DIR_3);
    STEPPERS_MJ_Init_left_steppers(s_left_motor_ids, 2U, &htim14, STEPPERS_MJ_stop_left);
    STEPPERS_MJ_Init_right_steppers(s_right_motor_ids, 2U, &htim13, STEPPERS_MJ_stop_right);
    STEPPERS_MJ_disable_left_steppers();
    STEPPERS_MJ_disable_right_steppers();

    /* Encoders and the 100 Hz odometry sampling timer --------------------- */
    ENCODERS_MJ_Init(ROBOT_ENCODER_ID_RIGHT, &htim5, ROBOT_ENCODER_INVERT_COUNT_RIGHT);
    ENCODERS_MJ_Init(ROBOT_ENCODER_ID_LEFT,  &htim8, ROBOT_ENCODER_INVERT_COUNT_LEFT);
    (void)HAL_TIM_Base_Start_IT(&htim7);

    /* Calibrated direction inversions + motion limits: the drivers are up,
       so the loaded settings can be pushed (idle, no segment runs yet). */
    ROBOT_SETTINGS_apply();

    /* Protocol, motion engine and user interface -------------------------- */
    MOTION_init();
    PROTO_init();
    ROBOT_STATUS_init();
    ROBOT_DISPLAY_boot_splash();      /* OLED logo                            */
    ROBOT_STATUS_boot_animation();    /* RGB colour walk (logo stays on screen) */

    /* Watchdog and I2C slave listen mode ---------------------------------- */
    BSP_watchdog_init();
    ROBOT_I2C_arm_listen();
    s_app_ready = 1U;
}

/** @brief Communication step: protocol execution + deferred I2C recovery. */
void ROBOT_APP_task_comms(void)
{
    PROTO_task();
    ROBOT_I2C_service_recovery();
}

/** @brief Motion step: runs the active segment and its watchdogs. */
void ROBOT_APP_task_motion(void)
{
    MOTION_task();
}

/** @brief Display step: OLED redraw (only when the content changes). */
void ROBOT_APP_task_display(void)
{
    ROBOT_DISPLAY_task(ROBOT_PORT_tick_ms());
}

/** @brief Status step: RGB LED indication + watchdog refresh. */
void ROBOT_APP_task_status(void)
{
    ROBOT_STATUS_task(ROBOT_PORT_tick_ms());
    BSP_watchdog_refresh();
}

/** @brief Bare-metal scheduler: never returns (RTOS builds omit this loop). */
void ROBOT_APP_run(void)
{
    for (;;)
    {
        ROBOT_APP_task_comms();
        ROBOT_APP_task_motion();
        ROBOT_APP_task_display();
        ROBOT_APP_task_status();
        ROBOT_PORT_idle();
    }
}

/* ===================== Application level HAL callbacks ===================== */

/** @brief Timer update dispatcher: HAL tick, odometry sampling, side counters. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
        return;
    }
    if (htim->Instance == TIM7)
    {
        /* 100 Hz odometry sampling: refresh the encoder caches first. */
        ENCODERS_MJ_poll();
        MOTION_odometry_tick();
        return;
    }
    /* Side counters (TIM13/14): only reachable through the stepper driver's
       open-loop API - the normal engine completes on the odometry encoders. */
    STEPPERS_MJ_set_left_target_callback(htim);
    STEPPERS_MJ_set_right_target_callback(htim);
}

/** @brief EXTI dispatcher: wall limit switches (hard stop) and user buttons. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ((GPIO_Pin == SW1_Pin) || (GPIO_Pin == SW4_Pin) || (GPIO_Pin == SW3_Pin))
    {
        MOTION_on_limit_switch(GPIO_Pin);
        return;
    }
    if ((GPIO_Pin == BUTTON1_Pin) || (GPIO_Pin == BUTTON2_Pin))
    {
        ROBOT_STATUS_button_event(GPIO_Pin);
        return;
    }
    /* unrelated EXTI line */
}

/** @brief Fallback completion callback for the left side group. */
void left_motors_reach_target_callback(void)
{
    STEPPERS_MJ_stop_left();
}

/** @brief Fallback completion callback for the right side group. */
void right_motors_reach_target_callback(void)
{
    STEPPERS_MJ_stop_right();
}

/** @brief Fatal error hook: safe state (motors released) then wait for the
 *         watchdog reset when the application was already running. */
void Error_Handler(void)
{
    __disable_irq();
    if (s_app_ready != 0U)
    {
        STEPPERS_MJ_stop_all();
        STEPPERS_MJ_disable_left_steppers();
        STEPPERS_MJ_disable_right_steppers();
    }
    while (1)
    {
        /* wait for the watchdog reset */
    }
}
