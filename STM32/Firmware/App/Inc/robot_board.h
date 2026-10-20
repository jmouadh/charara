/******************************************************************************
 * @file    robot_board.h
 * @brief   Board support layer: peripheral bring-up and handle ownership.
 *
 * @details Contains the initialization section that normally lives in the
 *          CubeMX generated main.c, organized and renamed in BSP_* form.
 *          main.c stays minimal and application logic lives in the App layer.
 *
 *          CubeMX workflow: if the .ioc is re-generated, take the regenerated
 *          bodies of the clock/MPU/MX_* functions and re-apply them here
 *          (they are preserved in Docs/reference/main_cubemx_reference.txt),
 *          then minimize main.c again as documented in Docs/02.
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#ifndef ROBOT_BOARD_H
#define ROBOT_BOARD_H

#include "main.h" /* pin map and HAL types, maintained together with the .ioc */

/* ------------------------- peripheral handle ownership ---------------------- */
extern I2C_HandleTypeDef hi2c1;   /**< I2C1: slave 0x0A + master for the OLED.  */
extern TIM_HandleTypeDef htim2;   /**< STEP1 generation.                        */
extern TIM_HandleTypeDef htim3;   /**< STEP2 generation.                        */
extern TIM_HandleTypeDef htim4;   /**< STEP3 generation.                        */
extern TIM_HandleTypeDef htim5;   /**< Right odometry encoder.                  */
extern TIM_HandleTypeDef htim7;   /**< 100 Hz odometry sampling timer.          */
extern TIM_HandleTypeDef htim8;   /**< Left odometry encoder.                   */
extern TIM_HandleTypeDef htim9;   /**< RGB LED PWM (R and G).                   */
extern TIM_HandleTypeDef htim11;  /**< RGB LED PWM (B).                         */
extern TIM_HandleTypeDef htim12;  /**< STEP4 generation.                        */
extern TIM_HandleTypeDef htim13;  /**< Right side target counter.               */
extern TIM_HandleTypeDef htim14;  /**< Left side target counter.                */
extern UART_HandleTypeDef huart1; /**< Debug UART (initialized, not used).      */
extern IWDG_HandleTypeDef hiwdg;  /**< Independent watchdog.                    */

/* ------------------------------- public API --------------------------------- */

/** @brief  Full board bring-up: MPU, caches, HAL, clock and all peripherals.
 *  @note   Replaces the generated initialization section of main.c; must be
 *          the first call of the application before any driver init. */
void BSP_init(void);

/** @brief Starts the independent watchdog (~500 ms, LSI based). */
void BSP_watchdog_init(void);

/** @brief Refreshes the watchdog; call from the main loop / a periodic task. */
void BSP_watchdog_refresh(void);

/** @brief Returns the APB1 timer kernel clock [Hz] (TIM2/3/4/6/7/12/13/14).
 *  @note  APB1 timers run at 2 x PCLK1 when the APB1 prescaler is not 1
 *         (108 MHz at 216 MHz SYSCLK with APB1 /4), at PCLK1 otherwise. */
uint32_t BSP_apb1_timer_clk_hz(void);

/** @brief Returns the APB2 timer kernel clock [Hz] (TIM1/8/9/10/11).
 *  @note  Same x2 rule as APB1: 216 MHz at 216 MHz SYSCLK with APB2 /2. */
uint32_t BSP_apb2_timer_clk_hz(void);

#endif /* ROBOT_BOARD_H */
