/******************************************************************************
 * @file    robot_app.h
 * @brief   Application entry points: one-time init, the individual task steps
 *          and the bare-metal scheduler.
 *
 * @details The four task step functions are independent and RTOS-ready:
 *          run them from a bare-metal super loop (ROBOT_APP_run) or from
 *          four FreeRTOS tasks (see Docs/02-firmware-architecture.md,
 *          section "RTOS porting").
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#ifndef ROBOT_APP_H
#define ROBOT_APP_H

/** @brief One-time bring-up: board, drivers, protocol, motion, UI and the
 *         watchdog. Must be called once before any task step. */
void ROBOT_APP_init(void);

/** @brief Communication step: protocol execution + I2C recovery. */
void ROBOT_APP_task_comms(void);

/** @brief Motion step: segment control, ramps and watchdogs. */
void ROBOT_APP_task_motion(void);

/** @brief Display step: throttled OLED status refresh. */
void ROBOT_APP_task_display(void);

/** @brief Status step: RGB LED + watchdog refresh. */
void ROBOT_APP_task_status(void);

/** @brief Bare-metal scheduler: never returns.
 *  @note  In FreeRTOS builds this function is unused; each task step runs in
 *         its own task instead. */
void ROBOT_APP_run(void);

#endif /* ROBOT_APP_H */
