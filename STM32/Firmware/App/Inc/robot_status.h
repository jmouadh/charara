/******************************************************************************
 * @file    robot_status.h
 * @brief   Status indication: RGB state LED (boot animation, periodic state
 *          update) and the two user button events (discrete LED toggles).
 *
 * @note    RGB colour code and behaviour: Docs/08-rgb-led-status.md;
 *          duty values in App/Inc/robot_configs.h.
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#ifndef ROBOT_STATUS_H
#define ROBOT_STATUS_H

#include <stdint.h>

/** @brief Starts the RGB PWM channels and shows the idle color (blue). */
void ROBOT_STATUS_init(void);

/** @brief Startup colour walk (red-green-blue-yellow-turquoise-purple-white)
 *         while the OLED logo is on screen, then back to the idle colour.
 *  @note  Blocking (~1.75s with the default step); call only from boot. */
void ROBOT_STATUS_boot_animation(void);

/** @brief Updates the LED color from the motion state (strafe yellow; e-stop
 *         blinks). Call periodically from the status task.
 *  @param now_ms  Current millisecond tick (drives the e-stop blink). */
void ROBOT_STATUS_task(uint32_t now_ms);

/** @brief Handles a user button EXTI event: toggles LED1 (BUTTON1) or LED2
 *         (BUTTON2). The RGB status LED is left untouched.
 *  @param gpio_pin  BUTTON1_Pin or BUTTON2_Pin. */
void ROBOT_STATUS_button_event(uint16_t gpio_pin);

#endif /* ROBOT_STATUS_H */
