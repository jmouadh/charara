/******************************************************************************
 * @file    robot_display.h
 * @brief   OLED screen manager: logo home screen (boot splash), temporary
 *          RPi messages and persistent manual mode.
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#ifndef ROBOT_DISPLAY_H
#define ROBOT_DISPLAY_H

#include <stdint.h>

/** @brief Initializes the OLED and draws the logo home screen (skipped when
 *         the display is absent or fails to answer). The logo stays on screen
 *         while the RGB boot animation runs (see ROBOT_STATUS_boot_animation). */
void ROBOT_DISPLAY_boot_splash(void);

/** @brief Screen manager: shows the logo home screen by default, the RPi text
 *         lines on DISPLAY_TEXT (for ROBOT_OLED_MESSAGE_HOLD_MS in AUTO mode,
 *         persistent in MANUAL mode) and blanks the screen in OFF mode.
 *         Redraws only when the content changes, when no request is pending and
 *         the motion engine is idle (the OLED master window never runs during
 *         an active segment).
 *  @param now_ms  Current millisecond tick. */
void ROBOT_DISPLAY_task(uint32_t now_ms);

#endif /* ROBOT_DISPLAY_H */
