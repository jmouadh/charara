/******************************************************************************
 * @file    robot_status.c
 * @brief   Status indication implementation: RGB state LED and the two user
 *          buttons.
 *
 * @details The RGB LED is reserved for the robot state: dim blue when idle,
 *          dim green while a closed-loop segment runs, dim yellow for the
 *          dead-reckoned strafe, red for a latched fault and blinking red
 *          for an e-stop. At startup a short colour walk
 *          (red-green-blue-yellow-turquoise-purple-white) runs while the
 *          OLED logo is on screen. Duty values live in robot_configs.h and
 *          the full colour code is documented in Docs/08-rgb-led-status.md.
 *          The user buttons toggle the discrete LEDs LED1/LED2 (PC3/PA8) and
 *          never touch the RGB LED.
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.0.0
 * @date    2026-09-28
 ******************************************************************************
 */
#include "robot_status.h"
#include "robot_board.h"
#include "robot_configs.h"
#include "robot_port.h"
#include "motion.h"

/** @brief Button activity counters (LED toggles). */
static uint8_t s_button1_counter = 0U;
static uint8_t s_button2_counter = 0U;

/** @brief Writes a packed 0xRRGGBB duty value (0..255 per channel) to the RGB
 *         LED (red = TIM9 CH1, green = TIM9 CH2, blue = TIM11 CH1). */
static void ROBOT_STATUS_set_rgb(uint32_t rgb)
{
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, (rgb >> 16) & 0xFFU);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, (rgb >> 8) & 0xFFU);
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, rgb & 0xFFU);
}

/** @brief Starts the RGB PWM channels and shows the idle color. */
void ROBOT_STATUS_init(void)
{
    (void)HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
    ROBOT_STATUS_set_rgb(ROBOT_RGB_IDLE_COLOR);
}

/** @brief Startup colour walk: red, green, blue (primaries), then yellow,
 *         turquoise, purple (their pairwise mixes), then white - 250 ms per
 *         colour (ROBOT_RGB_BOOT_STEP_MS) - and back to the idle colour.
 *  @note  Blocking by design (~1.75s with the default step) and only called
 *         from ROBOT_APP_init(), before the watchdog is started; the OLED
 *         logo is drawn first and stays on screen during the walk. */
void ROBOT_STATUS_boot_animation(void)
{
    static const uint32_t boot_colors[7] =
    {
        0x300000UL, /* red                 */
        0x003000UL, /* green               */
        0x000030UL, /* blue                */
        0x303000UL, /* yellow      (R+G)   */
        0x003030UL, /* turquoise   (G+B)   */
        0x300030UL, /* purple      (R+B)   */
        0x303030UL  /* white       (R+G+B) */
    };
    uint8_t index;

    for (index = 0U; index < 7U; index++)
    {
        ROBOT_STATUS_set_rgb(boot_colors[index]);
        ROBOT_PORT_delay_ms(ROBOT_RGB_BOOT_STEP_MS);
    }
    ROBOT_STATUS_set_rgb(ROBOT_RGB_IDLE_COLOR);
}

/** @brief Maps the motion state to a LED color (strafe yellow, e-stop blinks). */
void ROBOT_STATUS_task(uint32_t now_ms)
{
    uint8_t state = MOTION_get_state();

    if (state == MOTION_STATE_ESTOP)
    {
        if (((now_ms / ROBOT_RGB_ESTOP_BLINK_MS) % 2U) == 0U)
        {
            ROBOT_STATUS_set_rgb(0x000000UL);
        }
        else
        {
            ROBOT_STATUS_set_rgb(ROBOT_RGB_ESTOP_COLOR);
        }
    }
    else if (state == MOTION_STATE_FAULT)
    {
        ROBOT_STATUS_set_rgb(ROBOT_RGB_FAULT_COLOR);
    }
    else if (state == MOTION_STATE_STRAFE)
    {
        /* Caution colour: the only move without lateral feedback. */
        ROBOT_STATUS_set_rgb(ROBOT_RGB_STRAFE_COLOR);
    }
    else if (MOTION_is_busy() != 0U)
    {
        ROBOT_STATUS_set_rgb(ROBOT_RGB_BUSY_COLOR);
    }
    else
    {
        ROBOT_STATUS_set_rgb(ROBOT_RGB_IDLE_COLOR);
    }
}

/** @brief Handles a user button EXTI event: toggles the discrete LED of that
 *         button (LED1 for BUTTON1, LED2 for BUTTON2). */
void ROBOT_STATUS_button_event(uint16_t gpio_pin)
{
    if (gpio_pin == BUTTON1_Pin)
    {
        s_button1_counter++;
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
                          ((s_button1_counter % 2U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    else if (gpio_pin == BUTTON2_Pin)
    {
        s_button2_counter++;
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,
                          ((s_button2_counter % 2U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    else
    {
        /* unrelated pin */
    }
}
