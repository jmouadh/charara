/******************************************************************************
 * @file    robot_display.c
 * @brief   OLED screen manager: Charara logo home screen, temporary text
 *          messages from the RPi and persistent manual mode.
 *
 * @details Display policy (protocol: DISPLAY_MODE / DISPLAY_TEXT):
 *          - the logo home screen ("Charara") is the default content and is
 *            drawn at boot;
 *          - a DISPLAY_TEXT request shows the RPi text lines. In AUTO mode
 *            the text is displayed for ROBOT_OLED_MESSAGE_HOLD_MS and the
 *            home screen then comes back; in MANUAL mode the text stays;
 *          - DISPLAY_MODE = OFF blanks the screen once.
 *          The screen is redrawn only when its content changes (boot, new
 *          message, mode change, message hold expiry) - there is no periodic
 *          refresh. A redraw only starts when no request is pending, no motion
 *          segment is active and the bus has been silent for
 *          ROBOT_I2C_BUS_QUIET_MS; the ~100 ms bus window that follows is
 *          absorbed by the protocol retry loop. A failed transfer resets the
 *          I2C peripheral before listening resumes (see robot_i2c.c).
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 3.5.0
 * @date    2026-10-19
 ******************************************************************************
 */
#include "robot_display.h"
#include "robot_i2c.h"
#include "robot_configs.h"
#include "robot_protocol.h"
#include "robot_port.h"
#include "motion.h"
#include "OLED_MouadhJeridi.h"

/** @brief Identifies what is currently shown on the glass. */
typedef enum
{
    ROBOT_DISPLAY_SCREEN_NONE = 0,  /**< not drawn yet (boot / after error).  */
    ROBOT_DISPLAY_SCREEN_HOME,      /**< logo + "Charara".                     */
    ROBOT_DISPLAY_SCREEN_TEXT,      /**< RPi text (temporary or manual mode). */
    ROBOT_DISPLAY_SCREEN_OFF        /**< blanked (DISPLAY_MODE = off).        */
} ROBOT_DISPLAY_SCREEN;

static ROBOT_DISPLAY_SCREEN s_shown = ROBOT_DISPLAY_SCREEN_NONE;
static uint32_t s_shown_msg_seq = 0U;   /**< DISPLAY_TEXT counter of the shown text. */
static uint32_t s_next_refresh_ms = 0U; /**< earliest tick of the next redraw.       */
static uint8_t s_oled_ready = 0U;       /**< controller init succeeded.              */
static uint8_t s_boot_attempts = 0U;    /**< short-interval boot retries used.       */

/** @brief Runs the complete SSD1306 init (controller + mirroring) and records
 *         whether it succeeded. A failed init is retried by the task later:
 *         the OLED may still be powering up, or the RPi may be holding the
 *         shared I2C bus while it boots, so the first attempt is not final. */
static void ROBOT_DISPLAY_oled_init(void)
{
    ssd1306_ClearError();
    OLED_MJ_Init();
    OLED_MJ_mirror_horizental();
    OLED_MJ_mirror_vertical();
    if (ssd1306_GetError() == 0U)
    {
        s_oled_ready = 1U;
        s_boot_attempts = 0U;
    }
    else
    {
        s_oled_ready = 0U;
    }
}

/** @brief Schedules the next attempt after a failed OLED transfer.
 *  @note  The first ROBOT_OLED_BOOT_RETRY_COUNT attempts are spaced by
 *         ROBOT_OLED_BOOT_RETRY_MS so a display that powers up late (or a bus
 *         released by the Pi after its own boot) is picked up quickly. Once
 *         those are exhausted the normal 10 s back-off applies - an absent
 *         display must never hammer the shared bus. */
static void ROBOT_DISPLAY_oled_failed(uint32_t now_ms)
{
    ssd1306_ClearError();
    s_oled_ready = 0U;
    s_shown = ROBOT_DISPLAY_SCREEN_NONE;
    if (s_boot_attempts < (uint8_t)ROBOT_OLED_BOOT_RETRY_COUNT)
    {
        s_boot_attempts++;
        s_next_refresh_ms = now_ms + (uint32_t)ROBOT_OLED_BOOT_RETRY_MS;
    }
    else
    {
        s_next_refresh_ms = now_ms + (uint32_t)ROBOT_OLED_BACKOFF_MS;
    }
}

/** @brief Draws the logo home screen into the OLED buffer (update follows). */
static void ROBOT_DISPLAY_fill_home(void)
{
    ssd1306_Fill(Black);
    ssd1306_DrawBitmap(30, 0, fire_bitmap, 64, 40, White);
    OLED_MJ_write_in_middle("Charara", 45, Font_11x18, White);
}

/** @brief Returns the screen that should currently be on the glass.
 *         AUTO   : home screen, unless a message window (DISPLAY_TEXT hold)
 *                  is still open;
 *         MANUAL : the RPi text lines, persistent;
 *         OFF    : blank. */
static ROBOT_DISPLAY_SCREEN ROBOT_DISPLAY_desired(uint32_t now_ms)
{
    uint8_t mode = PROTO_get_display_mode();

    if (mode == PROTO_DISPLAY_OFF)
    {
        return ROBOT_DISPLAY_SCREEN_OFF;
    }
    if (mode == PROTO_DISPLAY_MANUAL)
    {
        return ROBOT_DISPLAY_SCREEN_TEXT;
    }
    if ((PROTO_message_seq() != 0U) &&
        ((uint32_t)(now_ms - PROTO_last_message_ms()) < (uint32_t)ROBOT_OLED_MESSAGE_HOLD_MS))
    {
        return ROBOT_DISPLAY_SCREEN_TEXT; /* temporary message window */
    }
    return ROBOT_DISPLAY_SCREEN_HOME;
}

/** @brief Initializes the OLED and draws the logo home screen (skipped when
 *         the display is absent or fails to answer).
 *  @note  Returns right after the drawing command: the caller keeps the logo
 *         on screen during the RGB boot animation (ROBOT_STATUS_boot_animation),
 *         which supplies the former fixed hold time. */
void ROBOT_DISPLAY_boot_splash(void)
{
    uint32_t now_ms = ROBOT_PORT_tick_ms();

    s_shown = ROBOT_DISPLAY_SCREEN_NONE;
    s_boot_attempts = 0U;
    ROBOT_DISPLAY_oled_init();
    if (s_oled_ready == 0U)
    {
        ROBOT_DISPLAY_oled_failed(now_ms);
        return;
    }

    ROBOT_DISPLAY_fill_home();
    ssd1306_UpdateScreen();
    if (ssd1306_GetError() == 0U)
    {
        ssd1306_ClearError();
        s_shown = ROBOT_DISPLAY_SCREEN_HOME;
    }
    else
    {
        ROBOT_DISPLAY_oled_failed(now_ms);
    }
}

/** @brief Screen manager: redraws only when the content must change - new
 *         message, mode change or message hold expiry (see the file header
 *         for the display policy). */
void ROBOT_DISPLAY_task(uint32_t now_ms)
{
    ROBOT_DISPLAY_SCREEN desired = ROBOT_DISPLAY_desired(now_ms);
    uint32_t msg_seq = PROTO_message_seq();

    if ((desired == s_shown) &&
        ((desired != ROBOT_DISPLAY_SCREEN_TEXT) || (msg_seq == s_shown_msg_seq)))
    {
        return; /* the screen is already up to date */
    }
    if ((int32_t)(now_ms - s_next_refresh_ms) < 0)
    {
        return; /* spacing / back-off after an OLED error */
    }
    if (PROTO_has_pending_requests() != 0U)
    {
        return; /* serve the RPi first, retry on the next pass */
    }
    if (MOTION_get_state() != (uint8_t)MOTION_STATE_IDLE)
    {
        return; /* never hold the shared bus while a segment runs */
    }
    if (PROTO_ms_since_bus_activity(now_ms) < ROBOT_I2C_BUS_QUIET_MS)
    {
        return; /* let the running transfer finish */
    }
    if (ROBOT_I2C_display_bus_acquire() == 0U)
    {
        return; /* the RPi won the race: abort this redraw */
    }

    if (s_oled_ready == 0U)
    {
        /* A retry after the boot attempt failed (display not powered yet, or
           the Pi was still holding the bus): run the FULL controller init
           again, not just a redraw of an uninitialized panel. */
        ROBOT_DISPLAY_oled_init();
        if (s_oled_ready == 0U)
        {
            ROBOT_DISPLAY_oled_failed(now_ms);
            ROBOT_I2C_display_bus_fault();
            return;
        }
    }

    switch (desired)
    {
        case ROBOT_DISPLAY_SCREEN_HOME:
            ROBOT_DISPLAY_fill_home();
            break;

        case ROBOT_DISPLAY_SCREEN_TEXT:
            ssd1306_Fill(Black);
            ssd1306_SetCursor(2, 0);
            ssd1306_WriteString((char *)PROTO_get_display_line(0U), Font_7x10, White);
            ssd1306_SetCursor(2, 22);
            ssd1306_WriteString((char *)PROTO_get_display_line(1U), Font_7x10, White);
            ssd1306_SetCursor(2, 44);
            ssd1306_WriteString((char *)PROTO_get_display_line(2U), Font_7x10, White);
            break;

        case ROBOT_DISPLAY_SCREEN_OFF:
            ssd1306_Fill(Black);
            break;

        default:
            break;
    }
    ssd1306_UpdateScreen();

    if (ssd1306_GetError() != 0U)
    {
        /* The panel may have lost its initialization (power glitch / bus
           error): retry the whole init next time, not just the drawing. */
        ROBOT_DISPLAY_oled_failed(now_ms);
        /* The master transfer failed: the peripheral may be left BUSY, so ask
           the bus manager for a full reset before listening again. */
        ROBOT_I2C_display_bus_fault();
    }
    else
    {
        s_shown = desired;
        s_shown_msg_seq = msg_seq;
        s_next_refresh_ms = now_ms + ROBOT_OLED_REFRESH_MS;
        ROBOT_I2C_display_bus_release();
    }
}
