/******************************************************************************
 * @file    OLED_MouadhJeridi.h
 * @brief   High level SSD1306 OLED helpers: init, mirroring, text placement
 *          and scrolling helpers used by the robot status screen.
 *
 * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
 * @version 2.3.3
 * @date    2026-09-25
 ******************************************************************************
 */
#ifndef __OLED_MOUADHJERIDI_H__
#define __OLED_MOUADHJERIDI_H__

#include "ssd1306_fonts.h"


#define OLED_MJ_WIDTH                                 SSD1306_WIDTH
#define OLED_MJ_HEIGHT                                SSD1306_HEIGHT
#define MIRROR_HORIZENTAL_NORMAL_CMD                  0xA1
#define MIRROR_HORIZENTAL_FLIPPED_CMD                 0xA0
#define MIRROR_VERTICAL_NORMAL_CMD                    0xC8
#define MIRROR_VERTICAL_FLIPPED_CMD                   0xC0

extern uint8_t current_mirror_horizental_pos;
extern uint8_t current_mirror_vertical_pos;
extern const uint8_t fire_bitmap[];

/** @brief  Initializes the SSD1306 controller and the mirroring state. */
void OLED_MJ_Init(void);

/** @brief  Toggles the horizontal segment remapping (A0/A1). */
void OLED_MJ_mirror_horizental(void);

/** @brief  Toggles the vertical COM scan direction (C0/C8). */
void OLED_MJ_mirror_vertical(void);

/** @brief  @return Rendered width in pixels of a string for the given font.
 *  @param  str   NUL terminated text (NULL yields 0).
 *  @param  Font  Font descriptor to measure with. */
uint8_t OLED_MJ_string_width(char* str, SSD1306_Font_t Font);

/** @brief  Writes text at an absolute position in the frame buffer.
 *  @param  str   NUL terminated text.
 *  @param  x     Horizontal start position in pixels.
 *  @param  y     Vertical start position in pixels.
 *  @param  Font  Font descriptor.
 *  @param  color Black or White. */
void OLED_MJ_write(char* str, int16_t x, uint8_t y, SSD1306_Font_t Font, SSD1306_COLOR color);

/** @brief  Writes horizontally centered text.
 *  @param  str   NUL terminated text.
 *  @param  y     Vertical start position in pixels.
 *  @param  Font  Font descriptor.
 *  @param  color Black or White. */
void OLED_MJ_write_in_middle(char* str, uint8_t y, SSD1306_Font_t Font, SSD1306_COLOR color);

/** @brief  Draws a marquee-animated string moving left to right.
 *  @param  str   NUL terminated text.
 *  @param  y     Vertical start position in pixels.
 *  @param  step  Horizontal step in pixels per call.
 *  @param  Font  Font descriptor.
 *  @param  color Black or White. */
void OLED_MJ_sweep_left_to_right(char* str, uint8_t y, uint8_t step, SSD1306_Font_t Font, SSD1306_COLOR color);

/** @brief  Draws a marquee-animated string moving right to left.
 *  @param  str   NUL terminated text.
 *  @param  y     Vertical start position in pixels.
 *  @param  step  Horizontal step in pixels per call.
 *  @param  Font  Font descriptor.
 *  @param  color Black or White. */
void OLED_MJ_sweep_right_to_left(char* str, uint8_t y, uint8_t step, SSD1306_Font_t Font, SSD1306_COLOR color);

#endif //__OLED_MJ_H__
