/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_LCD_H
#define APP_LCD_H

#include <stdint.h>
#include "app_lcd_panel.h"

/* Wildfire 4.3-inch 480x272 / 5-inch 800x480 RGB panel on the base-board LCD connector (J1),
 * driven by LTDC layer 0 from an RGB565 framebuffer at the start of SDRAM.
 * The display is optional: a failure is logged and the arm runs without it. */
#define APP_LCD_WIDTH   APP_LCD_UI_WIDTH
#define APP_LCD_HEIGHT  APP_LCD_UI_HEIGHT
/* Constants above are maximum buffer dimensions. Drawing APIs use native
 * pixels and clip to the runtime physical dimensions returned here. */
uint16_t AppLcd_PhysicalWidth(void);
uint16_t AppLcd_PhysicalHeight(void);
uint8_t AppLcd_PanelModel(void);

#define APP_LCD_RGB565(r, g, b) \
  ((uint16_t)((((r) & 0xF8U) << 8) | (((g) & 0xFCU) << 3) | ((b) >> 3)))

/* Brings up SDRAM and LTDC and draws the first boot frame before the
 * backlight is enabled. Call before the scheduler starts.
 * Returns 1 when the display is running. */
uint8_t AppLcd_Init(void (*first_frame)(uint32_t));

/* 1 when AppLcd_Init() succeeded. */
uint8_t AppLcd_Ready(void);
/* Application HAL tick at backlight enable; valid after successful init. */
uint32_t AppLcd_FirstFrameTick(void);

/* DMA2D register-to-memory fill.  The rectangle is clipped to the screen. */
void AppLcd_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/* The same fill into any RGB565 buffer `pitch` pixels wide (no clipping).
 * For cached memory the caller owns cache maintenance before/after DMA. */
void AppLcd_FillRectIn(uint16_t *buffer, uint16_t pitch, uint16_t x, uint16_t y, uint16_t w,
                       uint16_t h, uint16_t color);

/* DMA2D copy of a w x h RGB565 image (rows `source_pitch` pixels apart) to
 * the screen at (x, y).  The caller must clean a cached source before DMA. */
void AppLcd_Blit(const uint16_t *source, uint16_t source_pitch, uint16_t x, uint16_t y,
                 uint16_t w, uint16_t h);

/* Off-screen RGB565 drawing area in SDRAM, APP_LCD_SCRATCH_PIXELS pixels,
 * owned by the LCD task.  Cacheable by default: discard old cache lines
 * before a full DMA clear, invalidate again after DMA, draw, then clean
 * before AppLcd_Blit.  The whole enclosing 1 MB belongs to this buffer. */
#define APP_LCD_SCRATCH_PIXELS (APP_LCD_WIDTH * APP_LCD_HEIGHT)
uint16_t *AppLcd_Scratch(void);

/* Text in the 8x16 (size 16) or 12x24 (size 24) ASCII font, drawn with its
 * background; characters that do not fit on the screen are dropped. */
void AppLcd_DrawChar(uint16_t x, uint16_t y, char c, uint8_t size, uint16_t fg, uint16_t bg);
void AppLcd_DrawText(uint16_t x, uint16_t y, const char *text, uint8_t size,
                     uint16_t fg, uint16_t bg);
/* One non-ASCII character of the 4.3in Chinese UI: a 16x16 anti-aliased glyph
 * (two text columns), blended from bg to fg; '?' when the font has none. */
void AppLcd_DrawWide(uint16_t x, uint16_t y, uint16_t code, uint16_t fg, uint16_t bg);

#endif /* APP_LCD_H */
