/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_LCD_ZH_H
#define APP_LCD_ZH_H

#include <stdint.h>

/* Chinese text of the 4.3in UI: UTF-8 strings, 16x16 glyphs.
 *
 * A text column is 8 px (the 8x16 ASCII font); every non-ASCII character is
 * two columns wide.  Only the characters of app_lcd_ui_zh.inc have a glyph
 * (tools/gen_lcd_zh_font.py); pure C, no HAL, also built by host tests. */

#define APP_LCD_ZH_GLYPH_BYTES 128U   /* 16x16, 4-bit coverage, high nibble = left pixel */

/* Next character of *text (advanced past it); 0 at the end.  A malformed or
 * truncated sequence gives '?' and skips one byte; 4-byte sequences give '?'. */
uint16_t AppLcdZh_Next(const char **text);

/* Glyph of a non-ASCII character, NULL when the font has none. */
const uint8_t *AppLcdZh_Glyph(uint16_t code);

/* Display columns: ASCII 1, anything else 2. */
static inline unsigned AppLcdZh_Width(uint16_t code) { return code < 0x80U ? 1U : 2U; }
unsigned AppLcdZh_Columns(const char *text);

/* Appends the UTF-8 encoding of code (BMP) to out; returns its length. */
unsigned AppLcdZh_Encode(uint16_t code, char *out);

/* RGB565 colour of each of the 16 coverage levels between bg (0) and fg (15). */
void AppLcdZh_Palette(uint16_t fg, uint16_t bg, uint16_t out[16]);

#endif /* APP_LCD_ZH_H */
