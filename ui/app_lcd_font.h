/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_LCD_FONT_H
#define APP_LCD_FONT_H

#include <stdint.h>

/* Printable ASCII (' '..'~'), 8x16 and 12x24 pixels. */
extern const uint8_t app_lcd_font16[95][16];
extern const uint8_t app_lcd_font24[95][36];

#endif /* APP_LCD_FONT_H */
