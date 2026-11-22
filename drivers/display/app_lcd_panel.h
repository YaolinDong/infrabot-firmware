/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_LCD_PANEL_H
#define APP_LCD_PANEL_H
#include <stdint.h>
/* Maximum supported dimensions. All drawing/touch uses native pixels. */
#define APP_LCD_UI_WIDTH 800U
#define APP_LCD_UI_HEIGHT 480U
#define APP_LCD_PANEL_AUTO 0U
#define APP_LCD_PANEL_43 43U
#define APP_LCD_PANEL_5 5U
typedef struct {
  uint16_t width, height;
  uint16_t hsw, hbp, hfp, vsw, vbp, vfp;
  uint16_t pll_n, pll_r;
  uint8_t clock_mhz, model;
} AppLcdPanel;
const AppLcdPanel *AppLcdPanel_Get(uint8_t model, uint8_t legacy43);
/* Match factory touch range, not the chip ID (GT115x is used on both sizes). */
uint8_t AppLcdPanel_Detect(uint16_t x, uint16_t y);
uint16_t AppLcdPanel_ConfigRegister(const char *id);
/* Validate range/model and return native landscape coordinates; no scaling. */
uint8_t AppLcdPanel_Touch(uint16_t x, uint16_t y, uint16_t range_x, uint16_t range_y,
                          uint8_t model, uint16_t *logical_x, uint16_t *logical_y);
#endif
