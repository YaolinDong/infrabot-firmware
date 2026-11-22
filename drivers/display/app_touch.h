/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_TOUCH_H
#define APP_TOUCH_H

#include <stdint.h>

/* Goodix GT9xx/GT1x capacitive touch controller of the RGB panel (GT1158 on
 * the unit tested; GT911 on others), on I2C2
 * (SCL=PH4, SDA=PH5), RST=PI8, INT=PD13.  Polled from the LCD task; the INT
 * line is only used to pick the I2C address at reset. Coordinates are raw;
 * the UI validates and maps them before dispatching local control actions. */
#define APP_TOUCH_MAX_POINTS 5U

typedef struct
{
  uint16_t x;
  uint16_t y;
} AppTouchPoint;

/* Initializes once, before or after scheduler start. Reads product ID and
 * factory coordinate range; subsequent calls return the cached result. */
uint8_t AppTouch_Init(void);

/* 1 after a successful AppTouch_Init(). */
uint8_t AppTouch_Ready(void);

/* Product ID ("1158", "911") and 8-bit I2C address found by AppTouch_Init(). */
const char *AppTouch_ProductId(void);
uint8_t AppTouch_Address(void);
/* Raw factory X/Y coordinate extents. Returns 1 for a supported resolution. */
uint8_t AppTouch_Range(uint16_t *x, uint16_t *y);

/* Reads the latest report.  Returns 1 when the controller had a new report
 * (then *count = 0..APP_TOUCH_MAX_POINTS), 0 when none was ready, or 0xFF on
 * an I2C error. */
uint8_t AppTouch_Read(AppTouchPoint points[APP_TOUCH_MAX_POINTS], uint8_t *count);

#endif /* APP_TOUCH_H */
