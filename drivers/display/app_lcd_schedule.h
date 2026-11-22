/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_LCD_SCHEDULE_H
#define APP_LCD_SCHEDULE_H

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/* Fractional-tick cadence, anchored to the actual start of each frame.
 * An overrun may start the next frame immediately, but never accumulates
 * old deadlines that would be replayed as a burst after the load drops. */
static inline void AppLcdSchedule_Wait(TickType_t *last_wake, uint32_t *fraction,
                                      uint32_t frame_hz)
{
  *fraction += configTICK_RATE_HZ;
  const TickType_t period = (TickType_t)(*fraction / frame_hz);
  *fraction %= frame_hz;
  vTaskDelayUntil(last_wake, period);
  *last_wake = xTaskGetTickCount();
}

#endif
