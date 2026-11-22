/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_lcd_schedule.h"
#include <assert.h>
#include <stdio.h>

static TickType_t now, wake_latency;

TickType_t xTaskGetTickCount(void) { return now; }

void vTaskDelayUntil(TickType_t *last_wake, TickType_t period)
{
  const TickType_t elapsed = now - *last_wake;
  if (elapsed < period) now += period - elapsed;
  *last_wake += period;
  now += wake_latency;
}

static void CheckCadence(TickType_t initial)
{
  now = initial;
  TickType_t last_wake = now, previous = now;
  uint32_t fraction = 0;
  for (unsigned n = 0; n < 300; ++n)
  {
    AppLcdSchedule_Wait(&last_wake, &fraction, 30);
    assert(now - previous == (n % 3 == 2 ? 34U : 33U));
    assert(last_wake == now);
    previous = now;
    now += n % 2 ? 28U : 1U; /* rendering and unchanged-frame work */
  }
  assert(previous - initial == 10000U);
}

static void CheckRecovery(TickType_t initial)
{
  now = initial;
  TickType_t last_wake = now, previous = now;
  uint32_t fraction = 0;
  AppLcdSchedule_Wait(&last_wake, &fraction, 30);
  previous = now;
  for (unsigned n = 0; n < 250; ++n)
  {
    now += 45U; /* reproduce ~22 Hz dragging for more than 10 seconds */
    AppLcdSchedule_Wait(&last_wake, &fraction, 30);
    assert(now - previous == 45U);
    previous = now;
  }
  for (unsigned n = 0; n < 90; ++n)
  {
    now += n % 2 ? 28U : 1U; /* finger released: no catch-up burst */
    AppLcdSchedule_Wait(&last_wake, &fraction, 30);
    assert(now - previous >= 33U && now - previous <= 34U);
    previous = now;
  }
  now += 1200U; /* a page action that stalls for longer than one second */
  AppLcdSchedule_Wait(&last_wake, &fraction, 30);
  assert(last_wake == now);
  previous = now;
  now += 1U;
  AppLcdSchedule_Wait(&last_wake, &fraction, 30);
  assert(now - previous >= 33U && now - previous <= 34U);
  wake_latency = 5U;
  now += 1U;
  AppLcdSchedule_Wait(&last_wake, &fraction, 30);
  assert(last_wake == now); /* late wake is the new schedule anchor */
  wake_latency = 0U;
}

int main(void)
{
  CheckCadence(0);
  CheckCadence(UINT32_MAX - 5000U);
  CheckRecovery(0);
  CheckRecovery(UINT32_MAX - 5000U);
  puts("LCD cadence: 30 Hz, overload/release recovery, long stalls and tick wrap passed");
  return 0;
}
