/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_TASKS_H
#define APP_TASKS_H
#include "control_services.h"

#include "FreeRTOS.h"

#include <stdint.h>

/* What the status task measured over the last second, for readers other than
 * the UART line (the LCD).  The per-second counters themselves are reset by
 * the status task, so read them only through this copy. */
typedef struct
{
  uint32_t seconds;        /* status task seconds; 0 until the first update */
  uint32_t control_hz;
  uint32_t control_missed;
  uint32_t period_max_us;
  uint32_t can_tx;
  uint32_t can_rx;
  uint32_t can_tx_errors;
  uint32_t command_rx;
  uint32_t command_invalid;
  uint32_t telemetry_tx;
  uint32_t can_error_events; /* since boot, see dm_bus_stats.can_error_events */
  uint8_t can_tec;
  uint8_t can_rec;
  uint8_t can_error_passive;
  uint8_t can_bus_off;
} AppTasksStatusSnapshot;

/* Application task layer.  Board bring-up remains in main.c; this module owns
 * the periodic control task, low-priority status task, and their statistics. */
void AppTasks_InitControlProfile(void);
BaseType_t AppTasks_Create(void);
void AppTasks_ControlTick(void);
void AppTasks_DrainTrace(void);
void AppTasks_GetStatusSnapshot(AppTasksStatusSnapshot *snapshot);
/* Damiao drive state code (feedback byte) as text: ENABLED, OVERCURRENT, ... */
const char *AppTasks_MotorStateName(uint8_t state);

#endif /* APP_TASKS_H */
