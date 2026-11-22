/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef TOUCH_MOCK_TASK_H
#define TOUCH_MOCK_TASK_H
#include "FreeRTOS.h"
#define taskSCHEDULER_RUNNING 2
BaseType_t xTaskGetSchedulerState(void);
void vTaskDelay(TickType_t);
#endif
