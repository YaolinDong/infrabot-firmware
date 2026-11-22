/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef LCD_MOCK_TASK_H
#define LCD_MOCK_TASK_H
#include "FreeRTOS.h"
void vTaskDelayUntil(TickType_t *last_wake, TickType_t period);
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);
BaseType_t xTaskCreate(void (*task)(void *),const char *name,unsigned stack,void *argument,
                      unsigned priority,void *handle);
#endif
