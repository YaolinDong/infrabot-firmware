/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef LCD_MOCK_FREERTOS_H
#define LCD_MOCK_FREERTOS_H
#include <stdint.h>
typedef uint32_t TickType_t;
typedef int BaseType_t;
#define pdPASS 1
#define pdMS_TO_TICKS(ms) (ms)
#define configTICK_RATE_HZ 1000U
#endif
