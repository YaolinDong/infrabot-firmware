/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_ethernet.h"

/* Share the wrap-extended DWT clock with the network and telemetry tasks. */
static inline __attribute__((always_inline)) uint64_t platform_now_us(void) { return AppEthernet_GetTimeUs(); }
static inline __attribute__((always_inline)) uint32_t platform_now_ms(void) { return HAL_GetTick(); }
static inline __attribute__((always_inline)) void platform_enter_critical(void) { taskENTER_CRITICAL(); }
static inline __attribute__((always_inline)) void platform_exit_critical(void) { taskEXIT_CRITICAL(); }
