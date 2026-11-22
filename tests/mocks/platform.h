/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include "app_state.h"

uint64_t AppEthernet_GetTimeUs(void);
static inline uint64_t platform_now_us(void) { return AppEthernet_GetTimeUs(); }
static inline uint32_t platform_now_ms(void) { return HAL_GetTick(); }
static inline void platform_enter_critical(void) { taskENTER_CRITICAL(); }
static inline void platform_exit_critical(void) { taskEXIT_CRITICAL(); }
