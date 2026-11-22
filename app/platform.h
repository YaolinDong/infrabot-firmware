/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include "stdint.h"

#ifdef PLATFORM_IMPL
#define PLATFORM_DECL static inline
#else
#define PLATFORM_DECL extern
#endif

PLATFORM_DECL uint64_t platform_now_us(void);
PLATFORM_DECL uint32_t platform_now_ms(void);
PLATFORM_DECL void platform_enter_critical(void);
PLATFORM_DECL void platform_exit_critical(void);

#undef PLATFORM_DECL
#ifdef PLATFORM_IMPL
#include PLATFORM_IMPL
#endif
