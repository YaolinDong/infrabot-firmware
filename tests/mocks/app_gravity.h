/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifdef APP_GRAVITY_REAL_TEST
#include "../../control/app_gravity.h"
#else
#ifndef MOCK_APP_GRAVITY_H
#define MOCK_APP_GRAVITY_H
#include <stdint.h>
static inline void AppGravity_OnStop(uint8_t why) { (void)why; }
static inline uint8_t AppGravity_Recovering(void) { return 0; }
static inline uint8_t AppGravity_Owned(void) { return 0; }
static inline uint8_t AppGravity_CanStart(void) { return 0; }
static inline uint8_t AppGravity_CanStartMode(uint8_t mode){(void)mode;return AppGravity_CanStart();}
static inline void AppGravity_Start(uint8_t mode) { (void)mode; }
#endif
#endif
