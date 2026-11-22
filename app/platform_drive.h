/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include "dm_types.h"

#ifdef PLATFORM_DRIVE_IMPL
#define PLATFORM_DECL static inline
#else
#define PLATFORM_DECL extern
#endif

PLATFORM_DECL void platform_restart_arm_bus(void);
PLATFORM_DECL int platform_drive_enable(const DM_Motor_t *motor, uint16_t mode);
PLATFORM_DECL int platform_drive_disable(const DM_Motor_t *motor, uint16_t mode);
PLATFORM_DECL int platform_drive_clear_error(const DM_Motor_t *motor, uint16_t mode);
PLATFORM_DECL int platform_drive_request_feedback(const DM_Motor_t *motor);
PLATFORM_DECL int platform_drive_mit(const DM_Motor_t *motor, float pos, float vel, float kp, float kd, float torque);
PLATFORM_DECL int platform_drive_position(const DM_Motor_t *motor, float pos, float vel);
PLATFORM_DECL int platform_drive_position_current(const DM_Motor_t *motor, float pos, float vel, float ratio);
PLATFORM_DECL int platform_drive_speed(const DM_Motor_t *motor, float vel);

#undef PLATFORM_DECL
#ifdef PLATFORM_DRIVE_IMPL
#include PLATFORM_DRIVE_IMPL
#endif
