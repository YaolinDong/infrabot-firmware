/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <stdint.h>
#include "app_config.h"

typedef struct {
    volatile uint8_t control_enabled;
    volatile uint8_t zero_valid;
    volatile float zero_offset[DM_MOTOR_COUNT];
    volatile float command_position[DM_MOTOR_COUNT];
    volatile uint8_t profile_ready;
    volatile uint8_t enabled[DM_MOTOR_COUNT];
    /* v4 commands retain ownership until the next legacy all-motor ENABLE. */
    volatile uint8_t override[DM_MOTOR_COUNT];
    volatile uint8_t stopped[DM_MOTOR_COUNT];
} DmMotorRuntime;

extern DmMotorRuntime dm_motor_runtime;

/* Nonzero boot default: keep the per-drive KP_APR cache in initialized DTCM. */
extern volatile float dm_posvel_kp_apr[DM_MOTOR_COUNT];
