/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_STATE_H
#define APP_STATE_H

#include "app_config.h"
#include "dm_types.h"

/* Shared application state.  ISR-facing fields are volatile; ownership is
 * documented by the module that updates each field. */
extern DM_Motor_t dm_motors[DM_MOTOR_COUNT];
extern const char *const dm_motor_names[DM_MOTOR_COUNT];
extern const float dm_motor_position_max[DM_MOTOR_COUNT];
extern const float dm_motor_velocity_max[DM_MOTOR_COUNT];
extern const float dm_motor_torque_max[DM_MOTOR_COUNT];
extern const float dm_motor_kp[DM_MOTOR_COUNT];
extern const float dm_motor_kd[DM_MOTOR_COUNT];

#define DM_COMMANDS_PER_TICK \
  (((DM_MOTOR_COUNT * DM_MOTOR_COMMAND_HZ) + DM_CONTROL_HZ - 1U) / DM_CONTROL_HZ)

#endif /* APP_STATE_H */
