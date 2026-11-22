/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef MOCK_APP_STATE_H
#define MOCK_APP_STATE_H
#include <stdint.h>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#include "app_config.h"
#include "dm_types.h"
#include "bus_state.h"
#include "control_stats.h"
#include "dm_runtime.h"
#include "eth_state.h"
#include "uart_tx_state.h"
typedef DM_Motor_t MockMotor;
extern MockMotor dm_motors[8];
extern const char *const dm_motor_names[8];
extern const float dm_motor_position_max[8],dm_motor_velocity_max[8],dm_motor_torque_max[8];
uint32_t HAL_GetTick(void);
#endif
