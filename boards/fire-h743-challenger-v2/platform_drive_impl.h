/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include "dm_motor.h"
#include "app_fdcan.h"

/* Nonblocking operations on the arm bus. Use the same HAL and host-test
 * drivers. Only success/failure is used by the enable and power sequences. */
extern FDCAN_HandleTypeDef hfdcan2;
static inline __attribute__((always_inline)) void platform_restart_arm_bus(void)
{
    AppFdcan_StartForDmMotor(DM_CAN_USE_FD);
}

static inline __attribute__((always_inline)) int platform_drive_enable(const DM_Motor_t *motor, uint16_t mode)
{
    return DM_Motor_Enable(&hfdcan2, motor, mode) == HAL_OK;
}

static inline __attribute__((always_inline)) int platform_drive_disable(const DM_Motor_t *motor, uint16_t mode)
{
    return DM_Motor_Disable(&hfdcan2, motor, mode) == HAL_OK;
}

static inline __attribute__((always_inline)) int platform_drive_clear_error(const DM_Motor_t *motor, uint16_t mode)
{
    return DM_Motor_ClearError(&hfdcan2, motor, mode) == HAL_OK;
}

static inline __attribute__((always_inline)) int platform_drive_request_feedback(const DM_Motor_t *motor)
{
    return DM_Motor_RequestFeedback(&hfdcan2, motor) == HAL_OK;
}

static inline __attribute__((always_inline)) int platform_drive_mit(const DM_Motor_t *motor, float pos, float vel, float kp, float kd, float torque)
{
    return DM_Motor_MIT(&hfdcan2, motor, pos, vel, kp, kd, torque) == HAL_OK;
}

static inline __attribute__((always_inline)) int platform_drive_position(const DM_Motor_t *motor, float pos, float vel)
{
    return DM_Motor_Position(&hfdcan2, motor, pos, vel) == HAL_OK;
}

static inline __attribute__((always_inline)) int platform_drive_position_current(const DM_Motor_t *motor, float pos, float vel, float ratio)
{
    return DM_Motor_PositionCurrent(&hfdcan2, motor, pos, vel, ratio) == HAL_OK;
}

static inline __attribute__((always_inline)) int platform_drive_speed(const DM_Motor_t *motor, float vel)
{
    return DM_Motor_Speed(&hfdcan2, motor, vel) == HAL_OK;
}
