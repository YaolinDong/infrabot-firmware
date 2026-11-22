/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef MOCK_DM_MOTOR_H
#define MOCK_DM_MOTOR_H
/* Damiao CAN driver for host tests: the drives are MockMotor (app_state.h). */
#include "app_state.h"
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct { int unused; } FDCAN_HandleTypeDef;
typedef MockMotor DM_Motor_t;
extern FDCAN_HandleTypeDef hfdcan2;
#define __DMB() ((void)0)
#define DM_MODE_MIT 0x000U
#define DM_CAN_USE_FD 1U
HAL_StatusTypeDef DM_Motor_ReadRegister(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor,
                                        uint8_t register_id);
HAL_StatusTypeDef DM_Motor_WriteRegister(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor,
                                         uint8_t register_id, const uint8_t value[4]);
HAL_StatusTypeDef DM_Motor_SetZeroPosition(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor,
                                           uint16_t mode_id);
HAL_StatusTypeDef DM_Motor_SaveParameters(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor);
HAL_StatusTypeDef DM_Motor_Enable(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor, uint16_t mode_id);
HAL_StatusTypeDef DM_Motor_Disable(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor, uint16_t mode_id);
HAL_StatusTypeDef DM_Motor_ClearError(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor, uint16_t mode_id);
HAL_StatusTypeDef DM_Motor_RequestFeedback(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor);
HAL_StatusTypeDef DM_Motor_MIT(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor, float position,
                               float velocity, float kp, float kd, float torque);
HAL_StatusTypeDef DM_Motor_Position(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor, float position,
                                    float velocity);
HAL_StatusTypeDef DM_Motor_Speed(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor, float velocity);
HAL_StatusTypeDef DM_Motor_PositionCurrent(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor,
                                           float position, float velocity, float current);
#endif
