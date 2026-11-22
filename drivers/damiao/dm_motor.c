/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_motor.h"
#include "dm_protocol.h"

#include <string.h>

static uint8_t dm_can_fd_active = (DM_CAN_USE_FD != 0U) ? 1U : 0U;

void DM_Motor_SetCanFdMode(uint8_t enable)
{
    dm_can_fd_active = (enable != 0U) ? 1U : 0U;
}

__weak void DM_Motor_CanTxTrace(uint16_t can_id,
                                const uint8_t *data,
                                uint8_t length,
                                uint8_t fd_format,
                                uint8_t brs,
                                HAL_StatusTypeDef status)
{
    (void)can_id;
    (void)data;
    (void)length;
    (void)fd_format;
    (void)brs;
    (void)status;
}

static HAL_StatusTypeDef dm_send(FDCAN_HandleTypeDef *hfdcan,
                                 uint16_t can_id,
                                 const uint8_t *data,
                                 uint8_t length)
{
    FDCAN_TxHeaderTypeDef header = {0};
    HAL_StatusTypeDef status;

    if ((hfdcan == NULL) || (data == NULL) || (length > 8U))
    {
        return HAL_ERROR;
    }

    header.Identifier = can_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = (uint32_t)length;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    if (dm_can_fd_active != 0U)
    {
        header.BitRateSwitch = FDCAN_BRS_ON;
        header.FDFormat = FDCAN_FD_CAN;
    }
    else
    {
        header.BitRateSwitch = FDCAN_BRS_OFF;
        header.FDFormat = FDCAN_CLASSIC_CAN;
    }
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, (uint8_t *)data);
    DM_Motor_CanTxTrace(can_id,
                        data,
                        length,
                        (header.FDFormat == FDCAN_FD_CAN) ? 1U : 0U,
                        (header.BitRateSwitch == FDCAN_BRS_ON) ? 1U : 0U,
                        status);
    return status;
}

static HAL_StatusTypeDef dm_send_mode_command(FDCAN_HandleTypeDef *hfdcan,
                                              const DM_Motor_t *motor,
                                              uint16_t mode_id,
                                              uint8_t command)
{
    uint8_t data[8];
    DM_PackMode(command, data);

    (void)mode_id;
    if (motor == NULL)
    {
        return HAL_ERROR;
    }
    /* Lifecycle commands use the base receive ID. Mode offsets only apply to
       cyclic control frames (MIT/position/velocity/force-position). */
    return dm_send(hfdcan, motor->id, data, 8U);
}

static HAL_StatusTypeDef dm_send_register_write(FDCAN_HandleTypeDef *hfdcan,
                                                const DM_Motor_t *motor,
                                                uint8_t register_id,
                                                const uint8_t value[4])
{
    uint8_t data[8];

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    DM_PackRegister(motor->id, 0x55U, register_id, value, data);
    return dm_send(hfdcan, 0x7FFU, data, 8U);
}

void DM_Motor_Init(DM_Motor_t *motor,
                   uint16_t id,
                   uint16_t master_id,
                   float position_max,
                   float velocity_max,
                   float torque_max)
{
    if (motor == NULL)
    {
        return;
    }

    memset(motor, 0, sizeof(*motor));
    motor->id = id;
    motor->master_id = master_id;
    motor->position_max = position_max;
    motor->velocity_max = velocity_max;
    motor->torque_max = torque_max;
}

HAL_StatusTypeDef DM_Motor_Enable(FDCAN_HandleTypeDef *hfdcan,
                                  const DM_Motor_t *motor,
                                  uint16_t mode_id)
{
    return dm_send_mode_command(hfdcan, motor, mode_id, 0xFCU);
}

HAL_StatusTypeDef DM_Motor_Disable(FDCAN_HandleTypeDef *hfdcan,
                                   const DM_Motor_t *motor,
                                   uint16_t mode_id)
{
    return dm_send_mode_command(hfdcan, motor, mode_id, 0xFDU);
}

HAL_StatusTypeDef DM_Motor_ClearError(FDCAN_HandleTypeDef *hfdcan,
                                      const DM_Motor_t *motor,
                                      uint16_t mode_id)
{
    return dm_send_mode_command(hfdcan, motor, mode_id, 0xFBU);
}

HAL_StatusTypeDef DM_Motor_SetZeroPosition(FDCAN_HandleTypeDef *hfdcan,
                                            const DM_Motor_t *motor,
                                            uint16_t mode_id)
{
    return dm_send_mode_command(hfdcan, motor, mode_id, 0xFEU);
}

HAL_StatusTypeDef DM_Motor_SetControlMode(FDCAN_HandleTypeDef *hfdcan,
                                          const DM_Motor_t *motor,
                                          uint8_t control_mode)
{
    /* DM 协议：寄存器 10（CTRL_MODE），1=MIT，2=位置速度，3=速度，4=力位混控。 */
    const uint8_t value[4] = {control_mode, 0U, 0U, 0U};
    return dm_send_register_write(hfdcan, motor, 10U, value);
}

HAL_StatusTypeDef DM_Motor_ReadRegister(FDCAN_HandleTypeDef *hfdcan,
                                        const DM_Motor_t *motor,
                                        uint8_t register_id)
{
    uint8_t data[8] = {0U};

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    /* Damiao register requests use the complete motor ID in bytes 0..1. */
    DM_PackRegister(motor->id, 0x33U, register_id, NULL, data);
    return dm_send(hfdcan, 0x7FFU, data, 8U);
}

HAL_StatusTypeDef DM_Motor_WriteRegister(FDCAN_HandleTypeDef *hfdcan,
                                         const DM_Motor_t *motor,
                                         uint8_t register_id,
                                         const uint8_t value[4])
{
    if (value == NULL) return HAL_ERROR;
    return dm_send_register_write(hfdcan, motor, register_id, value);
}

HAL_StatusTypeDef DM_Motor_RequestFeedback(FDCAN_HandleTypeDef *hfdcan,
                                           const DM_Motor_t *motor)
{
    uint8_t data[8] = {0U};
    if (motor == NULL) return HAL_ERROR;
    DM_PackRegister(motor->id, 0xCCU, 0U, NULL, data);
    return dm_send(hfdcan, 0x7FFU, data, 8U);
}

HAL_StatusTypeDef DM_Motor_SaveParameters(FDCAN_HandleTypeDef *hfdcan,
                                          const DM_Motor_t *motor)
{
    uint8_t data[8] = {0U};

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    DM_PackRegister(motor->id, 0xAAU, 0x01U, NULL, data);
    return dm_send(hfdcan, 0x7FFU, data, 8U);
}

HAL_StatusTypeDef DM_Motor_MIT(FDCAN_HandleTypeDef *hfdcan,
                               const DM_Motor_t *motor,
                               float position,
                               float velocity,
                               float kp,
                               float kd,
                               float torque)
{
    uint8_t data[8];
    if (motor == NULL)
    {
        return HAL_ERROR;
    }
    DM_PackMIT(motor, position, velocity, kp, kd, torque, data);
    return dm_send(hfdcan, (uint16_t)(motor->id + DM_MODE_MIT), data, 8U);
}

HAL_StatusTypeDef DM_Motor_Position(FDCAN_HandleTypeDef *hfdcan,
                                    const DM_Motor_t *motor,
                                    float position,
                                    float velocity)
{
    uint8_t data[8];
    if (motor == NULL)
    {
        return HAL_ERROR;
    }
    DM_PackPosition(position, velocity, data);
    return dm_send(hfdcan, (uint16_t)(motor->id + DM_MODE_POSITION), data, 8U);
}

HAL_StatusTypeDef DM_Motor_Speed(FDCAN_HandleTypeDef *hfdcan,
                                 const DM_Motor_t *motor,
                                 float velocity)
{
    uint8_t data[8];
    if (motor == NULL)
    {
        return HAL_ERROR;
    }
    DM_PackSpeed(velocity, data);
    return dm_send(hfdcan, (uint16_t)(motor->id + DM_MODE_SPEED), data, 8U);
}

HAL_StatusTypeDef DM_Motor_PositionCurrent(FDCAN_HandleTypeDef *hfdcan,
                                           const DM_Motor_t *motor,
                                           float position,
                                           float velocity,
                                           float current)
{
    uint8_t data[8];
    if (motor == NULL)
    {
        return HAL_ERROR;
    }
    DM_PackPositionCurrent(position, velocity, current, data);
    return dm_send(hfdcan, (uint16_t)(motor->id + DM_MODE_POSITION_CURRENT), data, 8U);
}

uint8_t DM_Motor_ProcessFeedback(DM_Motor_t *motor,
                                  uint16_t can_id,
                                  const uint8_t *data,
                                  uint8_t length)
{
    if (!DM_DecodeFeedback(motor, can_id, data, length)) return 0U;
    motor->feedback_tick = HAL_GetTick();
    return 1U;
}
