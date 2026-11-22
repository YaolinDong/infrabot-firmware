/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "main.h"
#include "dm_types.h"

/* 总线上所有电机已经预先保存为 CAN_BR=9；FDCAN 控制器使用 1M/5M。
   运行阶段默认发送 FD+BRS，预检阶段可暂时发送 Classic 帧。 */
#ifndef DM_CAN_USE_FD
#define DM_CAN_USE_FD 1U
#endif

/* FDCAN kernel clock = 80 MHz：仲裁段 1 Mbps @75% 采样点，
   数据段 5 Mbps @75% 采样点。数据段时序与 motor-sdk/u2canfd 的
   canfd_sp=0.75 配置保持一致。 */
#define DM_CAN_NOMINAL_PRESCALER 1U
#define DM_CAN_NOMINAL_SEG1      59U
#define DM_CAN_NOMINAL_SEG2      20U
#define DM_CAN_NOMINAL_SJW       20U
#define DM_CAN_DATA_PRESCALER    1U
#define DM_CAN_DATA_SEG1         11U
#define DM_CAN_DATA_SEG2          4U
#define DM_CAN_DATA_SJW           2U

/* 5 Mbps FD 数据段开启发送端延迟补偿。ST 的 FDCAN 示例建议
   TDC offset = DataTimeSeg1 * DataPrescaler；当前数据段为 11/4，
   因此使用 offset=11，filter=0。若更换收发器/线缆，可据示波器
   测得的回路延迟重新调整这两个值。 */
#define DM_CAN_TDC_ENABLE         1U
#define DM_CAN_TDC_OFFSET         (DM_CAN_DATA_SEG1 * DM_CAN_DATA_PRESCALER)
#define DM_CAN_TDC_FILTER         0U

/* CAN 发送跟踪钩子；应用层可实现它通过 USART1 输出调试信息。 */
void DM_Motor_CanTxTrace(uint16_t can_id,
                         const uint8_t *data,
                         uint8_t length,
                         uint8_t fd_format,
                         uint8_t brs,
                         HAL_StatusTypeDef status);

void DM_Motor_Init(DM_Motor_t *motor,
                   uint16_t id,
                   uint16_t master_id,
                   float position_max,
                   float velocity_max,
                   float torque_max);

/* 选择后续控制帧格式。CAN FD 节点也接受 Classic CAN 控制帧，启动预检时
   使用 Classic CAN 可避免某台未切换到 CAN FD 时把整条总线打入 Bus-Off。 */
void DM_Motor_SetCanFdMode(uint8_t enable);

HAL_StatusTypeDef DM_Motor_Enable(FDCAN_HandleTypeDef *hfdcan,
                                  const DM_Motor_t *motor,
                                  uint16_t mode_id);
HAL_StatusTypeDef DM_Motor_Disable(FDCAN_HandleTypeDef *hfdcan,
                                   const DM_Motor_t *motor,
                                   uint16_t mode_id);
/* 清除电机内部锁存的错误（0xFB）。使能之前发一次，否则某些故障会让电机
   接受使能、状态也显示正常，但功率级一直被禁止输出。 */
HAL_StatusTypeDef DM_Motor_ClearError(FDCAN_HandleTypeDef *hfdcan,
                                      const DM_Motor_t *motor,
                                      uint16_t mode_id);
/* Persistent output-shaft zero command (FF FF FF FF FF FF FF FE).  The DM
   motor stores this zero in its own firmware/flash; it is not an MCU-side
   coordinate offset. */
HAL_StatusTypeDef DM_Motor_SetZeroPosition(FDCAN_HandleTypeDef *hfdcan,
                                           const DM_Motor_t *motor,
                                           uint16_t mode_id);
/* 通过 0x7FF 寄存器命令设置电机控制模式。模式值见 DM4340P 手册：
   1=MIT，2=位置速度，3=速度，4=力位混控。 */
HAL_StatusTypeDef DM_Motor_SetControlMode(FDCAN_HandleTypeDef *hfdcan,
                                          const DM_Motor_t *motor,
                                          uint8_t control_mode);
HAL_StatusTypeDef DM_Motor_ReadRegister(FDCAN_HandleTypeDef *hfdcan,
                                        const DM_Motor_t *motor,
                                        uint8_t register_id);
HAL_StatusTypeDef DM_Motor_WriteRegister(FDCAN_HandleTypeDef *hfdcan,
                                         const DM_Motor_t *motor,
                                         uint8_t register_id,
                                         const uint8_t value[4]);
HAL_StatusTypeDef DM_Motor_RequestFeedback(FDCAN_HandleTypeDef *hfdcan,
                                           const DM_Motor_t *motor);
HAL_StatusTypeDef DM_Motor_SaveParameters(FDCAN_HandleTypeDef *hfdcan,
                                          const DM_Motor_t *motor);

HAL_StatusTypeDef DM_Motor_MIT(FDCAN_HandleTypeDef *hfdcan,
                               const DM_Motor_t *motor,
                               float position,
                               float velocity,
                               float kp,
                               float kd,
                               float torque);
HAL_StatusTypeDef DM_Motor_Position(FDCAN_HandleTypeDef *hfdcan,
                                    const DM_Motor_t *motor,
                                    float position,
                                    float velocity);
HAL_StatusTypeDef DM_Motor_Speed(FDCAN_HandleTypeDef *hfdcan,
                                 const DM_Motor_t *motor,
                                 float velocity);
HAL_StatusTypeDef DM_Motor_PositionCurrent(FDCAN_HandleTypeDef *hfdcan,
                                           const DM_Motor_t *motor,
                                           float position,
                                           float velocity,
                                           float current);

/* 在 FDCAN 接收回调中调用；返回 1 表示已解析出该电机反馈。 */
uint8_t DM_Motor_ProcessFeedback(DM_Motor_t *motor,
                                  uint16_t can_id,
                                  const uint8_t *data,
                                  uint8_t length);

#endif /* DM_MOTOR_H */
