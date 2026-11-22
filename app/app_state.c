/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_runtime.h"
#include "app_state.h"

DM_Motor_t dm_motors[DM_MOTOR_COUNT];
const char *const dm_motor_names[DM_MOTOR_COUNT] = {
  "DM8009P-1", "DM8009P-2", "DM4340P-3", "DM4340P-4",
  "DM4310-5", "DM4310-6", "DM4310-7", "DM4310-8"
};
const float dm_motor_position_max[DM_MOTOR_COUNT] = {
  12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f
};
const float dm_motor_velocity_max[DM_MOTOR_COUNT] = {
  45.0f, 45.0f, 10.0f, 10.0f, 30.0f, 30.0f, 30.0f, 30.0f
};
const float dm_motor_torque_max[DM_MOTOR_COUNT] = {
  54.0f, 54.0f, 28.0f, 28.0f, 10.0f, 10.0f, 10.0f, 10.0f
};
const float dm_motor_kp[DM_MOTOR_COUNT] = {
  DM_MIT_KP_8009P, DM_MIT_KP_8009P, DM_MIT_KP_4340P, DM_MIT_KP_4340P,
  DM_MIT_KP_4310, DM_MIT_KP_4310, 30.0f, DM_MIT_KP_GRIPPER
};
const float dm_motor_kd[DM_MOTOR_COUNT] = {
  DM_MIT_KD_8009P, DM_MIT_KD_8009P, DM_MIT_KD_4340P, DM_MIT_KD_4340P,
  DM_MIT_KD_4310, DM_MIT_KD_4310, DM_MIT_KD_4310, DM_MIT_KD_GRIPPER
};

TaskHandle_t dm_control_task_handle;
TaskHandle_t dm_status_task_handle;
TaskHandle_t dm_ethernet_task_handle;
