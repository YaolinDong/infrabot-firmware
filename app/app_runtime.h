/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

#include "app_state.h"
#include "dm_motor.h"
#include "FreeRTOS.h"
#include "task.h"

extern FDCAN_HandleTypeDef hfdcan2;
extern UART_HandleTypeDef huart1;
extern TaskHandle_t dm_control_task_handle;
extern TaskHandle_t dm_status_task_handle;
extern TaskHandle_t dm_ethernet_task_handle;

#endif /* APP_RUNTIME_H */
