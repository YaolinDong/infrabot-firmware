/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "main.h"

#include "FreeRTOS.h"
#include "app_config.h"
#include "app_ethernet.h"
#include "app_lcd_ui.h"
#include "app_log_task.h"
#include "app_panel.h"
#include "app_startup.h"
#include "task.h"

#include <stdio.h>
#ifdef FW_NETBOOT_APP
#include "update.h"
#endif

void AppRtos_Start(void)
{
  if (AppEthernet_CreateTask() != pdPASS) Error_Handler();
#ifdef FW_NETBOOT_APP
  if(fw_is_trial()) {
    printf("NETBOOT: trial; network health check, motor tasks inhibited\r\n");
    vTaskStartScheduler();Error_Handler();return;
  }
#endif
  if (AppStartup_CreateMotorTask() != pdPASS) Error_Handler();
  if (AppPanel_CreateTask() != pdPASS) Error_Handler();
#if DM_LCD_ENABLE
  /* A display that cannot start must not stop the arm. */
  if (AppLcdUi_CreateTask() != pdPASS) printf("LCD: task create FAILED; display static\r\n");
#endif
  /* Nor can the run log. */
  if (AppLogTask_Create() != pdPASS) printf("LOG: task create FAILED; MCU run log disabled\r\n");
  printf("RTOS: ethernet/motor-startup/panel tasks created, network probe runs first, starting scheduler\r\n");
  vTaskStartScheduler();
  Error_Handler();
}

void vApplicationMallocFailedHook(void)
{
  taskDISABLE_INTERRUPTS();
  for (;;) { }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
  (void)task;
  (void)name;
  taskDISABLE_INTERRUPTS();
  for (;;) { }
}

void vApplicationTickHook(void)
{
  HAL_IncTick();
}
