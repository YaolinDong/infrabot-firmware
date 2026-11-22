/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "control_stats.h"
#include "dm_runtime.h"
#include "eth_state.h"
#include "app_panel.h"

#include "app_config.h"
#include "app_enable.h"
#include "app_fdcan.h"
#include "app_local_control.h"
#include "app_panel_logic.h"
#include "app_safety.h"
#include "app_startup.h"
#include "app_state.h"
#include "main.h"
#include "task.h"

#include <stdio.h>

/* Wildfire H743 board (核心板/底板原理图).  KEY1 = PA0 (WK_UP; on the base
 * board also WIFI_INT through 1K) and KEY2 = PC13, pulled down on the board,
 * high while pressed.  RGB LED D6 R/G/B = PH10/PH11/PH12, common anode: a low
 * pin lights.  PH10-PH12 are also camera-connector data lines; leave the
 * camera unplugged.  None of these pins are in fdcantest.ioc. */
#define PANEL_KEY1_PORT  GPIOA
#define PANEL_KEY1_PIN   GPIO_PIN_0
#define PANEL_KEY2_PORT  GPIOC
#define PANEL_KEY2_PIN   GPIO_PIN_13
#define PANEL_LED_PORT   GPIOH
#define PANEL_LED_RED    GPIO_PIN_10
#define PANEL_LED_GREEN  GPIO_PIN_11
#define PANEL_LED_BLUE   GPIO_PIN_12
#define PANEL_LED_PINS   (PANEL_LED_RED | PANEL_LED_GREEN | PANEL_LED_BLUE)

static void AppPanel_PinInit(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  HAL_GPIO_WritePin(PANEL_LED_PORT, PANEL_LED_PINS, GPIO_PIN_SET); /* off */
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = PANEL_LED_PINS;
  HAL_GPIO_Init(PANEL_LED_PORT, &gpio);

  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pin = PANEL_KEY1_PIN;
  HAL_GPIO_Init(PANEL_KEY1_PORT, &gpio);
  gpio.Pin = PANEL_KEY2_PIN;
  HAL_GPIO_Init(PANEL_KEY2_PORT, &gpio);
}

static uint8_t AppPanel_KeyDown(GPIO_TypeDef *port, uint16_t pin)
{
  return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
}

/* BSRR writes only: other tasks may initialize pins of the same ports. */
static void AppPanel_ShowColor(uint8_t color)
{
  uint16_t lit = 0U;
  if (color & APP_PANEL_RED) lit |= PANEL_LED_RED;
  if (color & APP_PANEL_GREEN) lit |= PANEL_LED_GREEN;
  if (color & APP_PANEL_BLUE) lit |= PANEL_LED_BLUE;
  PANEL_LED_PORT->BSRR = ((uint32_t)lit << 16) | (uint16_t)(PANEL_LED_PINS & ~lit);
}

static void AppPanel_Inputs(AppPanelInputs *in)
{
  InfrabotSafetyStatus stop = {0};

  taskENTER_CRITICAL();
  AppSafety_Report(&stop);
  taskEXIT_CRITICAL();
  in->arm_power_lost = AppStartup_ArmWaiting(NULL);
  in->starting = dm_control_stats.task_started == 0U;
  in->fault = AppFdcan_BusFault();
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; ++i)
    if (dm_motor_runtime.enabled[i] && INFRABOT_DRIVE_IS_FAULT(dm_motors[i].state)) in->fault = 1U;
  in->stop_reason = stop.stop_reason;
  in->setup = AppEnable_InProgress() || eth_maintenance.pending ||
              eth_maintenance.waiting || eth_maintenance.profile_pending;
  in->enabled = dm_motor_runtime.control_enabled;
  in->control_mode = eth_udp_command.control_mode;
}

/* A key before the control task runs would act on a half-started arm. */
static void AppPanel_Request(uint8_t action, const char *what)
{
  if (dm_control_stats.task_started == 0U)
  {
    printf("PANEL: %s ignored, motor bring-up still running\r\n", what);
    return;
  }
  AppLocalControl_Request(action);
  printf("PANEL: %s\r\n", what);
}

static void AppPanel_Task(void *argument)
{
  (void)argument;
  AppPanelKey stop_key, resume_key;
  uint8_t shown = 0xFFU;

  AppPanel_KeyInit(&stop_key, AppPanel_KeyDown(PANEL_KEY1_PORT, PANEL_KEY1_PIN));
  AppPanel_KeyInit(&resume_key, AppPanel_KeyDown(PANEL_KEY2_PORT, PANEL_KEY2_PIN));
  TickType_t wake_time = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(DM_PANEL_PERIOD_MS));
    if (AppPanel_KeySample(&stop_key, AppPanel_KeyDown(PANEL_KEY1_PORT, PANEL_KEY1_PIN),
                           DM_PANEL_PERIOD_MS, 0U) == APP_PANEL_KEY_PRESS)
      AppPanel_Request(APP_LOCAL_STOP, "KEY1 STOP");
    if (AppPanel_KeySample(&resume_key, AppPanel_KeyDown(PANEL_KEY2_PORT, PANEL_KEY2_PIN),
                           DM_PANEL_PERIOD_MS, DM_PANEL_RESUME_HOLD_MS) == APP_PANEL_KEY_LONG)
      AppPanel_Request(APP_LOCAL_RESUME, "KEY2 RESUME");

    AppPanelInputs inputs;
    AppPanel_Inputs(&inputs);
    const uint8_t color = AppPanel_LightColor(AppPanel_Light(&inputs), HAL_GetTick());
    if (color != shown)
    {
      AppPanel_ShowColor(color);
      shown = color;
    }
  }
}

BaseType_t AppPanel_CreateTask(void)
{
  AppPanel_PinInit();
  return xTaskCreate(AppPanel_Task, "panel", DM_PANEL_TASK_STACK, NULL, DM_PANEL_TASK_PRIORITY, NULL);
}
