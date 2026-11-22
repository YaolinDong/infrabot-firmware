/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_touch.h"

#include "main.h"
#include "app_lcd_panel.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* 50 kHz at the 120 MHz I2C123 kernel clock (PCLK1), from the Wildfire A-4-3
 * example.  A 5-point report (1 + 40 bytes) takes ~8 ms in the LCD task. */
#define APP_TOUCH_I2C_TIMING   0x90913232U
#define APP_TOUCH_I2C_TIMEOUT  10U

#define APP_TOUCH_RST_PORT  GPIOI
#define APP_TOUCH_RST_PIN   GPIO_PIN_8
#define APP_TOUCH_INT_PORT  GPIOD
#define APP_TOUCH_INT_PIN   GPIO_PIN_13

/* GT9xx registers (same map on GT911/GT1158). */
#define GT_REG_PRODUCT_ID  0x8140U
#define GT_REG_STATUS      0x814EU  /* bit 7: report ready, bits 3..0: points */
#define GT_REG_POINTS      0x814FU  /* 8 bytes per point */
#define GT_POINT_SIZE      8U

/* INT held low while RST rises selects 0xBA (0x5D); high selects 0x28
 * (0x14).  Both are tried in case the panel straps it differently. */
static const uint8_t app_touch_addresses[] = {0xBAU, 0x28U};

static I2C_HandleTypeDef app_touch_i2c;
static uint8_t app_touch_address;
static uint8_t app_touch_ready;
static char app_touch_id[5];
static uint8_t app_touch_initialized;
static uint16_t app_touch_range_x, app_touch_range_y;

static void AppTouch_Delay(uint32_t ms)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) vTaskDelay(pdMS_TO_TICKS(ms));
  else HAL_Delay(ms);
}


static void AppTouch_Reset(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();

  HAL_GPIO_WritePin(APP_TOUCH_RST_PORT, APP_TOUCH_RST_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(APP_TOUCH_INT_PORT, APP_TOUCH_INT_PIN, GPIO_PIN_RESET);
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = APP_TOUCH_RST_PIN;
  HAL_GPIO_Init(APP_TOUCH_RST_PORT, &gpio);
  gpio.Pin = APP_TOUCH_INT_PIN;
  HAL_GPIO_Init(APP_TOUCH_INT_PORT, &gpio);

  /* GT9xx power-on timing: RST low >= 100 us, INT kept low >= 5 ms after
   * RST rises, then INT released; I2C is usable ~50 ms later. */
  AppTouch_Delay(10U);
  HAL_GPIO_WritePin(APP_TOUCH_RST_PORT, APP_TOUCH_RST_PIN, GPIO_PIN_SET);
  AppTouch_Delay(10U);
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pin = APP_TOUCH_INT_PIN;
  HAL_GPIO_Init(APP_TOUCH_INT_PORT, &gpio);
  AppTouch_Delay(60U);
}

static uint8_t AppTouch_I2cInit(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_I2C2_CLK_ENABLE();

  /* SCL=PH4, SDA=PH5; the panel carries the pull-ups. */
  gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOH, &gpio);

  app_touch_i2c.Instance = I2C2;
  app_touch_i2c.Init.Timing = APP_TOUCH_I2C_TIMING;
  app_touch_i2c.Init.OwnAddress1 = 0U;
  app_touch_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  app_touch_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  app_touch_i2c.Init.OwnAddress2 = 0U;
  app_touch_i2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  app_touch_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  app_touch_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&app_touch_i2c) != HAL_OK) return 0U;
  if (HAL_I2CEx_ConfigAnalogFilter(&app_touch_i2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK) return 0U;
  return (HAL_I2CEx_ConfigDigitalFilter(&app_touch_i2c, 0U) == HAL_OK) ? 1U : 0U;
}

static uint8_t AppTouch_ReadReg(uint8_t address, uint16_t reg, uint8_t *data, uint16_t size)
{
  return (HAL_I2C_Mem_Read(&app_touch_i2c, address, reg, I2C_MEMADD_SIZE_16BIT, data, size,
                           APP_TOUCH_I2C_TIMEOUT) == HAL_OK) ? 1U : 0U;
}

static uint8_t AppTouch_WriteReg(uint8_t address, uint16_t reg, uint8_t value)
{
  return (HAL_I2C_Mem_Write(&app_touch_i2c, address, reg, I2C_MEMADD_SIZE_16BIT, &value, 1U,
                            APP_TOUCH_I2C_TIMEOUT) == HAL_OK) ? 1U : 0U;
}

/* A powered panel normally finishes booting while the A/B loader is running.
 * Only skip its hardware reset when the ID selects a known register map,
 * both range reads agree on a supported panel, and the stale report clears. */
static uint8_t AppTouch_Probe(uint8_t require_range)
{
  for (unsigned i=0;i<sizeof(app_touch_addresses);i++){
    const uint8_t addr=app_touch_addresses[i];
    char id[5]={0};
    if (!AppTouch_ReadReg(addr,GT_REG_PRODUCT_ID,(uint8_t *)id,4U)) continue;
    const uint16_t reg=AppLcdPanel_ConfigRegister(id);
    uint8_t range[4],confirm[4];
    uint16_t x=0,y=0;
    if(reg && AppTouch_ReadReg(addr,reg,range,sizeof(range)) &&
       AppTouch_ReadReg(addr,reg,confirm,sizeof(confirm)) && !memcmp(range,confirm,sizeof(range))){
      x=(uint16_t)(range[0]|((uint16_t)range[1]<<8));
      y=(uint16_t)(range[2]|((uint16_t)range[3]<<8));
    }
    if(require_range && !AppLcdPanel_Detect(x,y))continue;
    if(!AppTouch_WriteReg(addr,GT_REG_STATUS,0U))continue;
    memcpy(app_touch_id,id,sizeof(id));
    app_touch_address=addr;app_touch_range_x=x;app_touch_range_y=y;
    return 1U;
  }
  return 0U;
}

uint8_t AppTouch_Init(void)
{
  if (app_touch_initialized) return app_touch_ready;
  app_touch_initialized=1U;
  if (!AppTouch_I2cInit()) return 0U;
  if (!AppTouch_Probe(1U)){
    AppTouch_Reset(); /* unchanged GT reset/INT/address/settling sequence */
    app_touch_ready=AppTouch_Probe(0U);
  }else{
    /* Leave RST actively deasserted just as after a hardware reset. Set
     * the output latch before switching the pin mode to avoid a reset pulse. */
    GPIO_InitTypeDef gpio={.Pin=APP_TOUCH_RST_PIN,.Mode=GPIO_MODE_OUTPUT_PP,
                           .Pull=GPIO_NOPULL,.Speed=GPIO_SPEED_FREQ_LOW};
    __HAL_RCC_GPIOI_CLK_ENABLE();
    HAL_GPIO_WritePin(APP_TOUCH_RST_PORT,APP_TOUCH_RST_PIN,GPIO_PIN_SET);
    HAL_GPIO_Init(APP_TOUCH_RST_PORT,&gpio);
    app_touch_ready=1U;
  }
  return app_touch_ready;
}

uint8_t AppTouch_Ready(void)
{
  return app_touch_ready;
}

const char *AppTouch_ProductId(void)
{
  return app_touch_id;
}

uint8_t AppTouch_Address(void)
{
  return app_touch_address;
}

uint8_t AppTouch_Read(AppTouchPoint points[APP_TOUCH_MAX_POINTS], uint8_t *count)
{
  uint8_t status = 0U;
  uint8_t raw[APP_TOUCH_MAX_POINTS * GT_POINT_SIZE];
  uint8_t n;

  *count = 0U;
  if (app_touch_ready == 0U) return 0xFFU;
  if (AppTouch_ReadReg(app_touch_address, GT_REG_STATUS, &status, 1U) == 0U) return 0xFFU;
  if ((status & 0x80U) == 0U) return 0U;

  n = status & 0x0FU;
  if (n > APP_TOUCH_MAX_POINTS) n = APP_TOUCH_MAX_POINTS;
  if ((n != 0U) && (AppTouch_ReadReg(app_touch_address, GT_REG_POINTS, raw,
                                     (uint16_t)(n * GT_POINT_SIZE)) == 0U))
    return 0xFFU;
  /* Point record: track id, x lo/hi, y lo/hi, size lo/hi, reserved. */
  for (uint8_t i = 0U; i < n; i++)
  {
    const uint8_t *p = &raw[i * GT_POINT_SIZE];
    points[i].x = (uint16_t)(p[1] | ((uint16_t)(p[2] & 0x0FU) << 8));
    points[i].y = (uint16_t)(p[3] | ((uint16_t)(p[4] & 0x0FU) << 8));
  }
  *count = n;
  /* Clearing the ready flag lets the controller post the next report. */
  return AppTouch_WriteReg(app_touch_address, GT_REG_STATUS, 0U) ? 1U : 0xFFU;
}

uint8_t AppTouch_Range(uint16_t *x, uint16_t *y)
{
  *x = app_touch_range_x; *y = app_touch_range_y;
  return app_touch_ready && AppLcdPanel_Detect(*x, *y) != APP_LCD_PANEL_AUTO;
}
