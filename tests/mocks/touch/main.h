/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef TOUCH_MOCK_MAIN_H
#define TOUCH_MOCK_MAIN_H
#include <stdint.h>
typedef struct { unsigned unused; } GPIO_TypeDef;
extern GPIO_TypeDef touch_h,touch_i,touch_d;
#define GPIOH (&touch_h)
#define GPIOI (&touch_i)
#define GPIOD (&touch_d)
#define I2C2 ((void *)2)
#define GPIO_PIN_4 (1U<<4)
#define GPIO_PIN_5 (1U<<5)
#define GPIO_PIN_8 (1U<<8)
#define GPIO_PIN_13 (1U<<13)
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_MODE_INPUT 2
#define GPIO_MODE_AF_OD 3
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_LOW 0
#define GPIO_AF4_I2C2 4
#define I2C_ADDRESSINGMODE_7BIT 0
#define I2C_DUALADDRESS_DISABLE 0
#define I2C_OA2_NOMASK 0
#define I2C_GENERALCALL_DISABLE 0
#define I2C_NOSTRETCH_DISABLE 0
#define I2C_ANALOGFILTER_ENABLE 1
#define I2C_MEMADD_SIZE_16BIT 2
#define __HAL_RCC_GPIOH_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOI_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_I2C2_CLK_ENABLE() ((void)0)
typedef struct { unsigned Pin,Mode,Pull,Speed,Alternate; } GPIO_InitTypeDef;
typedef struct { void *Instance; struct {
 uint32_t Timing,OwnAddress1,AddressingMode,DualAddressMode,OwnAddress2,
 OwnAddress2Masks,GeneralCallMode,NoStretchMode;
} Init; } I2C_HandleTypeDef;
typedef enum { HAL_OK,HAL_ERROR } HAL_StatusTypeDef;
void HAL_GPIO_WritePin(GPIO_TypeDef *,uint16_t,unsigned);
void HAL_GPIO_Init(GPIO_TypeDef *,GPIO_InitTypeDef *);
void HAL_Delay(uint32_t);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *,uint16_t,uint16_t,uint16_t,uint8_t *,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *,uint16_t,uint16_t,uint16_t,uint8_t *,uint16_t,uint32_t);
#endif
