/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <stdint.h>
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define FLASH_BASE 0x08000000U
#define FLASH_TYPEPROGRAM_FLASHWORD 0
#define FLASH_TYPEERASE_SECTORS 0
#define FLASH_VOLTAGE_RANGE_3 3
#define FLASH_BANK_1 1
#define FLASH_BANK_2 2
#define FLASH_OPTSR_SWAP_BANK_OPT 1U
typedef struct {unsigned OPTSR_CUR;} MockFlash;
extern MockFlash *FLASH;
extern uint16_t flash_kb;
#define FLASHSIZE_BASE (&flash_kb)
#define __HAL_FLASH_CLEAR_FLAG(x) ((void)0)
typedef struct {unsigned TypeErase,Banks,Sector,NbSectors,VoltageRange;} FLASH_EraseInitTypeDef;
int HAL_FLASH_Unlock(void);
void HAL_FLASH_Lock(void);
int HAL_FLASH_Program(int type,uint32_t addr,uintptr_t source);
int HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e,uint32_t *error);
uint32_t HAL_GetTick(void);
void HAL_Delay(unsigned ms);
void NVIC_SystemReset(void);

extern volatile uint32_t host_mail[24];
#define FW_MAILBOX host_mail
#define __DSB() ((void)0)
