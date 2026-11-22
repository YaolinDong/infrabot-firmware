/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef MOCK_STM32H7XX_HAL_H
#define MOCK_STM32H7XX_HAL_H
/* Flash HAL for test_persist: the reserved sector is a RAM array, and the
 * address constants of app_persist.c are rebased onto it. */
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t TypeErase, Banks, Sector, NbSectors, VoltageRange; } FLASH_EraseInitTypeDef;
#define FLASH_TYPEERASE_SECTORS 0U
#define FLASH_SECTOR_7 7U
#define FLASH_BANK_2 2U
#define FLASH_VOLTAGE_RANGE_3 3U
#define FLASH_TYPEPROGRAM_FLASHWORD 1U
#define FLASH_FLAG_ALL_ERRORS_BANK2 0U
#define __HAL_FLASH_CLEAR_FLAG(f) ((void)(f))
extern uint8_t mock_flash[128U * 1024U];
extern uint32_t mock_flash_erases, mock_flash_programs, mock_flash_fail_after;
/* The firmware's real sector address, used as a token: reads go through
 * mock_flash_ptr(), programs map the 32-bit address back to the array.
 * The 32-bit data argument is the low half of a pointer on a 64-bit host,
 * so the test keeps the one flash-word buffer it may point to. */
#define DM_ZERO_FLASH_ADDRESS ((uint32_t)0x081E0000U)
#define MOCK_FLASH_ADDRESS 0x081E0000U
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *error);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint32_t data);
uint32_t HAL_FLASH_GetError(void);
void SCB_InvalidateDCache_by_Addr(volatile void *address, int32_t size);
#endif
