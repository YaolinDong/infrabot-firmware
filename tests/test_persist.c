/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* app_persist.c on a RAM model of the reserved flash sector: the control
 * (zero + profile), network and UI-language records survive each other's
 * saves byte for byte; invalid or failed writes keep the old values. */
#include "stm32h7xx_hal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

uint8_t mock_flash[128U * 1024U];
uint32_t mock_flash_erases, mock_flash_programs, mock_flash_fail_after = UINT32_MAX;
static uint8_t locked = 1U;
HAL_StatusTypeDef HAL_FLASH_Unlock(void) { locked = 0U; return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock(void) { locked = 1U; return HAL_OK; }
uint32_t HAL_FLASH_GetError(void) { return 0U; }
void SCB_InvalidateDCache_by_Addr(volatile void *address, int32_t size) { (void)address; (void)size; }
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *error)
{
  assert(!locked && erase->Sector == FLASH_SECTOR_7 && erase->Banks == FLASH_BANK_2 && erase->NbSectors == 1U);
  *error = 0xFFFFFFFFU;
  memset(mock_flash, 0xFF, sizeof(mock_flash));
  ++mock_flash_erases;
  return HAL_OK;
}
/* The data argument is (uint32_t)flash_word; on the host only its low 32
 * bits survive.  Recover the source from the last memcpy into flash_word. */
static const uint8_t *last_word;
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint32_t data)
{
  assert(!locked && type == FLASH_TYPEPROGRAM_FLASHWORD && last_word);
  assert((uint32_t)(uintptr_t)last_word == data);
  const uint32_t offset = address - MOCK_FLASH_ADDRESS;
  assert(offset % 32U == 0U && offset + 32U <= sizeof(mock_flash));
  if (mock_flash_programs++ >= mock_flash_fail_after) return HAL_ERROR;
  for (unsigned i = 0U; i < 32U; ++i) assert(mock_flash[offset + i] == 0xFFU);   /* programmed only once after erase */
  memcpy(&mock_flash[offset], last_word, 32U);
  return HAL_OK;
}

/* Reads of the sector: app_persist.c casts the address to a pointer. */
static void *mock_memcpy(void *dst, const void *src, size_t n)
{
  if ((uintptr_t)src == (uintptr_t)MOCK_FLASH_ADDRESS) src = mock_flash;
  void *r = memcpy(dst, src, n);
  if (n == 32U) last_word = dst;   /* the flash-word staging copy */
  return r;
}
#include "app_state.h"
MockMotor dm_motors[8];

const float dm_motor_velocity_max[8] = {45, 45, 10, 10, 30, 30, 30, 30};
const float dm_motor_torque_max[8] = {54, 54, 28, 28, 10, 10, 10, 10};
#include <string.h>
#define memcpy mock_memcpy
#include "app_persist.c"
#undef memcpy

static void profile(float base)
{
  for (unsigned i = 0U; i < DM_MOTOR_COUNT; ++i)
  {
    dm_motor_runtime.zero_offset[i] = base + .01f * (float)i;
    eth_udp_command.target_kp[i] = 10.f + base;
    eth_udp_command.target_kd[i] = .5f;
    eth_udp_command.target_velocity[i] = 0.f;
    eth_udp_command.target_torque[i] = 0.f;
    eth_udp_command.velocity_limit[i] = 1.f;
    eth_udp_command.torque_limit_ratio[i] = .5f;
    eth_udp_command.motor_mode[i] = i == DM_GRIPPER_INDEX ? DM_ETH_MODE_FORCE_POS : DM_ETH_MODE_MIT;
  }
}

int main(void)
{
  memset(mock_flash, 0xFF, sizeof(mock_flash));
  uint8_t language = 0xAAU;
  AppNetConfig net;
  /* Empty sector: nothing loads, defaults stay. */
  assert(!AppPersist_LoadLanguage(&language) && language == 0xAAU);
  assert(!AppPersist_LoadNetwork(&net) && !AppPersist_LoadZero());

  /* Language alone on an empty sector. */
  assert(AppPersist_SaveLanguage(APP_UI_LANGUAGE_ZH));
  assert(AppPersist_LoadLanguage(&language) && language == APP_UI_LANGUAGE_ZH);
  assert(!AppPersist_LoadNetwork(&net) && !AppPersist_LoadZero());

  /* Zero + profile, then network: the language survives both. */
  profile(.25f);
  assert(AppPersist_SaveZero());
  const AppNetConfig address = {{192, 168, 0, 50}, {255, 255, 255, 0}, {192, 168, 0, 1}};
  assert(AppPersist_SaveNetwork(&address));
  assert(AppPersist_LoadLanguage(&language) && language == APP_UI_LANGUAGE_ZH);
  uint8_t before[DM_PERSIST_SPAN];
  memcpy(before, mock_flash, sizeof(before));

  /* The same language again: no erase. */
  const uint32_t erases = mock_flash_erases;
  assert(AppPersist_SaveLanguage(APP_UI_LANGUAGE_ZH) && mock_flash_erases == erases);

  /* Changing the language keeps the control and network records byte for byte. */
  assert(AppPersist_SaveLanguage(APP_UI_LANGUAGE_EN) && mock_flash_erases == erases + 1U);
  assert(!memcmp(mock_flash, before, 256U + 32U));
  assert(AppPersist_LoadLanguage(&language) && language == APP_UI_LANGUAGE_EN);
  assert(AppPersist_LoadNetwork(&net) && !memcmp(&net, &address, sizeof(net)));
  for(unsigned i=0;i<DM_MOTOR_COUNT;i++)dm_motor_runtime.zero_offset[i]=0;
  assert(AppPersist_LoadZero() && dm_motor_runtime.zero_offset[3] == .25f + .03f);

  /* A control-profile save keeps the language and the network. */
  profile(.5f);
  assert(AppPersist_SaveControlProfile());
  assert(AppPersist_LoadLanguage(&language) && language == APP_UI_LANGUAGE_EN);
  assert(AppPersist_LoadNetwork(&net) && !memcmp(&net, &address, sizeof(net)));

  /* Invalid language: refused, flash untouched. */
  memcpy(before, mock_flash, sizeof(before));
  assert(!AppPersist_SaveLanguage(APP_UI_LANGUAGES) && !memcmp(mock_flash, before, sizeof(before)));

  /* A corrupted UI record reads as absent and does not disturb the others. */
  mock_flash[256U + 32U + 6U] ^= 1U;
  assert(!AppPersist_LoadLanguage(&language));
  assert(AppPersist_LoadNetwork(&net) && AppPersist_LoadZero());
  /* ... and the next network save drops it rather than rewriting garbage. */
  assert(AppPersist_SaveNetwork(&address) && !AppPersist_LoadLanguage(&language));
  for (unsigned i = 0U; i < 32U; ++i) assert(mock_flash[256U + 32U + i] == 0xFFU);

  /* A failed program reports failure (the caller keeps showing the old choice). */
  assert(AppPersist_SaveLanguage(APP_UI_LANGUAGE_ZH));
  mock_flash_fail_after = mock_flash_programs + 9U;   /* control (8 words) + network ok, UI fails */
  assert(!AppPersist_SaveLanguage(APP_UI_LANGUAGE_EN));
  mock_flash_fail_after = UINT32_MAX;
  assert(!app_persist_busy);

  /* A busy sector: refused at once, nothing written. */
  app_persist_busy = 1U;
  memcpy(before, mock_flash, sizeof(before));
  assert(!AppPersist_SaveLanguage(APP_UI_LANGUAGE_ZH) && !memcmp(mock_flash, before, sizeof(before)));
  app_persist_busy = 0U;
  puts("persist: control, network and UI-language records preserve each other; invalid, busy and failed saves handled");
  return 0;
}
