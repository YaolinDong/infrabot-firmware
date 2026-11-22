/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_sdram.h"

#include "main.h"

#include <stdio.h>

/* SDCLK = FMC kernel clock (HCLK3 = 240 MHz) / 2 = 120 MHz.
 * Refresh: 64 ms / 8192 rows = 7.81 us -> 7.81 us * 120 MHz - 20 = 917. */
#define APP_SDRAM_REFRESH_COUNT  917U
#define APP_SDRAM_TIMEOUT        0xFFFFU

/* JEDEC mode register: burst length 1, sequential, CAS 3, single write burst. */
#define APP_SDRAM_MODE_REGISTER  0x0230U

static SDRAM_HandleTypeDef app_sdram;
#ifndef FW_LOADER_BUILD
static uint8_t app_sdram_ready;
#endif

void AppSdram_ConfigMpu(void)
{
  MPU_Region_InitTypeDef region = {0};

  HAL_MPU_Disable();
  region.Enable = MPU_REGION_ENABLE;
  region.Number = MPU_REGION_NUMBER3;
  region.BaseAddress = APP_SDRAM_BASE;
  region.Size = MPU_REGION_SIZE_32MB;
  region.SubRegionDisable = 0x00U;
  region.TypeExtField = MPU_TEX_LEVEL1;
  region.AccessPermission = MPU_REGION_FULL_ACCESS;
  region.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  region.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  region.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  region.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&region);
#if APP_LCD_SCRATCH_CACHE
  region.Number = MPU_REGION_NUMBER4;
  region.BaseAddress = APP_SDRAM_LCD_SCRATCH;
  region.Size = MPU_REGION_SIZE_1MB;
  /* TEX=1, C=1, B=1: Normal, write-back, write-allocate. */
  region.IsCacheable = MPU_ACCESS_CACHEABLE;
  region.IsBufferable = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&region);
#endif
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/* The NAND flash shares the FMC data lines with the SDRAM.  Hold it
 * deselected (NCE=PG9 high, NWE=PD5 high, CLE=PD11/ALE=PD12 low) so it never
 * reacts to SDRAM bus traffic. */
static void AppSdram_DeselectNand(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);

  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = GPIO_PIN_9;
  HAL_GPIO_Init(GPIOG, &gpio);
  gpio.Pin = GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12;
  HAL_GPIO_Init(GPIOD, &gpio);
}

static void AppSdram_PinInit(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF12_FMC;

  /* A0..A5, RAS (PF11), A6..A9 */
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
             GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOF, &gpio);
  /* A10..A12, BA0/BA1, CLK (PG8), CAS (PG15) */
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 |
             GPIO_PIN_8 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOG, &gpio);
  /* D0..D3, D13..D15 */
  gpio.Pin = GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 |
             GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOD, &gpio);
  /* LDQM/UDQM, D4..D12 */
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
             GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &gpio);
  /* SDNE1 (PH6), SDCKE1 (PH7) */
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOH, &gpio);
  /* SDNWE (PC0) */
  gpio.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOC, &gpio);
}

static HAL_StatusTypeDef AppSdram_Command(uint32_t mode, uint32_t refresh, uint32_t mode_register)
{
  FMC_SDRAM_CommandTypeDef command = {0};

  command.CommandMode = mode;
  command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
  command.AutoRefreshNumber = refresh;
  command.ModeRegisterDefinition = mode_register;
  return HAL_SDRAM_SendCommand(&app_sdram, &command, APP_SDRAM_TIMEOUT);
}

/* Walks every address line (byte address bits 2..24) with a distinct value,
 * then checks a few data patterns.  Catches open/shorted address and data
 * lines in well under a millisecond, unlike a full 32 MB sweep. */
static uint8_t AppSdram_Check(void)
{
  volatile uint32_t *const base = (volatile uint32_t *)APP_SDRAM_BASE;
  static const uint32_t patterns[] = {0x00000000U, 0xFFFFFFFFU, 0xAAAA5555U, 0x5555AAAAU};

  for (uint32_t bit = 2U; bit < 25U; bit++)
    base[(1UL << bit) / 4U] = 0xA5000000U | bit;
  base[0] = 0xA5FFFFFFU;
#if APP_LCD_SCRATCH_CACHE
  /* Address bit 20 probes the cached scratch region.  Force that word to
   * physical SDRAM before checking, otherwise cache could hide a fault. */
  SCB_CleanInvalidateDCache_by_Addr((void *)APP_SDRAM_LCD_SCRATCH, 32);
#endif
  for (uint32_t bit = 2U; bit < 25U; bit++)
    if (base[(1UL << bit) / 4U] != (0xA5000000U | bit)) return 0U;
  if (base[0] != 0xA5FFFFFFU) return 0U;

  for (uint32_t i = 0U; i < sizeof(patterns) / sizeof(patterns[0]); i++)
  {
    base[1] = patterns[i];
    base[2] = ~patterns[i];
    if ((base[1] != patterns[i]) || (base[2] != ~patterns[i])) return 0U;
  }
  return 1U;
}

uint8_t AppSdram_Init(void)
{
  RCC_PeriphCLKInitTypeDef clock = {0};
  FMC_SDRAM_TimingTypeDef timing = {0};

  clock.PeriphClockSelection = RCC_PERIPHCLK_FMC;
  clock.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) return 0U;

  AppSdram_DeselectNand();
  AppSdram_PinInit();
  __HAL_RCC_FMC_CLK_ENABLE();

  app_sdram.Instance = FMC_SDRAM_DEVICE;
  app_sdram.Init.SDBank = FMC_SDRAM_BANK2;
  app_sdram.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;
  app_sdram.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
  app_sdram.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  app_sdram.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  app_sdram.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  app_sdram.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  app_sdram.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  app_sdram.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  app_sdram.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;

  /* Timings in SDCLK cycles (8.33 ns), from the Wildfire A-3 SDRAM example. */
  timing.LoadToActiveDelay = 2U;
  timing.ExitSelfRefreshDelay = 8U;
  timing.SelfRefreshTime = 5U;
  timing.RowCycleDelay = 8U;
  timing.WriteRecoveryTime = 2U;
  timing.RPDelay = 2U;
  timing.RCDDelay = 2U;
  if (HAL_SDRAM_Init(&app_sdram, &timing) != HAL_OK) return 0U;

  if (AppSdram_Command(FMC_SDRAM_CMD_CLK_ENABLE, 1U, 0U) != HAL_OK) return 0U;
  HAL_Delay(1U);
  if (AppSdram_Command(FMC_SDRAM_CMD_PALL, 1U, 0U) != HAL_OK) return 0U;
  if (AppSdram_Command(FMC_SDRAM_CMD_AUTOREFRESH_MODE, 4U, 0U) != HAL_OK) return 0U;
  if (AppSdram_Command(FMC_SDRAM_CMD_LOAD_MODE, 1U, APP_SDRAM_MODE_REGISTER) != HAL_OK) return 0U;
  if (HAL_SDRAM_ProgramRefreshRate(&app_sdram, APP_SDRAM_REFRESH_COUNT) != HAL_OK) return 0U;

#ifdef FW_LOADER_BUILD
  /* The resident loader stays byte-identical: it has no run log. */
  return AppSdram_Check();
#else
  app_sdram_ready = AppSdram_Check();
  return app_sdram_ready;
#endif
}

#ifndef FW_LOADER_BUILD
uint8_t AppSdram_Ready(void) { return app_sdram_ready; }
#endif
