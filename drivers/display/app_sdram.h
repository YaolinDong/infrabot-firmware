/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_SDRAM_H
#define APP_SDRAM_H

#include <stdint.h>

/* Core-board W9825G6KH-6: 32 MB, 16-bit, FMC SDRAM bank 2 (SDNE1 = PH6). */
#define APP_SDRAM_BASE  0xD0000000UL
#define APP_SDRAM_SIZE  (32UL * 1024UL * 1024UL)

/* LCD scratch owns this entire, MPU-aligned 1 MB region.  The displayed
 * framebuffer lives below it and stays non-cacheable. */
#define APP_SDRAM_LCD_SCRATCH       (APP_SDRAM_BASE + 0x100000UL)
#define APP_SDRAM_LCD_SCRATCH_SIZE  0x100000UL
#ifndef APP_LCD_SCRATCH_CACHE
#define APP_LCD_SCRATCH_CACHE  1
#endif

/* Region 3: Normal non-cacheable SDRAM.  Region 4 overrides only the LCD
 * scratch with write-back, write-allocate caching when enabled.  Its owner
 * must maintain D-cache around DMA2D transfers.  Install before enabling
 * D-cache or accessing SDRAM; generated region 0 denies this address range. */
void AppSdram_ConfigMpu(void);

/* Configures FMC bank 2, runs the JEDEC init sequence and a short
 * address/data line check.  Returns 1 when the SDRAM answered correctly.
 * Must run before the scheduler starts (uses HAL_Delay). */
uint8_t AppSdram_Init(void);
/* 1 after AppSdram_Init succeeded (LCD init runs it; with the LCD disabled
 * the SDRAM is never started). */
uint8_t AppSdram_Ready(void);

/* SDRAM map.  0-2 MiB: LCD framebuffer + scratch, and the loader's upgrade
 * staging area (it starts at APP_SDRAM_BASE).  From 2 MiB: the MCU run log
 * (app_log_task.c).  The rest is free. */
#define APP_SDRAM_LOG_BASE  (APP_SDRAM_BASE + 0x200000UL)

#endif /* APP_SDRAM_H */
