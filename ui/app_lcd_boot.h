/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_LCD_BOOT_H
#define APP_LCD_BOOT_H
#include <stdint.h>
#define APP_LCD_BOOT_MS 3000U
/* First called at LCD init, then only by the LCD task. Time since first frame (ms). */
void AppLcdBoot_Draw(uint32_t elapsed_ms);
#endif
