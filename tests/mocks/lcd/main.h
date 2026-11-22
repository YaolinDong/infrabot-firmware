/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef LCD_MOCK_MAIN_H
#define LCD_MOCK_MAIN_H
#include <stdint.h>

typedef struct { uint32_t CYCCNT; } MockDwt;
extern MockDwt lcd_dwt;
#define DWT (&lcd_dwt)
void SCB_InvalidateDCache_by_Addr(volatile void *address, int32_t size);
void SCB_CleanDCache_by_Addr(volatile void *address, int32_t size);
#endif
