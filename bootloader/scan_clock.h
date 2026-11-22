/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef FW_SCAN_CLOCK_H
#define FW_SCAN_CLOCK_H
#include <stdint.h>
/* Pre-HAL, single-owner, ordinary H743 boot only. Leave before flash writes,
 * key debounce, SysTick/HAL, maintenance and the application handoff. */
struct fw_scan_clock {
    uint32_t pll_source, pll_cfg, pll_div, d1;
    unsigned changed;
};
uint32_t fw_scan_clock_enter(struct fw_scan_clock *s);
void fw_scan_clock_leave(struct fw_scan_clock *s);
#endif
