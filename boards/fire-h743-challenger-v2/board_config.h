/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

/* LCD presence, panel detection defaults and board timing selection. */

#define DM_LCD_ENABLE               1U
/* AUTO=0, force 4.3-inch=43, force 5-inch=5. Unknown/missing touch falls
 * back explicitly; never infer the display size from GT115x ID alone. */
#ifndef DM_LCD_PANEL
#define DM_LCD_PANEL 0U
#endif
#ifndef DM_LCD_FALLBACK_PANEL
#define DM_LCD_FALLBACK_PANEL 43U
#endif
/* 0: GL043056B0-40-N (new 4.3-inch); 1: old compatible-example timing. */
#ifndef DM_LCD_43_LEGACY_TIMING
#define DM_LCD_43_LEGACY_TIMING 0U
#endif
