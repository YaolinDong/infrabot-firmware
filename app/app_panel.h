/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_PANEL_H
#define APP_PANEL_H

#include "FreeRTOS.h"

/* The development board's status light and keys (Wildfire H743 挑战者):
 *   KEY1 pressed: STOP, a controlled stop that keeps the arm enabled.
 *   KEY2 held DM_PANEL_RESUME_HOLD_MS: RESUME.
 * Both post the same requests as the LCD's STOP/RESUME buttons
 * (app_local_control.h); enable and disable stay on the LCD and the PC.  The
 * RGB light shows the controller state (app_panel_logic.h).  Its own task, so
 * the keys work without the display. */

/* Configures the pins and creates the task; call before the scheduler
 * starts (the pins share GPIO ports with other boot-time init). */
BaseType_t AppPanel_CreateTask(void);

#endif /* APP_PANEL_H */
