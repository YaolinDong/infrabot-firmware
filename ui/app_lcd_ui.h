/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_LCD_UI_H
#define APP_LCD_UI_H

#include "FreeRTOS.h"

/* RGB LCD user interface: a navigation bar over two pages.  STATUS shows
 * controller status, control/CAN/Ethernet rates, the latched controlled stop,
 * one row per joint and the TCP pose.  NETWORK shows the address in use and
 * edits/saves a new static IP (only with every drive disabled).  Nothing on
 * the screen commands arm motion. */

/* Lowest-priority display task; does nothing when AppLcd_Init() failed. */
BaseType_t AppLcdUi_CreateTask(void);

#endif /* APP_LCD_UI_H */
