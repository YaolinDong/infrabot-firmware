/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_STARTUP_H
#define APP_STARTUP_H
#include "control_services.h"

#include "FreeRTOS.h"

/* Application bring-up and runtime hand-off.  main.c only initializes the
 * board peripherals; this module owns motor discovery, configuration and the
 * transition to FreeRTOS. */
void AppStartup_DwtInit(void);
void AppStartup_ConfigEthernetMemory(void);
void AppStartup_Run(void);
BaseType_t AppStartup_CreateMotorTask(void);

#endif /* APP_STARTUP_H */
