/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_CONTROL_MODE_H
#define APP_CONTROL_MODE_H
#include <stdint.h>
#include "infrabot/protocol.h"
static inline uint8_t AppControlMode_IsLocalCompliant(uint8_t mode) {
  return mode>=INFRABOT_CONTROL_GRAVITY_FREE && mode<=INFRABOT_CONTROL_CARTESIAN_ADMITTANCE;
}

/* Called by the control task; network ingress only reads these gates. */
uint8_t AppControlMode_Profile(void); /* 0=mixed/invalid, 1=MIT, 2=POS_VEL */
uint8_t AppControlMode_ProfileReady(void);
uint8_t AppControlMode_Switch(uint8_t mode); /* DMAR status; never enables drives */
uint8_t AppControlMode_Telemetry(void);
void AppControlMode_Hold(void);
#endif
