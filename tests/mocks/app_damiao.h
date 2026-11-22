/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef MOCK_APP_DAMIAO_H
#define MOCK_APP_DAMIAO_H
#include <stdint.h>
#include "dm_motor.h"
#include "infrabot/protocol.h"
void AppDamiao_Process(void);
void AppDamiao_Cancel(void);
void AppDamiao_CancelMotors(uint32_t mask);
uint8_t AppDamiao_RegisterRx(const DM_CanRxFrame_t *frame);
uint16_t AppDamiao_ModeId(uint8_t mode);
uint8_t AppDamiao_RegisterIsUint(uint8_t rid);
void AppDamiao_ReportSelfCheck(InfrabotSafetyStatus *status);
uint8_t AppDamiao_StartLocalProfile(uint8_t mode);
uint8_t AppDamiao_LocalProfileStatus(void);
#endif
