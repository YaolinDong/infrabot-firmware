/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_DAMIAO_H
#define APP_DAMIAO_H
#include "control_services.h"
#include "app_runtime.h"
#include "can_types.h"
#include "infrabot/protocol.h"
/* Called only from the 1 kHz control owner, never sleeps/waits for CAN. */
void AppDamiao_Process(void);
/* ISR hook: capture register responses BEFORE ordinary feedback decoding. */
uint8_t AppDamiao_RegisterRx(const DM_CanRxFrame_t *frame);
/* 1: the register holds a u32 (else f32), from the protocol's register table. */
uint8_t AppDamiao_RegisterIsUint(uint8_t rid);
/* The selfcheck* fields of the telemetry safety block: the parameter
 * self-check of the last SET_ARM_MODE. */
void AppDamiao_ReportSelfCheck(InfrabotSafetyStatus *status);
#endif
