/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include "dm_types.h"

/* All payloads are eight bytes. The mode command uses the base motor ID;
 * register requests use CAN ID 0x7FF. A null register value emits four zeros. */
void DM_PackMode(uint8_t command, uint8_t data[8]);
void DM_PackRegister(uint16_t id, uint8_t command, uint8_t rid, const uint8_t value[4], uint8_t data[8]);
static inline uint32_t DM_RegisterValue(const uint8_t data[8])
{
    return (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
           ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
}

void DM_PackMIT(const DM_Motor_t *motor, float position, float velocity, float kp, float kd, float torque, uint8_t data[8]);
void DM_PackPosition(float position, float velocity, uint8_t data[8]);
void DM_PackSpeed(float velocity, uint8_t data[8]);
void DM_PackPositionCurrent(float position, float velocity, float current, uint8_t data[8]);

/* A successful decode updates feedback_count; the transport owns the timestamp. */
uint8_t DM_DecodeFeedback(DM_Motor_t *motor,
                                  uint16_t can_id,
                                  const uint8_t *data,
                                  uint8_t length);
