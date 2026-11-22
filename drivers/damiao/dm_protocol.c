/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_protocol.h"

#include <string.h>

#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f
#define DM_KD_MAX 5.0f

static float dm_clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static uint16_t dm_float_to_uint(float value, float min_value, float max_value, uint8_t bits)
{
    const float span = max_value - min_value;
    const uint32_t max_int = ((uint32_t)1U << bits) - 1U;

    value = dm_clamp(value, min_value, max_value);
    return (uint16_t)(((value - min_value) * (float)max_int / span) + 0.5f);
}

static float dm_uint_to_float(uint16_t value, float min_value, float max_value, uint8_t bits)
{
    const uint32_t max_int = ((uint32_t)1U << bits) - 1U;
    return ((float)value * (max_value - min_value) / (float)max_int) + min_value;
}

static void dm_put_float_le(uint8_t *dst, float value)
{
    memcpy(dst, &value, sizeof(value));
}

void DM_PackMode(uint8_t command, uint8_t data[8])
{
    memset(data, 0xFF, 7U);
    data[7] = command;
}

void DM_PackRegister(uint16_t id, uint8_t command, uint8_t rid,
                     const uint8_t value[4], uint8_t data[8])
{
    data[0] = (uint8_t)id;
    data[1] = (uint8_t)(id >> 8);
    data[2] = command;
    data[3] = rid;
    if (value != NULL) memcpy(data + 4, value, 4U);
    else memset(data + 4, 0, 4U);
}

void DM_PackMIT(const DM_Motor_t *motor, float position, float velocity, float kp, float kd, float torque, uint8_t data[8])
{
    uint16_t position_int;
    uint16_t velocity_int;
    uint16_t kp_int;
    uint16_t kd_int;
    uint16_t torque_int;


    position_int = dm_float_to_uint(position, -motor->position_max, motor->position_max, 16U);
    velocity_int = dm_float_to_uint(velocity, -motor->velocity_max, motor->velocity_max, 12U);
    torque_int = dm_float_to_uint(torque, -motor->torque_max, motor->torque_max, 12U);
    kp_int = dm_float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12U);
    kd_int = dm_float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12U);

    data[0] = (uint8_t)(position_int >> 8);
    data[1] = (uint8_t)position_int;
    data[2] = (uint8_t)(velocity_int >> 4);
    data[3] = (uint8_t)((velocity_int << 4) | (kp_int >> 8));
    data[4] = (uint8_t)kp_int;
    data[5] = (uint8_t)(kd_int >> 4);
    data[6] = (uint8_t)((kd_int << 4) | (torque_int >> 8));
    data[7] = (uint8_t)torque_int;
}

void DM_PackPosition(float position, float velocity, uint8_t data[8])
{
    dm_put_float_le(&data[0], position);
    dm_put_float_le(&data[4], velocity);
}

void DM_PackSpeed(float velocity, uint8_t data[8])
{
    memset(data, 0, 8U);
    dm_put_float_le(data, velocity);
}

void DM_PackPositionCurrent(float position, float velocity, float current, uint8_t data[8])
{
    /* Fourth mode encodes a 0..1 current/torque-limit ratio, NOT amperes/Nm. */
    uint16_t velocity_int = (uint16_t)(dm_clamp(velocity, 0.0f, 100.0f) * 100.0f);
    uint16_t current_int = (uint16_t)(dm_clamp(current, 0.0f, 1.0f) * 10000.0f);


    dm_put_float_le(&data[0], position);
    data[4] = (uint8_t)velocity_int;
    data[5] = (uint8_t)(velocity_int >> 8);
    data[6] = (uint8_t)current_int;
    data[7] = (uint8_t)(current_int >> 8);
}

uint8_t DM_DecodeFeedback(DM_Motor_t *motor,
                                  uint16_t can_id,
                                  const uint8_t *data,
                                  uint8_t length)
{
    uint16_t position_int;
    uint16_t velocity_int;
    uint16_t torque_int;
    uint8_t feedback_id;

    if ((motor == NULL) || (data == NULL) || (length < 8U))
    {
        return 0U;
    }

    /* 参考例程中电机反馈使用 Master ID，data[0] 低四位为电机 ID。 */
    feedback_id = (uint8_t)(data[0] & 0x0FU);
    if ((can_id != motor->master_id) || (feedback_id != (motor->id & 0x0FU)))
    {
        return 0U;
    }

    motor->feedback_id = feedback_id;
    motor->state = (uint8_t)(data[0] >> 4);
    position_int = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    velocity_int = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
    torque_int = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8) | data[5]);

    motor->position = dm_uint_to_float(position_int,
                                       -motor->position_max,
                                       motor->position_max,
                                       16U);
    motor->velocity = dm_uint_to_float(velocity_int,
                                       -motor->velocity_max,
                                       motor->velocity_max,
                                       12U);
    motor->torque = dm_uint_to_float(torque_int,
                                     -motor->torque_max,
                                     motor->torque_max,
                                     12U);
    motor->temperature_mos = data[6];
    motor->temperature_coil = data[7];
    memcpy(motor->raw, data, sizeof(motor->raw));
    motor->feedback_count++;
    return 1U;
}
