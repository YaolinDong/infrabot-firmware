/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <stdint.h>

/* 达妙/达秒 DM 系列电机的模式 ID。 */
#define DM_MODE_MIT       0x000U
#define DM_MODE_POSITION  0x100U
#define DM_MODE_SPEED     0x200U
#define DM_MODE_POSITION_CURRENT 0x300U

typedef struct
{
    uint16_t id;          /* 电机 CAN ID；当前测试为 0x01..0x08。 */
    uint16_t master_id;   /* 电机反馈返回的 Master ID；当前测试为 0x11..0x18。 */

    /* 这些范围必须和电机型号/参数一致：
       DM4340P=12.5/10/28，DM8009P=12.5/45/54，DM4310=12.5/30/10。 */
    float position_max;
    float velocity_max;
    float torque_max;

    uint8_t feedback_id;
    uint8_t state;
    uint8_t temperature_mos;
    uint8_t temperature_coil;
    float position;
    float velocity;
    float torque;
    uint32_t feedback_count;
    uint32_t feedback_tick;
    uint8_t raw[8];       /* 最近一帧反馈的原始字节，便于直接比对协议。 */
} DM_Motor_t;
