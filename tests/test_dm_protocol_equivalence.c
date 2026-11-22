/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Generate expected records with the original pre-refactor driver, or compare the current standalone
 * codec against them. Both paths use the same deterministic inputs. */
#ifdef GENERATE_VECTORS
#include "dm_motor.h"
#else
#include "dm_protocol.h"
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t seed = 12345;
static unsigned model, sample;
static const char *operation;

static uint32_t random_word(void)
{
    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;
    return seed;
}

static float input(float lo, float hi, unsigned levels)
{
    uint32_t bits = random_word();
    float value;
    switch (bits % 12) {
    case 0: return lo;
    case 1: return hi;
    case 2: return 0.0f;
    case 3: return -0.0f;
    case 4: return lo - (hi-lo);
    case 5: return hi + (hi-lo);
    case 6:
        do { bits = random_word(); } while ((bits & 0x7f800000U) == 0x7f800000U);
        memcpy(&value, &bits, sizeof(value));
        return value;
    case 7: case 8: case 9:
        value = lo + (hi-lo) * (((float)(random_word() % levels) + 0.5f) / levels);
        if (bits % 12 == 7) return nextafterf(value, lo);
        if (bits % 12 == 8) return nextafterf(value, hi);
        return value;
    default: return lo + (hi-lo) * (float)(random_word() & 0xffffffU) / 16777215.0f;
    }
}

static void record(const void *data, size_t size)
{
#ifdef GENERATE_VECTORS
    if (fwrite(data, 1, size, stdout) == size) return;
#else
    unsigned char expected[8];
    assert(size <= sizeof(expected));
    if (fread(expected, 1, size, stdin) == size && !memcmp(data, expected, size)) return;
#endif
    fprintf(stderr, "vector error at range %u, sample %u, %s\n", model, sample, operation);
    exit(1);
}

#ifdef GENERATE_VECTORS
uint32_t HAL_GetTick(void) { return 1234; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *bus,
    const FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    (void)bus;
    (void)header;
    record(data, 8);
    return HAL_OK;
}
#endif

int main(void)
{
    _Static_assert(sizeof(float) == 4, "requires binary32");
    const float limits[][3] = {{12.5f,45,54}, {12.5f,10,28}, {12.5f,30,10}, {3.1416f,280,1}};
    for (model=0; model<4; ++model) {
        DM_Motor_t motor = {.id=model+1, .master_id=model+17,
            .position_max=limits[model][0], .velocity_max=limits[model][1], .torque_max=limits[model][2]};
        for (sample=0; sample<25000; ++sample) {
            float p = input(-motor.position_max, motor.position_max, 65535);
            float v = input(-motor.velocity_max, motor.velocity_max, 4095);
            float kp = input(0,500,4095);
            float kd = input(0,5,4095);
            float t = input(-motor.torque_max,motor.torque_max,4095);
            float cv = input(0,100,10000);
            float ci = input(0,1,10000);
#ifdef GENERATE_VECTORS
            FDCAN_HandleTypeDef bus = {0};
            operation = "MIT"; DM_Motor_MIT(&bus,&motor,p,v,kp,kd,t);
            operation = "position"; DM_Motor_Position(&bus,&motor,p,v);
            operation = "speed"; DM_Motor_Speed(&bus,&motor,v);
            operation = "position/current"; DM_Motor_PositionCurrent(&bus,&motor,p,cv,ci);
#else
            uint8_t data[8];
            operation = "MIT"; DM_PackMIT(&motor,p,v,kp,kd,t,data); record(data,8);
            operation = "position"; DM_PackPosition(p,v,data); record(data,8);
            operation = "speed"; DM_PackSpeed(v,data); record(data,8);
            operation = "position/current"; DM_PackPositionCurrent(p,cv,ci,data); record(data,8);
#endif
            operation = "feedback";
            uint8_t fb[8];
            for (unsigned i=0; i<sizeof(fb); ++i) fb[i] = random_word() >> 24;
            fb[0] = (fb[0] & 0xf0) | (motor.id + (sample%5 == 1));
            uint16_t id = motor.master_id + (sample%5 == 2);
            uint8_t length = (sample%5 == 3) ? (sample % 8) : 8;
#ifdef GENERATE_VECTORS
            uint8_t ok = DM_Motor_ProcessFeedback(&motor,id,fb,length);
#else
            uint8_t ok = DM_DecodeFeedback(&motor,id,fb,length);
#endif
            record(&ok,1);
            /* Rejected frames must preserve the preceding sample's feedback. */
            record(&motor.position,4); record(&motor.velocity,4); record(&motor.torque,4);
            record(&motor.state,1); record(&motor.feedback_id,1);
            record(&motor.temperature_mos,1); record(&motor.temperature_coil,1);
            record(motor.raw,8); record(&motor.feedback_count,4);
        }
    }
#ifndef GENERATE_VECTORS
    if (getchar() != EOF || ferror(stdin)) return 1;
    puts("100000 deterministic inputs, four motor ranges, 500000 frame comparisons passed");
#endif
    return 0;
}
