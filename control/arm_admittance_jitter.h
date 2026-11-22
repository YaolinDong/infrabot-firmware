/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef ARM_ADMITTANCE_JITTER_H
#define ARM_ADMITTANCE_JITTER_H
#include <stdint.h>
typedef struct {uint32_t count,reversals[7];float max_step[7],friction_step_max[7],error_sum[7];} ArmAdmittanceJitterReport;
typedef struct {ArmAdmittanceJitterReport report;float previous[7],previous_friction[7];int8_t sign[7];uint8_t initialized,has_friction;} ArmAdmittanceJitter;
void ArmAdmittanceJitter_RecordFriction(ArmAdmittanceJitter *s,const float friction[7]);
void ArmAdmittanceJitter_Tick(ArmAdmittanceJitter *s,const float ff[7],const float drive[7],const float velocity[7],const float vmax[7]);
void ArmAdmittanceJitter_Take(ArmAdmittanceJitter *s,ArmAdmittanceJitterReport *out);
#endif
