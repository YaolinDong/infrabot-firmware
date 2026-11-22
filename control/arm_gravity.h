/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef ARM_GRAVITY_H
#define ARM_GRAVITY_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define ARM_GRAVITY_AXES 7
/* Pure, allocation-free joint-frame model. torque includes gravity, the
 * identified H7C v1 offset/spring terms, optional velocity friction and the 20 Nm/rad wall.
 * twist = base-frame TCP linear/angular velocity (m/s, rad/s), may be NULL. */
int ArmGravity_Evaluate(const float q[7], const float dq[7], float friction_scale,
                        float torque[7], float twist[6]);
/* Same model with an adjustable velocity deadband (rad/s). */
int ArmGravity_EvaluateTuned(const float q[7], const float dq[7], float friction_scale,
                            float friction_deadband, float torque[7], float twist[6]);
/* Same result, optionally reporting just the velocity-dependent friction
 * feedforward. Gravity/offset/spring/wall are excluded from friction. */
int ArmGravity_EvaluateParts(const float q[7], const float dq[7], float friction_scale,
                            float friction_deadband, float torque[7], float twist[6],
                            float friction[7]);
const char *ArmGravity_ModelName(void);
const char *ArmGravity_ModelHash(void);
const float *ArmGravity_Coulomb(void);
const float *ArmGravity_Viscous(void);
const float *ArmGravity_Lower(void);
const float *ArmGravity_Upper(void);
#ifdef __cplusplus
}
#endif
#endif
