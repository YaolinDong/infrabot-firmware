/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_IMPEDANCE_H
#define APP_IMPEDANCE_H
#include <stdint.h>
#include "arm_impedance.h"
enum {
  APP_IMP_JOINT_K, APP_IMP_JOINT_D, APP_IMP_TRANSLATION_K, APP_IMP_TRANSLATION_D,
  APP_IMP_ROTATION_K, APP_IMP_ROTATION_D, APP_IMP_JOINT_DAMPING, APP_IMP_PARAMETERS
};
typedef struct {
  float requested[APP_IMP_PARAMETERS];
  float applied[APP_IMP_PARAMETERS];
} AppImpedanceParameters;
/* Independent of gravity FREE/LOCK settings; RAM only, reset at reboot. */
uint8_t AppImpedance_SetParameter(uint8_t index,float value);
void AppImpedance_GetParameters(AppImpedanceParameters *out);
float AppImpedance_ParameterMin(uint8_t index);
float AppImpedance_ParameterMax(uint8_t index);
/* Control-task only. Bound/slew changes, including while modes are inactive. */
void AppImpedance_Tick(float dt,ArmImpedanceGains *gains);
#endif
