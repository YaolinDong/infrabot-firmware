/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_ADMITTANCE_H
#define APP_ADMITTANCE_H
#include "arm_admittance.h"
enum { APP_ADM_JOINT_K,APP_ADM_JOINT_MASS,APP_ADM_LINEAR_K,APP_ADM_ROTATION_K,APP_ADM_MASS,APP_ADM_ZETA,
 APP_ADM_STATIC_RATIO,APP_ADM_FRICTION,APP_ADM_DRIVE_GAIN,APP_ADM_NULL_K,APP_ADM_NULL_D,APP_ADM_RETURN_SPEED,APP_ADM_PARAMETERS };
typedef struct {float requested[APP_ADM_PARAMETERS],applied[APP_ADM_PARAMETERS];} AppAdmittanceParameters;
typedef struct {
  uint8_t mode,frozen,returning,fault,joint;
  uint32_t fault_sequence;
  uint8_t drift_frozen,loaded,bias_learning;
  float release;
  float lag_time,bias[7];
  float force[3],moment[3],effective_force[3],error[6],rate[6],external[7],rest_raw[7],tracking[7],pd[7],q[7],velocity[7],equilibrium[7];
  ArmAdmittanceReturn ret;
} AppAdmittanceStatus;
uint8_t AppAdmittance_BiasLearning(void);
void AppAdmittance_SetBiasLearning(uint8_t enabled);
uint8_t AppAdmittance_SetParameter(uint8_t index,float value);
void AppAdmittance_GetParameters(AppAdmittanceParameters *out);
float AppAdmittance_ParameterMin(uint8_t index);
float AppAdmittance_ParameterMax(uint8_t index);
float AppAdmittance_ParameterRate(uint8_t index); /* largest change of the applied value, units per second */
float AppAdmittance_ParameterDefault(uint8_t index);
ArmAdmittanceParameters AppAdmittance_Applied(void); /* control-task owner */
void AppAdmittance_ParametersTick(float dt);
uint8_t AppAdmittance_EntryAllowed(const float q[7]);
void AppAdmittance_Start(uint8_t mode,const float q[7]);
void AppAdmittance_Stop(void);
int AppAdmittance_Evaluate(const float q[7],const float dq[7],const float gravity[7],const float drive[7],
  const ArmIkPose *actual,float dt,int run,uint64_t now,float reference[7],float velocity[7],float kp[7],float kd[7],float ff[7]);
void AppAdmittance_RecordOutput(const float ff[7],const float drive[7],const float velocity[7]);
uint8_t AppAdmittance_FaultJoint(void);
void AppAdmittance_Status(AppAdmittanceStatus *out);
void AppAdmittance_Print(void); /* low-priority status task, never the control task */
#endif
