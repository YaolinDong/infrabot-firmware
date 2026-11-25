/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "platform.h"
#include "app_impedance.h"
#include "app_state.h"
#include <math.h>
#define INITIAL_GAINS {DM_IMPEDANCE_JOINT_K, DM_IMPEDANCE_JOINT_D, \
  DM_IMPEDANCE_TRANSLATION_K, DM_IMPEDANCE_TRANSLATION_D, \
  DM_IMPEDANCE_ROTATION_K, DM_IMPEDANCE_ROTATION_D, DM_IMPEDANCE_JOINT_DAMPING}
static AppImpedanceParameters parameters={INITIAL_GAINS,INITIAL_GAINS};
static const float low[APP_IMP_PARAMETERS]={.5f,.1f,5.f,1.f,.1f,.05f,.05f};
static const float high[APP_IMP_PARAMETERS]={30.f,3.f,300.f,50.f,15.f,5.f,3.f};
/* Units per second, not per UI event. */
static const float rate[APP_IMP_PARAMETERS]={20.f,2.f,100.f,20.f,5.f,2.f,2.f};
float AppImpedance_ParameterMin(uint8_t i){return i<APP_IMP_PARAMETERS?low[i]:0.f;}
float AppImpedance_ParameterMax(uint8_t i){return i<APP_IMP_PARAMETERS?high[i]:0.f;}
static const float initial_value[APP_IMP_PARAMETERS]=INITIAL_GAINS;
float AppImpedance_ParameterRate(uint8_t i){return i<APP_IMP_PARAMETERS?rate[i]:0.f;}
float AppImpedance_ParameterDefault(uint8_t i){return i<APP_IMP_PARAMETERS?initial_value[i]:0.f;}
uint8_t AppImpedance_SetParameter(uint8_t i,float value)
{
  if(i>=APP_IMP_PARAMETERS || !isfinite(value) || value<low[i] || value>high[i]) return 0;
  platform_enter_critical();parameters.requested[i]=value;platform_exit_critical();
  return 1;
}
void AppImpedance_GetParameters(AppImpedanceParameters *out)
{
  if(!out)return;
  platform_enter_critical();*out=parameters;platform_exit_critical();
}
void AppImpedance_Tick(float dt,ArmImpedanceGains *out)
{
  if(!out)return;
  /* Never catch up a stalled loop with an unbounded gain jump. */
  if(isfinite(dt) && dt>0.f && dt<=.01f) {
    for(unsigned i=0;i<APP_IMP_PARAMETERS;i++) {
      float desired;
      platform_enter_critical();desired=parameters.requested[i];platform_exit_critical();
      const float delta=desired-parameters.applied[i],step=rate[i]*dt;
      parameters.applied[i]+=fmaxf(-step,fminf(step,delta));
    }
  }
  *out=(ArmImpedanceGains){parameters.applied[APP_IMP_JOINT_K],parameters.applied[APP_IMP_JOINT_D],
    parameters.applied[APP_IMP_TRANSLATION_K],parameters.applied[APP_IMP_TRANSLATION_D],
    parameters.applied[APP_IMP_ROTATION_K],parameters.applied[APP_IMP_ROTATION_D],
    parameters.applied[APP_IMP_JOINT_DAMPING]};
}
