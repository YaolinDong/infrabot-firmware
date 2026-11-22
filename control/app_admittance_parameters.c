/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "platform.h"
#include "app_admittance.h"
#include "app_state.h"
#include <math.h>
#define ADM_INITIAL {8,1,100,4,3,1,1.15f,.7f,1,3,1.5f,150}
static AppAdmittanceParameters parameters={ADM_INITIAL,ADM_INITIAL};
static volatile uint8_t bias_learning=1;
uint8_t AppAdmittance_BiasLearning(void){return bias_learning;}
void AppAdmittance_SetBiasLearning(uint8_t enabled){bias_learning=enabled!=0;}
static const float low[]={1,.5f,10,.5f,1,.6f,.5f,0,.3f,0,.5f,20};
static const float high[]={40,3,400,15,10,2,2,.9f,1,10,5,300};
static const float rate[]={20,1,100,5,2,1,.5f,.3f,.5f,5,2,100};
float AppAdmittance_ParameterMin(uint8_t i){return i<APP_ADM_PARAMETERS?low[i]:0;}
float AppAdmittance_ParameterMax(uint8_t i){return i<APP_ADM_PARAMETERS?high[i]:0;}
uint8_t AppAdmittance_SetParameter(uint8_t i,float v){
  if(i>=APP_ADM_PARAMETERS||!isfinite(v)||v<low[i]||v>high[i])return 0;
  platform_enter_critical();parameters.requested[i]=v;platform_exit_critical();return 1;
}
void AppAdmittance_GetParameters(AppAdmittanceParameters *out){
  if(!out)return;
  platform_enter_critical();*out=parameters;platform_exit_critical();
}
void AppAdmittance_ParametersTick(float dt){
  if(!isfinite(dt)||dt<=0||dt>.01f)return;
  for(int i=0;i<APP_ADM_PARAMETERS;i++){
    float requested;platform_enter_critical();requested=parameters.requested[i];platform_exit_critical();
    parameters.applied[i]+=fmaxf(-rate[i]*dt,fminf(rate[i]*dt,requested-parameters.applied[i]));
  }
}
ArmAdmittanceParameters AppAdmittance_Applied(void){
  const float *a=parameters.applied;
  return (ArmAdmittanceParameters){a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8],a[9],a[10],a[11]*.001f};
}
