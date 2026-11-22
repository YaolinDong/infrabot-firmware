/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_admittance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void){
  AppAdmittanceParameters first,now;AppAdmittance_GetParameters(&first);
  for(int i=0;i<APP_ADM_PARAMETERS;i++){
    assert(first.applied[i]==first.requested[i]);
    assert(!AppAdmittance_SetParameter(i,NAN));
    assert(!AppAdmittance_SetParameter(i,AppAdmittance_ParameterMin(i)-.001f));
    assert(!AppAdmittance_SetParameter(i,AppAdmittance_ParameterMax(i)+.001f));
    assert(AppAdmittance_SetParameter(i,AppAdmittance_ParameterMin(i)));
  }
  AppAdmittance_ParametersTick(.1f);AppAdmittance_GetParameters(&now);
  for(int i=0;i<APP_ADM_PARAMETERS;i++)assert(now.applied[i]==first.applied[i]);
  AppAdmittance_ParametersTick(.001f);AppAdmittance_GetParameters(&now);
  for(int i=0;i<APP_ADM_PARAMETERS;i++)assert(now.applied[i]<first.applied[i] && now.applied[i]>now.requested[i]);
  for(int n=0;n<10000;n++)AppAdmittance_ParametersTick(.001f);
  AppAdmittance_GetParameters(&now);
  for(int i=0;i<APP_ADM_PARAMETERS;i++)assert(now.applied[i]==now.requested[i]);
  puts("ADM parameters: 12 independent RAM settings, finite bounds, time-based slew, no stalled catch-up");
}
