/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_impedance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void)
{
  AppImpedanceParameters initial,p;AppImpedance_GetParameters(&initial);
  ArmImpedanceGains gains;
  for(unsigned i=0;i<APP_IMP_PARAMETERS;i++) {
    assert(initial.requested[i]==initial.applied[i]);
    assert(!AppImpedance_SetParameter(i,NAN));assert(!AppImpedance_SetParameter(i,INFINITY));
    assert(!AppImpedance_SetParameter(i,AppImpedance_ParameterMin(i)-.01f));
    assert(!AppImpedance_SetParameter(i,AppImpedance_ParameterMax(i)+.01f));
    assert(AppImpedance_SetParameter(i,AppImpedance_ParameterMax(i)));
  }
  assert(!AppImpedance_SetParameter(APP_IMP_PARAMETERS,1.f));
  AppImpedance_Tick(NAN,&gains);AppImpedance_Tick(.1f,&gains);
  AppImpedance_GetParameters(&p);
  for(unsigned i=0;i<APP_IMP_PARAMETERS;i++)assert(p.applied[i]==initial.applied[i]);
  AppImpedance_Tick(.001f,&gains);AppImpedance_GetParameters(&p);
  for(unsigned i=0;i<APP_IMP_PARAMETERS;i++)assert(p.applied[i]>initial.applied[i] && p.applied[i]<p.requested[i]);
  for(unsigned n=0;n<10000;n++)AppImpedance_Tick(.001f,&gains);
  AppImpedance_GetParameters(&p);
  for(unsigned i=0;i<APP_IMP_PARAMETERS;i++) {
    assert(p.applied[i]==p.requested[i]);
    assert(AppImpedance_SetParameter(i,AppImpedance_ParameterMin(i)));
  }
  for(unsigned n=0;n<10000;n++)AppImpedance_Tick(.001f,&gains);
  AppImpedance_GetParameters(&p);
  for(unsigned i=0;i<APP_IMP_PARAMETERS;i++)assert(p.applied[i]==p.requested[i]);
  assert(gains.joint_k==p.applied[APP_IMP_JOINT_K] && gains.joint_d==p.applied[APP_IMP_JOINT_D]);
  assert(gains.translation_k==p.applied[APP_IMP_TRANSLATION_K] && gains.translation_d==p.applied[APP_IMP_TRANSLATION_D]);
  assert(gains.rotation_k==p.applied[APP_IMP_ROTATION_K] && gains.rotation_d==p.applied[APP_IMP_ROTATION_D]);
  assert(gains.joint_damping==p.applied[APP_IMP_JOINT_DAMPING]);
  puts("Impedance parameters: all seven bounds, finite checks, slew/no jump, convergence, gain mapping passed");
}
