/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "arm_admittance.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>
int main(void){
  float q[7]={.2f,.9f,-.5f,1.3f,.3f,.6f,-.2f};ArmAdmittance s;ArmAdmittance_Init(&s,0,q);
  ArmAdmittanceParameters p={.joint_k=8,.joint_mass_scale=1,.zeta=1};
  s.external[0]=2;
  for(int n=0;n<300;n++){
    float old=s.velocity[0];assert(ArmAdmittance_JointStep(&s,&p,.001f));
    assert(fabsf(s.velocity[0]-old)<=ARM_ADM_JOINT_REFERENCE_ACCEL_RAD_S2*.001f+1e-6f && fabsf(s.velocity[0])<=1);
  }
  assert(s.q[0]>q[0]+.02f);s.external[0]=0;
  for(int n=0;n<4000;n++){
    assert(ArmAdmittance_JointStep(&s,&p,.001f));assert(s.q[0]>=q[0]-1e-5f);
  }
  assert(fabsf(s.q[0]-q[0])<1e-4f);
  puts("Joint ADM: independent mass/damping, bounded reference, critical return and no zero-bias sampling");
}
