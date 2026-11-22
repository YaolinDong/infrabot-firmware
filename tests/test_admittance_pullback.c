/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "arm_admittance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void){
  const float q[7]={.2f,.9f,-.5f,1.3f,.3f,.6f,-.2f},dt=.001f;
  ArmAdmittance s;ArmAdmittance_Init(&s,1,q);
  ArmAdmittanceParameters p={.null_d=1.5f,.null_k=3,.mass=3,.linear_k=100,
                              .rotation_k=4,.zeta=1,.return_speed=.15f};
  s.error[0]=.1f;s.error[3]=.4f;
  for(int tick=1;tick<=650;tick++){
    float before=s.error[0],angle=s.error[3];
    ArmAdmittanceFrame frame;assert(ArmAdmittance_Factor(s.q,&frame));
    ArmAdmittance_DesiredPose(&s,&s.reference);
    assert(ArmAdmittance_Reference(&s,&frame,&p,dt));
    for(int i=0;i<7;i++)assert(fabsf(s.velocity[i])<=.050001f);
    assert(fabsf(s.error[0]-before)<=.05001f*dt);
    assert(fabsf(s.error[3]-angle)<=.20002f*dt);
    assert(p.linear_k*fabsf(s.error[0]-before)<.1f);
    assert(s.drift_frozen==(tick>=500));
  }
  float zero[3]={0};s.effective_force[0]=5;
  ArmAdmittance_Dynamics(&s,&p,dt);assert(s.rate[0]==0);
  ArmAdmittance_Pullback(&s,zero,zero,dt);assert(s.drift_frozen);
  s.effective_force[0]=0;
  for(int i=0;i<160;i++)ArmAdmittance_Dynamics(&s,&p,dt);
  float before=s.error[0];ArmAdmittance_Pullback(&s,zero,zero,dt);
  assert(!s.drift_frozen && s.error[0]==before && s.rate[0]==0);
  puts("Slow IK (0.05 rad/s): bounded spring correction, 500 ms latch, unloaded recovery");
}
