/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "arm_impedance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void) {
  float q[7]={.12f,.6f,-.25f,.8f,.3f,-.2f,.1f},dq[7]={0},eq[7],tau[7];
  for(unsigned i=0;i<7;i++)eq[i]=q[i];
  ArmIkPose target;ArmIk_Forward(eq,&target);
  ArmImpedanceGains g={8,.8f,80,12,3,.6f,.3f};
  for(int cart=0;cart<2;cart++) {
    assert(ArmImpedance_Compute(cart,q,dq,eq,&target,&g,tau));
    for(unsigned i=0;i<7;i++)assert(fabsf(tau[i])<1e-4f);
  }
  eq[1]+=.02f;
  assert(ArmImpedance_Compute(0,q,dq,eq,&target,&g,tau));assert(fabsf(tau[1]-.16f)<1e-5f);
  eq[1]=q[1];
  /* J^T maps an x-directed spring to the correct joint torque. */
  float jv[3][7],jw[3][7];ArmIk_Jacobian(q,jv,jw);
  target.position[0]+=.01f;
  assert(ArmImpedance_Compute(1,q,dq,eq,&target,&g,tau));
  for(unsigned i=0;i<7;i++)assert(fabsf(tau[i]-.8f*jv[0][i])<1e-4f);
  ArmImpedanceDiagnostics diag;
  float observed[7];
  assert(ArmImpedance_Evaluate(1,q,dq,eq,&target,&g,observed,&diag));
  assert(diag.valid && !diag.error_clamped);
  for(unsigned i=0;i<7;i++)assert(observed[i]==tau[i]);
  assert(fabsf(diag.position_error[0]-.01f)<1e-6f && fabsf(diag.force[0]-.8f)<1e-5f);
  target.position[0]+=.14f;
  assert(ArmImpedance_Evaluate(1,q,dq,eq,&target,&g,observed,&diag));
  assert(diag.error_clamped&1U);
  assert(fabsf(diag.position_error[0]-.15f)<1e-6f && fabsf(diag.force[0]-8.f)<1e-5f);
  assert(ArmImpedance_Evaluate(0,q,dq,eq,&target,&g,observed,&diag));assert(!diag.valid);
  q[0]=NAN;diag.valid=1;
  assert(!ArmImpedance_Evaluate(1,q,dq,eq,&target,&g,observed,&diag));assert(!diag.valid);q[0]=eq[0];
  ArmIk_Forward(eq,&target);
  /* Small base-frame rotation error must give positive restoring torque. */
  float r[3][3],rv[3]={0,0,.05f};ArmIk_RotationExp(rv,r);
  ArmIkPose rotated=target;
  for(unsigned a=0;a<3;a++)for(unsigned b=0;b<3;b++) {
    rotated.rotation[a][b]=0;
    for(unsigned k=0;k<3;k++)rotated.rotation[a][b]+=r[a][k]*target.rotation[k][b];
  }
  assert(ArmImpedance_Compute(1,q,dq,eq,&rotated,&g,tau));
  for(unsigned i=0;i<7;i++)assert(fabsf(tau[i]-.15f*jw[2][i])<1e-4f);
  for(unsigned i=0;i<7;i++)dq[i]=.1f*((int)i-3);
  assert(ArmImpedance_Compute(1,q,dq,eq,&target,&g,tau));
  float power=0;for(unsigned i=0;i<7;i++)power+=tau[i]*dq[i];assert(power<0);
  q[0]=NAN;assert(!ArmImpedance_Compute(1,q,dq,eq,&target,&g,tau));q[0]=eq[0];
  g.rotation_d=-1;assert(!ArmImpedance_Compute(1,q,dq,eq,&target,&g,tau));
  puts("impedance math: equilibrium, joint spring, J-transpose/rotation signs, dissipative damping, invalid inputs passed");
}
