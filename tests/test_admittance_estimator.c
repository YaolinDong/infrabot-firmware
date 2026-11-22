/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "arm_admittance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void){
  ArmAdmittanceFrame frame={0};
  for(int i=0;i<6;i++){frame.j[i][i]=1;frame.lower[i][i]=sqrtf(1.0004f);}
  float f[3]={5,-3,4},m[3]={.3f,-.2f,.4f},tau[7],force[3],moment[3],ns[7];
  for(int i=0;i<6;i++)tau[i]=i<3?.2f*f[i]:m[i-3];
  tau[6]=.7f;
  assert(ArmAdmittance_Map(&frame,tau,force,moment,ns));
  for(int i=0;i<3;i++){assert(fabsf(force[i]-f[i])<.003f);assert(fabsf(moment[i]-m[i])<.001f);}
  assert(ns[6]==.7f);
  float v[3]={3,4,0};ArmAdmittance_SoftDeadzone(v,3,2);assert(fabsf(v[0]-1.8f)<1e-6f&&fabsf(v[1]-2.4f)<1e-6f);
  ArmAdmittance s={0};float fc[7]={.65f,.62f,.42f,.5f,.09f,.11f,.08f},fv[7]={0},g[7]={0},drive[7];
  ArmAdmittanceParameters p={.static_ratio=1,.friction=.7f};
  float dq[7]={0};
  for(int i=0;i<7;i++)drive[i]=.9f*fc[i];
  for(int n=0;n<100;n++)assert(ArmAdmittance_Estimate(&s,dq,g,drive,fc,fv,&p,.001f));
  for(int i=0;i<7;i++)assert(s.external[i]==0);
  drive[0]=-1.f;
  for(int n=0;n<2000;n++){
    s.velocity[0]=.01f*sinf(31.415927f*n*.001f);
    assert(ArmAdmittance_Estimate(&s,dq,g,drive,fc,fv,&p,.001f));
    if(n>200)assert(fabsf(s.external[0]-.35f)<1e-5f);
  }
  dq[0]=.2f;s.velocity[0]=.15f;drive[0]=p.friction*fc[0]*tanhf(10)-1.f;
  for(int n=0;n<200;n++)assert(ArmAdmittance_Estimate(&s,dq,g,drive,fc,fv,&p,.001f));
  assert(fabsf(s.raw[0]-1.f)<1e-6f && fabsf(s.external[0]-.89f)<.001f);
  /* Sensor torque includes the measured-motion friction. Use a strong synthetic
   * wrench to bound the known soft-deadzone attenuation relative to signal. */
  for(int i=0;i<7;i++){
    dq[i]=.18f;s.velocity[i]=.13f;fv[i]=.17f;
    drive[i]=p.friction*(fc[i]*tanhf(dq[i]/.02f)+fv[i]*dq[i])-25.f*tau[i];
  }
  for(int n=0;n<500;n++)assert(ArmAdmittance_Estimate(&s,dq,g,drive,fc,fv,&p,.001f));
  assert(ArmAdmittance_Map(&frame,s.external,force,moment,ns));
  for(int i=0;i<3;i++){
    assert(fabsf(force[i]-25*f[i])<.1f*fabsf(25*f[i]));
    assert(fabsf(moment[i]-25*m[i])<.1f*fabsf(25*m[i]));
  }
  /* With Fc below the model-error floor, even a small static_ratio cannot
   * open the observer. Exercise all seven floors and deterministic white noise. */
  const float floors[7]={.11f,.32f,.19f,.23f,.06f,.07f,.06f};
  unsigned rng=19;
  for(unsigned step=0;step<=6;step++){
    s=(ArmAdmittance){0};p.static_ratio=.5f+.25f*step;
    for(int n=0;n<2000;n++){
      for(int i=0;i<7;i++){
        dq[i]=0;fc[i]=.2f*floors[i];rng=1664525U*rng+1013904223U;
        drive[i]=floors[i]*(.90f+.1f*(2.f*(rng>>8)/16777216.f-1.f));
      }
      assert(ArmAdmittance_Estimate(&s,dq,g,drive,fc,fv,&p,.001f));
      for(int i=0;i<7;i++)assert(s.external[i]==0);
    }
    /* Alternating samples cross the deadzone; the filtered raw stays inside. */
    for(int n=0;n<2000;n++){
      for(int i=0;i<7;i++)drive[i]=floors[i]*(n%2?1.09f:.89f);
      assert(ArmAdmittance_Estimate(&s,dq,g,drive,fc,fv,&p,.001f));
      for(int i=0;i<7;i++)assert(s.external[i]==0);
    }
    for(int i=0;i<7;i++)drive[i]=2.f*floors[i];
    for(int n=0;n<250;n++)assert(ArmAdmittance_Estimate(&s,dq,g,drive,fc,fv,&p,.001f));
    for(int i=0;i<7;i++)assert(fabsf(s.external[i]+floors[i])<1e-5f);
  }
  float q[7]={.2f,.9f,-.5f,1.3f,.3f,.6f,-.2f};
  assert(ArmAdmittance_Factor(q,&frame));
  float rhs[6]={1,2,3,4,5,6},out[6];assert(ArmAdmittance_Solve(&frame,rhs,out));
  for(int a=0;a<6;a++){
    double restored=0;
    for(int b=0;b<6;b++){
      double x=a==b?.0004:0;for(int k=0;k<7;k++)x+=(double)frame.j[a][k]*frame.j[b][k];
      restored+=x*out[b];
    }
    assert(fabs(restored-rhs[a])<.0005f);
  }
  /* A fresh session starts at zero. Hold learning until contact, motion,
   * tracking/overload and the full quiet dwell all permit an update. */
  ArmAdmittance_Init(&s,1,q);s.release=1.f;
  for(int i=0;i<7;i++)s.raw[i]=1;
  for(int n=0;n<1499;n++)ArmAdmittance_UpdateBias(&s,.001f);
  assert(s.bias[0]==0);
  for(int n=0;n<5000;n++)ArmAdmittance_UpdateBias(&s,.001f);
  assert(s.bias[0]==.30f && s.bias[6]==.15f);
  for(int i=0;i<7;i++)s.raw[i]=-1;
  for(int condition=0;condition<7;condition++){
    s.bias_learning=condition!=0;s.release=condition==1?0:1;
    s.loaded=condition==2;s.measured_motion[0]=condition==3?.005f:0;
    s.tracking_time[0]=condition==4?.1f:0;
    s.overload_time[6]=condition==5?.01f:0;s.drift_frozen=condition==6;
    for(int n=0;n<2000;n++)ArmAdmittance_UpdateBias(&s,.001f);
    assert(s.bias[0]==.30f && s.bias[6]==.15f && s.still_time==0);
  }
  ArmAdmittance_Init(&s,1,q);assert(s.bias[0]==0 && s.bias[6]==0);
  puts("ADM estimator: sign, shrinking torque deadzone, 20Hz LP, wrench/nullspace split, regularized solve");
}
