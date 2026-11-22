/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "arm_admittance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static float length(const float *v){return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);}
static void release_case(const float dp[3]){
  float eq[7]={.2f,.9f,-.5f,1.3f,.3f,.6f,-.2f},rv[3]={.08f,-.04f,.06f};
  ArmAdmittance s;ArmAdmittance_Init(&s,1,eq);
  memcpy(s.error,dp,3*sizeof(float));memcpy(s.error+3,rv,sizeof(rv));ArmAdmittance_DesiredPose(&s,&s.reference);
  ArmIkOptions o;ArmIk_DefaultOptions(&o);o.time_budget_us=0;o.max_iterations=1000;o.tolerance=2e-6f;o.posture_weight=0;
  ArmIkResult start=ArmIk_Solve(&s.reference,eq,&o,NULL);assert(start.position_error<.0001f);
  memcpy(s.q,start.q,sizeof(s.q));
  float fc[7]={.65f,.62f,.42f,.5f,.09f,.11f,.08f},fv[7]={0},gravity[7]={0};
  ArmAdmittanceParameters p={8,1,100,4,3,1,1,.7f,1,3,1.5f,.15f};
  float maxdev=0,maxsync=0,previous=1;
  for(int n=0;n<2500;n++){
    ArmIkPose actual;ArmIk_Forward(s.q,&actual);
    float drive[7];ArmAdmittance_Friction(s.velocity,fc,fv,p.friction,drive); /* plant friction, not the drive feedforward */
    float measured[7];memcpy(measured,s.q,sizeof(measured));
    assert(ArmAdmittance_Tick(&s,measured,s.velocity,gravity,drive,&actual,fc,fv,&p,.001f,1)==0);
    float path[3],off[3],dot=0,d2=0;
    for(int i=0;i<3;i++){path[i]=actual.position[i]-s.goal.position[i];dot+=path[i]*dp[i];d2+=dp[i]*dp[i];}
    for(int i=0;i<3;i++)off[i]=path[i]-dot/d2*dp[i];
    maxdev=fmaxf(maxdev,length(off));
    float linear=s.error[0]/dp[0],angular=s.error[3]/rv[0];
    maxsync=fmaxf(maxsync,fabsf(linear-angular));
    assert(linear>=-1e-5f && linear<=previous+1e-5f);previous=linear;
  }
  printf("ADM real H7C FK: release dev %.5fmm, translation/rotation progress mismatch %.6f, peak correction=%.1fum\n",
    maxdev*1000,maxsync,1e6f*s.correction_peak_m);
  assert(maxdev<.0001f && maxsync<.001f);
}
int main(void){
  const float small[3]={.03f,.01f,-.005f},hundred_mm[3]={.08f,.05f,-.03316625f};
  release_case(small);release_case(hundred_mm);
}
