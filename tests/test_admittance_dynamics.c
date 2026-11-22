/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "arm_admittance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static float length(const float *v){return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);}
int main(void){
  ArmAdmittanceParameters p={.linear_k=100,.rotation_k=4,.mass=3,.zeta=1,.return_speed=.15f};
  ArmAdmittance s={0};s.cartesian=1;s.error[0]=.1f;s.error[1]=.04f;s.error[3]=.3f;s.error[4]=.12f;
  float previous=s.error[0];
  for(int i=0;i<2500;i++){
    ArmAdmittance_Dynamics(&s,&p,.001f);
    assert(s.error[0]>=0 && s.error[0]<=previous);
    assert(fabsf(s.error[1]-.4f*s.error[0])<1e-6f);
    assert(fabsf(s.error[3]-3*s.error[0])<1e-5f);
    previous=s.error[0];
  }
  assert(length(s.error)<.001f);
  s=(ArmAdmittance){0};s.cartesian=1;s.error[0]=.1f;s.rate[1]=.05f;
  float from[3]={0},dev=0,pre=0;int captured=0;
  for(int i=0;i<2500;i++){
    ArmAdmittance_Dynamics(&s,&p,.001f);
    if(!captured && s.release>0){for(int j=0;j<3;j++)from[j]=s.error[j];captured=1;}
    if(!captured)pre=fmaxf(pre,fabsf(s.error[1]));
    else{
      float mag=length(from),dot=0;for(int j=0;j<3;j++)dot+=s.error[j]*from[j]/mag;
      float off[3];for(int j=0;j<3;j++)off[j]=s.error[j]-dot*from[j]/mag;
      dev=fmaxf(dev,length(off));
    }
  }
  printf("50mm/s lateral release: before detected release %.3fmm, after g_rel %.3fmm\n",pre*1000,dev*1000);
  assert(dev<.002f);
  s=(ArmAdmittance){0};s.cartesian=1;s.rate[0]=1;s.rate[1]=2;s.rate[3]=3;s.rate[4]=6;
  ArmAdmittance_Dynamics(&s,&p,.001f);
  assert(length(s.rate)<=.300001f && length(s.rate+3)<=1.20001f);
  assert(fabsf(s.rate[1]/s.rate[0]-2)<1e-5f && fabsf(s.rate[4]/s.rate[3]-2)<1e-5f);
  s=(ArmAdmittance){0};s.error[0]=.26f;s.rate[0]=.2f;s.effective_force[0]=40;ArmAdmittance_Dynamics(&s,&p,.001f);
  assert(length(s.error)<=.250001f && s.effective_force[0]==0);
  for(int cart=0;cart<2;cart++){
    const float q[7]={0,.8f,0,.8f,0,0,0};
    ArmAdmittance_Init(&s,cart,q);p.joint_k=8;p.joint_mass_scale=1;
    for(int n=0;n<900;n++){
      s.effective_force[0]=n<400?.5f:n<800?(n%2?.5f:.7f):.39f;
      if(cart)ArmAdmittance_Dynamics(&s,&p,.001f);
      else assert(ArmAdmittance_JointStep(&s,&p,.001f));
      if(n==399)assert(!s.loaded && s.release==1);
      if(n>=400 && n<800)assert(s.loaded && s.release==0);
      if(n>=800)assert(!s.loaded);
    }
    s.effective_force[0]=0;s.ns[6]=.26f;
    ArmAdmittance_Dynamics(&s,&p,.001f);assert(s.loaded);
    s.ns[6]=.17f;s.effective_moment[0]=.06f;
    ArmAdmittance_Dynamics(&s,&p,.001f);assert(s.loaded);
    s.effective_moment[0]=.04f;
    ArmAdmittance_Dynamics(&s,&p,.001f);assert(!s.loaded);
  }
  puts("ADM dynamics: synchronized critical decay, detected-release line, vector limits/workspace passed");
}
