/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "arm_admittance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static ArmAdmittanceParameters params(void){
  return (ArmAdmittanceParameters){8,1,100,4,3,1,1,.7f,1,3,1.5f,.15f};
}
int main(void){
  float q[7]={.2f,.9f,-.5f,1.3f,.3f,.6f,-.2f},g[7]={0},drive[7]={0};
  const float dq[7]={0};
  float fc[7]={.65f,.62f,.42f,.5f,.09f,.11f,.08f},fv[7]={0};
  ArmAdmittanceParameters p=params();ArmAdmittance s;ArmAdmittance_Init(&s,1,q);ArmIkPose actual=s.goal;
  drive[0]=-29;int frozen_at=-1,fault=0;float held[7],error[6];
  for(int n=0;n<1150;n++){
    fault=ArmAdmittance_Tick(&s,q,dq,g,drive,&actual,fc,fv,&p,.001f,1);
    if(s.frozen && frozen_at<0){frozen_at=n;memcpy(held,s.q,sizeof(held));memcpy(error,s.error,sizeof(error));}
    if(s.frozen && !fault){
      for(int i=0;i<7;i++)assert(s.velocity[i]==0 && s.q[i]==held[i]);
      for(int i=0;i<6;i++)assert(s.error[i]==error[i]);
    }
    if(fault){assert(n>=1000 && n<1100);break;}
  }
  assert(frozen_at>=29 && frozen_at<100 && fault==ARM_ADM_OVERLOAD && s.fault_joint==0);
  printf("ADM overload: frozen at %dms, then controlled-stop fault\n",frozen_at);
  ArmAdmittance_Init(&s,0,q);drive[0]=0;
  float measured[7];memcpy(measured,q,sizeof(q));measured[0]-=.1f;ArmIk_Forward(measured,&actual);
  for(int n=0;n<499;n++)assert(ArmAdmittance_Tick(&s,measured,dq,g,drive,&actual,fc,fv,&p,.001f,1)==0);
  assert(ArmAdmittance_Tick(&s,measured,dq,g,drive,&actual,fc,fv,&p,.001f,1)==ARM_ADM_TRACKING);
  ArmAdmittance_Init(&s,1,q);g[0]=NAN;
  assert(ArmAdmittance_Tick(&s,q,dq,g,drive,&actual,fc,fv,&p,.001f,1)==ARM_ADM_NONFINITE);g[0]=0;
  for(int i=0;i<7;i++)drive[i]=-5;
  for(int n=0;n<20;n++){
    assert(!ArmAdmittance_Tick(&s,q,dq,g,drive,&s.goal,fc,fv,&p,.001f,1));
    float f=0,m=0;for(int i=0;i<3;i++){f+=s.effective_force[i]*s.effective_force[i];m+=s.effective_moment[i]*s.effective_moment[i];}
    assert(sqrtf(f)<=40.0001f && sqrtf(m)<=4.0001f);
  }
  puts("ADM safety: overload freeze/timeout, tracking dwell, nonfinite rejection, wrench caps");
}
