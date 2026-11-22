/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#define _POSIX_C_SOURCE 200809L
#include "arm_admittance.h"
#include "arm_impedance.h"
#include "arm_gravity.h"
#include <stdio.h>
#include <time.h>
static double seconds(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}
int main(void){
  const float eq[7]={.2f,.9f,-.5f,1.3f,.3f,.6f,-.2f};float q[7],dq[7]={0},g[7],twist[6],tau[7],drive[7];
  for(int i=0;i<7;i++)q[i]=eq[i];
  ArmIkPose goal;ArmIk_Forward(eq,&goal);ArmAdmittance s;
  ArmAdmittanceParameters p={8,1,100,4,3,1,1,.7f,1,3,1.5f,.15f};
  ArmImpedanceGains gains={8,1,80,12,3,.6f,.3f};ArmImpedanceDiagnostics diag;
  volatile float sink=0;const int N=200000;
  for(int kind=0;kind<2;kind++){
    double start=seconds();ArmAdmittance_Init(&s,1,eq);
    for(int n=0;n<N;n++){
      ArmGravity_Evaluate(q,dq,0,g,twist);
      if(!kind){ArmImpedance_Evaluate(1,q,dq,eq,&goal,&gains,tau,&diag);sink+=tau[0];}
      else{ArmIkPose actual;ArmIk_Forward(q,&actual);for(int i=0;i<7;i++)drive[i]=g[i];
        int result=ArmAdmittance_Tick(&s,q,dq,g,drive,&actual,ArmGravity_Coulomb(),ArmGravity_Viscous(),&p,.001f,1);
        if(result)return 1;
        ArmAdmittance_Friction(s.velocity,ArmGravity_Coulomb(),ArmGravity_Viscous(),p.friction,tau);sink+=tau[0];}
    }
    printf("%s native average %.3f us\n",kind?"CART ADM kernel+gravity+actual FK":"CART IMP kernel+gravity",(seconds()-start)*1e6/N);
  }
  return sink>1e20f;
}

