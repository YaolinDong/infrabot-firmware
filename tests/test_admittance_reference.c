/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "arm_admittance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void){
  float q[7]={.2f,.9f,-.5f,1.3f,.3f,.6f,-.2f};
  ArmAdmittance s;ArmAdmittance_Init(&s,1,q);
  /* Two axes ask for acceleration in the same tick; one tick is counted,
   * peak and axis use velocity-scaled demand. A smaller next tick retains it. */
  float fast[7]={1,0,0,0,0,-3,0};
  assert(ArmAdmittance_LimitReference(&s,fast,.001f));
  assert(s.limiter.velocity_ticks==1 && s.limiter.accel_ticks==1);
  assert(s.limiter.peak_joint==6 && fabsf(s.limiter.peak_accel-1500.f)<.001f);
  float still[7]={0};
  assert(ArmAdmittance_LimitReference(&s,still,.001f));
  assert(s.limiter.accel_ticks==1 && s.limiter.peak_joint==6);
  ArmAdmittance_Init(&s,1,q);
  assert(ArmAdmittance_LimitReference(&s,still,.001f));
  assert(!s.limiter.velocity_ticks && !s.limiter.accel_ticks && !s.limiter.peak_joint);
  ArmAdmittanceParameters p={.null_d=1.5f,.null_k=3};
  ArmAdmittanceFrame f;assert(ArmAdmittance_Factor(s.q,&f));
  float posture[7];s.ns[6]=.1f;assert(ArmAdmittance_NullVelocity(&s,&f,&p,posture));
  float leak=0;for(int a=0;a<6;a++){float r=0;for(int i=0;i<7;i++)r+=f.j[a][i]*posture[i];leak+=r*r;}
  assert(sqrtf(leak)<.005f);s.ns[6]=0;
  float maximum=0;
  for(int n=0;n<1200;n++){
    assert(ArmAdmittance_Factor(s.q,&f));
    s.reference=s.goal;s.reference.position[0]+=.0001f*n;s.rate[0]=n<200?.1f*n/200:.1f;
    /* Start with a ramped input; a velocity step cannot obey finite acceleration with zero lag. */
    static float target=0;target+=s.rate[0]*.001f;s.reference.position[0]=s.goal.position[0]+target;
    float previous[7];for(int i=0;i<7;i++)previous[i]=s.velocity[i];
    assert(ArmAdmittance_Reference(&s,&f,&p,.001f));
    for(int i=0;i<7;i++)assert(fabsf(s.velocity[i]-previous[i])<=ARM_ADM_REFERENCE_ACCEL_RAD_S2*.001f+1e-6f);
    if(n>400){
      ArmIkPose actual;ArmIk_Forward(s.q,&actual);
      float e=fabsf(actual.position[0]-(s.goal.position[0]+target+s.rate[0]*.001f));
      maximum=fmaxf(maximum,e);
    }
  }
  printf("100mm/s reference: steady FK error %.5fmm\n",maximum*1000);
  assert(maximum<.0001f);
  s.error[0]=.2f;
  for(int n=0;n<100;n++){
    assert(ArmAdmittance_Factor(s.q,&f));ArmAdmittance_DesiredPose(&s,&s.reference);
    if(!ArmAdmittance_Reference(&s,&f,&p,.001f))break;
  }
  assert(s.correction_peak_m>0 && s.correction_peak_m<=.00005001f);
  /* Folded, almost singular arm, and a reference next to every hard margin:
   * remain finite/bounded or refuse the conflicting reference atomically. */
  for(int scenario=0;scenario<3;scenario++){
    float edge[7]={0,.031f,0,.031f,0,0,0};
    if(scenario)for(int i=0;i<7;i++)edge[i]=scenario==1?ArmIk_JointLower()[i]+.03001f:ArmIk_JointUpper()[i]-.03001f;
    ArmAdmittance_Init(&s,1,edge);
    for(int n=0;n<1000;n++){
      assert(ArmAdmittance_Factor(s.q,&f));
      s.reference=s.goal;s.reference.position[1]+=.2f;s.rate[1]=.3f;
      float before_q[7],before_v[7];
      for(int i=0;i<7;i++){before_q[i]=s.q[i];before_v[i]=s.velocity[i];}
      int ok=ArmAdmittance_Reference(&s,&f,&p,.001f);
      for(int i=0;i<7;i++){
        assert(isfinite(s.q[i]) && isfinite(s.velocity[i]));
        assert(s.q[i]>=ArmIk_JointLower()[i]+.03f-1e-7f);
        assert(s.q[i]<=ArmIk_JointUpper()[i]-.03f+1e-7f);
        assert(fabsf(s.velocity[i])<=(i<4?1.f:1.5f)+1e-6f);
        assert(fabsf(s.velocity[i]-before_v[i])<=ARM_ADM_REFERENCE_ACCEL_RAD_S2*.001f+1e-6f);
        if(!ok)assert(s.q[i]==before_q[i] && s.velocity[i]==before_v[i]);
      }
      if(!ok)break;
    }
  }
  puts("ADM reference: shared factorization, nullspace, finite accel, singular/limit bounds and reprojection");
}
