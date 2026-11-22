/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "arm_impedance.h"
#include <math.h>
static float bounded(float x,float cap){return fmaxf(-cap,fminf(x,cap));}
int ArmImpedance_Evaluate(int cartesian,const float q[7],const float dq[7],
  const float equilibrium[7],const ArmIkPose *pose,const ArmImpedanceGains *g,float tau[7],ArmImpedanceDiagnostics *diag)
{
  if(diag) {diag->valid=0;diag->error_clamped=0;}
  if(!q||!dq||!equilibrium||!pose||!g||!tau) return 0;
  const float gains[]={g->joint_k,g->joint_d,g->translation_k,g->translation_d,
    g->rotation_k,g->rotation_d,g->joint_damping};
  for(unsigned i=0;i<7;i++)
    if(!isfinite(q[i])||!isfinite(dq[i])||!isfinite(equilibrium[i])||
       !isfinite(gains[i])||gains[i]<0.f) return 0;
  if(!cartesian) {
    for(unsigned i=0;i<7;i++) tau[i]=g->joint_k*bounded(equilibrium[i]-q[i],.35f)-g->joint_d*dq[i];
  } else {
    ArmIkPose current,error={0};float jv[3][7],jw[3][7],angle[3],force[3],moment[3];
    ArmIk_Forward(q,&current);ArmIk_Jacobian(q,jv,jw);
    for(unsigned a=0;a<3;a++) {
      if(!isfinite(pose->position[a])) return 0;
      for(unsigned b=0;b<3;b++) {
        if(!isfinite(pose->rotation[a][b])) return 0;
        for(unsigned k=0;k<3;k++) error.rotation[a][b]+=pose->rotation[a][k]*current.rotation[b][k];
      }
    }
    ArmIk_RotationLog(&error,angle);
    for(unsigned a=0;a<3;a++) {
      float v=0.f,w=0.f;
      for(unsigned i=0;i<7;i++){v+=jv[a][i]*dq[i];w+=jw[a][i]*dq[i];}
      force[a]=g->translation_k*bounded(pose->position[a]-current.position[a],.10f)-g->translation_d*v;
      moment[a]=g->rotation_k*bounded(angle[a],.5f)-g->rotation_d*w;
    }
    if(diag) for(unsigned a=0;a<3;a++) {
      diag->target[a]=pose->position[a];diag->measured[a]=current.position[a];
      diag->position_error[a]=pose->position[a]-current.position[a];diag->rotation_error[a]=angle[a];
      diag->force[a]=force[a];diag->moment[a]=moment[a];
      if(fabsf(diag->position_error[a])>.10f)diag->error_clamped|=(uint8_t)(1U<<a);
      if(fabsf(angle[a])>.5f)diag->error_clamped|=(uint8_t)(1U<<(3U+a));
    }
    for(unsigned i=0;i<7;i++) {
      tau[i]=-g->joint_damping*dq[i];
      for(unsigned a=0;a<3;a++) tau[i]+=jv[a][i]*force[a]+jw[a][i]*moment[a];
    }
  }
  for(unsigned i=0;i<7;i++) if(!isfinite(tau[i]))return 0;
  if(diag)diag->valid=cartesian!=0;
  return 1;
}

int ArmImpedance_Compute(int cartesian,const float q[7],const float dq[7],
  const float equilibrium[7],const ArmIkPose *pose,const ArmImpedanceGains *g,float tau[7])
{
  return ArmImpedance_Evaluate(cartesian,q,dq,equilibrium,pose,g,tau,0);
}
