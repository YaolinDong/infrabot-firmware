/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "arm_admittance.h"
#include <math.h>
#include <string.h>

static float dot(const float *a,const float *b,unsigned n){float r=0;for(unsigned i=0;i<n;i++)r+=a[i]*b[i];return r;}
static float norm(const float *a,unsigned n){return sqrtf(dot(a,a,n));}
static int valid_vector(const float *a,unsigned n){for(unsigned i=0;i<n;i++)if(!isfinite(a[i]))return 0;return 1;}
void ArmAdmittance_Init(ArmAdmittance *s,int cartesian,const float q[7]){
  memset(s,0,sizeof(*s));s->cartesian=cartesian!=0;s->return_armed=1;s->bias_learning=1;s->fault_joint=0xff;
  static const float limits[7]={36,36,25,25,9,9,9};memcpy(s->torque_limit,limits,sizeof(limits));
  memcpy(s->q,q,sizeof(s->q));memcpy(s->equilibrium,q,sizeof(s->equilibrium));
  ArmIk_Forward(q,&s->goal);s->reference=s->goal;
  for(int i=0;i<5;i++)s->measured_history[i]=s->goal;
}
void ArmAdmittance_SoftDeadzone(float *v,unsigned n,float threshold){
  float length=norm(v,n),scale=length>threshold?1-threshold/length:0;
  for(unsigned i=0;i<n;i++)v[i]*=scale;
}
int ArmAdmittance_Factor(const float q[7],ArmAdmittanceFrame *f){
  if(!valid_vector(q,7))return 0;
  ArmIk_ForwardJacobian(q,&f->pose,f->linear,f->angular);
  for(int a=0;a<6;a++)for(int i=0;i<7;i++)f->j[a][i]=a<3?f->linear[a][i]/ARM_ADM_LEVER:f->angular[a-3][i];
  for(int a=0;a<6;a++)for(int b=0;b<=a;b++){
    float x=(a==b?.0004f:0)+dot(f->j[a],f->j[b],7);
    for(int k=0;k<b;k++)x-=f->lower[a][k]*f->lower[b][k];
    if(a==b){if(!(x>0) || !isfinite(x))return 0;f->lower[a][a]=sqrtf(x);}
    else f->lower[a][b]=x/f->lower[b][b];
  }
  return 1;
}
int ArmAdmittance_Solve(const ArmAdmittanceFrame *f,const float rhs[6],float out[6]){
  for(int a=0;a<6;a++){
    out[a]=rhs[a];for(int i=0;i<a;i++)out[a]-=f->lower[a][i]*out[i];
    out[a]/=f->lower[a][a];
  }
  for(int a=5;a>=0;a--){
    for(int i=a+1;i<6;i++)out[a]-=f->lower[i][a]*out[i];
    out[a]/=f->lower[a][a];
  }
  return valid_vector(out,6);
}
int ArmAdmittance_Map(const ArmAdmittanceFrame *f,const float tau[7],float force[3],float moment[3],float ns[7]){
  float rhs[6],y[6];for(int a=0;a<6;a++)rhs[a]=dot(f->j[a],tau,7);
  if(!ArmAdmittance_Solve(f,rhs,y))return 0;
  for(int a=0;a<3;a++){force[a]=y[a]/ARM_ADM_LEVER;moment[a]=y[a+3];}
  for(int i=0;i<7;i++){ns[i]=tau[i];for(int a=0;a<6;a++)ns[i]-=f->j[a][i]*y[a];}
  return valid_vector(ns,7);
}
int ArmAdmittance_Project(const ArmAdmittanceFrame *f,const float joint[7],float out[7]){
  float force[3],moment[3];return ArmAdmittance_Map(f,joint,force,moment,out);
}
static void friction_model(const float velocity[7],const float fc[7],const float fv[7],
                           float beta,float width,float out[7]){
  for(int i=0;i<7;i++){
    out[i]=beta*(fc[i]*tanhf(velocity[i]/width)+fv[i]*velocity[i]);
  }
}

void ArmAdmittance_Friction(const float velocity[7],const float fc[7],const float fv[7],float beta,float out[7]){
  friction_model(velocity,fc,fv,beta,.02f,out);
}

void ArmAdmittance_FrictionFeedforward(const float velocity[7],const float fc[7],const float fv[7],float beta,float out[7]){
  friction_model(velocity,fc,fv,beta,ARM_ADM_FEEDFORWARD_WIDTH_RAD_S,out);
}

int ArmAdmittance_Estimate(ArmAdmittance *s,const float measured_velocity[7],const float gravity[7],const float drive[7],
  const float fc[7],const float fv[7],const ArmAdmittanceParameters *p,float dt){
  /* 1.5 x held-out H7C v1 static RMS, 20 poses: J1..J7 (Nm). */
  static const float delta[7]={.11f,.32f,.19f,.23f,.06f,.07f,.06f};
  if(!valid_vector(measured_velocity,7))return 0;
  const float velocity_alpha=1.f-expf(-62.831853f*dt);
  for(int i=0;i<7;i++)
    s->estimated_velocity[i]+=velocity_alpha*(measured_velocity[i]-s->estimated_velocity[i]);
  float ff[7];ArmAdmittance_Friction(s->estimated_velocity,fc,fv,p->friction,ff);
  const float alpha=1-expf(-125.663706f*dt);
  for(int i=0;i<7;i++){
    const float unbiased=gravity[i]+ff[i]-drive[i];
    s->raw[i]=unbiased-s->bias[i];
    s->unbiased_filtered[i]+=alpha*(unbiased-s->unbiased_filtered[i]);
    float sigma=fabsf(tanhf(s->estimated_velocity[i]/.02f));
    float db=delta[i]+fmaxf(0.f,p->static_ratio*fc[i]-delta[i])*(1.f-sigma);
    s->filtered_raw[i]+=alpha*(s->raw[i]-s->filtered_raw[i]);
    float filtered=s->filtered_raw[i];
    s->external[i]=copysignf(fmaxf(0.f,fabsf(filtered)-db),filtered);
  }
  return valid_vector(s->raw,7)&&valid_vector(s->external,7);
}


#include "arm_admittance_dynamics.inc"

#include "arm_admittance_ik.inc"

#include "arm_admittance_safety.inc"
#include "arm_admittance_runtime.inc"
