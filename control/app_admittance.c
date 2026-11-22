/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "platform.h"
#include "app_admittance.h"
#include "app_state.h"
#include "arm_gravity.h"
#include "arm_admittance_jitter.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
static ArmAdmittance kernel;
static ArmAdmittanceJitter jitter;
static AppAdmittanceStatus status;
static uint64_t return_start;
static const float torque_limit[7]=DM_ARM_TORQUE_LIMIT_NM;
uint8_t AppAdmittance_EntryAllowed(const float q[7]){
  for(int i=0;i<7;i++)if(!isfinite(q[i]) || q[i]<ArmIk_JointLower()[i]+.03f || q[i]>ArmIk_JointUpper()[i]-.03f)return 0;
  return 1;
}
void AppAdmittance_Start(uint8_t mode,const float q[7]){
  ArmAdmittanceReturn old=kernel.ret;ArmAdmittance_Init(&kernel,mode==DM_ETH_CONTROL_CARTESIAN_ADMITTANCE,q);kernel.ret=old;
  memcpy(kernel.torque_limit,torque_limit,sizeof(torque_limit));
  memset(&jitter,0,sizeof(jitter));status.mode=mode;status.frozen=status.returning=status.fault=0;return_start=0;
}
void AppAdmittance_Stop(void){status.mode=status.returning=status.frozen=0;memset(kernel.velocity,0,sizeof(kernel.velocity));kernel.active_return=0;}
int AppAdmittance_Evaluate(const float q[7],const float dq[7],const float gravity[7],const float drive[7],
  const ArmIkPose *actual,float dt,int run,uint64_t now,float reference[7],float velocity[7],float kp[7],float kd[7],float ff[7]){
  ArmAdmittanceParameters p=AppAdmittance_Applied();
  uint32_t old_seq=kernel.ret.sequence;uint8_t returning=kernel.active_return;
  kernel.bias_learning=AppAdmittance_BiasLearning();
  int fault=ArmAdmittance_Tick(&kernel,q,dq,gravity,drive,actual,ArmGravity_Coulomb(),ArmGravity_Viscous(),&p,dt,run);
  if(!returning && (kernel.active_return || old_seq!=kernel.ret.sequence))return_start=now;
  if(old_seq!=kernel.ret.sequence){kernel.ret.start_us=return_start;kernel.ret.end_us=now;}
  if(fault){status.fault=(uint8_t)fault;status.joint=kernel.fault_joint;status.fault_sequence++;return fault;}
  static const float kp_base[7]={300,300,150,150,40,40,30},kd_base[7]={2.5f,2.5f,2.5f,2.5f,.8f,.8f,.8f};
  float friction[7];ArmAdmittance_FrictionFeedforward(kernel.velocity,ArmGravity_Coulomb(),ArmGravity_Viscous(),p.friction,friction);
  ArmAdmittanceJitter_RecordFriction(&jitter,friction);
  for(int i=0;i<7;i++){
    reference[i]=kernel.q[i];velocity[i]=kernel.velocity[i];
    kp[i]=p.drive_gain*kp_base[i];kd[i]=sqrtf(p.drive_gain)*kd_base[i];ff[i]=gravity[i]+friction[i];
    status.tracking[i]=kernel.q[i]-q[i];
  }
  status.release=kernel.release;status.loaded=kernel.loaded;status.bias_learning=kernel.bias_learning;
  memcpy(status.bias,kernel.bias,sizeof(status.bias));
  memcpy(status.force,kernel.force,sizeof(status.force));memcpy(status.moment,kernel.moment,sizeof(status.moment));
  memcpy(status.effective_force,kernel.effective_force,sizeof(status.effective_force));memcpy(status.error,kernel.error,sizeof(status.error));
  memcpy(status.rate,kernel.rate,sizeof(status.rate));memcpy(status.external,kernel.external,sizeof(status.external));
  memcpy(status.q,kernel.q,sizeof(status.q));memcpy(status.velocity,kernel.velocity,sizeof(status.velocity));memcpy(status.equilibrium,kernel.equilibrium,sizeof(status.equilibrium));
  status.frozen=kernel.frozen;status.returning=kernel.active_return;status.drift_frozen=kernel.drift_frozen;status.lag_time=kernel.projection_time;status.ret=kernel.ret;
  return ARM_ADM_OK;
}
void AppAdmittance_RecordOutput(const float ff[7],const float drive[7],const float velocity[7]){
  float vmax[7];for(int i=0;i<7;i++){status.pd[i]=drive[i]-ff[i];vmax[i]=dm_motors[i].velocity_max;}
  ArmAdmittanceJitter_Tick(&jitter,ff,drive,velocity,vmax);
}
uint8_t AppAdmittance_FaultJoint(void){return status.joint;}
void AppAdmittance_Status(AppAdmittanceStatus *out){if(!out)return;
  platform_enter_critical();*out=status;platform_exit_critical();}
static long milli(float x){return isfinite(x)&&fabsf(x)<2147483.f?lroundf(x*1000):0;}
static void vector(const char *name,const float *v,unsigned n,float scale){
  printf(" %s=",name);for(unsigned i=0;i<n;i++)printf("%s%ld",i?",":"",milli(v[i]*scale));
}
static void vector_si(const char *name,const float *v,unsigned n){
  printf(" %s=",name);
  for(unsigned i=0;i<n;i++){
    long q=milli(v[i]),magnitude=q<0?-q:q;
    printf("%s%s%ld.%03ld",i?",":"",q<0?"-":"",magnitude/1000,magnitude%1000);
  }
}
void AppAdmittance_Print(void){
  static uint32_t printed,printed_fault;static AppAdmittanceStatus s;
  float rest[7];uint32_t count;ArmAdmittanceJitterReport jr;
  platform_enter_critical();ArmAdmittanceJitter_Take(&jitter,&jr);count=kernel.rest_count;
  for(int i=0;i<7;i++)rest[i]=count?kernel.rest_sum[i]/count:0;
  const ArmAdmittanceLimiterStats limiter=kernel.limiter;
  kernel.limiter=(ArmAdmittanceLimiterStats){0};
  const long correction_um=lroundf(kernel.correction_peak_m*1e6f);
  const long correction_urad=lroundf(kernel.correction_peak_rad*1e6f);
  kernel.correction_peak_m=kernel.correction_peak_rad=0.f;
  memset(kernel.rest_sum,0,sizeof(kernel.rest_sum));kernel.rest_count=0;
  memcpy(status.rest_raw,rest,sizeof(rest));s=status;platform_exit_critical();
  if(s.fault_sequence!=printed_fault){
    printed_fault=s.fault_sequence;
    printf("ADM_FAULT seq=%lu reason=%s joint=%u\r\n",(unsigned long)s.fault_sequence,
      s.fault==ARM_ADM_TRACKING?"ADM_TRACKING":s.fault==ARM_ADM_OVERLOAD?"ADM_OVERLOAD":"ADM_INVALID",s.joint<7?(unsigned)s.joint+1:0U);
  }
  if(s.mode){
    printf("ADM mode=%u frozen=%u drift_frozen=%u lag_ms=%ld corr_um=%ld corr_urad=%ld loaded=%u release_permille=%ld bias_learn=%u rest_n=%lu",
      s.mode,s.frozen,s.drift_frozen,milli(s.lag_time),correction_um,correction_urad,s.loaded,milli(s.release),s.bias_learning,(unsigned long)count);
    const long tenths=lroundf(limiter.peak_accel*10.f);
    printf(" accel_bind_ms=%lu accel_need=%ld.%ld accel_joint=%u vel_bind_ms=%lu",
      (unsigned long)limiter.accel_ticks,tenths/10,tenths%10,
      (unsigned)limiter.peak_joint,(unsigned long)limiter.velocity_ticks);
    vector_si("F_N",s.force,3);vector_si("M_Nm",s.moment,3);vector_si("Feff_N",s.effective_force,3);
    vector("e_mm",s.error,3,1);vector("e_mrad",s.error+3,3,1);vector("edot_mm_s",s.rate,3,1);
    vector("text_mNm",s.external,7,1);vector("raw_rest_mNm",rest,7,1);vector("bias_mNm",s.bias,7,1);vector("trk_mrad",s.tracking,7,1);vector("pd_mNm",s.pd,7,1);printf("\r\n");
  }
  if(jr.count){
    printf("ADM JITTER n=%lu",(unsigned long)jr.count);
    vector("dtau_max_mNm",jr.max_step,7,1);
    vector("ff_step_max_mNm",jr.friction_step_max,7,1);
    float rms[7];for(int i=0;i<7;i++)rms[i]=sqrtf(jr.error_sum[i]/jr.count);
    vector("ffdrv_rms_mNm",rms,7,1);printf(" rev=");
    for(int i=0;i<7;i++)printf("%s%lu",i?",":"",(unsigned long)jr.reversals[i]);
    printf("\r\n");
  }
  if(s.ret.sequence!=printed){
    printed=s.ret.sequence;
    printf("ADM RET seq=%lu t_ms=%ld dev_mm=%ld peak_mm_s=%ld final_mm=%ld final_mrad=%ld ns_mrad=%ld result=%s start_us=%llu end_us=%llu qeq_urad=",
      (unsigned long)s.ret.sequence,milli(s.ret.elapsed),milli(sqrtf(s.ret.deviation2)),milli(s.ret.peak_speed),milli(s.ret.final_position),
      milli(s.ret.final_rotation),milli(s.ret.ns_error),s.ret.result?"timeout":"ok",(unsigned long long)s.ret.start_us,(unsigned long long)s.ret.end_us);
    for(int i=0;i<7;i++)printf("%s%ld",i?",":"",milli(s.ret.equilibrium[i]*1000));
    printf("\r\n");
  }
}
