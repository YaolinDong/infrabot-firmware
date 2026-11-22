/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
int ArmAdmittance_CheckSafety(ArmAdmittance *s,const float measured[7],float dt){
  s->frozen=0;
  for(int i=0;i<7;i++){
    s->overload_time[i]=fmaxf(fabsf(s->unbiased_filtered[i]),fabsf(s->external[i]))>.6f*s->torque_limit[i]?s->overload_time[i]+dt:0;
    s->tracking_time[i]=fabsf(s->q[i]-measured[i])>(i<4?.08f:.15f)?s->tracking_time[i]+dt:0;
    if(s->overload_time[i]+1e-6f>=.030f)s->frozen=1;
    if(s->overload_time[i]+1e-5f>=1.f){s->fault_joint=(uint8_t)i;return ARM_ADM_OVERLOAD;}
    if(s->tracking_time[i]+1e-5f>=.5f){s->fault_joint=(uint8_t)i;return ARM_ADM_TRACKING;}
  }
  return ARM_ADM_OK;
}
