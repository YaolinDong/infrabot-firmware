/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "arm_admittance_jitter.h"
#include <math.h>
#include <string.h>
/* Keep the previous sample across reporting windows. Start clears the whole
 * jitter state so re-entry cannot include a step from a previous controller. */
void ArmAdmittanceJitter_RecordFriction(ArmAdmittanceJitter *s,const float friction[7])
{
  for(int i=0;i<7;i++){
    if(s->has_friction){
      const float step=fabsf(friction[i]-s->previous_friction[i]);
      s->report.friction_step_max[i]=fmaxf(s->report.friction_step_max[i],step);
    }
    s->previous_friction[i]=friction[i];
  }
  s->has_friction=1;
}

void ArmAdmittanceJitter_Tick(ArmAdmittanceJitter *s,const float ff[7],const float drive[7],const float velocity[7],const float vmax[7])
{
  for(int i=0;i<7;i++){
    if(s->initialized)s->report.max_step[i]=fmaxf(s->report.max_step[i],fabsf(ff[i]-s->previous[i]));
    float e=ff[i]-drive[i];s->report.error_sum[i]+=e*e;s->previous[i]=ff[i];
    if(fabsf(velocity[i])>4.f*vmax[i]/4095.f){
      int8_t sign=velocity[i]>0?1:-1;
      if(s->sign[i] && sign!=s->sign[i])s->report.reversals[i]++;
      s->sign[i]=sign;
    }
  }
  s->initialized=1;s->report.count++;
}
void ArmAdmittanceJitter_Take(ArmAdmittanceJitter *s,ArmAdmittanceJitterReport *out)
{
  *out=s->report;memset(&s->report,0,sizeof(s->report));
}
