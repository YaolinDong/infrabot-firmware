/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "arm_admittance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

typedef void (*Friction)(const float *,const float *,const float *,float,float *);

/* One second of a 50 Hz square wave sampled by the 1 kHz MCU. */
static float peak_step(Friction model,float viscous){
  const float fc[7]={.64855034f},fv[7]={viscous};
  float v[7]={0},out[7],previous=0,peak=0;
  for(int tick=0;tick<1000;tick++){
    v[0]=(tick/10)%2?.005f:-.005f;
    model(v,fc,fv,.7f,out);
    if(tick)peak=fmaxf(peak,fabsf(out[0]-previous));
    previous=out[0];
  }
  return peak;
}

int main(void){
  const float scale=.7f*.64855034f, fv=.11700428f;
  const float narrow=peak_step(ArmAdmittance_Friction,0);
  const float wide=peak_step(ArmAdmittance_FrictionFeedforward,0);
  assert(narrow>.4f*scale && wide<.1f*scale);
  /* The linear viscous step adds 2*beta*Fv*0.005 even with a wider tanh. */
  const float total=peak_step(ArmAdmittance_FrictionFeedforward,fv);
  assert(fabsf(total-wide-.01f*.7f*fv)<1e-6f);
  printf("50 Hz, +/-5 mrad/s: Coulomb step %.5f -> %.5f beta*Fc; H7C J1 total %.2f -> %.2f mNm\n",
         narrow/scale,wide/scale,1000*peak_step(ArmAdmittance_Friction,fv),1000*total);
}
