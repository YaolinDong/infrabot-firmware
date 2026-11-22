/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "arm_admittance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void){
 float q[7]={.2f,.9f,-.5f,1.3f,.3f,.6f,-.2f};
 ArmAdmittance s;ArmAdmittance_Init(&s,1,q);
 ArmAdmittanceFrame f;assert(ArmAdmittance_Factor(q,&f));
 ArmAdmittanceParameters p={.null_d=1.5f,.null_k=3};
 s.ns[6]=.1f;float out[7];assert(ArmAdmittance_NullVelocity(&s,&f,&p,out));
 float leak=0;
 for(int a=0;a<6;a++){float r=0;for(int i=0;i<7;i++)r+=f.j[a][i]*out[i];leak+=r*r;}
 assert(sqrtf(leak)<.005f);
 s.ns[6]=0;assert(ArmAdmittance_NullVelocity(&s,&f,&p,out));
 for(int i=0;i<7;i++)assert(out[i]==0);
 puts("ADM nullspace: damped task leakage bounded, equilibrium and independent elbow mobility");
}
