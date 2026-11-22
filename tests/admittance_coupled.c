/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Folded single-joint/endpoint model, SI units.
 * Prismatic local Jacobian x=.2q; actual production observer, virtual
 * dynamics, safety and reference limiter. A scalar IK/reprojection is the
 * exact 1-D reduction, not a substitute for the separate H7C IK tests.
 * Drive PD 10kHz, MCU 1kHz; total torque feedback delayed 2ms, 12bit ±54Nm.
 * Human impedance: Fh=Kh(h-x)+Dh(hdot-v)-Mh*xddot.
 * The last term is reflected human mass. Hands detach after 3s.
 */
#include "arm_admittance.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
static double clip(double x,double cap){return fmax(-cap,fmin(cap,x));}
static void coupled_run(double mass,double zeta,double human_mass,double human_k,double human_d,
                     double physical_mass,double friction_estimate,int release_test,double result[12],
                     int scenario,int learn,FILE *curve){
  const int joint=scenario?3:0;
  const double lever=.2,fc=scenario?.497:.65,fs=1.7*fc,fv=scenario?.243:.117;
  const double kp=scenario?150:300,drive_range=scenario?28:54;
  const double rated=scenario?25:36,ff_limit=.85*rated;
  const double model_error=scenario==1?.5:0.;
  float fc_est[7]={0},fv_est[7]={0},g[7]={0},drive[7]={0};
  fc_est[joint]=(float)(fc*friction_estimate);fv_est[joint]=(float)(fv*friction_estimate);
  for(int i=0;i<7;i++)if(i!=joint){fc_est[i]=.1f;fv_est[i]=.02f;}
  float eq[7]={0,.9f,-.5f,1.3f,.3f,.6f,-.2f};
  ArmAdmittance s;ArmAdmittance_Init(&s,1,eq);
  memset(&s.goal,0,sizeof(s.goal));for(int a=0;a<3;a++)s.goal.rotation[a][a]=1;
  s.reference=s.goal;
  ArmAdmittanceParameters p={8,1,100,4,(float)mass,(float)zeta,1,.7f,1,3,1.5f,.15f};
  s.bias_learning=learn;p.static_ratio=1.15f;g[joint]=(float)model_error;
  double x=release_test?.1:0,v=0,tau_delay[2]={0},last_tau=0,ff=0,brake=0,brake_previous_v=0;
  if(release_test){s.q[joint]=eq[joint]+.5f;s.error[0]=.1f;}
  double maximum=0,overshoot=0,min_tail=1,max_tail=-1,max_velocity=0,settle=-1;
  int freezes=0,fault=0,drift_ticks=0,still_count=0,torque_high=0;
  const int ticks=scenario?30000:6000;
  double peak_ext=0,peak_virtual=0,previous_x=0,max_push_reversal=0;
  for(int n=0;n<ticks;n++){
    double t=n*.001;
    double h=0,hv=0;int touching=!scenario&&!release_test&&t<3;
    if(touching){
      double fade=1-exp(-8*t);
      h=fade*(.05*sin(1.884955592*t)+.002*sin(25.13274123*t)+.0005*sin(75.39822369*t));
      hv=fade*(.0942477796*cos(1.884955592*t)+.05026548246*cos(25.13274123*t)+.03769911184*cos(75.39822369*t))
        +8*exp(-8*t)*(.05*sin(1.884955592*t)+.002*sin(25.13274123*t)+.0005*sin(75.39822369*t));
    }
    drive[joint]=(float)tau_delay[n%2];
    float measured_velocity[7]={0};measured_velocity[joint]=(float)(v/lever);
    if(!ArmAdmittance_Estimate(&s,measured_velocity,g,drive,fc_est,fv_est,&p,.001f)){fault=1;break;}
    float measured[7];memcpy(measured,eq,sizeof(measured));measured[joint]=eq[joint]+(float)(x/lever);
    fault=ArmAdmittance_CheckSafety(&s,measured,.001f);if(fault)break;
    if(s.frozen){freezes++;memset(s.velocity,0,sizeof(s.velocity));memset(s.rate,0,sizeof(s.rate));}
    else{
      memset(s.effective_force,0,sizeof(s.effective_force));memset(s.effective_moment,0,sizeof(s.effective_moment));memset(s.ns,0,sizeof(s.ns));
      s.effective_force[0]=s.external[joint]/(float)lever;
      ArmAdmittance_SoftDeadzone(s.effective_force,3,1);
      s.effective_force[0]=(float)clip(s.effective_force[0],40);
      float old_target=s.error[0],qr_before=s.q[joint]-eq[joint];
      ArmAdmittance_Dynamics(&s,&p,.001f);
      float desired[7]={0};desired[joint]=(s.rate[0]+30*(old_target-qr_before*(float)lever))/(float)lever;
      if(!ArmAdmittance_LimitReference(&s,desired,.001f)){fault=2;break;}
      float position_lag[3]={(float)(old_target-qr_before*lever),0,0},rotation_lag[3]={0};
      ArmAdmittance_Pullback(&s,position_lag,rotation_lag,.001f);
      if(s.drift_frozen)drift_ticks++;
      s.measured_motion[0]=(float)v;
      ArmAdmittance_UpdateBias(&s,.001f);

    }
    float friction[7];ArmAdmittance_FrictionFeedforward(s.velocity,fc_est,fv_est,p.friction,friction);
    /* Inject model_error into the observer only: a known DC residual even
     * when the stationary plant is supported by stiction. */
    ff+=clip(friction[joint]-ff,ff_limit*.005);
    double qr=s.q[joint]-eq[joint],vr=s.velocity[joint];
    /* MIT reference position has no exact zero code either. */
    qr=floor((qr+12.5)*65535/25)*25/65535-12.5;
    double joint_speed=fabs(v/lever);
    if(joint_speed<=1.57 || brake_previous_v*v<0)brake=0;
    brake_previous_v=v;
    double u=fmax(0,fmin(1,(joint_speed-1.57)/1.57));
    brake+=clip(.25*rated*u*u*(3-2*u)-brake,.25*rated*.02);
    double final_ff=clip(ff-copysign(brake,v),ff_limit);
    for(int sub=0;sub<10;sub++){
      double motor=clip(kp*(qr-x/lever)+2.5*(vr-v/lever)+final_ff,drive_range);
      double human=touching?human_k*(h-x)+human_d*(hv-v):0;
      if(scenario==2 && t>=1 && t<3)human=fmin(10.,50.*(t-1.));
      double effective_mass=physical_mass+(touching?human_mass:0);
      double effort=motor/lever+human;
      double drag=copysign(fc/lever,v)+(fv/(lever*lever))*v;
      double next=v+.0001*(effort-drag)/effective_mass;
      if((v==0 || next*v<=0) && fabs(effort)<=fs/lever)next=0;
      else if(v==0)next=.0001*(effort-copysign(fc/lever,effort))/effective_mass;
      v=next;x+=.0001*v;last_tau=motor;
    }
    tau_delay[n%2]=floor((clip(last_tau,drive_range)+drive_range)*4095/(2*drive_range))*(2*drive_range)/4095-drive_range;
    torque_high=fabs(last_tau)>=rated?torque_high+1:0;
    if(torque_high>=100){fault=100;break;}
    if(fabs(v/lever)>3.14*1.25){fault=101;break;}
    maximum=fmax(maximum,fabs(x));max_velocity=fmax(max_velocity,fabs(v));
    if(release_test)overshoot=fmax(overshoot,-x);
    if(scenario){
      peak_virtual=fmax(peak_virtual,fabs(s.error[0]));
      peak_ext=fmax(peak_ext,fabs(s.external[joint]));
      if(t>=1 && t<3)max_push_reversal=fmax(max_push_reversal,previous_x-x);
      previous_x=x;
      if(curve && n%10==0)fprintf(curve,"%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u\n",
          t,x*1000,s.error[0]*1000,s.raw[joint],s.external[joint],s.bias[joint],s.release,v*1000,s.drift_frozen);
      if(scenario==2 && t>=3){
        overshoot=fmax(overshoot,-x);
        if(fabs(x)>=.002)settle=-1;
        else if(settle<0)settle=t-3;
      }
    }
    if(n>=ticks-1000){min_tail=fmin(min_tail,x);max_tail=fmax(max_tail,x);}
    if(release_test && fabs(x)<.001){
      still_count++;if(still_count>=200 && settle<0)settle=t-.199;
    }else still_count=0;
    if(!isfinite(x)||fabs(x)>.30 ||fabs(v)>2){fault=99;break;}
  }
  result[0]=fault;result[1]=freezes;result[2]=maximum;result[3]=max_velocity;
  result[4]=max_tail-min_tail;result[5]=x;result[6]=v;result[7]=settle;
  result[8]=overshoot;result[9]=drift_ticks;result[10]=last_tau;result[11]=s.error[0];
  if(scenario){result[12]=peak_virtual;result[13]=peak_ext;result[14]=s.bias[joint];result[15]=max_push_reversal;}
}


void adm_coupled_run(double mass,double zeta,double human_mass,double human_k,double human_d,
                     double physical_mass,double friction_estimate,int release_test,double result[12]){
  coupled_run(mass,zeta,human_mass,human_k,human_d,physical_mass,friction_estimate,
              release_test,result,0,1,NULL);
}

int adm_coupled_v9(int scenario,int learn,const char *path,double result[16]){
  FILE *curve=fopen(path,"w");
  if(!curve)return 0;
  fputs("t_s,actual_mm,virtual_mm,raw_Nm,external_Nm,bias_Nm,release,v_mm_s,drift_frozen\n",curve);
  coupled_run(3,1,0,0,0,2,1,0,result,scenario,learn,curve);
  return fclose(curve)==0;
}
