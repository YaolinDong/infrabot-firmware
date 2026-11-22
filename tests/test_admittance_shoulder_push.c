/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* 7-DoF coupled host simulation around the PRODUCTION admittance kernel.
 * Plant: rigid-body inertia from infrabot-H7C_v1.urdf (+ reflected rotor inertia),
 * gravity assumed perfectly compensated, no Coriolis (|dq|<1.5 rad/s),
 * joint friction = viscous + Coulomb/static solved as bounded impulses (PGS).
 * Drive: MIT PD at 10 kHz. MCU: 1 kHz, quantised/delayed feedback like the firmware.
 * Human: force (or spring-damper hand) at a point of a chosen link.
 */
#include "arm_admittance.h"
#include "admittance_plant/h7c_links.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const double FC[7]={0.6485503358,0.6212831549,0.4227659217,0.4968016721,0.09435067175,0.1117289144,0.08116653803};
static const double FV[7]={0.1170042814,0.05456692991,0.3060837998,0.2433215176,0.01604760697,0.01774517461,0.02539361766};
static const double KP[7]={300,300,150,150,40,40,30},KD[7]={2.5,2.5,2.5,2.5,.8,.8,.8};
static const double TMAX[7]={54,54,28,28,10,10,10},VMAX[7]={45,45,10,10,30,30,30};
static const double TLIM[7]={36,36,25,25,9,9,9};
static const double DIR[7]={1,-1,1,-1,1,1,1};
static const double ROTOR[7]={.010,.010,.029,.029,.0018,.0018,.0018};


/* Self-contained plant equations and nominal parameters. The optional external
 * simulator used for the separate seven-axis sweep is not needed by this test.
 * Positive Y side push at the link origin. Only measure the settled hold. */
static void cross(const double a[3],const double b[3],double c[3]){c[0]=a[1]*b[2]-a[2]*b[1];c[1]=a[2]*b[0]-a[0]*b[2];c[2]=a[0]*b[1]-a[1]*b[0];}

static void frames(const double q[7],float R[9][3][3],float P[9][3]){float qf[7];for(int i=0;i<7;i++)qf[i]=(float)q[i];ArmIk_LinkFrames(qf,R,P);}

static void mass_matrix(float R[9][3][3],float P[9][3],const double rotor[7],double M[7][7]){
  memset(M,0,49*sizeof(double));
  for(int i=1;i<=7;i++){
    const SimLink *L=&sim_links[i-1];double pc[3],Iw[3][3],T[3][3];
    for(int a=0;a<3;a++){pc[a]=P[i][a];for(int b=0;b<3;b++)pc[a]+=R[i][a][b]*L->c[b];}
    for(int a=0;a<3;a++)for(int b=0;b<3;b++){T[a][b]=0;for(int k=0;k<3;k++)T[a][b]+=R[i][a][k]*L->I[k][b];}
    for(int a=0;a<3;a++)for(int b=0;b<3;b++){Iw[a][b]=0;for(int k=0;k<3;k++)Iw[a][b]+=T[a][k]*R[i][b][k];}
    double Jv[3][7]={{0}},Jw[3][7]={{0}};
    for(int j=1;j<=i;j++){
      double z[3]={R[j][0][2],R[j][1][2],R[j][2][2]},d[3]={pc[0]-P[j][0],pc[1]-P[j][1],pc[2]-P[j][2]},c[3];
      cross(z,d,c);for(int a=0;a<3;a++){Jv[a][j-1]=c[a];Jw[a][j-1]=z[a];}
    }
    for(int r=0;r<7;r++)for(int c=0;c<7;c++){
      double x=0;for(int a=0;a<3;a++){x+=L->m*Jv[a][r]*Jv[a][c];for(int b=0;b<3;b++)x+=Jw[a][r]*Iw[a][b]*Jw[b][c];}
      M[r][c]+=x;
    }
  }
  for(int i=0;i<7;i++)M[i][i]+=rotor[i];
}
static int invert(double A[7][7],double X[7][7]){
  double a[7][14];
  for(int i=0;i<7;i++)for(int j=0;j<7;j++){a[i][j]=A[i][j];a[i][j+7]=i==j;}
  for(int c=0;c<7;c++){
    int p=c;for(int r=c+1;r<7;r++)if(fabs(a[r][c])>fabs(a[p][c]))p=r;
    if(fabs(a[p][c])<1e-14)return 0;
    if(p!=c)for(int k=0;k<14;k++){double t=a[c][k];a[c][k]=a[p][k];a[p][k]=t;}
    double d=a[c][c];for(int k=0;k<14;k++)a[c][k]/=d;
    for(int r=0;r<7;r++)if(r!=c){double f=a[r][c];for(int k=0;k<14;k++)a[r][k]-=f*a[c][k];}
  }
  for(int i=0;i<7;i++)for(int j=0;j<7;j++)X[i][j]=a[i][j+7];
  return 1;
}
/* Damiao float<->uint: truncation, no exact zero code for symmetric ranges. */
static double quant(double x,double range,int bits,double direction){
  double span=2*range,levels=(double)((1<<bits)-1),motor=x*direction;
  motor=fmin(range,fmax(-range,motor));
  double code=floor((motor+range)*levels/span);
  return (code*span/levels-range)*direction;
}

enum { HAND_ON=1000, RAMP_TICKS=400, SETTLE_TICKS=2500, HOLD_SAMPLES=3000 };
static const double q0[7]={.049,.475,-.007,.594,-.010,-.046,-.053};

static void contact_jacobian(int link,float R[9][3][3],float P[9][3],
                             double point[3],double J[3][7]){
  memset(J,0,21*sizeof(double));
  for(int a=0;a<3;a++)point[a]=P[link][a];
  for(int j=1;j<=link;j++){
    double axis[3],arm[3],column[3];
    for(int a=0;a<3;a++){axis[a]=R[j][a][2];arm[a]=point[a]-P[j][a];}
    cross(axis,arm,column);
    for(int a=0;a<3;a++)J[a][j-1]=column[a];
  }
}

typedef struct {
  unsigned samples,limited,reversals[7];
  int last_sign[7];
  double lo[3],hi[3];
} Hold;

static int push(int link,int stiffness,int cartesian,unsigned delay_ms,double tremor_m){
  const int hold_start=HAND_ON+RAMP_TICKS+SETTLE_TICKS;
  const double dt=.0001;
  double q[7],v[7]={0},torque_history[7][7]={{0}},ff[7]={0},origin[3];
  int stuck[7]={1,1,1,1,1,1,1};
  float initial[7],fc[7],fv[7],zero[7]={0};
  for(int i=0;i<7;i++){q[i]=q0[i];initial[i]=q[i];fc[i]=FC[i];fv[i]=FV[i];}
  ArmAdmittance state;ArmAdmittance_Init(&state,cartesian,initial);
  ArmAdmittanceParameters params={8,1,100,4,3,1,1.15f,.7f,1,3,1.5f,.15f};
  Hold hold={0};
  for(int tick=0;tick<hold_start+HOLD_SAMPLES;tick++){
    float R[9][3][3],P[9][3],qm[7],dqm[7],drive[7],friction[7];
    double M[7][7],inverse[7][7],point[3],J[3][7],qr[7],vr[7];
    frames(q,R,P);mass_matrix(R,P,ROTOR,M);
    if(!invert(M,inverse))return 0;
    contact_jacobian(link,R,P,point,J);
    if(tick==0)memcpy(origin,point,sizeof(origin));
    for(int i=0;i<7;i++){
      qm[i]=quant(q[i],12.5,16,DIR[i]);dqm[i]=quant(v[i],VMAX[i],12,DIR[i]);
      drive[i]=quant(torque_history[delay_ms][i],TMAX[i],12,DIR[i]);
    }
    ArmIkPose pose;ArmIk_Forward(qm,&pose);
    const uint32_t prior_bind=state.limiter.accel_ticks;
    int fault=ArmAdmittance_Tick(&state,qm,dqm,zero,drive,&pose,fc,fv,&params,.001f,1);
    if(fault || state.drift_frozen || state.frozen){
      fprintf(stderr,"link %d K %d at tick %d: fault %d or freeze\n",link,stiffness,tick,fault);
      return 0;
    }
    if(tick>=hold_start){
      hold.limited+=state.limiter.accel_ticks-prior_bind;
      for(int a=0;a<3;a++){
        if(!hold.samples)hold.lo[a]=hold.hi[a]=point[a];
        hold.lo[a]=fmin(hold.lo[a],point[a]);hold.hi[a]=fmax(hold.hi[a],point[a]);
      }
      for(int i=0;i<7;i++){
        if(fabs(dqm[i])<=4*VMAX[i]/4095)continue;
        int sign=dqm[i]>0?1:-1;
        if(hold.last_sign[i] && hold.last_sign[i]!=sign)hold.reversals[i]++;
        hold.last_sign[i]=sign;
      }
      hold.samples++;
    }
    ArmAdmittance_FrictionFeedforward(state.velocity,fc,fv,params.friction,friction);
    for(int i=0;i<7;i++){
      const double cap=.85*TLIM[i],step=cap*.001/.2;
      double target=fmin(cap,fmax(-cap,friction[i]));
      ff[i]+=fmin(step,fmax(-step,target-ff[i]));
      qr[i]=quant(state.q[i],12.5,16,DIR[i]);vr[i]=quant(state.velocity[i],VMAX[i],12,DIR[i]);
    }
    double motor[7];
    double ramp=fmin(1.,fmax(0.,(tick-HAND_ON)/(double)RAMP_TICKS));
    const double held_y=.05*ramp*ramp*(3.-2.*ramp);
    for(int sub=0;sub<10;sub++){
      const double time=tick*.001+sub*dt;
      const double y=held_y+tremor_m*sin(6.283185307*9.*time);
      double hand[3]={0},effort[7],next_v[7],impulse[7]={0},bound[7];
      for(int a=0;a<3;a++){
        double speed=0;
        for(int i=0;i<7;i++)speed+=J[a][i]*v[i];
        if(tick>HAND_ON)hand[a]=stiffness*(origin[a]+(a==1?y:0)-point[a])-40*speed;
      }
      for(int i=0;i<7;i++){
        double torque=KP[i]*(qr[i]-q[i])+KD[i]*(vr[i]-v[i])+ff[i];
        motor[i]=fmin(TMAX[i],fmax(-TMAX[i],torque));
        effort[i]=motor[i]-FV[i]*v[i];
        for(int a=0;a<3;a++)effort[i]+=J[a][i]*hand[a];
        bound[i]=dt*FC[i]*(stuck[i]?1.7:1.);
      }
      for(int i=0;i<7;i++){
        next_v[i]=v[i];
        for(int j=0;j<7;j++)next_v[i]+=dt*inverse[i][j]*effort[j];
      }
      /* Bounded friction impulses via projected Gauss-Seidel. */
      for(int it=0;it<30;it++)for(int i=0;i<7;i++){
        double desired=fmin(bound[i],fmax(-bound[i],impulse[i]-next_v[i]/inverse[i][i]));
        double change=desired-impulse[i];
        if(change!=0){
          for(int k=0;k<7;k++)next_v[k]+=inverse[k][i]*change;
          impulse[i]=desired;
        }
      }
      for(int i=0;i<7;i++){
        stuck[i]=fabs(next_v[i])<1e-7;v[i]=stuck[i]?0:next_v[i];q[i]+=dt*v[i];
        if(!isfinite(q[i]) || !isfinite(v[i]))return 0;
      }
      for(int a=0;a<3;a++)for(int i=0;i<7;i++)point[a]+=dt*J[a][i]*v[i];
    }
    memmove(torque_history+1,torque_history,delay_ms*sizeof(motor));
    memcpy(torque_history[0],motor,sizeof(motor));
  }
  double ripple2=0,worst_rev=0;
  for(int a=0;a<3;a++){double span=hold.hi[a]-hold.lo[a];ripple2+=span*span;}
  for(int i=0;i<7;i++)worst_rev=fmax(worst_rev,1000.*hold.reversals[i]/hold.samples);
  double ripple_mm=1000*sqrt(ripple2),limited_pct=100.*hold.limited/hold.samples;
  printf("%s %d %d %u %.1f %.4f %.4f %.3f\n",cartesian?"CART":"JOINT",
         link,stiffness,delay_ms,tremor_m*1000,worst_rev,ripple_mm,limited_pct);
  if(tremor_m>0)return worst_rev<3.;
  return worst_rev<1 && ripple_mm<1 && limited_pct<1;
}

int main(void){
  const int stiffness[]={1000,4000,10000,30000};
  int passed=1;
  puts("mode link hand_k_N_m delay_ms tremor_mm worst_rev_s ripple_mm accel_bind_pct");
  for(int link=3;link<=5;link+=2)
    for(unsigned k=0;k<4;k++)passed &= push(link,stiffness[k],1,2,0.);
  for(int cartesian=0;cartesian<=1;cartesian++)
    for(int link=3;link<=5;link+=2)
      for(unsigned k=0;k<3;k++)
        for(unsigned delay=2;delay<=6;delay+=4)
          passed &= push(link,stiffness[k],cartesian,delay,.0004);
  return passed?EXIT_SUCCESS:EXIT_FAILURE;
}
