/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
static void rotation_delta(const ArmIkPose *from,const ArmIkPose *to,float out[3]){
  ArmIkPose r={0};
  for(int a=0;a<3;a++)for(int b=0;b<3;b++)for(int k=0;k<3;k++)r.rotation[a][b]+=to->rotation[a][k]*from->rotation[b][k];
  ArmIk_RotationLog(&r,out);
}
void ArmAdmittance_DesiredPose(const ArmAdmittance *s,ArmIkPose *pose){
  float r[3][3];ArmIk_RotationExp(s->error+3,r);
  for(int a=0;a<3;a++){
    pose->position[a]=s->goal.position[a]+s->error[a];
    for(int b=0;b<3;b++){pose->rotation[a][b]=0;for(int k=0;k<3;k++)pose->rotation[a][b]+=r[a][k]*s->goal.rotation[k][b];}
  }
}
int ArmAdmittance_NullVelocity(const ArmAdmittance *s,const ArmAdmittanceFrame *f,
  const ArmAdmittanceParameters *p,float out[7]){
  float joint[7];
  for(int i=0;i<7;i++)joint[i]=(s->ns[i]-p->null_k*(s->q[i]-s->equilibrium[i]))/p->null_d;
  return ArmAdmittance_Project(f,joint,out);
}
/* Left Jacobian of Exp: rotation-vector derivative -> base angular velocity. */
static void angular_velocity(const float e[3],const float rate[3],float out[3]){
  float theta=norm(e,3),a=.5f,b=1.f/6.f;
  if(theta>1e-4f){a=(1-cosf(theta))/(theta*theta);b=(theta-sinf(theta))/(theta*theta*theta);}
  float cross[3]={e[1]*rate[2]-e[2]*rate[1],e[2]*rate[0]-e[0]*rate[2],e[0]*rate[1]-e[1]*rate[0]};
  float twice[3]={e[1]*cross[2]-e[2]*cross[1],e[2]*cross[0]-e[0]*cross[2],e[0]*cross[1]-e[1]*cross[0]};
  for(int i=0;i<3;i++)out[i]=rate[i]+a*cross[i]+b*twice[i];
}
/* The slow-IK host fixture can tighten the shipped joint velocity caps. */
#ifndef ARM_ADM_TEST_SPEED_CAP
#define ARM_ADM_TEST_SPEED_CAP 1.5f
#endif
int ArmAdmittance_LimitReference(ArmAdmittance *s,float desired[7],float dt){
  float scale=1,accel=1,maxv=0;
  for(int i=0;i<7;i++){scale=fminf(scale,fminf(i<4?1.f:1.5f,ARM_ADM_TEST_SPEED_CAP)/fmaxf(fabsf(desired[i]),1e-8f));maxv=fmaxf(maxv,fabsf(desired[i]));}
  for(int i=0;i<7;i++){
    float distance=desired[i]>=0?ArmIk_JointUpper()[i]-.03f-s->q[i]:s->q[i]-ArmIk_JointLower()[i]-.03f;
    float denom=fabsf(desired[i])*maxv;
    if(denom>1e-10f)scale=fminf(scale,sqrtf(16.f*fmaxf(0,distance-fabsf(s->velocity[i])*dt)/denom));
  }
  /* Retain the conservative stopping-distance envelope near a joint limit.
   * Only CART's tracking of the requested reference may accelerate faster. */
  const float max_step=dt*(s->cartesian ? ARM_ADM_REFERENCE_ACCEL_RAD_S2 :
                                          ARM_ADM_JOINT_REFERENCE_ACCEL_RAD_S2);
  float largest_delta=0.f;
  unsigned largest_axis=0;
  for(int i=0;i<7;i++){
    desired[i]*=scale;
    const float delta=fabsf(desired[i]-s->velocity[i]);
    if(delta>largest_delta){largest_delta=delta;largest_axis=(unsigned)i;}
  }
  if(largest_delta>max_step)accel=max_step/largest_delta;

  if(scale<1.f)++s->limiter.velocity_ticks;
  if(accel<1.f)++s->limiter.accel_ticks;
  const float needed=largest_delta/dt;
  if(needed>s->limiter.peak_accel){
    s->limiter.peak_accel=needed;
    s->limiter.peak_joint=(uint8_t)(largest_axis+1);
  }
  float v[7],q[7];
  for(int i=0;i<7;i++){
    v[i]=s->velocity[i]+accel*(desired[i]-s->velocity[i]);q[i]=s->q[i]+dt*v[i];
    /* Never solve conflicting hard bounds by issuing a discontinuous reference. */
    if(!isfinite(q[i])||q[i]<ArmIk_JointLower()[i]+.03f-1e-7f||q[i]>ArmIk_JointUpper()[i]-.03f+1e-7f)return 0;
  }
  memcpy(s->q,q,sizeof(q));memcpy(s->velocity,v,sizeof(v));return 1;
}
/* Both lag vectors refer to the start of this tick, like frame->pose.
 * A reachable reference advances during integration; comparing it against
 * yesterday's FK would subtract one velocity step on every tick. */
void ArmAdmittance_Pullback(ArmAdmittance *s,const float position_lag[3],
                            const float rotation_lag[3],float dt)
{
  const float distance=norm(position_lag,3);
  const float angle=norm(rotation_lag,3);
  const int lagging=distance>.005f || angle>.030f;
  s->projection_time=lagging?s->projection_time+dt:0.f;
  s->drift_time=(distance>.020f || angle>.15f)?s->drift_time+dt:0.f;
  if(s->drift_time+1e-5f>=.5f)s->drift_frozen=1;
  else if(!lagging && s->release>0.f)s->drift_frozen=0;

  /* Smooth proportional convergence inside the rate cap. Do not chase FK
   * roundoff or damp a synchronized, already-feasible trajectory. */
  const float fraction=fminf(1.f,8.f*dt);
  const float position_step=fminf(distance*fraction,.05f*dt);
  const float angle_step=fminf(angle*fraction,.20f*dt);
  s->correction_peak_m=fmaxf(s->correction_peak_m,position_step);
  s->correction_peak_rad=fmaxf(s->correction_peak_rad,angle_step);
  for(int i=0;i<3;i++){
    if(distance>0.f)s->error[i]-=position_lag[i]*(position_step/distance);
    if(angle>0.f)s->error[i+3]-=rotation_lag[i]*(angle_step/angle);
  }
  /* Cancel only the outward component while pullback is rate-limited. */
  if(distance*fraction>.05f*dt)remove_outward(s->rate,position_lag);
  if(angle*fraction>.20f*dt)remove_outward(s->rate+3,rotation_lag);
  if(s->drift_frozen)memset(s->rate,0,sizeof(s->rate));
}

int ArmAdmittance_Reference(ArmAdmittance *s,const ArmAdmittanceFrame *f,
  const ArmAdmittanceParameters *p,float dt){
  float position[3],rot[3],omega[3],rhs[6],y[6],ns[7],desired[7];
  rotation_delta(&f->pose,&s->reference,rot);
  angular_velocity(s->error+3,s->rate+3,omega);
  for(int i=0;i<3;i++){
    position[i]=s->reference.position[i]-f->pose.position[i];
    rhs[i]=(s->rate[i]+30*position[i])/ARM_ADM_LEVER;
    rhs[i+3]=omega[i]+30*rot[i];
  }
  if(!ArmAdmittance_Solve(f,rhs,y)||!ArmAdmittance_NullVelocity(s,f,p,ns))return 0;
  for(int i=0;i<7;i++){desired[i]=ns[i];for(int a=0;a<6;a++)desired[i]+=f->j[a][i]*y[a];}
  if(!ArmAdmittance_LimitReference(s,desired,dt))return 0;
  ArmAdmittance_Pullback(s,position,rot,dt);
  ArmAdmittance_DesiredPose(s,&s->reference);
  return valid_vector(s->q,7)&&valid_vector(s->velocity,7);
}

int ArmAdmittance_JointStep(ArmAdmittance *s,const ArmAdmittanceParameters *p,float dt){
  static const float mass[7]={.15f,.15f,.08f,.08f,.02f,.02f,.02f};
  float desired[7];
  release_gate(s,dt);
  for(int i=0;i<7;i++){
    float m=mass[i]*p->joint_mass_scale,d=2*p->zeta*sqrtf(p->joint_k*m);
    desired[i]=s->velocity[i]+dt*(s->external[i]-d*s->velocity[i]-p->joint_k*(s->q[i]-s->equilibrium[i]))/m;
  }
  return ArmAdmittance_LimitReference(s,desired,dt);
}
