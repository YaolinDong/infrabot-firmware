/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
static int parameters_valid(const ArmAdmittanceParameters *p){
  const float a[]={p->joint_k,p->joint_mass_scale,p->linear_k,p->rotation_k,p->mass,p->zeta,
    p->static_ratio,p->friction,p->drive_gain,p->null_k,p->null_d,p->return_speed};
  return valid_vector(a,12) && p->joint_k>0 && p->joint_mass_scale>0 && p->linear_k>0 && p->rotation_k>0 &&
    p->mass>0 && p->zeta>0 && p->static_ratio>0 && p->friction>=0 && p->friction<=.9f &&
    p->drive_gain>=.3f && p->drive_gain<=1.f && p->null_k>=0 && p->null_d>0 && p->return_speed>0;
}
static void measured_motion(ArmAdmittance *s,const ArmIkPose *pose){
  if(++s->history_ticks==10){
    s->history_ticks=0;s->history_head=(s->history_head+1)%5;s->measured_history[s->history_head]=*pose;
    const ArmIkPose *old=&s->measured_history[(s->history_head+1)%5];
    float dp[3],dr[3];rotation_delta(old,pose,dr);
    for(int i=0;i<3;i++)dp[i]=pose->position[i]-old->position[i];
    float scale_p=norm(dp,3)<=.00015f?0:25,scale_r=norm(dr,3)<=.0005f?0:25;
    for(int i=0;i<3;i++){s->measured_motion[i]=dp[i]*scale_p;s->measured_motion[i+3]=dr[i]*scale_r;}
  }
}
static void reference_from_joints(ArmAdmittance *s,const ArmAdmittanceFrame *f){
  ArmIk_Forward(s->q,&s->reference);
  for(int a=0;a<3;a++){
    s->error[a]=s->reference.position[a]-s->goal.position[a];s->rate[a]=s->rate[a+3]=0;
    for(int i=0;i<7;i++){s->rate[a]+=f->linear[a][i]*s->velocity[i];s->rate[a+3]+=f->angular[a][i]*s->velocity[i];}
  }
  rotation_delta(&s->goal,&s->reference,s->error+3);
}
static void return_diagnostics(ArmAdmittance *s,const float measured[7],const ArmIkPose *actual,float dt,int loaded){
  if(loaded){s->active_return=0;return;}
  float maximum_joint=0,virtual_joint=0;
  for(int i=0;i<7;i++){maximum_joint=fmaxf(maximum_joint,fabsf(measured[i]-s->equilibrium[i]));virtual_joint=fmaxf(virtual_joint,fabsf(s->q[i]-s->equilibrium[i]));}
  if(s->release>0 && s->return_armed && !s->active_return &&
     (norm(s->error,3)>.0005f || norm(s->error+3,3)>.00349066f || virtual_joint>.005f)){
    s->active_return=1;s->return_armed=0;s->return_time=s->return_dev2=s->return_peak=0;
    for(int i=0;i<3;i++){s->return_origin[i]=actual->position[i];s->return_delta[i]=s->goal.position[i]-actual->position[i];}
  }
  if(!s->active_return)return;
  float from_start[3],off[3],error[3],rotation[3];
  for(int i=0;i<3;i++){from_start[i]=actual->position[i]-s->return_origin[i];error[i]=actual->position[i]-s->goal.position[i];}
  rotation_delta(&s->goal,actual,rotation);
  float line2=dot(s->return_delta,s->return_delta,3);
  float u=line2>1e-12f?fmaxf(0,fminf(1,dot(from_start,s->return_delta,3)/line2)):0;
  for(int i=0;i<3;i++)off[i]=from_start[i]-u*s->return_delta[i];
  s->return_dev2=fmaxf(s->return_dev2,dot(off,off,3));
  s->return_peak=fmaxf(s->return_peak,norm(s->measured_motion,3));s->return_time+=dt;
  /* Match the stated final accuracy, including measured attitude and elbow.
   * Do not report OK while a redundant posture is still far from equilibrium. */
  int done=norm(s->error,3)<.0005f && norm(s->error+3,3)<.00349066f &&
    norm(error,3)<.002f && norm(rotation,3)<.00872665f && maximum_joint<.010f;
  if(done || s->return_time>=4){
    s->ret.sequence++;s->ret.result=done?0:1;s->ret.elapsed=s->return_time;
    s->ret.deviation2=s->return_dev2;s->ret.peak_speed=s->return_peak;
    s->ret.final_position=norm(error,3);s->ret.final_rotation=norm(rotation,3);s->ret.ns_error=maximum_joint;memcpy(s->ret.equilibrium,s->equilibrium,sizeof(s->equilibrium));
    s->active_return=0;
  }
}
/* Learn only on an uninterrupted settled interval, after this tick's
 * contact classification and safety timers have been evaluated. */
void ArmAdmittance_UpdateBias(ArmAdmittance *s,float dt){
  int allowed=s->bias_learning && !s->frozen && !s->drift_frozen &&
              !s->loaded && s->release==1.f &&
              norm(s->measured_motion,3)<.004f && norm(s->measured_motion+3,3)<.030f;
  for(int i=0;i<7;i++){
    if(s->tracking_time[i]>0.f || s->overload_time[i]>0.f)allowed=0;
  }
  if(!allowed){s->still_time=0.f;return;}
  s->still_time=fminf(1.5f,s->still_time+dt);
  if(s->still_time<1.5f)return;
  const float gain=1.f-expf(-.5f*dt);
  for(int i=0;i<7;i++){
    const float limit=i<4?.30f:.15f;
    s->bias[i]=fminf(limit,fmaxf(-limit,s->bias[i]+gain*s->raw[i]));
  }
}

int ArmAdmittance_Tick(ArmAdmittance *s,const float measured[7],const float measured_velocity[7],const float gravity[7],
  const float drive[7],const ArmIkPose *actual,const float fc[7],const float fv[7],
  const ArmAdmittanceParameters *p,float dt,int run){
  if(!parameters_valid(p)||!valid_vector(measured,7)||!valid_vector(gravity,7)||!valid_vector(drive,7)||
     !valid_vector(s->q,7)||!valid_vector(s->velocity,7)||!valid_vector(s->error,6)||
     !valid_vector(s->rate,6)||!isfinite(dt)||dt<=0||dt>.01f)return ARM_ADM_NONFINITE;
  measured_motion(s,actual);
  if(!ArmAdmittance_Estimate(s,measured_velocity,gravity,drive,fc,fv,p,dt))return ARM_ADM_NONFINITE;
  if(norm(s->measured_motion,3)<.004f && norm(s->measured_motion+3,3)<.03f){
    for(int i=0;i<7;i++)s->rest_sum[i]+=s->raw[i]+s->bias[i]; /* calibration needs pre-bias raw */
    s->rest_count++;
  }
  if(!run){
    s->still_time=0.f;
    memset(s->external,0,sizeof(s->external));memset(s->velocity,0,sizeof(s->velocity));
    return ARM_ADM_OK;
  }
  int safety=ArmAdmittance_CheckSafety(s,measured,dt);
  if(safety)return safety;
  if(s->frozen){
    s->still_time=0.f;
    memset(s->velocity,0,sizeof(s->velocity));memset(s->rate,0,sizeof(s->rate));s->active_return=0;
    return ARM_ADM_OK;
  }
  ArmAdmittanceFrame frame;
  if(!ArmAdmittance_Factor(s->q,&frame))return ARM_ADM_FACTOR;
  if(!ArmAdmittance_Map(&frame,s->external,s->force,s->moment,s->ns))return ARM_ADM_FACTOR;
  memcpy(s->effective_force,s->force,sizeof(s->force));memcpy(s->effective_moment,s->moment,sizeof(s->moment));
  ArmAdmittance_SoftDeadzone(s->effective_force,3,1.f);ArmAdmittance_SoftDeadzone(s->effective_moment,3,.1f);ArmAdmittance_SoftDeadzone(s->ns,7,.2f);
  float force_scale=fminf(1,40/fmaxf(norm(s->effective_force,3),1e-10f)),moment_scale=fminf(1,4/fmaxf(norm(s->effective_moment,3),1e-10f));
  for(int i=0;i<3;i++){s->effective_force[i]*=force_scale;s->effective_moment[i]*=moment_scale;}
  if(s->cartesian){
    /* Both reference states start this tick at the same time. Integrate
     * semi-implicitly with the new virtual velocity; avoid a one-tick lead. */
    ArmAdmittance_DesiredPose(s,&s->reference);
    ArmAdmittance_Dynamics(s,p,dt);
    if(!ArmAdmittance_Reference(s,&frame,p,dt))return ARM_ADM_FACTOR;
  }else{
    if(!ArmAdmittance_JointStep(s,p,dt))return ARM_ADM_FACTOR;
    reference_from_joints(s,&frame);
  }
  ArmAdmittance_UpdateBias(s,dt);
  return_diagnostics(s,measured,actual,dt,s->loaded);
  return valid_vector(s->q,7)&&valid_vector(s->velocity,7)&&valid_vector(s->error,6)&&valid_vector(s->rate,6)?ARM_ADM_OK:ARM_ADM_NONFINITE;
}
