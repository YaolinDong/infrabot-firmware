/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Private pure dynamics shared by the host simulations and the MCU Tick. */
static void remove_outward(float vector[3],const float direction[3]){
  float d=dot(direction,direction,3);
  if(d>1e-16f){float component=dot(vector,direction,3)/d;if(component>0)for(int i=0;i<3;i++)vector[i]-=component*direction[i];}
}
static void workspace(float e[3],float v[3],float force[3],float maximum){
  float length=norm(e,3);
  if(length>=maximum){
    remove_outward(force,e);remove_outward(v,e);
    if(length>maximum)for(int i=0;i<3;i++)e[i]*=maximum/length;
  }
}
static int adm_loaded(const ArmAdmittance *s){
  const float force=s->loaded?.4f:.6f;
  const float moment=s->loaded?.05f:.08f;
  const float null_torque=s->loaded?.18f:.25f;
  for(int i=0;i<7;i++)if(fabsf(s->ns[i])>null_torque)return 1;
  return norm(s->effective_force,3)>force || norm(s->effective_moment,3)>moment;
}

static void release_gate(ArmAdmittance *s,float dt){
  s->loaded=(uint8_t)adm_loaded(s);
  if(s->loaded){s->quiet=0;s->release=0;s->return_armed=1;}
  else{
    s->quiet+=dt;
    if(s->quiet>=.15f)s->release=fminf(1,s->release+dt/.2f);
  }
}
void ArmAdmittance_Dynamics(ArmAdmittance *s,const ArmAdmittanceParameters *p,float dt){
  if(s->frozen){memset(s->rate,0,sizeof(s->rate));return;}
  release_gate(s,dt);
  if(s->drift_frozen){memset(s->rate,0,sizeof(s->rate));return;}
  const float wn=sqrtf(p->linear_k/p->mass);
  const float inertia=p->rotation_k/(wn*wn);
  const float dlin=2*p->zeta*sqrtf(p->linear_k*p->mass);
  const float drot=2*p->zeta*sqrtf(p->rotation_k*inertia);
  workspace(s->error,s->rate,s->effective_force,.25f);
  workspace(s->error+3,s->rate+3,s->effective_moment,1.2f);
  float scaled_e[6],scaled_v[6],accel[6];
  for(int i=0;i<3;i++){
    accel[i]=(s->effective_force[i]-dlin*s->rate[i]-p->linear_k*s->error[i])/p->mass;
    accel[i+3]=(s->effective_moment[i]-drot*s->rate[i+3]-p->rotation_k*s->error[i+3])/inertia;
    scaled_e[i]=s->error[i]/ARM_ADM_LEVER;scaled_v[i]=s->rate[i]/ARM_ADM_LEVER;
    scaled_e[i+3]=s->error[i+3];scaled_v[i+3]=s->rate[i+3];
  }
  float e2=dot(scaled_e,scaled_e,6),along=e2>1e-12f?dot(scaled_e,scaled_v,6)/e2:0;
  for(int i=0;i<6;i++){
    float perpendicular=scaled_v[i]-along*scaled_e[i];
    accel[i]-=s->release*6*wn*perpendicular*(i<3?ARM_ADM_LEVER:1.f);
    s->rate[i]+=dt*accel[i];
  }
  float vmax=s->release>0?p->return_speed:.30f;
  float wmax=s->release>0?1.2f*(p->return_speed/.30f):1.2f;
  float scale=fminf(1.f,fminf(vmax/fmaxf(norm(s->rate,3),1e-10f),wmax/fmaxf(norm(s->rate+3,3),1e-10f)));
  for(int i=0;i<6;i++){s->rate[i]*=scale;s->error[i]+=dt*s->rate[i];}
  workspace(s->error,s->rate,s->effective_force,.25f);
  workspace(s->error+3,s->rate+3,s->effective_moment,1.2f);
}
