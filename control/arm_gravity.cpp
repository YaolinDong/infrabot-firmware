/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "arm_gravity.h"
#include <cmath>

namespace {
struct V { float x, y, z; };
struct M { float a[9]; };
struct Link { V xyz; M rotation; V axis; float mass; V first; float offset, spring, viscous, coulomb; };
#include "arm_gravity_model.inc"
V add(V a,V b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
V sub(V a,V b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
V scale(V a,float s) { return {a.x*s,a.y*s,a.z*s}; }
V cross(V a,V b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
float dot(V a,V b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
V mul(M a,V v) { return {a.a[0]*v.x+a.a[1]*v.y+a.a[2]*v.z,
                        a.a[3]*v.x+a.a[4]*v.y+a.a[5]*v.z,
                        a.a[6]*v.x+a.a[7]*v.y+a.a[8]*v.z}; }
M mul(M a,M b) {
  M r{};
  for (unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j)
    for(unsigned k=0;k<3;++k) r.a[3*i+j]+=a.a[3*i+k]*b.a[3*k+j];
  return r;
}
M rotation(V u,float q) {
  const float c=cosf(q),s=sinf(q),t=1-c;
  return {{c+u.x*u.x*t,u.x*u.y*t-u.z*s,u.x*u.z*t+u.y*s,
           u.y*u.x*t+u.z*s,c+u.y*u.y*t,u.y*u.z*t-u.x*s,
           u.z*u.x*t-u.y*s,u.z*u.y*t+u.x*s,c+u.z*u.z*t}};
}
}
extern "C" const char *ArmGravity_ModelName() { return k_name; }
extern "C" const char *ArmGravity_ModelHash() { return k_hash; }
extern "C" const float *ArmGravity_Coulomb(){
  static constexpr float c[]={k_links[0].coulomb,k_links[1].coulomb,k_links[2].coulomb,k_links[3].coulomb,k_links[4].coulomb,k_links[5].coulomb,k_links[6].coulomb};return c;
}
extern "C" const float *ArmGravity_Viscous(){
  static constexpr float c[]={k_links[0].viscous,k_links[1].viscous,k_links[2].viscous,k_links[3].viscous,k_links[4].viscous,k_links[5].viscous,k_links[6].viscous};return c;
}
extern "C" const float *ArmGravity_Lower() { return k_lower; }
extern "C" const float *ArmGravity_Upper() { return k_upper; }
extern "C" int ArmGravity_Evaluate(const float q[7],const float dq[7],float friction_scale,
                                    float torque[7],float twist[6]) {
  return ArmGravity_EvaluateTuned(q,dq,friction_scale,.15f,torque,twist);
}
extern "C" int ArmGravity_EvaluateTuned(const float q[7],const float dq[7],float friction_scale,
                                         float deadband,float torque[7],float twist[6]) {
  return ArmGravity_EvaluateParts(q,dq,friction_scale,deadband,torque,twist,nullptr);
}
extern "C" int ArmGravity_EvaluateParts(const float q[7],const float dq[7],float friction_scale,
                                         float deadband,float torque[7],float twist[6],float friction[7]) {
  if(!q || !dq || !torque || !std::isfinite(friction_scale) || friction_scale<0 || friction_scale>1) return 0;
  if(!std::isfinite(deadband) || deadband<.02f || deadband>.5f) return 0;
  for(unsigned i=0;i<7;++i) if(!std::isfinite(q[i]) || !std::isfinite(dq[i])) return 0;
  V origins[7],axes[7],forces[7],moments[7];
  M world{{1,0,0,0,1,0,0,0,1}};
  V position{0,0,0};
  const V acceleration=scale(k_gravity,-1);
  for(unsigned i=0;i<7;++i) {
    const Link &l=k_links[i];
    position=add(position,mul(world,l.xyz));
    world=mul(world,l.rotation);
    axes[i]=mul(world,l.axis); origins[i]=position;
    world=mul(world,rotation(l.axis,q[i]));
    forces[i]=scale(acceleration,l.mass);
    moments[i]=cross(mul(world,l.first),acceleration);
  }
  V force{0,0,0},moment{0,0,0};
  for(int i=6;i>=0;--i) {
    if(i<6) moment=add(moment,cross(sub(origins[i+1],origins[i]),force));
    moment=add(moment,moments[i]); force=add(force,forces[i]);
    const Link &l=k_links[i];
    float tau=dot(axes[i],moment)+l.offset+l.spring*q[i];
    float friction_tau=0.f;
    if (friction_scale>0.f && fabsf(dq[i])>deadband) {
      const float ramp=fminf((fabsf(dq[i])-deadband)/deadband,1.f);
      friction_tau=friction_scale*ramp*(l.viscous*dq[i]+l.coulomb*tanhf(dq[i]/.05f));
    }
    tau+=friction_tau;
    tau+=20.f*(fmaxf(k_lower[i]-q[i],0)-fmaxf(q[i]-k_upper[i],0));
    if(!std::isfinite(tau)) return 0;
    torque[i]=tau;
    if(friction) friction[i]=friction_tau;
  }
  if(twist) {
    const V tcp=add(position,mul(world,k_tcp));
    V v{0,0,0},w{0,0,0};
    for(unsigned i=0;i<7;++i) {
      w=add(w,scale(axes[i],dq[i]));
      v=add(v,scale(cross(axes[i],sub(tcp,origins[i])),dq[i]));
    }
    twist[0]=v.x;twist[1]=v.y;twist[2]=v.z;twist[3]=w.x;twist[4]=w.y;twist[5]=w.z;
  }
  return 1;
}
