/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Offline ideal-tracking checks, not a motor/loaded-arm performance claim. */
#include "arm_motion.h"
#include "app_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static float norm3(const float x[3]) { return sqrtf(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]); }
int main(void) {
  assert(fabsf(DM_ETH_COMMAND_SLEW_RAD_S-3.14f)<1e-6f);
  assert(fabsf(ArmMotion_StreamPositionTarget(0,1,DM_ETH_COMMAND_SLEW_RAD_S,.001f)-.00314f)<1e-8f);
  assert(fabsf(ArmMotion_StreamPositionTarget(0,-1,DM_ETH_COMMAND_SLEW_RAD_S,.001f)+.00314f)<1e-8f);
  /* A hold/reference offset must not make movej retiming underestimate
   * distance and silently exceed 3.14 at the profile's velocity peak. */
  ArmMoveFeedback fb={0}; ArmMoveCommand cmd={0}; ArmMotion move={0};
  fb.ready=fb.command_valid=1; fb.enabled_mask=fb.fresh_mask=fb.healthy_mask=ARM_MASK;
  cmd.timeout_ms=10000;cmd.dwell_ms=100;cmd.position_tolerance=.01f;cmd.velocity_tolerance=.05f;
  for(unsigned j=0;j<7;++j) {
    fb.position[j]=.05f;fb.command[j]=0;fb.lower[j]=-2;fb.upper[j]=2;
    fb.velocity_max[j]=3.14f;fb.torque_max[j]=10;
    cmd.target[j]=1.9f;cmd.velocity_max[j]=3.14f;cmd.acceleration_max[j]=10;
    cmd.kp[j]=30;cmd.kd[j]=1;
  }
  assert(ArmMotion_Request(&move,ARM_MOVEJ,1,1,&cmd,&fb,0)==ARM_OK);
  for(unsigned k=1;k<=move.duration_ms;++k) {
    move.last_heartbeat_ms=k; ArmMotion_Tick(&move,&fb,k);
    for(unsigned j=0;j<7;++j) {
      assert(fabsf(move.velocity[j])<=3.14001f);
      fb.position[j]=move.position[j]; fb.velocity[j]=move.velocity[j];
    }
  }
  const float seeds[2][7]={{.3f,1.f,-.2f,1.2f,.2f,.3f,-.5f},{0,.02f,0,.04f,0,0,0}};
  const char *names[]={"x +200mm","y +200mm","z -250mm","wrist rotation +2rad","home z +300mm"};
  for(unsigned c=0;c<5;++c) {
    ArmCartesianLimits limits;
    uint16_t wire[4]={0}; assert(ArmMotion_CartesianLimitsFromWire(wire,&limits));
    for(unsigned j=0;j<7;++j) assert(limits.joint_velocity_max[j]<=3.140001f);
    ArmCartesianTracker t; ArmCartesianTracker_Start(&t,seeds[c==4],NULL,NULL); t.ik_options.time_budget_us=0;
    ArmIkPose start,target,previous;
    ArmIk_Forward(t.q,&start); target=previous=start;
    if(c<3) target.position[c]+= c==2 ? -.25f : .2f;
    else if(c==4) target.position[2]+=.3f;
    else {
      float rv[3],delta[3][3];
      for(unsigned i=0;i<3;++i) rv[i]=2.f*start.rotation[i][2];
      ArmIk_RotationExp(rv,delta);
      for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) {
        target.rotation[i][j]=0;
        for(unsigned k=0;k<3;++k) target.rotation[i][j]+=delta[i][k]*start.rotation[k][j];
      }
    }
    float peak_v=0,peak_w=0,peak_joint=0,min_scale=1;
    unsigned limited=0;
    for(unsigned k=0;k<4000;++k) {
      ArmCartesianTracker_Step(&t,&target,&limits,.001f);
      for(unsigned j=0;j<7;++j) {
        assert(fabsf(t.v[j])<=3.141f);
        peak_joint=fmaxf(peak_joint,fabsf(t.v[j]));
      }
      ArmIkPose p; ArmIk_Forward(t.q,&p);
      float dp[3],r[3][3]={0},w[3];
      for(unsigned i=0;i<3;++i) {
        dp[i]=(p.position[i]-previous.position[i])*1000;
        for(unsigned j=0;j<3;++j) for(unsigned n=0;n<3;++n) r[i][j]+=p.rotation[i][n]*previous.rotation[j][n];
      }
      w[0]=(r[2][1]-r[1][2])*500; w[1]=(r[0][2]-r[2][0])*500; w[2]=(r[1][0]-r[0][1])*500;
      peak_v=fmaxf(peak_v,norm3(dp)); peak_w=fmaxf(peak_w,norm3(w));
      if(t.scale<.999f) { ++limited; min_scale=fminf(min_scale,t.scale); }
      previous=p;
    }
    printf("%s: peak TCP %.1f mm/s, %.1f deg/s; joint %.3f rad/s; limited=%u min_scale=%.3f blocked=%u\n",
           names[c],peak_v*1000,peak_w*57.29578f,peak_joint,limited,min_scale,t.blocked);
  }
  puts("all J1-J7 command velocities <= 3.14 rad/s (+float tolerance)");
}
