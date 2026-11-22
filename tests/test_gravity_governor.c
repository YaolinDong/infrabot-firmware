/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Exercise the real controller output, plus a small inertial plant with
 * perfect gravity cancellation. This checks signs/limits/dynamics, not the
 * physical arm's attainable speed under an arbitrary external force. */
#define main app_motion_tests_main
#include "test_app_motion.c"
#undef main
#include "app_control_mode.h"
#include "app_gravity.h"
#include "arm_gravity.h"

uint8_t AppFdcan_BusFault(void) { return 0; }
uint8_t AppTasks_Enabling(void) { return 0; }
static const float speed_limit[7]=DM_GRAVITY_MAX_SPEED_RAD_S;
static const float torque_limit[7]=DM_ARM_TORQUE_LIMIT_NM;
static void step(void) {
  ++tick;
  for(unsigned i=0;i<7;++i) dm_motors[i].feedback_tick=tick;
  AppGravity_Tick(); AppSafety_Tick();
}
static AppGravityStatus status(void) { AppGravityStatus s; AppGravity_Status(&s); return s; }
static void setup(uint8_t mode) {
  dm_motor_runtime.control_enabled=0; AppGravity_Tick(); ready(DM_ETH_MODE_MIT);
  eth_udp_command.control_mode=DM_ETH_CONTROL_IDLE;
  const float vmax[7]={45,45,10,10,30,30,30};
  for(unsigned i=0;i<7;++i) {
    const float q=.5f*(ArmGravity_Lower()[i]+ArmGravity_Upper()[i]);
    dm_motors[i].position=dm_motor_runtime.zero_offset[i]+DM_MOTOR_DIRECTION(i)*q;
    dm_motors[i].velocity=0; dm_motors[i].velocity_max=vmax[i];
    dm_motor_runtime.command_position[i]=dm_motors[i].position;
    eth_udp_command.target_kp[i]=50; eth_udp_command.target_kd[i]=2;
    eth_udp_command.target_velocity[i]=eth_udp_command.target_torque[i]=0;
    dm_motor_runtime.stopped[i]=0;
  }
  assert(!AppGravity_SetParameter(APP_GRAVITY_DAMPING,.05f)); /* below the Kd minimum */
  assert(AppGravity_SetParameter(APP_GRAVITY_DAMPING,.1f)); /* limiter must work at the lowest Kd */
  assert(AppControlMode_Switch(mode)==0);
  for(unsigned t=0;t<1500;++t) step();
}
static void speed(unsigned j,float v) { dm_motors[j].velocity=DM_MOTOR_DIRECTION(j)*v; }
static void model(float out[7]) {
  float q[7],dq[7];
  for(unsigned i=0;i<7;++i) {
    q[i]=(dm_motors[i].position-dm_motor_runtime.zero_offset[i])*DM_MOTOR_DIRECTION(i);
    dq[i]=dm_motors[i].velocity*DM_MOTOR_DIRECTION(i);
  }
  assert(ArmGravity_Evaluate(q,dq,0.f,out,NULL));
}
int main(void) {
  dm_motor_runtime.zero_valid=1;
  float baseline[7];
  for(unsigned mode=DM_ETH_CONTROL_GRAVITY_FREE;mode<=DM_ETH_CONTROL_GRAVITY_LOCK;++mode)
  for(unsigned j=0;j<7;++j) for(unsigned sign=0;sign<2;++sign) {
    const float direction=sign ? -1.f : 1.f;
    setup((uint8_t)mode);
    speed(j,direction*.45f*speed_limit[j]); step(); model(baseline);
    AppGravityStatus s=status();
    assert(!s.speed_limited_mask);
    assert(fabsf(s.torque[j]-baseline[j])<1e-5f); /* original compensation below soft zone */
    const float v=direction*1.01f*speed_limit[j];
    speed(j,v);
    float previous=0;
    for(unsigned t=0;t<100;++t) {
      step(); s=status();
      assert(s.mode==mode && !s.recovering && (s.speed_limited_mask & (1U<<j)));
      assert(s.speed_brake[j]*v<=0); /* no extra propulsive power */
      assert(fabsf(s.speed_brake[j])<=DM_GRAVITY_SPEED_BRAKE_RATIO*torque_limit[j]+1e-5f);
      assert(fabsf(s.speed_brake[j]-previous)<=DM_GRAVITY_SPEED_BRAKE_RATIO*torque_limit[j]/
             (DM_CONTROL_HZ*DM_GRAVITY_SPEED_BRAKE_RAMP_S)+1e-5f);
      previous=s.speed_brake[j];
    }
    model(baseline); /* the brake is the only addition to the model */
    assert(fabsf(s.torque[j]-s.speed_brake[j]-baseline[j])<1e-5f);
    assert(eth_udp_command.target_kp[j]==(mode==DM_ETH_CONTROL_GRAVITY_FREE ? 0 : DM_GRAVITY_LOCK_KP));
    /* Reverse/release: never retain a brake in the old direction. */
    speed(j,-v); step(); s=status(); assert(s.speed_brake[j]*(-v)<=0);
    speed(j,0); step(); s=status();
    assert(!s.speed_limited_mask && s.speed_brake[j]==0);
    model(baseline); assert(fabsf(s.torque[j]-baseline[j])<1e-5f);
    /* An explicit STOP during limiting removes the temporary brake as
     * holding gains recover, so it never remains as a static torque bias. */
    speed(j,v);
    for(unsigned t=0;t<100;++t) step();
    s=status(); const float holding=s.torque[j]-s.speed_brake[j];
    AppSafety_Stop(INFRABOT_STOP_REQUESTED,INFRABOT_NO_JOINT,0);
    speed(j,0);
    for(unsigned t=0;t<600;++t) step();
    s=status();
    assert(!s.mode && !s.recovering && !s.speed_limited_mask && s.speed_brake[j]==0);
    assert(fabsf(eth_udp_command.target_torque[j]-holding)<1e-5f && dm_motor_runtime.enabled[j]);
  }
  /* Continued modest hand torque: without a governor, the lowest Kd (0.1) would keep
   * accelerating. Here every axis returns below the nominal speed while
   * retaining FREE, using only bounded dissipative braking. */
  for(unsigned j=0;j<7;++j) for(unsigned sign=0;sign<2;++sign) {
    setup(DM_ETH_CONTROL_GRAVITY_FREE);
    const float direction=sign ? -1.f : 1.f;
    float v=direction*1.1f*speed_limit[j];
    float q=.5f*(ArmGravity_Lower()[j]+ArmGravity_Upper()[j]);
    float peak=fabsf(v);
    for(unsigned t=0;t<120;++t) {
      dm_motors[j].position=dm_motor_runtime.zero_offset[j]+DM_MOTOR_DIRECTION(j)*q;
      speed(j,v); step(); model(baseline);
      const AppGravityStatus s=status();
      assert(s.mode==DM_ETH_CONTROL_GRAVITY_FREE && !s.recovering);
      const float net=eth_udp_command.target_torque[j]-baseline[j]+direction*.05f*torque_limit[j];
      v+=net/(.002f*torque_limit[j])/DM_CONTROL_HZ;
      q+=v/DM_CONTROL_HZ;
      peak=fmaxf(peak,fabsf(v));
    }
    assert(fabsf(v)<speed_limit[j]);
    assert(peak<speed_limit[j]*DM_GRAVITY_SPEED_TRIP_RATIO);
  }
  /* Joint-limit brake: lowest Kd (0.1), the arm thrown toward each limit at the speed
   * limit (hand torque for the first 0.2 s, then released). The dissipative
   * limit brake must stop it before the position-trip boundary, in FREE. */
  for(unsigned j=0;j<7;++j) for(unsigned upper=0;upper<2;++upper) {
    setup(DM_ETH_CONTROL_GRAVITY_FREE);
    const float direction=upper ? 1.f : -1.f;
    const float limit=upper ? ArmGravity_Upper()[j] : ArmGravity_Lower()[j];
    float q=.5f*(ArmGravity_Lower()[j]+ArmGravity_Upper()[j]);
    float v=direction*speed_limit[j];
    float reached=q;
    for(unsigned t=0;t<4000;++t) {
      dm_motors[j].position=dm_motor_runtime.zero_offset[j]+DM_MOTOR_DIRECTION(j)*q;
      speed(j,v); step(); model(baseline);
      const AppGravityStatus s=status();
      assert(s.mode==DM_ETH_CONTROL_GRAVITY_FREE && !s.recovering);
      assert((eth_udp_command.target_torque[j]-baseline[j])*v<=1e-5f); /* never pushes along the motion */
      const float hand=t<200U ? direction*.02f*torque_limit[j] : 0.f;
      const float net=eth_udp_command.target_torque[j]-baseline[j]+hand;
      v+=net/(.002f*torque_limit[j])/DM_CONTROL_HZ;
      q+=v/DM_CONTROL_HZ;
      reached=upper ? fmaxf(reached,q) : fminf(reached,q);
    }
    assert(upper ? reached<limit+.05236f : reached>limit-.05236f); /* before the 3 deg trip boundary */
  }
  /* Away from the limits and below 50% of the speed limit nothing is added. */
  setup(DM_ETH_CONTROL_GRAVITY_FREE);
  for(unsigned j=0;j<7;++j) speed(j,.4f*speed_limit[j]);
  step(); model(baseline);
  { const AppGravityStatus s=status(); assert(!s.speed_limited_mask);
    for(unsigned j=0;j<7;++j) assert(fabsf(s.torque[j]-baseline[j])<1e-5f); }
  puts("gravity governor: no friction assist, FREE/LOCK stays active at 101% of 3.14 rad/s, no added propulsive power, bounded brake slew, release/reversal, STOP bias removal; 14 inertial-plant cases decelerate below limit; 14 throws stop before the joint limit: OK");
}
