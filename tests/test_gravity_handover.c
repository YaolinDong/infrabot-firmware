/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Actual mode/gravity/safety adapters, with a synthetic stationary load.
 * Assert motor-side MIT torque (Kp error + Kd error + feedforward), not just
 * a gain or the model output: those alone missed the resumption drop. */
#define main app_motion_tests_main
#include "test_app_motion.c"
#undef main
#include "app_control_mode.h"
#include "app_gravity.h"
#include "arm_gravity.h"

uint8_t AppFdcan_BusFault(void) { return 0; }
uint8_t AppTasks_Enabling(void) { return 0; }
static const float kp[7]={300,300,150,150,40,40,30};
static const float kd[7]={2.5f,2.5f,2.5f,2.5f,.8f,.8f,.8f};
static const float pose[7]={.12f,.6f,-.25f,.8f,.3f,-.2f,.1f};
static float max_torque_error;
static unsigned handovers;
static float joint_q(unsigned i) {
  return (dm_motors[i].position-dm_motor_runtime.zero_offset[i])*DM_MOTOR_DIRECTION(i);
}
static float output(unsigned i) {
  const float error=(dm_motor_runtime.command_position[i]-dm_motors[i].position)*DM_MOTOR_DIRECTION(i);
  return eth_udp_command.target_kp[i]*error+
         eth_udp_command.target_kd[i]*((dm_motor_runtime.stopped[i] ? 0.f : eth_udp_command.target_velocity[i])-
                                  dm_motors[i].velocity*DM_MOTOR_DIRECTION(i))+
         (dm_motor_runtime.stopped[i] ? 0.f : eth_udp_command.target_torque[i]);
}
static void step(void) {
  ++tick;
  for(unsigned i=0;i<7;++i) dm_motors[i].feedback_tick=tick;
  AppGravity_Tick(); AppSafety_Tick();
}
static void setup(float feedforward_fraction) {
  dm_motor_runtime.zero_valid=1;
  dm_motor_runtime.control_enabled=0; AppGravity_Tick(); /* end preceding test session */
  ready(DM_ETH_MODE_MIT); eth_udp_command.control_mode=DM_ETH_CONTROL_IDLE;
  float zero[7]={0}, load[7];
  assert(ArmGravity_Evaluate(pose,zero,0,load,NULL));
  for(unsigned i=0;i<7;++i) {
    dm_motor_runtime.zero_offset[i]=.03f*(i+1);
    dm_motors[i].position=dm_motor_runtime.zero_offset[i]+DM_MOTOR_DIRECTION(i)*pose[i];
    dm_motors[i].velocity=0; dm_motors[i].torque=load[i]*DM_MOTOR_DIRECTION(i);
    dm_motor_runtime.command_position[i]=dm_motors[i].position+
                          DM_MOTOR_DIRECTION(i)*(1-feedforward_fraction)*load[i]/kp[i];
    eth_udp_command.target_kp[i]=kp[i]; eth_udp_command.target_kd[i]=kd[i];
    eth_udp_command.target_torque[i]=feedforward_fraction*load[i];
    eth_udp_command.target_velocity[i]=0;
    dm_motor_runtime.stopped[i]=0;
  }
  eth_udp_command.target_torque[7]=.3f; eth_udp_command.target_velocity[7]=.2f;
}
static void check_handover(uint8_t mode) {
  ++handovers;
  float before[7], q[7], dq[7], gravity[7], initial_command[7];
  for(unsigned i=0;i<7;++i) {
    before[i]=output(i); q[i]=joint_q(i); dq[i]=0;
    initial_command[i]=dm_motor_runtime.command_position[i];
  }
  assert(ArmGravity_Evaluate(q,dq,0,gravity,NULL));
  const float gripper_command=dm_motor_runtime.command_position[7];
  eth_udp_command.queue_head=3; eth_udp_command.queue_tail=1;
  eth_udp_command.queue_count=2; eth_udp_command.sequence_valid=1;
  assert(AppControlMode_Switch(mode)==0);
  assert(!eth_udp_command.queue_count && !eth_udp_command.sequence_valid && !eth_udp_command.active);
  /* The mode switch itself must not erase a STOP's retained load support. */
  for(unsigned i=0;i<7;++i) {
    assert(fabsf(output(i)-before[i])<5e-5f);
    assert(fabsf(dm_motor_runtime.command_position[i]-initial_command[i])<1e-6f);
  }
  for(unsigned n=1;n<=1100;++n) {
    step();
    const float s=fminf(n/(1000.f*DM_GRAVITY_RAMP_S),1.f), a=s*s*(3-2*s);
    for(unsigned i=0;i<7;++i) {
      const float expected=(1-a)*before[i]+a*gravity[i];
      const float error=fabsf(output(i)-expected);
      max_torque_error=fmaxf(max_torque_error,error);
      assert(error<1e-4f);
      assert(isfinite(eth_udp_command.target_torque[i]));
    }
    assert(dm_motor_runtime.control_enabled && eth_udp_command.control_mode==mode);
  }
  assert(dm_motor_runtime.command_position[7]==gripper_command);
  assert(eth_udp_command.target_torque[7]==.3f && eth_udp_command.target_velocity[7]==.2f);
  for(unsigned i=0;i<7;++i) {
    assert(eth_udp_command.target_kp[i]==(mode==DM_ETH_CONTROL_GRAVITY_LOCK ? DM_GRAVITY_LOCK_KP : 0.f));
    assert(fabsf(eth_udp_command.target_torque[i]-gravity[i])<1e-5f);
  }
}
#ifndef GRAVITY_HANDOVER_ENTRY
#define GRAVITY_HANDOVER_ENTRY main
#endif
int GRAVITY_HANDOVER_ENTRY(void) {
  const float shares[]={1.f,0.f,.4f}; /* STOP hold, initial PD hold, mixed hold */
  for(unsigned mode=DM_ETH_CONTROL_GRAVITY_FREE;mode<=DM_ETH_CONTROL_GRAVITY_LOCK;++mode)
    for(unsigned k=0;k<sizeof(shares)/sizeof(shares[0]);++k) {
      setup(shares[k]); check_handover((uint8_t)mode);
      for(unsigned repeat=0;repeat<3;++repeat) {
        AppSafety_Stop(INFRABOT_STOP_REQUESTED,INFRABOT_NO_JOINT,0);
        for(unsigned n=0;n<600;++n) step();
        assert(!AppGravity_Owned() && dm_motor_runtime.control_enabled);
        /* A small sag after STOP leaves part of the load on the hold PD. */
        dm_motors[1].position+=DM_MOTOR_DIRECTION(1)*.001f;
        check_handover((uint8_t)mode);
      }
    }
  /* Even an IDLE MIT hold with a velocity bias blends as a complete PD
   * controller. A v4 stopped drive must not revive masked FF/velocity. */
  for(unsigned mode=DM_ETH_CONTROL_GRAVITY_FREE;mode<=DM_ETH_CONTROL_GRAVITY_LOCK;++mode) {
    setup(.4f);
    for(unsigned i=0;i<7;++i) {
      eth_udp_command.target_velocity[i]=(i%2) ? .02f : -.02f;
      eth_udp_command.target_torque[i]-=kd[i]*eth_udp_command.target_velocity[i];
    }
    check_handover((uint8_t)mode);
    setup(0.f);
    for(unsigned i=0;i<7;++i) {
      dm_motor_runtime.stopped[i]=1; eth_udp_command.target_torque[i]=1.f; eth_udp_command.target_velocity[i]=.02f;
    }
    check_handover((uint8_t)mode);
  }
  setup(1.f);
  const float command=dm_motor_runtime.command_position[1];
  eth_udp_command.target_torque[1]=NAN;
  assert(AppControlMode_Switch(DM_ETH_CONTROL_GRAVITY_FREE)==INFRABOT_REPLY_INVALID);
  assert(eth_udp_command.control_mode==DM_ETH_CONTROL_IDLE && dm_motor_runtime.command_position[1]==command && !AppGravity_Owned());
  printf("gravity handover: %u first-entry/re-entry ramps, PD + FF support, directions/zero offsets, masked outputs, J8 unchanged; max torque error %.8f Nm\n",handovers,max_torque_error);
  return 0;
}
