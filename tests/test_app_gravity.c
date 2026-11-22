/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Use real safety+mode adapters and mocked CAN state; no physical IO. */
#define main app_motion_tests_main
#include "test_app_motion.c"
#undef main
#include "app_control_mode.h"
#include "app_gravity.h"
#include "arm_gravity.h"

static uint8_t bus_fault;
uint8_t AppFdcan_BusFault(void) { return bus_fault; }
uint8_t AppTasks_Enabling(void) { return 0; }
static void gravity_tick(void) {
  ++tick;
  /* Keep all drive feedback fresh without any PC heartbeat. */
  for(unsigned i=0;i<7;++i) dm_motors[i].feedback_tick=HAL_GetTick();
  AppGravity_Tick();
  AppSafety_Tick();
}
int main(void) {
  dm_motor_runtime.zero_valid=1;
  ready(DM_ETH_MODE_MIT);
  dm_motor_runtime.profile_ready=1;
  eth_udp_command.control_mode=DM_ETH_CONTROL_IDLE;
  for(unsigned i=0;i<7;++i) {
    dm_motors[i].position=dm_motor_runtime.zero_offset[i]+DM_MOTOR_DIRECTION(i)*.1f;
    dm_motor_runtime.command_position[i]=dm_motors[i].position;
    eth_udp_command.target_kp[i]=50; eth_udp_command.target_kd[i]=2;
  }
  assert(AppControlMode_Switch(DM_ETH_CONTROL_GRAVITY_FREE)==0);
  for(unsigned i=0;i<1100;++i) gravity_tick();
  assert(eth_udp_command.control_mode==DM_ETH_CONTROL_GRAVITY_FREE);
  for(unsigned i=0;i<7;++i) assert(eth_udp_command.target_kp[i]==0);
  assert(dm_motor_runtime.enabled[7]); /* J8 unchanged */
  AppSafety_Stop(INFRABOT_STOP_REQUESTED,INFRABOT_NO_JOINT,0);
  assert(AppGravity_Recovering());
  AppSafety_Stop(INFRABOT_STOP_TORQUE_LIMIT,0,0);
  for(unsigned i=0;i<7;++i) assert(eth_udp_command.target_torque[i]==0);
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_JOINT)==INFRABOT_REPLY_BUSY);
  for(unsigned i=0;i<550;++i) gravity_tick();
  assert(!AppGravity_Owned());
  for(unsigned i=0;i<7;++i) assert(fabsf(eth_udp_command.target_kp[i]-50)<1e-5);
  assert(dm_motor_runtime.control_enabled && eth_udp_command.control_mode==DM_ETH_CONTROL_IDLE);
  assert(AppControlMode_Switch(DM_ETH_CONTROL_GRAVITY_LOCK)==0);
  for(unsigned i=0;i<1100;++i) gravity_tick();
  for(unsigned i=0;i<7;++i) assert(eth_udp_command.target_kp[i]==DM_GRAVITY_LOCK_KP);
  /* Live tuning is bounded and slewed; stopped hold gains remain intact. */
  assert(!AppGravity_SetParameter(APP_GRAVITY_DAMPING,NAN));
  assert(!AppGravity_SetParameter(APP_GRAVITY_STIFFNESS,31.f));
  assert(!AppGravity_SetParameter(APP_GRAVITY_PARAMETERS,1.f));
  assert(AppGravity_SetParameter(APP_GRAVITY_DAMPING,2.f));
  assert(AppGravity_SetParameter(APP_GRAVITY_STIFFNESS,20.f));
  gravity_tick();
  assert(fabsf(eth_udp_command.target_kp[0]-DM_GRAVITY_LOCK_KP)<=.02001f);
  assert(fabsf(eth_udp_command.target_kd[0]-DM_GRAVITY_KD)<=.00201f);
  for(unsigned i=0;i<1100;++i) gravity_tick();
  AppGravityParameters params; AppGravity_GetParameters(&params);
  for(unsigned i=0;i<APP_GRAVITY_PARAMETERS;++i) assert(params.applied[i]==params.value[i]);
  for(unsigned i=0;i<7;++i) assert(eth_udp_command.target_kp[i]==20 && eth_udp_command.target_kd[i]==2);
  bus_fault=1; gravity_tick();
  assert(eth_udp_command.control_mode==DM_ETH_CONTROL_IDLE && AppGravity_Recovering());
  for(unsigned i=0;i<7;++i) assert(eth_udp_command.target_torque[i]==0);
  for(unsigned i=0;i<550;++i) gravity_tick();
  for(unsigned i=0;i<7;++i) assert(eth_udp_command.target_kp[i]==50);
  assert(AppControlMode_Switch(DM_ETH_CONTROL_GRAVITY_FREE)==INFRABOT_REPLY_NOT_READY);
  bus_fault=0;dm_motors[0].temperature_mos=100;
  assert(AppControlMode_Switch(DM_ETH_CONTROL_GRAVITY_FREE)==INFRABOT_REPLY_NOT_READY);
  assert(ArmMotion_StreamPositionTarget(0,.5f,0,.001f)==.5f);
  assert(ArmMotion_StreamPositionTarget(1,-.5f,0,.001f)==-.5f);
  assert(fabsf(ArmMotion_StreamPositionTarget(0,.5f,.5f,.001f)-.0005f)<1e-8);
  puts("gravity free/lock, no PC stream, STOP gain recovery, faults, no RT slew: OK");
}
