/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Real LCD action coordinator, modes, gravity and stop state machines.
 * CAN setup/enable are simulated here and independently tested with drives. */
#define main app_motion_tests_main
#include "test_app_motion.c"
#undef main
#include "app_local_control.h"
#include "app_control_mode.h"
#include "app_gravity.h"
#include "app_enable.h"

static unsigned profile_starts, enable_starts, disables;
static uint8_t profile_result, enabling;
uint8_t AppFdcan_BusFault(void) { return 0; }
uint8_t AppTasks_Enabling(void) { return enabling; }
uint8_t AppEnable_InProgress(void) { return enabling; }
void AppEnable_CancelAndHold(void) { enabling=0; }
uint8_t AppEnable_Start(uint32_t mask) {
  assert(mask==DM_ARM_MOTOR_MASK); assert(dm_motor_runtime.profile_ready);
  ++enable_starts; enabling=1; return 1;
}
void AppEnable_Disable(uint32_t mask) {
  assert(mask==DM_ARM_MOTOR_MASK); ++disables;
  for(unsigned i=0;i<7;++i) dm_motor_runtime.enabled[i]=0;
  dm_motor_runtime.control_enabled=dm_motor_runtime.enabled[7]; eth_udp_command.control_mode=DM_ETH_CONTROL_IDLE;
}
uint8_t AppDamiao_StartLocalProfile(uint8_t mode) {
  assert(mode==DM_ETH_MODE_MIT); assert(!dm_motor_runtime.control_enabled);
  ++profile_starts; eth_maintenance.waiting=1; profile_result=4; return 0;
}
uint8_t AppDamiao_LocalProfileStatus(void) { return profile_result; }
void AppDamiao_Cancel(void) { eth_maintenance.waiting=0; profile_result=4; }
void AppDamiao_CancelMotors(uint32_t mask) { assert(mask==DM_ARM_MOTOR_MASK); }
static AppLocalControlStatus local_status(void) {
  AppLocalControlStatus s; AppLocalControl_Status(&s); return s;
}
static void action(uint8_t a) { AppLocalControl_Request(a); AppLocalControl_Process(); }
static void advance(unsigned ticks) {
  for(unsigned n=0;n<ticks;++n) {
    ++tick;
    for(unsigned i=0;i<8;++i) dm_motors[i].feedback_tick=tick;
    AppLocalControl_Process(); AppMotion_Tick(enabling); AppGravity_Tick(); AppSafety_Tick();
  }
}
int main(void) {
  dm_motor_runtime.zero_valid=1;
  ready(DM_ETH_MODE_MIT);
  eth_udp_command.control_mode=DM_ETH_CONTROL_IDLE;
  dm_motor_runtime.control_enabled=dm_motor_runtime.profile_ready=0;
  for(unsigned i=0;i<DM_MOTOR_COUNT;i++)dm_motor_runtime.enabled[i]=0;
  for(unsigned i=0;i<7;++i) {
    dm_motor_runtime.command_position[i]=dm_motors[i].position;
    eth_udp_command.target_kp[i]=50; eth_udp_command.target_kd[i]=2;
  }
  dm_motor_runtime.zero_valid=0;
  action(APP_LOCAL_ENABLE); assert(local_status().result==7 && profile_starts==0);
  dm_motor_runtime.zero_valid=1;
  action(APP_LOCAL_ENABLE); assert(profile_starts==1 && enable_starts==0);
  eth_udp_command.enable_request=DM_ARM_MOTOR_MASK;
  eth_udp_command.control_request=DM_ETH_CMD_ENABLE;
  action(APP_LOCAL_STOP); assert(!eth_maintenance.waiting);
  assert(!eth_udp_command.enable_request && !eth_udp_command.control_request);
  advance(10); assert(enable_starts==0); /* no enable after cancelled setup */
  action(APP_LOCAL_ENABLE); assert(profile_starts==2);
  profile_result=6; eth_maintenance.waiting=0;
  AppLocalControl_Process(); assert(local_status().result==6 && enable_starts==0);
  action(APP_LOCAL_ENABLE); assert(profile_starts==3);
  profile_result=0; eth_maintenance.waiting=0; dm_motor_runtime.profile_ready=1;
  AppLocalControl_Process(); assert(enabling && enable_starts==1);
  for(unsigned i=0;i<7;++i) dm_motor_runtime.enabled[i]=1;
  dm_motor_runtime.control_enabled=1; enabling=0;
  advance(1); assert(local_status().state==APP_LOCAL_ENABLED && !dm_motor_runtime.enabled[7]);
  assert(AppSafety_LocalHold());
  action(APP_LOCAL_FREE); assert(local_status().result==0);
  advance(1100); assert(eth_udp_command.control_mode==DM_ETH_CONTROL_GRAVITY_FREE);
  action(APP_LOCAL_STOP); assert(AppGravity_Recovering());
  action(APP_LOCAL_RESUME); assert(local_status().result==4); /* cannot cut recovery short */
  advance(600); assert(dm_motor_runtime.control_enabled && !AppGravity_Owned());
  assert(!AppSafety_Stopping()); /* no PC required for STOP to settle */
  float retained[7];
  for(unsigned i=0;i<7;++i) retained[i]=eth_udp_command.target_torque[i];
  action(APP_LOCAL_RESUME); assert(local_status().result==0);
  for(unsigned i=0;i<7;++i) assert(eth_udp_command.target_torque[i]==retained[i]);
  assert(eth_udp_command.control_mode==DM_ETH_CONTROL_GRAVITY_FREE);
  advance(1100);
  action(APP_LOCAL_STOP); advance(600);
  for(unsigned i=0;i<7;++i) retained[i]=eth_udp_command.target_torque[i];
  action(APP_LOCAL_FREE); assert(local_status().result==0);
  for(unsigned i=0;i<7;++i) assert(eth_udp_command.target_torque[i]==retained[i]);
  advance(1100);
  action(APP_LOCAL_STOP); advance(600);
  /* External control takeover restores the external watchdog policy. */
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_JOINT)==0);
  assert(!AppSafety_LocalHold());
  eth_udp_command.last_tick=tick-DM_ETH_COMMAND_TIMEOUT_MS-1U;
  AppSafety_Stop(INFRABOT_STOP_STREAM_LOST,INFRABOT_NO_JOINT,0);
  advance(600); assert(AppSafety_Stopping());
  dm_motors[2].feedback_tick=tick-60;
  action(APP_LOCAL_RESUME); assert(local_status().result==7);
  dm_motors[2].feedback_tick=tick;
  action(APP_LOCAL_RESUME); assert(local_status().result==0);
  assert(eth_udp_command.control_mode==DM_ETH_CONTROL_IDLE && !AppSafety_Stopping());
  AppSafety_Stop(INFRABOT_STOP_TORQUE_LIMIT,2,0); advance(600);
  action(APP_LOCAL_RESUME); assert(local_status().result==7); /* no silent fault reset */
  AppSafety_Clear();
  const uint8_t imp_modes[]={DM_ETH_CONTROL_JOINT_IMPEDANCE,DM_ETH_CONTROL_CARTESIAN_IMPEDANCE};
  const uint8_t imp_actions[]={APP_LOCAL_JOINT_IMP,APP_LOCAL_CART_IMP};
  for(unsigned k=0;k<2;k++) {
    AppLocalControl_Request(imp_actions[k]);
    assert(local_status().pending_actions&(1U<<imp_actions[k]));
    AppLocalControl_Process();assert(!local_status().result && !local_status().pending_actions);
    AppGravityStatus indicator;AppGravity_Status(&indicator);assert(indicator.entry_permille==0);
    advance(200);AppGravity_Status(&indicator);assert(indicator.entry_permille>=190 && indicator.entry_permille<=210);
    advance(900);AppGravity_Status(&indicator);assert(indicator.entry_permille==1000);
    assert(eth_udp_command.control_mode==imp_modes[k]);
    action(APP_LOCAL_STOP);advance(600);assert(!AppGravity_Owned() && dm_motor_runtime.control_enabled);
    action(APP_LOCAL_RESUME);assert(!local_status().result && eth_udp_command.control_mode==imp_modes[k]);
    advance(1100);
    AppLocalControl_Request(imp_actions[k]);action(APP_LOCAL_STOP);advance(600);
    assert(eth_udp_command.control_mode==DM_ETH_CONTROL_IDLE && !AppGravity_Owned());
  }
  /* Explicit disable leaves the independently controlled gripper alone,
   * clears gravity ownership and restores normal gains for the next enable. */
  AppSafety_Clear(); action(APP_LOCAL_FREE); advance(1100);
  dm_motor_runtime.enabled[7]=1;
  AppLocalControl_Request(APP_LOCAL_ENABLE);
  AppLocalControl_Request(APP_LOCAL_RESUME);
  AppLocalControl_Request(APP_LOCAL_DISABLE);
  AppLocalControl_Process(); advance(1);
  assert(disables==1 && dm_motor_runtime.enabled[7] && !AppGravity_Owned());
  assert(enable_starts==1);
  for(unsigned i=0;i<7;++i) assert(!dm_motor_runtime.enabled[i] && eth_udp_command.target_kp[i]==50);
  /* A network disable overtakes an LCD request, without a late auto-enable. */
  eth_udp_command.disable_request=DM_ARM_MOTOR_MASK;
  action(APP_LOCAL_ENABLE); assert(enable_starts==1);
  eth_udp_command.disable_request=0; advance(10); assert(enable_starts==1);
  puts("LCD control: setup/cancel/failure, arm-only enable, STOP/recovery/resume offline, PC takeover, fault gates, disable priority: OK");
}
