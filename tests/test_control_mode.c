/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Actual mode adapter with mocked drives; never sends network/CAN traffic. */
#define main app_motion_tests_main
#include "test_app_motion.c"
#undef main
#include "app_control_mode.h"
static uint8_t enabling;
uint8_t AppTasks_Enabling(void) { return enabling; }

int main(void)
{
  for(unsigned mode=0;mode<=10;mode++)assert(AppControlMode_IsLocalCompliant(mode)==(mode>=5));
  ready(DM_ETH_MODE_MIT);
  eth_udp_command.control_mode = DM_ETH_CONTROL_IDLE;
  dm_motor_runtime.profile_ready = 0;
  assert(AppControlMode_Profile() == DM_ETH_MODE_MIT);
  assert(!AppControlMode_ProfileReady());
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_JOINT) == 6);
  dm_motor_runtime.profile_ready = 1;
  enabling = 1;
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_JOINT) == 4);
  enabling = 0;
  dm_motor_runtime.enabled[3] = 0;
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_JOINT) == 7);
  for(unsigned mode=0;mode<=10;mode++)assert(AppControlMode_IsLocalCompliant(mode)==(mode>=5));
  ready(DM_ETH_MODE_MIT); eth_udp_command.control_mode = DM_ETH_CONTROL_IDLE;
  dm_motors[3].velocity = .1f;
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_JOINT) == 4);
  dm_motors[3].velocity = 0;
  eth_udp_command.target_position[0] = 10;
  eth_udp_command.target_velocity[0] = 3;
  eth_udp_command.target_torque[0] = 9;
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_JOINT) == 0);
  assert(fabsf(dm_motor_runtime.command_position[0] - .2f) < 1e-6f);
  assert(fabsf(eth_udp_command.target_position[0] - .1f) < 1e-6f);
  assert(eth_udp_command.target_velocity[0] == 0 && eth_udp_command.target_torque[0] == 0);
  assert((AppControlMode_Telemetry() & 0xf7) == 0xd1);
  assert(AppControlMode_Switch(DM_ETH_CONTROL_NRT_JOINT) == 6);
  assert(AppControlMode_Switch(DM_ETH_CONTROL_IDLE) == 0);
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_CARTESIAN) == 0);
  assert(eth_udp_command.control_mode == DM_ETH_CONTROL_RT_CARTESIAN);
  assert(AppControlMode_Switch(DM_ETH_CONTROL_IDLE) == 0);
  dm_motor_runtime.command_position[0] = .25f; /* Gravity sag below the command: kept, no drop. */
  dm_motor_runtime.command_position[1] = .5f;  /* Far from feedback (blocked joint): re-latched. */
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_JOINT) == 0);
  assert(fabsf(dm_motor_runtime.command_position[0] - .25f) < 1e-6f);
  assert(fabsf(eth_udp_command.target_position[0] - .15f) < 1e-6f);
  assert(fabsf(dm_motor_runtime.command_position[1] - dm_motors[1].position) < 1e-6f);
  assert(AppControlMode_Switch(DM_ETH_CONTROL_IDLE) == 0);
  assert(dm_motor_runtime.control_enabled); /* IDLE is hold, not disable. */
  assert(eth_udp_command.active); /* watched by the PC's heartbeat */
  assert(AppControlMode_Switch(DM_ETH_CONTROL_NRT_JOINT) == 0);
  AppMotionRequest r = request(10);
  assert(exchange(&r, 123).result == ARM_OK);
  assert(AppControlMode_Switch(DM_ETH_CONTROL_IDLE) == 4);
  for (tick = 1; tick < 4200; ++tick) {
    for (unsigned i = 0; i < 7; ++i) {
      dm_motors[i].feedback_tick = tick;
      dm_motors[i].position = dm_motor_runtime.command_position[i];
    }
    if (tick % 10 == 0) assert(exchange(&r, 123).result == ARM_OK);
    else AppMotion_Tick(0);
  }
  dm_motors[7].feedback_tick = tick;
  assert(AppControlMode_Switch(DM_ETH_CONTROL_IDLE) == 0);
  assert(dm_motor_runtime.control_enabled && !AppMotion_Owned());
  assert(exchange(&r, 123).result == ARM_BAD_SESSION); /* Old motion cannot resume. */
  assert(dm_motor_runtime.control_enabled && eth_udp_command.control_request == 0);
  tick += 100;
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_JOINT) == 7);
  assert(eth_udp_command.control_mode == DM_ETH_CONTROL_IDLE);
  /* The modes need the arm only: a gripper that is disabled, silent and
   * moving does not block the switch, and its command (squeezing, 0.3 rad
   * past its angle) is not re-latched, which would let go of the object. */
  for(unsigned mode=0;mode<=10;mode++)assert(AppControlMode_IsLocalCompliant(mode)==(mode>=5));
  ready(DM_ETH_MODE_MIT); eth_udp_command.control_mode = DM_ETH_CONTROL_IDLE;
  dm_motor_runtime.enabled[7] = 0;
  dm_motors[7].feedback_tick = 0; dm_motors[7].velocity = 1.0f; dm_motors[7].state = 0;
  dm_motor_runtime.command_position[7] = dm_motors[7].position + 0.3f;
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_JOINT) == 0);
  assert(fabsf(dm_motor_runtime.command_position[7] - (dm_motors[7].position + 0.3f)) < 1e-6f);
  assert(AppControlMode_Switch(DM_ETH_CONTROL_IDLE) == 0);
  dm_motor_runtime.enabled[7] = 1;
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_CARTESIAN) == 0);
  assert(fabsf(dm_motor_runtime.command_position[7] - (dm_motors[7].position + 0.3f)) < 1e-6f);
  assert(AppControlMode_Switch(DM_ETH_CONTROL_IDLE) == 0);
  dm_motor_runtime.enabled[6] = 0; /* but every arm joint */
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_JOINT) == 7);
  ready(DM_ETH_MODE_POS_VEL); eth_udp_command.control_mode = DM_ETH_CONTROL_IDLE;
  assert(AppControlMode_Profile() == DM_ETH_MODE_POS_VEL);
  eth_udp_command.motor_mode[7] = DM_ETH_MODE_MIT;
  assert(!AppControlMode_ProfileReady());
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_JOINT) == 6);
  /* A controlled stop leaves the control mode for IDLE and records it
   * (stop_mode): the PC resumes by entering it again.  While braking, an
   * IDLE retry does not acknowledge the stop; a joint still moving (a
   * position-velocity drive trailing its stopped command) refuses the
   * resume as busy. */
  for(unsigned mode=0;mode<=10;mode++)assert(AppControlMode_IsLocalCompliant(mode)==(mode>=5));
  ready(DM_ETH_MODE_MIT); eth_udp_command.control_mode = DM_ETH_CONTROL_RT_JOINT;
  AppSafety_Stop(INFRABOT_STOP_REQUESTED, INFRABOT_NO_JOINT, 0);
  InfrabotSafetyStatus s;
  AppSafety_Report(&s);
  assert(eth_udp_command.control_mode == DM_ETH_CONTROL_IDLE && s.stop_reason == INFRABOT_STOP_REQUESTED);
  assert(s.stop_mode == DM_ETH_CONTROL_RT_JOINT && s.stop_phase == INFRABOT_STOP_PHASE_BRAKING);
  assert(AppControlMode_Switch(DM_ETH_CONTROL_IDLE) == 0);
  AppSafety_Report(&s);
  assert(s.stop_reason == INFRABOT_STOP_REQUESTED && s.stop_mode == DM_ETH_CONTROL_RT_JOINT);
  /* The brake is done (the command was still: one tick), the PC is here. */
  for (unsigned i = 0; i < 8; ++i) dm_motors[i].feedback_tick = tick;
  eth_udp_command.last_tick = tick;
  AppSafety_Tick();
  AppSafety_Report(&s);
  assert(s.stop_phase == INFRABOT_STOP_PHASE_NONE && s.stop_reason == INFRABOT_STOP_REQUESTED);
  dm_motors[2].velocity = 0.2f;
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_JOINT) == 4);
  AppSafety_Report(&s);
  assert(s.stop_reason == INFRABOT_STOP_REQUESTED && s.stop_mode == DM_ETH_CONTROL_RT_JOINT);
  dm_motors[2].velocity = 0.0f;
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_JOINT) == 0);
  AppSafety_Report(&s);
  assert(eth_udp_command.control_mode == DM_ETH_CONTROL_RT_JOINT);
  assert(s.stop_reason == INFRABOT_STOP_NONE && s.stop_phase == INFRABOT_STOP_PHASE_NONE &&
         s.stop_mode == DM_ETH_CONTROL_IDLE);
  /* A stop in IDLE: nothing to re-enter; IDLE -> IDLE acknowledges it. */
  assert(AppControlMode_Switch(DM_ETH_CONTROL_IDLE) == 0);
  AppSafety_Stop(INFRABOT_STOP_TORQUE_LIMIT, 2, 0);
  for (unsigned i = 0; i < 8; ++i) dm_motors[i].feedback_tick = tick;
  eth_udp_command.last_tick = tick;
  AppSafety_Tick();
  AppSafety_Report(&s);
  assert(s.stop_reason == INFRABOT_STOP_TORQUE_LIMIT && s.stop_mode == DM_ETH_CONTROL_IDLE &&
         s.stop_phase == INFRABOT_STOP_PHASE_NONE);
  assert(AppControlMode_Switch(DM_ETH_CONTROL_IDLE) == 0);
  AppSafety_Report(&s);
  assert(s.stop_reason == INFRABOT_STOP_NONE && eth_udp_command.control_mode == DM_ETH_CONTROL_IDLE);
  /* A movej whose keepalive was lost: the arm stops and holds enabled, the
   * failure stays latched, and a mode switch releases it. */
  for(unsigned mode=0;mode<=10;mode++)assert(AppControlMode_IsLocalCompliant(mode)==(mode>=5));
  ready(DM_ETH_MODE_MIT);
  AppMotionRequest lost = request(20);
  assert(exchange(&lost, 123).result == ARM_OK && AppMotion_Owned());
  tick += 150;
  for (unsigned i = 0; i < 8; ++i) dm_motors[i].feedback_tick = tick;
  AppMotion_Tick(0);
  assert(AppMotion_Failed() && dm_motor_runtime.control_enabled && eth_udp_command.control_mode == DM_ETH_CONTROL_IDLE);
  AppSafety_Report(&s);
  assert(s.stop_reason == INFRABOT_STOP_MOVE_LOST);
  assert(AppControlMode_Switch(DM_ETH_CONTROL_RT_JOINT) == 0 && !AppMotion_Owned());
  puts("control modes: profile, enable, idle, handover, Cartesian modes, stale feedback, ownership, arm-only, controlled stop passed");
  return 0;
}
