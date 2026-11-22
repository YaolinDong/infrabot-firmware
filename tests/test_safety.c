/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Controlled stop (app_safety.c) with mocked drives: braking, holding with
 * no PC for as long as it takes (never disabling), a PC that comes back, and
 * the drive supervisors; no HAL, network or RTOS. */
#include "app_safety.h"
#include "app_motion.h"
#include "app_state.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

MockMotor dm_motors[8];

static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }

/* The move a stop ends: how often and with which DMOS reason. */
static int aborts;
static uint8_t abort_reason;
void AppMotion_Abort(uint8_t reason) { aborts++; abort_reason = reason; }

#define DT (1.0f / (float)DM_CONTROL_HZ)
static uint8_t pc = 1;     /* the PC's heartbeat arrives every tick */
static uint8_t silent;     /* drives whose feedback stopped (bit i) */

static float joint(uint8_t i)
{
  return (dm_motor_runtime.command_position[i] - dm_motor_runtime.zero_offset[i]) * DM_MOTOR_DIRECTION(i);
}
static void command(uint8_t i, float q)
{
  dm_motor_runtime.command_position[i] = dm_motor_runtime.zero_offset[i] + DM_MOTOR_DIRECTION(i) * q;
}
static float measured(uint8_t i)
{
  return (dm_motors[i].position - dm_motor_runtime.zero_offset[i]) * DM_MOTOR_DIRECTION(i);
}
static void measure(uint8_t i, float q)
{
  dm_motors[i].position = dm_motor_runtime.zero_offset[i] + DM_MOTOR_DIRECTION(i) * q;
}
static InfrabotSafetyStatus report(void)
{
  InfrabotSafetyStatus s;
  AppSafety_Report(&s);
  return s;
}

/* One control tick: feedback and heartbeat arrive, the producers have
 * written this tick's command, then the module runs. */
static void begin_tick(void)
{
  ++tick;
  for (unsigned i = 0; i < 8; ++i)
    if (!(silent & (1U << i))) dm_motors[i].feedback_tick = tick;
  if (pc) eth_udp_command.last_tick = tick;
}
static void step(void)
{
  begin_tick();
  AppSafety_Tick();
}
/* The PC goes silent long enough for the watchdog to expire. */
static void lose_pc(void)
{
  pc = 0;
  for (unsigned k = 0; k <= DM_ETH_COMMAND_TIMEOUT_MS; ++k) step();
}
/* Producers move joint i at v[i] for n ticks (the commands the windows see). */
static void move(const float v[8], unsigned n)
{
  for (unsigned k = 0; k < n; ++k) {
    begin_tick();
    for (unsigned i = 0; i < 7; ++i) {
      command((uint8_t)i, joint((uint8_t)i) + v[i] * DT);
      measure((uint8_t)i, joint((uint8_t)i));
    }
    AppSafety_Tick();
  }
}
/* Ticks until the brake is done (asserting every tick that all moving joints
 * keep the same fraction of their start speed: they stop together). */
static unsigned brake(const float v0[8])
{
  unsigned n = 0;
  while (AppSafety_Braking()) {
    step();
    ++n;
    assert(n < 2000);
    float fraction = -1.0f;
    for (unsigned i = 0; i < 7; ++i) {
      if (v0[i] == 0.0f) continue;
      const float f = eth_udp_command.target_velocity[i] / v0[i];
      if (fraction < 0.0f) fraction = f;
      assert(fabsf(f - fraction) < 1e-4f);
    }
  }
  return n;
}

/* Eight drives enabled, healthy and fresh, streaming RT joint; the arm still
 * at q = 0.1 (i + 1), each with a torque feed-forward. */
static void reset(void)
{
  tick = 1000;
  pc = 1;
  silent = 0;
  AppSafety_Clear();
  for (unsigned i = 0; i < 8; ++i) {
    dm_motors[i] = (MockMotor){.position_max = 12.5f, .velocity_max = 10.0f, .torque_max = 28.0f,
                               .feedback_count = 1, .feedback_tick = tick, .state = 1};
    dm_motor_runtime.zero_offset[i] = 0.05f * (float)(i + 1);
    dm_motor_runtime.enabled[i] = 1;
    dm_motor_runtime.stopped[i] = dm_motor_runtime.override[i] = 0;
    eth_udp_command.motor_mode[i] = i == 7 ? DM_ETH_MODE_FORCE_POS : DM_ETH_MODE_MIT;
    command((uint8_t)i, 0.1f * (float)(i + 1));
    measure((uint8_t)i, 0.1f * (float)(i + 1));
    eth_udp_command.target_velocity[i] = 0.0f;
    eth_udp_command.target_torque[i] = 0.5f * (float)(i + 1);
  }
  dm_motor_runtime.control_enabled = 1;
  eth_udp_command.control_mode = DM_ETH_CONTROL_RT_JOINT;
  eth_udp_command.control_request = 0;
  eth_udp_command.active = 1;
  eth_udp_command.sequence_valid = 1;
  eth_udp_command.queue_count = 3;
  eth_udp_command.cartesian_target_valid = 1;
  eth_udp_command.last_tick = tick;
  for (unsigned k = 0; k < 20; ++k) step(); /* the velocity windows settle */
  aborts = 0;
  abort_reason = 0xFF;
}

/* No DISABLE requested, every arm drive still enabled. */
static int still_enabled(void)
{
  for (unsigned i = 0; i < 7; ++i)
    if (!dm_motor_runtime.enabled[i]) return 0;
  return dm_motor_runtime.control_enabled && eth_udp_command.control_request == 0;
}

static void test_synchronized_brake_and_hold(void)
{
  reset();
  lose_pc(); /* the PC is gone: its stream stopped */
  const float v0[8] = {1.0f, -0.5f, 0.25f, 0, 0, 0, 0, 0};
  move(v0, 20);
  float q0[8], gripper = dm_motor_runtime.command_position[7];
  for (unsigned i = 0; i < 7; ++i) q0[i] = joint((uint8_t)i);
  begin_tick();
  AppSafety_Stop(INFRABOT_STOP_STREAM_LOST, INFRABOT_NO_JOINT, 0);
  /* Out of the control mode at once; the move (if any) ends. */
  assert(eth_udp_command.control_mode == DM_ETH_CONTROL_IDLE && !eth_udp_command.active);
  assert(!eth_udp_command.sequence_valid && !eth_udp_command.queue_count &&
         !eth_udp_command.cartesian_target_valid);
  assert(aborts == 1 && abort_reason == ARM_COMM_TIMEOUT);
  assert(AppSafety_Braking() && AppSafety_Stopping());
  AppSafety_Tick(); /* the first braking step, same tick */
  /* J1 brakes from 1 rad/s at 8 rad/s^2: 0.125 s, the others with it. */
  const unsigned n = 1 + brake(v0);
  assert(n >= 124 && n <= 126);
  for (unsigned i = 0; i < 7; ++i) {
    assert(fabsf(joint((uint8_t)i) - (q0[i] + v0[i] * 0.125f * 0.5f)) < 2e-4f);
    assert(eth_udp_command.target_velocity[i] == 0.0f);
    assert(fabsf(eth_udp_command.target_torque[i] - 0.5f * (float)(i + 1)) < 1e-6f); /* kept */
  }
  assert(dm_motor_runtime.command_position[7] == gripper); /* the gripper is not braked */
  /* No PC (the GUI closed, the cable pulled): hold, however long -- ten
   * minutes here -- enabled, the command where the brake left it. */
  InfrabotSafetyStatus s = report();
  assert(s.stop_phase == INFRABOT_STOP_PHASE_HOLDING && s.stop_reason == INFRABOT_STOP_STREAM_LOST);
  assert(s.stop_joint == INFRABOT_NO_JOINT);
  assert(AppSafety_Stopping() && !AppSafety_Braking());
  float held[8];
  for(unsigned i=0;i<8;i++)held[i]=dm_motor_runtime.command_position[i];
  for (unsigned k = 0; k < 10U * 60U * 1000U; ++k) {
    step();
    if (k % 1000U == 0U) assert(still_enabled() && report().stop_phase == INFRABOT_STOP_PHASE_HOLDING);
  }
  assert(still_enabled() && AppSafety_Stopping());
  float final_hold[8];
  for(unsigned i=0;i<8;i++)final_hold[i]=dm_motor_runtime.command_position[i];
  assert(memcmp(held, final_hold, sizeof(held)) == 0);
  /* The PC comes back: an IDLE hold; the reason stays until acknowledged. */
  pc = 1;
  step();
  s = report();
  assert(s.stop_phase == INFRABOT_STOP_PHASE_NONE && s.stop_reason == INFRABOT_STOP_STREAM_LOST);
  assert(still_enabled() && eth_udp_command.active && !AppSafety_Stopping());
  AppSafety_Clear();
  assert(report().stop_reason == INFRABOT_STOP_NONE && report().stop_phase == INFRABOT_STOP_PHASE_NONE);
}

static void test_pc_back(void)
{
  /* Holding without a PC: its heartbeat takes the arm over as an IDLE hold. */
  reset();
  lose_pc();
  begin_tick();
  AppSafety_Stop(INFRABOT_STOP_STREAM_LOST, INFRABOT_NO_JOINT, 0);
  AppSafety_Tick(); /* still: no braking */
  assert(report().stop_phase == INFRABOT_STOP_PHASE_HOLDING);
  for (unsigned k = 0; k < 1000; ++k) step();
  pc = 1;
  step();
  assert(report().stop_phase == INFRABOT_STOP_PHASE_NONE && eth_udp_command.active);
  assert(report().stop_reason == INFRABOT_STOP_STREAM_LOST); /* latched for the PC */
  assert(!AppSafety_Stopping());
  /* A PC that stays (a requested stop): straight to the IDLE hold. */
  reset();
  const float v0[8] = {1.0f, 0, 0, 0, 0, 0, 0, 0};
  move(v0, 20);
  AppSafety_RequestStop(INFRABOT_STOP_REQUESTED);
  assert(!AppSafety_Braking()); /* only the control task starts it */
  step();
  assert(AppSafety_Braking() && abort_reason == ARM_CANCEL);
  (void)brake(v0);
  assert(report().stop_phase == INFRABOT_STOP_PHASE_NONE && eth_udp_command.active);
  assert(report().stop_reason == INFRABOT_STOP_REQUESTED);
}

static void test_stop_mode(void)
{
  /* The stop records the mode it interrupted (RT Cartesian here); a second
   * stop before the PC acknowledged the first happens in IDLE and keeps it:
   * the mode to resume. */
  reset();
  eth_udp_command.control_mode = DM_ETH_CONTROL_RT_CARTESIAN;
  begin_tick();
  AppSafety_Stop(INFRABOT_STOP_NOT_FOLLOWING, 3, 0);
  AppSafety_Tick();
  InfrabotSafetyStatus s = report();
  assert(s.stop_mode == DM_ETH_CONTROL_RT_CARTESIAN && eth_udp_command.control_mode == DM_ETH_CONTROL_IDLE);
  while (AppSafety_Braking()) step();
  assert(report().stop_phase == INFRABOT_STOP_PHASE_NONE);
  begin_tick();
  AppSafety_Stop(INFRABOT_STOP_TORQUE_LIMIT, 1, 0);
  AppSafety_Tick();
  s = report();
  assert(s.stop_reason == INFRABOT_STOP_TORQUE_LIMIT && s.stop_joint == 1 &&
         s.stop_mode == DM_ETH_CONTROL_RT_CARTESIAN);
  /* Acknowledged: no mode to resume any more; the next stop records anew. */
  AppSafety_Clear();
  assert(report().stop_mode == DM_ETH_CONTROL_IDLE && report().stop_reason == INFRABOT_STOP_NONE);
  eth_udp_command.control_mode = DM_ETH_CONTROL_NRT_JOINT;
  begin_tick();
  AppSafety_Stop(INFRABOT_STOP_MOVE_LOST, INFRABOT_NO_JOINT, 0);
  assert(report().stop_mode == DM_ETH_CONTROL_NRT_JOINT);
}

static void test_repeated_stop_is_one(void)
{
  /* The PC sends STOP twice: the brake runs on unchanged. */
  reset();
  const float v0[8] = {1.0f, 0, 0, 0, 0, 0, 0, 0};
  move(v0, 20);
  const float q0 = joint(0);
  AppSafety_RequestStop(INFRABOT_STOP_REQUESTED);
  unsigned n = 0;
  while (n == 0 || AppSafety_Braking()) {
    if (n == 10) AppSafety_RequestStop(INFRABOT_STOP_REQUESTED);
    step();
    ++n;
  }
  assert(n >= 124 && n <= 126 && aborts == 1);
  assert(fabsf(joint(0) - (q0 + 0.0625f)) < 2e-4f);
}

static void test_brake_time_cap_and_limits(void)
{
  /* 8 rad/s needs 1 s at 8 rad/s^2: capped to 0.5 s. */
  reset();
  lose_pc();
  float v0[8] = {0, 0, 0, 0, 8.0f, 0, 0, 0};
  move(v0, 20);
  float q0 = joint(4);
  begin_tick();
  AppSafety_Stop(INFRABOT_STOP_STREAM_LOST, INFRABOT_NO_JOINT, 0);
  AppSafety_Tick();
  unsigned n = 1 + brake(v0);
  assert(n >= 499 && n <= 501);
  assert(fabsf(joint(4) - (q0 + 8.0f * 0.5f * 0.5f)) < 2e-3f);
  /* J2 (lower limit 0) braking towards it stops on it, never past. */
  reset();
  lose_pc();
  command(1, 0.02f);
  measure(1, 0.02f);
  for (unsigned k = 0; k < 15; ++k) step(); /* the window restarts at the jump */
  float v1[8] = {0, -0.5f, 0, 0, 0, 0, 0, 0};
  move(v1, 20); /* down to 0.01 */
  begin_tick();
  AppSafety_Stop(INFRABOT_STOP_STREAM_LOST, INFRABOT_NO_JOINT, 0);
  while (AppSafety_Braking()) {
    AppSafety_Tick();
    assert(joint(1) >= -1e-6f);
    begin_tick();
  }
  assert(fabsf(joint(1)) < 1e-6f && eth_udp_command.target_velocity[1] == 0.0f);
  /* A command jump (a mode switch re-latching it) is no speed to brake from. */
  reset();
  command(0, 1.5f);
  step();
  AppSafety_Stop(INFRABOT_STOP_REQUESTED, INFRABOT_NO_JOINT, 0);
  step();
  assert(!AppSafety_Braking() && fabsf(joint(0) - 1.5f) < 1e-6f);
}

static void test_blocked_joint_and_v4_stop(void)
{
  /* J3 left 0.3 rad behind its command: held where it is, not pushed on;
   * J1 brakes; J5 is held by a v4 STOP and not moved. */
  reset();
  lose_pc();
  const float v0[8] = {1.0f, 0, 0.25f, 0, 0, 0, 0, 0};
  move(v0, 20);
  measure(2, joint(2) - 0.3f);
  dm_motor_runtime.stopped[4] = 1;
  const float held = dm_motor_runtime.command_position[4];
  begin_tick();
  AppSafety_Stop(INFRABOT_STOP_NOT_FOLLOWING, 2, 0);
  assert(fabsf(joint(2) - measured(2)) < 1e-6f && eth_udp_command.target_velocity[2] == 0.0f);
  const float v_brake[8] = {1.0f, 0, 0, 0, 0, 0, 0, 0};
  AppSafety_Tick();
  (void)brake(v_brake);
  assert(fabsf(joint(2) - measured(2)) < 1e-6f);
  assert(dm_motor_runtime.command_position[4] == held);
  InfrabotSafetyStatus s = report();
  assert(s.stop_reason == INFRABOT_STOP_NOT_FOLLOWING && s.stop_joint == 2);
  assert(abort_reason == ARM_MOTOR_FAULT);
}

static void test_torque_supervisor(void)
{
  reset();
  dm_motors[3].torque = 25.0f; /* J4 at its limit */
  for (unsigned k = 0; k < 99; ++k) {
    step();
    assert(AppSafety_TorqueLimitMask() == 0x08 && !AppSafety_Braking());
  }
  measure(3, 0.37f); /* pushed away from its command */
  step();
  InfrabotSafetyStatus s = report();
  assert(s.stop_reason == INFRABOT_STOP_TORQUE_LIMIT && s.stop_joint == 3);
  assert(fabsf(joint(3) - 0.37f) < 1e-6f); /* yields: held where it was pushed */
  dm_motors[3].torque = -10.0f;
  step();
  assert(AppSafety_TorqueLimitMask() == 0);
  /* An interruption restarts the count; the gripper and disabled joints are
   * not supervised. */
  reset();
  dm_motors[7].torque = 100.0f;
  dm_motor_runtime.enabled[0] = 0;
  dm_motors[0].torque = 100.0f;
  dm_motors[1].torque = -40.0f;
  for (unsigned k = 0; k < 50; ++k) step();
  dm_motors[1].torque = -35.9f;
  step();
  dm_motors[1].torque = -36.0f;
  for (unsigned k = 0; k < 99; ++k) step();
  assert(!AppSafety_Braking() && report().stop_reason == INFRABOT_STOP_NONE);
  assert(AppSafety_TorqueLimitMask() == 0x02);
  step();
  assert(report().stop_reason == INFRABOT_STOP_TORQUE_LIMIT && report().stop_joint == 1);
}

static void test_drive_fault_and_feedback(void)
{
  /* A drive fault stops on its onset, once. */
  reset();
  dm_motors[5].state = INFRABOT_DRIVE_OVERCURRENT;
  step();
  InfrabotSafetyStatus s = report();
  assert(s.stop_reason == INFRABOT_STOP_DRIVE_FAULT && s.stop_joint == 5 &&
         s.stop_detail == INFRABOT_DRIVE_OVERCURRENT && abort_reason == ARM_MOTOR_FAULT);
  AppSafety_Clear();
  for (unsigned k = 0; k < 10; ++k) step();
  assert(report().stop_reason == INFRABOT_STOP_NONE);
  dm_motors[5].state = 1;
  step();
  dm_motors[5].state = INFRABOT_DRIVE_OVERLOAD;
  step();
  assert(report().stop_reason == INFRABOT_STOP_DRIVE_FAULT && report().stop_detail == INFRABOT_DRIVE_OVERLOAD);
  /* Feedback silent for more than DM_STOP_FEEDBACK_LOST_MS. */
  reset();
  silent = 0x01;
  for (unsigned k = 0; k < DM_STOP_FEEDBACK_LOST_MS; ++k) step();
  assert(report().stop_reason == INFRABOT_STOP_NONE);
  step();
  s = report();
  assert(s.stop_reason == INFRABOT_STOP_FEEDBACK_LOST && s.stop_joint == 0);
  assert(abort_reason == ARM_FEEDBACK_STALE);
}

static void test_disable_wins(void)
{
  /* A disable ends the stop; the reason stays latched. */
  reset();
  lose_pc();
  begin_tick();
  AppSafety_Stop(INFRABOT_STOP_STREAM_LOST, INFRABOT_NO_JOINT, 0);
  AppSafety_Tick();
  assert(AppSafety_Stopping());
  dm_motor_runtime.control_enabled = 0;
  step();
  assert(!AppSafety_Stopping() && report().stop_phase == INFRABOT_STOP_PHASE_NONE);
  assert(report().stop_reason == INFRABOT_STOP_STREAM_LOST);
  /* A pending DISABLE: no stop. */
  reset();
  eth_udp_command.control_request = DM_ETH_CMD_DISABLE;
  AppSafety_Stop(INFRABOT_STOP_REQUESTED, INFRABOT_NO_JOINT, 0);
  assert(!AppSafety_Braking() && eth_udp_command.control_mode == DM_ETH_CONTROL_RT_JOINT && aborts == 0);
  /* Nothing enabled: nothing to stop. */
  reset();
  dm_motor_runtime.control_enabled = 0;
  AppSafety_Stop(INFRABOT_STOP_REQUESTED, INFRABOT_NO_JOINT, 0);
  assert(!AppSafety_Braking());
}

static void test_can_fault_holds(void)
{
  /* A CAN controller fault (RT Cartesian, app_tasks.c) is a controlled stop
   * like any other: braked and held with no PC, never disabled. */
  reset();
  lose_pc();
  const float v0[8] = {0.5f, 0, 0, 0, 0, 0, 0, 0};
  move(v0, 20);
  begin_tick();
  AppSafety_Stop(INFRABOT_STOP_DRIVE_FAULT, INFRABOT_NO_JOINT, 0);
  AppSafety_Tick();
  assert(AppSafety_Braking() && eth_udp_command.control_mode == DM_ETH_CONTROL_IDLE);
  (void)brake(v0);
  for (unsigned k = 0; k < 60000U; ++k) step();
  const InfrabotSafetyStatus s = report();
  assert(s.stop_reason == INFRABOT_STOP_DRIVE_FAULT && s.stop_joint == INFRABOT_NO_JOINT);
  assert(s.stop_phase == INFRABOT_STOP_PHASE_HOLDING && still_enabled());
}

int main(void)
{
  test_synchronized_brake_and_hold();
  test_pc_back();
  test_stop_mode();
  test_repeated_stop_is_one();
  test_brake_time_cap_and_limits();
  test_blocked_joint_and_v4_stop();
  test_torque_supervisor();
  test_drive_fault_and_feedback();
  test_disable_wins();
  test_can_fault_holds();
  puts("safety: synchronized brake, 10 min hold without a PC (never disabled), PC back, stop mode, repeated stop, "
       "brake cap, joint limit, command jump, blocked joint, v4 stop, torque supervisor, drive fault, "
       "feedback lost, CAN fault holds, disable passed");
  return 0;
}
