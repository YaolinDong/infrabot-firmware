/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Exercise the actual MCU adapter with synthetic feedback; no HAL/network. */
#include "app_motion.h"
#include "app_safety.h"
#include "app_state.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
MockMotor dm_motors[8];

static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
uint64_t AppEthernet_GetTimeUs(void) { return (uint64_t)tick * 1000ULL; }

static InfrabotSafetyStatus safety(void)
{
  InfrabotSafetyStatus s;
  AppSafety_Report(&s);
  return s;
}

static void ready(uint8_t mode)
{
  AppSafety_Clear();
  dm_motor_runtime.control_enabled = 1;
  dm_motor_runtime.profile_ready = 1;
  eth_udp_command.control_mode = DM_ETH_CONTROL_NRT_JOINT;
  eth_udp_command.control_request = 0;
  for (unsigned i = 0; i < 8; ++i) {
    dm_motor_runtime.enabled[i] = 1;
    eth_udp_command.motor_mode[i] = i == 7 ? 4 : mode;
    const float physical_position = (i == 1U || i == 3U) ? -0.1f : 0.2f;
    dm_motors[i] = (MockMotor){.position = physical_position, .velocity = 0,
      .position_max = 12.5f, .velocity_max = 10, .torque_max = 10,
      .feedback_count = 1, .feedback_tick = tick, .state = 1};
    dm_motor_runtime.zero_offset[i] = 0.1f;
  }
  dm_motor_runtime.command_position[7] = 0.3f; /* Gripper must not become an eighth planned joint. */
}

static AppMotionRequest request(uint64_t session)
{
  AppMotionRequest r = {.magic = 0x444d4f56, .version = 1, .op = ARM_MOVEJ,
    .session = session, .motion_id = 1};
  r.command.joint.timeout_ms = 10000; r.command.joint.dwell_ms = 100;
  r.command.joint.position_tolerance = 0.01f; r.command.joint.velocity_tolerance = 0.05f;
  for (unsigned i = 0; i < 7; ++i) {
    r.command.joint.target[i] = 0.3f;
    r.command.joint.velocity_max[i] = 0.1f; r.command.joint.acceleration_max[i] = 0.2f;
    r.command.joint.kp[i] = 30; r.command.joint.kd[i] = 0.8f;
  }
  return r;
}

static AppMotionReply exchange(const AppMotionRequest *r, uint32_t source)
{
  AppMotionReply reply;
  uint32_t ip; uint16_t port;
  assert(AppMotion_Publish(r, source, 5009));
  AppMotion_Tick(0);
  assert(AppMotion_TakeReply(&reply, &ip, &port));
  assert(ip == source && port == 5009);
  return reply;
}

int main(void)
{
  AppMotionRequest r = request(1);
  ready(1);
  eth_udp_command.control_mode = DM_ETH_CONTROL_RT_JOINT;
  assert(exchange(&r, 123).result == ARM_NOT_READY && !AppMotion_Owned());
  eth_udp_command.control_mode = DM_ETH_CONTROL_IDLE;
  assert(exchange(&r, 123).result == ARM_NOT_READY && !AppMotion_Owned());
  ready(1);
  /* All-or-nothing: one missing/faulted/wrong-mode joint prevents start. */
  dm_motor_runtime.enabled[3] = 0;
  assert(exchange(&r, 123).result == ARM_NOT_READY && !AppMotion_Owned());
  ready(1); eth_udp_command.motor_mode[3] = 2;
  assert(exchange(&r, 123).result == ARM_NOT_READY);
  ready(1); dm_motors[3].state = 8;
  assert(exchange(&r, 123).result == ARM_MOTOR_FAULT);
  ready(1);
  eth_udp_command.queue_count = 8; eth_udp_command.sequence_valid = 1;
  assert(exchange(&r, 123).result == ARM_OK && AppMotion_Owned());
  assert(!AppMotion_ReleaseHold()); /* Running motion cannot switch ownership. */
  assert(!eth_udp_command.queue_count && !eth_udp_command.sequence_valid);
  assert(fabsf(dm_motor_runtime.command_position[0] - 0.2f) < 1e-6f); /* No initial position jump. */
  assert(dm_motor_runtime.command_position[7] == 0.3f);
  assert(exchange(&r, 999).result == ARM_BAD_SESSION); /* Endpoint also owns session. */
  for (tick = 1; tick < 50; ++tick) {
    for (unsigned i = 0; i < 7; ++i) dm_motors[i].feedback_tick = tick;
    if (tick % 10 == 0) assert(exchange(&r, 123).result == ARM_OK);
    else AppMotion_Tick(0);
  }
  assert(dm_motor_runtime.command_position[0] > 0.2f && dm_motor_runtime.command_position[7] == 0.3f);
  /* A late heartbeat cannot conceal watchdog expiry before acceptance. */
  tick = 150;
  for (unsigned i = 0; i < 7; ++i) dm_motors[i].feedback_tick = tick;
  AppMotionReply failed = exchange(&r, 123);
  assert(failed.state == ARM_FAILED && failed.reason == ARM_COMM_TIMEOUT);
  /* A lost keepalive is a controlled stop, not a disable: the arm stays
   * enabled in IDLE and brakes; the failure stays latched for the PC. */
  assert(dm_motor_runtime.control_enabled && eth_udp_command.control_request == 0);
  assert(eth_udp_command.control_mode == DM_ETH_CONTROL_IDLE && AppSafety_Braking());
  assert(safety().stop_reason == INFRABOT_STOP_MOVE_LOST);
  assert(AppMotion_Owned() && AppMotion_Failed());
  assert(exchange(&r, 123).state == ARM_FAILED); /* DMOS keeps reporting it */
  assert(safety().stop_reason == INFRABOT_STOP_MOVE_LOST); /* one stop per failure */
  /* The PC cancels the failed move (its session cleanup): released, and the
   * stop it already made is not replaced by a requested one. */
  AppMotionRequest cancel_failed = r;
  cancel_failed.op = ARM_CANCEL_MOVE;
  assert(exchange(&cancel_failed, 123).state == ARM_CANCELLED && !AppMotion_Owned());
  assert(safety().stop_reason == INFRABOT_STOP_MOVE_LOST);
  /* A mode switch acknowledges a failure too... */
  ready(1); r = request(9);
  assert(exchange(&r, 123).result == ARM_OK);
  tick += 150;
  for (unsigned i = 0; i < 7; ++i) dm_motors[i].feedback_tick = tick;
  AppMotion_Tick(0);
  assert(AppMotion_Failed());
  assert(AppMotion_ReleaseHold() && !AppMotion_Owned());
  /* ...and so does a disable without a cancel (a register read found a
   * drive in another control mode): the next enable is not refused as owned
   * by the old move. */
  ready(1); r = request(8);
  assert(exchange(&r, 123).result == ARM_OK);
  tick += 150;
  for (unsigned i = 0; i < 7; ++i) dm_motors[i].feedback_tick = tick;
  AppMotion_Tick(0);
  assert(AppMotion_Failed());
  dm_motor_runtime.control_enabled = 0;
  AppMotion_Tick(0);
  assert(!AppMotion_Owned());
  ready(2); r = request(2); /* POS_VEL adapter uses same profile, passes speed limit. */
  assert(exchange(&r, 123).result == ARM_OK);
  assert(eth_udp_command.velocity_limit[0] == 0.1f);
  AppMotion_TxFailed();
  AppMotion_Tick(0); /* the next tick turns it into a controlled stop */
  assert(dm_motor_runtime.control_enabled && eth_udp_command.control_request == 0 && AppSafety_Braking());
  assert(safety().stop_reason == INFRABOT_STOP_MOVE_FAILED && safety().stop_detail == ARM_TX_ERROR);
  AppMotion_RequestCancel(); AppMotion_Tick(0);
  /* DISABLE still removes power at once. */
  assert(!dm_motor_runtime.control_enabled && eth_udp_command.control_request == DM_ETH_CMD_DISABLE);
  /* The PC cancels a running move: a controlled stop, no disable. */
  ready(1); r = request(6);
  assert(exchange(&r, 123).result == ARM_OK && AppMotion_Owned());
  AppMotionRequest cancel = r;
  cancel.op = ARM_CANCEL_MOVE;
  AppMotionReply cancelled = exchange(&cancel, 123);
  assert(cancelled.result == ARM_OK && cancelled.state == ARM_CANCELLED && !AppMotion_Owned());
  assert(dm_motor_runtime.control_enabled && eth_udp_command.control_request == 0 && AppSafety_Braking());
  assert(safety().stop_reason == INFRABOT_STOP_REQUESTED);
  /* A supervisor stop during a move fails it with a matching DMOS reason. */
  ready(1); r = request(7);
  assert(exchange(&r, 123).result == ARM_OK);
  AppSafety_Stop(INFRABOT_STOP_FEEDBACK_LOST, 2, 0);
  assert(AppMotion_Failed() && exchange(&r, 123).reason == ARM_FEEDBACK_STALE);
  assert(safety().stop_reason == INFRABOT_STOP_FEEDBACK_LOST && safety().stop_joint == 2);
  AppMotion_RequestRelease(); AppMotion_Tick(0);
  /* Cancel/disable wins against a start already in the mailbox. */
  ready(1); r = request(3);
  assert(AppMotion_Publish(&r, 123, 5009));
  AppMotion_RequestCancel(); AppMotion_Tick(0);
  uint32_t ip; uint16_t port;
  assert(AppMotion_TakeReply(&failed, &ip, &port));
  assert(failed.result == ARM_NOT_READY && !AppMotion_Owned());
  /* The arm disabled alone (DISABLE_ARM) ends a move without the full
   * DISABLE of a cancel: an enabled gripper keeps holding.  A full cancel
   * pending at the same time stays one. */
  ready(1); r = request(4);
  assert(exchange(&r, 123).result == ARM_OK && AppMotion_Owned());
  AppMotion_RequestRelease(); AppMotion_Tick(0);
  assert(!AppMotion_Owned() && dm_motor_runtime.control_enabled && eth_udp_command.control_request == 0);
  ready(1); r = request(5);
  assert(exchange(&r, 123).result == ARM_OK && AppMotion_Owned());
  AppMotion_RequestCancel(); AppMotion_RequestRelease(); AppMotion_Tick(0);
  assert(!AppMotion_Owned() && eth_udp_command.control_request == DM_ETH_CMD_DISABLE);
  puts("app_motion: owner, mailbox, all-axis readiness, output mapping, faults as controlled stops, cancel and arm release tests passed");
  return 0;
}
