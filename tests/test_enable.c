/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Staged enable/disable of the drive groups (arm J1-J7, gripper J8) and the
 * gripper command, with mocked CAN drives; no HAL, network or RTOS. */
#include "app_enable.h"
#include "app_gripper.h"
#include "app_state.h"
#include "dm_motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

MockMotor dm_motors[8];

FDCAN_HandleTypeDef hfdcan2;
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
uint16_t AppDamiao_ModeId(uint8_t mode) { return (uint16_t)((mode - 1U) * 0x100U); }

/* The drives: a connected one answers every frame with feedback; Enable and
 * Disable switch its power stage. */
static uint8_t present[8], powered[8], fail_enable[8];
static unsigned frames[8], enables[8], disables[8];
static float last_hold[8];

static unsigned drive(const DM_Motor_t *motor) { return (unsigned)(motor - dm_motors); }
static HAL_StatusTypeDef frame(const DM_Motor_t *motor)
{
  const unsigned i = drive(motor);
  frames[i]++;
  if (present[i]) {
    dm_motors[i].feedback_count++;
    dm_motors[i].feedback_tick = tick;
  }
  return HAL_OK;
}
HAL_StatusTypeDef DM_Motor_Enable(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor, uint16_t mode_id)
{
  (void)hfdcan; (void)mode_id;
  const unsigned i = drive(motor);
  if (fail_enable[i]) return HAL_ERROR;
  enables[i]++;
  if (present[i]) powered[i] = 1;
  return frame(motor);
}
HAL_StatusTypeDef DM_Motor_Disable(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor, uint16_t mode_id)
{
  (void)hfdcan; (void)mode_id;
  disables[drive(motor)]++;
  powered[drive(motor)] = 0;
  return frame(motor);
}
HAL_StatusTypeDef DM_Motor_ClearError(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor, uint16_t mode_id)
{
  (void)hfdcan; (void)mode_id;
  return frame(motor);
}
HAL_StatusTypeDef DM_Motor_RequestFeedback(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor)
{
  (void)hfdcan;
  return frame(motor);
}
HAL_StatusTypeDef DM_Motor_MIT(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor, float position,
                               float velocity, float kp, float kd, float torque)
{
  (void)hfdcan; (void)velocity; (void)kp; (void)kd; (void)torque;
  last_hold[drive(motor)] = position;
  return frame(motor);
}
HAL_StatusTypeDef DM_Motor_Position(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor, float position,
                                    float velocity)
{
  (void)hfdcan; (void)velocity;
  last_hold[drive(motor)] = position;
  return frame(motor);
}
HAL_StatusTypeDef DM_Motor_Speed(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor, float velocity)
{
  (void)hfdcan; (void)velocity;
  return frame(motor);
}
HAL_StatusTypeDef DM_Motor_PositionCurrent(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor,
                                           float position, float velocity, float current)
{
  (void)hfdcan; (void)velocity; (void)current;
  last_hold[drive(motor)] = position;
  return frame(motor);
}

/* All drives connected, disabled, last heard from long ago; arm MIT, gripper
 * force-position. */
static void reset(void)
{
  tick = 1000;
  for (unsigned i = 0; i < 8; ++i) {
    dm_motors[i] = (MockMotor){.position = 0.1f * (float)(i + 1), .position_max = 12.5f,
                               .velocity_max = 10, .torque_max = 10, .feedback_count = 1};
    present[i] = 1;
    powered[i] = fail_enable[i] = 0;
    frames[i] = enables[i] = disables[i] = 0;
    last_hold[i] = NAN;
    dm_motor_runtime.enabled[i] = 0;
    eth_udp_command.motor_mode[i] = i == 7 ? DM_ETH_MODE_FORCE_POS : DM_ETH_MODE_MIT;
    dm_motor_runtime.command_position[i] = dm_motor_runtime.zero_offset[i] = 0.0f;
    eth_udp_command.velocity_limit[i] = eth_udp_command.torque_limit_ratio[i] = 1.0f;
  }
  dm_motor_runtime.control_enabled = 0;
  eth_udp_command.control_mode = DM_ETH_CONTROL_IDLE;
  eth_udp_command.active = 0;
}

/* Control ticks until the sequence ends, as the 1 kHz task runs it. */
static void run(void)
{
  for (unsigned n = 0; AppEnable_InProgress(); ++n) {
    assert(n < 500);
    ++tick;
    if (AppEnable_Step()) AppEnable_HoldFrames();
  }
}

int main(void)
{
  /* The arm alone: J8 gets no frame at all. */
  reset();
  assert(AppEnable_Start(DM_ARM_MOTOR_MASK));
  assert(!AppEnable_Start(DM_GRIPPER_MOTOR_MASK)); /* one sequence at a time */
  run();
  for (unsigned i = 0; i < 7; ++i) {
    assert(dm_motor_runtime.enabled[i] && powered[i] && enables[i] == 1);
    assert(dm_motor_runtime.command_position[i] == dm_motors[i].position); /* rebased: a true hold */
  }
  assert(!dm_motor_runtime.enabled[7] && frames[7] == 0);
  assert(dm_motor_runtime.control_enabled && eth_udp_command.control_mode == DM_ETH_CONTROL_IDLE);
  assert(!AppEnable_Start(DM_ARM_MOTOR_MASK)); /* enabled already: not a new epoch */

  /* The gripper while the arm holds a command 20 mrad above its sagging
   * angle: the arm keeps that command throughout and is not re-enabled. */
  for (unsigned i = 0; i < 7; ++i) dm_motor_runtime.command_position[i] = dm_motors[i].position + 0.02f;
  assert(AppEnable_Start(DM_GRIPPER_MOTOR_MASK));
  run();
  assert(dm_motor_runtime.enabled[7] && powered[7] && dm_motor_runtime.control_enabled);
  assert(dm_motor_runtime.command_position[7] == dm_motors[7].position);
  for (unsigned i = 0; i < 7; ++i) {
    assert(dm_motor_runtime.enabled[i] && enables[i] == 1 && disables[i] == 0);
    assert(fabsf(dm_motor_runtime.command_position[i] - (dm_motors[i].position + 0.02f)) < 1e-6f);
    assert(fabsf(last_hold[i] - dm_motor_runtime.command_position[i]) < 1e-6f);
  }

  /* Disabling the gripper leaves the arm running in its mode... */
  eth_udp_command.control_mode = DM_ETH_CONTROL_RT_JOINT;
  eth_udp_command.active = 1;
  AppEnable_Disable(DM_GRIPPER_MOTOR_MASK);
  assert(!dm_motor_runtime.enabled[7] && !powered[7] && disables[7] == 1);
  assert(eth_udp_command.control_mode == DM_ETH_CONTROL_RT_JOINT && dm_motor_runtime.control_enabled && eth_udp_command.active);
  for (unsigned i = 0; i < 7; ++i) assert(dm_motor_runtime.enabled[i] && powered[i] && disables[i] == 0);
  /* ...disabling the arm returns to IDLE while the gripper holds. */
  eth_udp_command.control_mode = DM_ETH_CONTROL_IDLE;
  assert(AppEnable_Start(DM_GRIPPER_MOTOR_MASK));
  run();
  eth_udp_command.control_mode = DM_ETH_CONTROL_RT_CARTESIAN;
  eth_udp_command.cartesian_target_valid = 1;
  AppEnable_Disable(DM_ARM_MOTOR_MASK);
  assert(eth_udp_command.control_mode == DM_ETH_CONTROL_IDLE && !eth_udp_command.cartesian_target_valid);
  assert(dm_motor_runtime.control_enabled && eth_udp_command.active && dm_motor_runtime.enabled[7] && powered[7]);
  for (unsigned i = 0; i < 7; ++i) assert(!dm_motor_runtime.enabled[i] && !powered[i]);
  AppEnable_Disable(DM_GRIPPER_MOTOR_MASK);
  assert(!dm_motor_runtime.control_enabled && !eth_udp_command.active);

  /* No gripper connected: enabling both fails before any drive is enabled
   * (the probe), the arm alone comes up, and a gripper enable then fails
   * without touching it. */
  reset();
  present[7] = 0;
  assert(AppEnable_Start(DM_ALL_MOTOR_MASK));
  run();
  for (unsigned i = 0; i < 8; ++i) assert(!dm_motor_runtime.enabled[i] && !powered[i] && enables[i] == 0);
  assert(!dm_motor_runtime.control_enabled);
  assert(AppEnable_Start(DM_ARM_MOTOR_MASK));
  run();
  unsigned arm_disables[7];
  for (unsigned i = 0; i < 7; ++i) arm_disables[i] = disables[i];
  assert(AppEnable_Start(DM_GRIPPER_MOTOR_MASK));
  run();
  assert(!dm_motor_runtime.enabled[7] && dm_motor_runtime.control_enabled);
  for (unsigned i = 0; i < 7; ++i) assert(dm_motor_runtime.enabled[i] && powered[i] && disables[i] == arm_disables[i]);

  /* A drive refusing its enable frame rolls back the ones already up. */
  reset();
  fail_enable[2] = 1;
  assert(AppEnable_Start(DM_ARM_MOTOR_MASK));
  run();
  assert(enables[0] == 1 && enables[1] == 1);
  for (unsigned i = 0; i < 8; ++i) assert(!dm_motor_runtime.enabled[i] && !powered[i]);
  assert(!dm_motor_runtime.control_enabled);

  /* Any disable during a sequence aborts it, with the drives it brought up. */
  reset();
  assert(AppEnable_Start(DM_ALL_MOTOR_MASK));
  while (!dm_motor_runtime.enabled[0]) {
    ++tick;
    if (AppEnable_Step()) AppEnable_HoldFrames();
  }
  AppEnable_Disable(DM_GRIPPER_MOTOR_MASK);
  assert(!AppEnable_InProgress() && !dm_motor_runtime.control_enabled);
  for (unsigned i = 0; i < 8; ++i) assert(!dm_motor_runtime.enabled[i] && !powered[i]);

  /* Position-velocity arm: hold frames in that mode, gripper force-position. */
  reset();
  assert(AppEnable_Start(DM_ARM_MOTOR_MASK));
  while (!dm_motor_runtime.enabled[0]) {
    ++tick;
    if (AppEnable_Step()) AppEnable_HoldFrames();
  }
  AppEnable_CancelAndHold();
  assert(!AppEnable_InProgress() && dm_motor_runtime.control_enabled && powered[0]);
  for (unsigned n=0;n<100;++n) { ++tick; assert(!AppEnable_Step()); }
  for (unsigned i=0;i<8;++i) {
    assert(disables[i]==0);
    if (i) assert(!powered[i] && enables[i]==0);
  }
  assert(dm_motor_runtime.command_position[0]==dm_motors[0].position);
  /* Completing an explicitly restarted sequence keeps J1 powered. */
  assert(AppEnable_Start(DM_ARM_MOTOR_MASK)); run();
  assert(enables[0]==1);
  for (unsigned i=0;i<7;++i) assert(powered[i]);

  /* Position-velocity arm: hold frames in that mode, gripper force-position. */
  reset();
  for (unsigned i = 0; i < 7; ++i) eth_udp_command.motor_mode[i] = DM_ETH_MODE_POS_VEL;
  assert(AppEnable_Start(DM_ALL_MOTOR_MASK));
  run();
  for (unsigned i = 0; i < 8; ++i) assert(dm_motor_runtime.enabled[i] && powered[i] && last_hold[i] == dm_motors[i].position);

  /* The gripper command: position, speed limit and force (current limit)
   * in one go; rejected while J8 is disabled or out of range, and never
   * touching the arm. */
  reset();
  assert(AppEnable_Start(DM_ARM_MOTOR_MASK));
  run();
  float arm[7];
  for (unsigned i = 0; i < 7; ++i) arm[i] = dm_motor_runtime.command_position[i];
  const float before = dm_motor_runtime.command_position[7];
  assert(AppGripper_Command(0.6f, 2.0f, 0.4f) == 7); /* not enabled */
  assert(dm_motor_runtime.command_position[7] == before);
  assert(AppEnable_Start(DM_GRIPPER_MOTOR_MASK));
  run();
  dm_motor_runtime.zero_offset[7] = 0.05f;
  dm_motor_runtime.stopped[7] = 1;
  assert(AppGripper_Command(0.6f, 2.0f, 0.4f) == 0);
  /* Joint frame to encoder: J8 is mounted reversed (DM_GRIPPER_DIRECTION). */
  assert(fabsf(dm_motor_runtime.command_position[7] - (0.05f + DM_GRIPPER_DIRECTION * 0.6f)) < 1e-6f);
  assert(eth_udp_command.target_position[7] == 0.6f && !dm_motor_runtime.stopped[7]);
  assert(eth_udp_command.velocity_limit[7] == 2.0f && eth_udp_command.torque_limit_ratio[7] == 0.4f);
  /* The whole travel, 0..5.94 rad, and the force to 0.01. */
  assert(AppGripper_Command(5.94f, 2.0f, 0.01f) == 0);
  assert(fabsf(dm_motor_runtime.command_position[7] - (0.05f + DM_GRIPPER_DIRECTION * 5.94f)) < 1e-5f);
  assert(eth_udp_command.torque_limit_ratio[7] == 0.01f);
  assert(AppGripper_Command(0.0f, 2.0f, 0.4f) == 0 && eth_udp_command.target_position[7] == 0.0f);
  assert(AppGripper_Command(0.6f, 2.0f, 0.4f) == 0);
  const float applied = dm_motor_runtime.command_position[7];
  const float bad[][3] = {{5.95f, 2.0f, 0.4f}, {-0.1f, 2.0f, 0.4f}, {0.5f, 0.0f, 0.4f},
                          {0.5f, 11.0f, 0.4f}, {0.5f, 2.0f, 1.5f}, {0.5f, 2.0f, -0.1f},
                          {NAN, 2.0f, 0.4f}, {0.5f, NAN, 0.4f}, {0.5f, 2.0f, NAN}};
  for (unsigned k = 0; k < sizeof bad / sizeof bad[0]; ++k)
    assert(AppGripper_Command(bad[k][0], bad[k][1], bad[k][2]) == 3);
  assert(dm_motor_runtime.command_position[7] == applied && eth_udp_command.velocity_limit[7] == 2.0f &&
         eth_udp_command.torque_limit_ratio[7] == 0.4f);
  eth_udp_command.motor_mode[7] = DM_ETH_MODE_MIT; /* not force-position */
  assert(AppGripper_Command(0.5f, 2.0f, 0.4f) == 3);
  for (unsigned i = 0; i < 7; ++i) assert(dm_motor_runtime.command_position[i] == arm[i]);

  puts("enable: arm alone, gripper while the arm holds, group disable, absent gripper, rollback, abort, gripper command passed");
  return 0;
}
