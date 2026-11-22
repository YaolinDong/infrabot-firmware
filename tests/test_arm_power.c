/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Arm power lost with the MCU on (app_arm_power.c), with a mocked bus: the
 * arm is powered or not; unpowered, nothing ACKs and the bus faults. */
#include "app_arm_power.h"
#include "app_damiao.h"
#include "app_enable.h"
#include "app_fdcan.h"
#include "app_state.h"
#include "dm_motor.h"
#include <assert.h>
#include <stdio.h>

MockMotor dm_motors[8];

FDCAN_HandleTypeDef hfdcan2;
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }

static uint8_t powered[8], bus_fault, enabling;
static unsigned resets, disables, cancels, requests;
uint8_t AppFdcan_BusFault(void) { return bus_fault; }
void AppFdcan_StartForDmMotor(uint8_t use_fd) { (void)use_fd; resets++; bus_fault = 0; }
uint8_t AppEnable_InProgress(void) { return enabling; }
void AppEnable_Disable(uint32_t mask)
{
  disables++;
  for (unsigned i = 0; i < 8; ++i)
    if (mask & (1UL << i)) dm_motor_runtime.enabled[i] = 0;
  dm_motor_runtime.control_enabled = 0;
  enabling = 0;
}
void AppDamiao_Cancel(void) { cancels++; }
HAL_StatusTypeDef DM_Motor_RequestFeedback(FDCAN_HandleTypeDef *hfdcan, const DM_Motor_t *motor)
{
  (void)hfdcan;
  const unsigned i = (unsigned)(motor - dm_motors);
  requests++;
  if (powered[i]) { dm_motors[i].feedback_count++; dm_motors[i].feedback_tick = tick; }
  else bus_fault = 1; /* no ACK: error passive within a few retransmissions */
  return HAL_OK;
}

/* The enabled arm's cyclic frames: answered when powered, else bus fault;
 * ACKed frames bring the error counter back down. */
static void step(unsigned ms)
{
  for (unsigned n = 0; n < ms; ++n) {
    ++tick;
    if (dm_motor_runtime.control_enabled) {
      uint8_t acked = 1;
      for (unsigned i = 0; i < 7; ++i) {
        if (powered[i]) { dm_motors[i].feedback_count++; dm_motors[i].feedback_tick = tick; }
        else acked = 0;
      }
      bus_fault = !acked;
    }
    AppArmPower_Watch();
  }
}

static void power(uint8_t on) { for (unsigned i = 0; i < 8; ++i) powered[i] = on; }

int main(void)
{
  uint8_t n = 99;
  tick = 1000;
  power(1);
  dm_motor_runtime.profile_ready = 1;
  dm_motor_runtime.control_enabled = 1;
  for (unsigned i = 0; i < 7; ++i) dm_motor_runtime.enabled[i] = 1;
  step(2000);
  assert(!AppArmPower_Waiting(&n) && disables == 0 && resets == 0);

  /* One brief glitch (bus fault, feedback 150 ms old) is not power loss. */
  bus_fault = 1;
  powered[0] = 0;
  step(150);
  assert(!AppArmPower_Waiting(NULL) && disables == 0);
  powered[0] = 1;
  bus_fault = 0;
  step(10);

  /* Enabled arm switched off: marked disabled once, after 200 ms of silence. */
  power(0);
  step(DM_ARM_POWER_LOST_MS);
  assert(!AppArmPower_Waiting(NULL) && disables == 0);
  step(2);
  assert(AppArmPower_Waiting(&n) && n == 0);
  assert(disables == 1 && cancels == 1 && !dm_motor_runtime.control_enabled && !dm_motor_runtime.enabled[0]);
  assert(!dm_motor_runtime.profile_ready);

  /* Waiting: one CAN reset and seven feedback requests per second, never more. */
  const unsigned r0 = resets, q0 = requests;
  step(5000);
  assert(resets - r0 >= 4 && resets - r0 <= 6 && requests - q0 == 7 * (resets - r0));
  assert(AppArmPower_Waiting(&n) && n == 0 && disables == 1);

  /* Maintenance owns the bus: no probe meanwhile. */
  eth_maintenance.waiting = 1;
  const unsigned r1 = resets;
  step(3000);
  assert(resets == r1);
  eth_maintenance.waiting = 0;

  /* Arm partly up: still waiting, the count is reported. */
  for (unsigned i = 0; i < 4; ++i) powered[i] = 1;
  step(2100);
  assert(AppArmPower_Waiting(&n) && n == 4);

  /* All seven answer two probes in a row: back, CAN reset once more, the
   * profile stays cleared (ENABLE re-applies it), nothing gets enabled. */
  power(1);
  step(1100);
  assert(AppArmPower_Waiting(NULL));
  const unsigned r2 = resets;
  step(1100);
  assert(!AppArmPower_Waiting(&n) && n == 7);
  assert(resets >= r2 + 2 && !bus_fault);
  assert(!dm_motor_runtime.profile_ready && !dm_motor_runtime.control_enabled && disables == 1);
  const unsigned r3 = resets;
  step(5000);
  assert(resets == r3); /* no probing once back */

  /* Disabled arm switched off: nothing to disable; the next frame (an
   * ENABLE probe here) faults the bus and the watch starts. */
  power(0);
  step(1000);
  assert(!AppArmPower_Waiting(NULL));
  (void)DM_Motor_RequestFeedback(&hfdcan2, &dm_motors[0]);
  step(1);
  assert(AppArmPower_Waiting(NULL) && disables == 1);

  /* ENABLE went through between two probes (arm back): waiting ends, the
   * running arm's CAN is never reset. */
  power(1);
  dm_motor_runtime.control_enabled = 1;
  const unsigned r4 = resets;
  step(3000);
  assert(!AppArmPower_Waiting(NULL) && resets == r4 && disables == 1);

  /* ... and losing power again while enabled is caught again. */
  power(0);
  step(DM_ARM_POWER_LOST_MS + 2);
  assert(AppArmPower_Waiting(NULL) && disables == 2 && !dm_motor_runtime.control_enabled);

  /* An enable sequence running into an unpowered arm is aborted too. */
  power(1);
  step(2200);
  assert(!AppArmPower_Waiting(NULL));
  power(0);
  enabling = 1;
  bus_fault = 1;
  step(DM_ARM_POWER_LOST_MS + 2);
  assert(AppArmPower_Waiting(NULL) && disables == 3 && !enabling);

  printf("arm power loss: detected after %u ms, drives marked disabled, CAN reset and J1-J7 "
         "re-probed every 1 s, back after two full probes, never enables\n", (unsigned)DM_ARM_POWER_LOST_MS);
  return 0;
}
