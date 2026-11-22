/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_runtime.h"
#include "eth_state.h"
#include "control_services.h"
#include "platform.h"
#include "app_arm_power.h"

#include "app_config.h"
#include "app_enable.h"
#include "app_state.h"
#include "platform_drive.h"

#include <stdio.h>

#define ARM_POWER_PROBE_MS 1000U  /* re-probe period while waiting */
#define ARM_POWER_REPLY_MS 50U    /* feedback window of a probe */

static uint8_t waiting;
static uint8_t responding;
static uint8_t probing;
static uint8_t full_probes;
static uint32_t probe_tick;
static uint32_t before[DM_ARM_MOTOR_COUNT];

uint8_t AppArmPower_Waiting(uint8_t *n)
{
  if (n != NULL) *n = responding;
  return waiting;
}

/* None of J1-J7 answered for DM_ARM_POWER_LOST_MS.  Signed age: a frame
 * received after now was read is fresh. */
static uint8_t arm_silent(uint32_t now)
{
  for (uint8_t i = 0U; i < DM_ARM_MOTOR_COUNT; i++)
    if (dm_motors[i].feedback_count != 0U &&
        (int32_t)(now - dm_motors[i].feedback_tick) <= (int32_t)DM_ARM_POWER_LOST_MS)
      return 0U;
  return 1U;
}

void AppArmPower_Watch(void)
{
  const uint32_t now = platform_now_ms();

  const uint8_t driving = (dm_motor_runtime.control_enabled || AppEnable_InProgress()) ? 1U : 0U;
  if (((waiting == 0U) || (driving != 0U)) && AppFdcan_BusFault() && arm_silent(now))
  {
    if (driving != 0U)
    {
      printf("CAN: bus fault and no feedback from J1-J7 for %u ms: arm power lost; "
             "drives marked disabled\r\n", (unsigned)DM_ARM_POWER_LOST_MS);
      AppDamiao_Cancel();
      AppEnable_Disable(DM_ALL_MOTOR_MASK);
    }
    if (waiting == 0U)
    {
      printf("CAN: bus fault, J1-J7 silent: waiting for arm power, "
             "resetting CAN and re-probing every 1 s\r\n");
      waiting = 1U;
      responding = 0U;
      full_probes = 0U;
      dm_motor_runtime.profile_ready = 0U;
    }
    probing = 0U;
    probe_tick = now - ARM_POWER_PROBE_MS;
    return;
  }
  if (waiting == 0U) return;

  /* An ENABLE that went through while waiting (the arm came back between
   * two probes): the drives answer, the bus works; never reset under it. */
  if (driving != 0U)
  {
    if (dm_motor_runtime.control_enabled && !AppFdcan_BusFault()) waiting = 0U;
    probing = 0U;
    return;
  }
  if (eth_maintenance.pending || eth_maintenance.waiting) return;

  if (probing == 0U)
  {
    if ((uint32_t)(now - probe_tick) < ARM_POWER_PROBE_MS) return;
    platform_restart_arm_bus();   /* drop un-ACKed frames, clear TEC/REC */
    for (uint8_t i = 0U; i < DM_ARM_MOTOR_COUNT; i++)
    {
      before[i] = dm_motors[i].feedback_count;
      (void)platform_drive_request_feedback(&dm_motors[i]);
    }
    probe_tick = now;
    probing = 1U;
    return;
  }
  if ((uint32_t)(now - probe_tick) < ARM_POWER_REPLY_MS) return;
  probing = 0U;
  uint8_t answered = 0U;
  for (uint8_t i = 0U; i < DM_ARM_MOTOR_COUNT; i++)
    if (dm_motors[i].feedback_count != before[i]) answered++;
  responding = answered;
  full_probes = (answered == DM_ARM_MOTOR_COUNT) ? (uint8_t)(full_probes + 1U) : 0U;
  if (full_probes < 2U) return;
  platform_restart_arm_bus();     /* resume with clean error counters */
  waiting = 0U;
  printf("CAN: all %u arm motors respond again; ENABLE re-applies the profile\r\n",
         (unsigned)DM_ARM_MOTOR_COUNT);
}
