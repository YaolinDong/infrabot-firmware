/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "control_stats.h"
#include "dm_runtime.h"
#include "eth_state.h"
#include "control_services.h"
#include "platform.h"
#include "app_enable.h"
#include "app_state.h"
#include "platform_drive.h"
#include <stdio.h>

/* Serialize the staged enable.  Startup and the explicit SET_ARM_MODE
 * operation already configure CTRL_MODE while the drives are disabled.
 * Rewriting CTRL_MODE on every enable can make some Damiao firmware revisions
 * jump to their internal default target, so the runtime sequence is
 * deliberately: clear error, send a measured-position hold frame in the
 * already-selected mode, enable, then repeat the hold frame immediately after
 * enable.  The post-enable frame prevents a drive from using an old internal
 * target during the first control tick.  Before the first drive is enabled
 * every drive of the sequence must answer a feedback request, so a missing
 * one (a gripper that is not connected) fails the sequence with nothing
 * enabled, rather than after the others came up. */
enum { PROBE, PROBE_WAIT, STAGE };
static volatile uint8_t enable_in_progress;
static uint8_t enable_phase;
static uint8_t enable_motor;
static uint8_t enable_stage;
static uint8_t enable_retry;
static uint8_t enable_failed;
static uint32_t enable_mask;         /* drives this sequence brings up */
static uint32_t enable_success_mask; /* of those, enabled so far */
static uint32_t enable_feedback_before;
static uint32_t probe_feedback_before[DM_MOTOR_COUNT];
static uint32_t probe_tick;          /* last feedback request of the probe */

uint8_t AppEnable_InProgress(void) { return enable_in_progress; }

void AppEnable_CancelAndHold(void)
{
  if (!enable_in_progress) return;
  /* The enabled subset was receiving measured-position hold frames. Latch
   * that same hold once before handing back to normal cyclic output. */
  uint8_t any = 0U;
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; ++i) {
    if (!dm_motor_runtime.enabled[i]) continue;
    any = 1U;
    if (enable_mask & (1UL << i)) dm_motor_runtime.command_position[i] = dm_motors[i].position;
  }
  enable_in_progress = 0U;
  dm_motor_runtime.control_enabled = any;
  eth_udp_command.active = 0U;
}

static const char *group_name(uint32_t mask)
{
  if (mask == DM_ALL_MOTOR_MASK) return "arm+gripper";
  if (mask == DM_ARM_MOTOR_MASK) return "arm J1-J7";
  if (mask == DM_GRIPPER_MOTOR_MASK) return "gripper J8";
  return "drives";
}

/* Feedback of drive i newer than `before` and at most 50 ms old.  Signed age:
 * a frame received after now was read is fresh, not ~49 days old. */
static uint8_t answered(uint8_t i, uint32_t before, uint32_t now)
{
  return dm_motors[i].feedback_count != before &&
         (int32_t)(now - dm_motors[i].feedback_tick) <= 50;
}

/* One cyclic frame holding drive i at hold_position (encoder rad) in its
 * selected mode.  The MIT torque feed-forward is in the joint frame, so it is
 * turned into the drive frame as the 1 kHz tick does: an arm holding against
 * gravity in IDLE keeps that support while the gripper is brought up, instead
 * of getting it reversed on J2/J4 for the length of each attempt. */
static int hold_frame(uint8_t i, float hold_position)
{
  switch (eth_udp_command.motor_mode[i])
  {
    case DM_ETH_MODE_POS_VEL:
      return platform_drive_position(&dm_motors[i], hold_position,
                               eth_udp_command.velocity_limit[i]);
    case DM_ETH_MODE_FORCE_POS:
    {
      float ratio = eth_udp_command.torque_limit_ratio[i];
      if (ratio < 0.0f) ratio = 0.0f;
      if (ratio > 1.0f) ratio = 1.0f;
      return platform_drive_position_current(&dm_motors[i], hold_position,
                                      eth_udp_command.velocity_limit[i], ratio);
    }
    case DM_ETH_MODE_VEL:
      return platform_drive_speed(&dm_motors[i], 0.0f);
    case DM_ETH_MODE_MIT:
    default:
      return platform_drive_mit(&dm_motors[i], hold_position, 0.0f,
                          eth_udp_command.target_kp[i], eth_udp_command.target_kd[i],
                          dm_motor_runtime.stopped[i] ? 0.0f :
                          DM_MOTOR_DIRECTION(i) * eth_udp_command.target_torque[i]);
  }
}

/* While the sequence runs, the normal control tick is skipped.  Keep every
 * enabled drive under an explicit cyclic hold: one this sequence brought up
 * holds where it stands (rebased when the sequence completes), otherwise it
 * would run on its old internal target for the rest of the sequence; one
 * enabled before keeps its command, so an arm holding in IDLE does not sag
 * while the gripper is brought up. */
void AppEnable_HoldFrames(void)
{
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    if (dm_motor_runtime.enabled[i] == 0U) continue;
    if (enable_mask & (1UL << i)) dm_motor_runtime.command_position[i] = dm_motors[i].position;
    (void)hold_frame(i, dm_motor_runtime.command_position[i]);
  }
}

uint8_t AppEnable_Start(uint32_t mask)
{
  uint32_t staged = 0U;
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
    if ((mask & (1UL << i)) && dm_motor_runtime.enabled[i] == 0U) staged |= 1UL << i;
  if (staged == 0U || enable_in_progress != 0U) return 0U;
  platform_enter_critical();
  enable_in_progress = 1U;
  enable_phase = PROBE;
  enable_mask = staged;
  enable_success_mask = 0U;
  enable_motor = 0U;
  enable_stage = 0U;
  enable_retry = 0U;
  enable_failed = 0U;
  eth_udp_command.control_mode = DM_ETH_CONTROL_IDLE;
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    if ((staged & (1UL << i)) == 0U) continue;
    dm_motor_runtime.override[i] = 0U;
    dm_motor_runtime.stopped[i] = 0U;
    eth_udp_command.target_velocity[i] = eth_udp_command.target_torque[i] = 0.0f;
    /* Start from the measured physical angle, never the ENABLE packet's
     * target fields or a stale target left by an earlier stream.  The MCU
     * is authoritative for the first hold target: convert the measured raw
     * angle back to the external joint coordinate used by the UDP stream
     * (including the gripper direction and zero offset), so the first 1 kHz
     * frame is a true hold, not a move. */
    dm_motor_runtime.command_position[i] = dm_motors[i].position;
    eth_udp_command.target_position[i] =
        (dm_motors[i].position - dm_motor_runtime.zero_offset[i]) * DM_MOTOR_DIRECTION(i);
  }
  platform_exit_critical();
  printf("ENABLE: %s staged sequence started\r\n", group_name(staged));
  return 1U;
}

static void finish(void)
{
  const uint8_t ok = enable_failed == 0U && enable_success_mask == enable_mask;
  platform_enter_critical();
  enable_in_progress = 0U;
  if (ok)
  {
    /* The drives are enabled one at a time.  While the remaining drives are
     * being brought up, gravity/coupling can move an already enabled joint
     * away from the position sampled at the start of this sequence.  Reusing
     * that old command on the first normal 1 kHz tick creates a high-gain
     * snap back (especially with the arm profile Kp=300/150).  Rebase every
     * command this sequence enabled to the latest measured position
     * immediately before publishing the enabled state.  The first cyclic
     * frame is therefore a true zero-error hold, independent of the enable
     * ordering and CAN latency. */
    for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
    {
      if ((enable_mask & (1UL << i)) == 0U) continue;
      dm_motor_runtime.command_position[i] = dm_motors[i].position;
      eth_udp_command.target_position[i] =
          (dm_motors[i].position - dm_motor_runtime.zero_offset[i]) * DM_MOTOR_DIRECTION(i);
      eth_udp_command.target_velocity[i] = 0.0f;
      eth_udp_command.target_torque[i] = 0.0f;
    }
    dm_motor_runtime.control_enabled = 1U;
  }
  eth_udp_command.last_tick = platform_now_ms();
  dm_control_stats.last_cycle = 0U;
  platform_exit_critical();
  if (ok)
  {
    printf("ENABLE: rebased hold positions_mrad=");
    for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
      printf("%ld%s", (long)(dm_motor_runtime.command_position[i] * 1000.0f),
             (i + 1U == DM_MOTOR_COUNT) ? "" : ",");
    printf("\r\n");
  }
  else
  {
    /* Never leave a partial group: a missing joint must not let the others
     * move against an unpowered load.  Disable every drive of the sequence,
     * also one that may have taken its enable frame before failing. */
    AppEnable_Disable(enable_mask);
  }
  printf("ENABLE: %s staged sequence complete mask=0x%02lX expected=0x%02lX%s\r\n",
         group_name(enable_mask), (unsigned long)enable_success_mask,
         (unsigned long)enable_mask, ok ? "" : " (failed: these drives stay disabled)");
}

uint8_t AppEnable_Step(void)
{
  if (enable_in_progress == 0U) return 0U;
  if (enable_failed != 0U)
  {
    finish();
    return 0U;
  }
  const uint32_t now = platform_now_ms();

  if (enable_phase == PROBE)
  {
    while (enable_motor < DM_MOTOR_COUNT && (enable_mask & (1UL << enable_motor)) == 0U)
      enable_motor++;
    if (enable_motor >= DM_MOTOR_COUNT)
    {
      enable_phase = PROBE_WAIT;
      return 1U;
    }
    probe_feedback_before[enable_motor] = dm_motors[enable_motor].feedback_count;
    if (!platform_drive_request_feedback(&dm_motors[enable_motor]))
    {
      if (enable_retry++ < 5U) return 1U;
      printf("ENABLE: J%u feedback request failed\r\n", (unsigned)enable_motor + 1U);
      enable_failed = 1U;
      return 1U;
    }
    enable_retry = 0U;
    probe_tick = now;
    enable_motor++;
    return 1U;
  }
  if (enable_phase == PROBE_WAIT)
  {
    uint32_t missing = 0U;
    for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
      if ((enable_mask & (1UL << i)) != 0U && !answered(i, probe_feedback_before[i], now))
        missing |= 1UL << i;
    if (missing == 0U)
    {
      enable_phase = STAGE;
      enable_motor = 0U;
      enable_stage = 0U;
      return 1U;
    }
    if (now - probe_tick <= 20U) return 1U;
    for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
      if (missing & (1UL << i))
        printf("ENABLE: J%u no feedback (not connected or not powered?)\r\n", (unsigned)i + 1U);
    enable_failed = 1U;
    return 1U;
  }

  while (enable_motor < DM_MOTOR_COUNT && (enable_mask & (1UL << enable_motor)) == 0U)
    enable_motor++;
  if (enable_motor >= DM_MOTOR_COUNT)
  {
    finish();
    return 0U;
  }
  DM_Motor_t *motor = &dm_motors[enable_motor];
  const uint16_t mode_id = AppDamiao_ModeId(eth_udp_command.motor_mode[enable_motor]);
  int result;
  if (enable_stage == 0U)
  {
    enable_feedback_before = motor->feedback_count;
    result = platform_drive_clear_error(motor, mode_id);
    if (result) result = platform_drive_request_feedback(motor);
  }
  else if ((enable_stage == 1U) || (enable_stage == 3U))
  {
    if (enable_stage == 1U && !answered(enable_motor, enable_feedback_before, now))
    {
      if (enable_retry++ < 20U) return 1U;
      printf("ENABLE: J%u feedback missing; aborting\r\n", (unsigned)enable_motor + 1U);
      enable_failed = 1U;
      return 1U;
    }
    result = hold_frame(enable_motor, motor->position);
  }
  else
  {
    result = platform_drive_enable(motor, mode_id);
    if (result)
    {
      /* Queue the first cyclic hold immediately after 0xFC.  Waiting for the
       * next 1 ms tick leaves a drive briefly executing its previous internal
       * target, which is visible as a sharp enable jerk on a loaded joint. */
      result = hold_frame(enable_motor, motor->position);
    }
  }

  if (!result)
  {
    if (enable_retry < 5U)
    {
      enable_retry++;
      return 1U;
    }
    printf("ENABLE: motor=%u stage=%u failed\r\n",
           (unsigned)(enable_motor + 1U), (unsigned)enable_stage);
    enable_failed = 1U;
    return 1U;
  }

  enable_retry = 0U;
  enable_stage++;
  if (enable_stage >= 4U)
  {
    dm_motor_runtime.enabled[enable_motor] = 1U;
    enable_success_mask |= 1UL << enable_motor;
    enable_motor++;
    enable_stage = 0U;
  }
  return 1U;
}

void AppEnable_Disable(uint32_t mask)
{
  if (enable_in_progress != 0U)
  {
    mask |= enable_mask;
    enable_in_progress = 0U;
  }
  uint8_t any = 0U, arm = 1U;
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    if (mask & (1UL << i))
    {
      (void)platform_drive_disable(&dm_motors[i],
                             AppDamiao_ModeId(eth_udp_command.motor_mode[i]));
      dm_motor_runtime.enabled[i] = 0U;
    }
    if (dm_motor_runtime.enabled[i] != 0U) any = 1U;
    else if (i < DM_ARM_MOTOR_COUNT) arm = 0U;
  }
  platform_enter_critical();
  dm_motor_runtime.control_enabled = any;
  if (arm == 0U)
  {
    /* The control modes move the arm: without all seven joints, IDLE. */
    eth_udp_command.control_mode = DM_ETH_CONTROL_IDLE;
    eth_udp_command.cartesian_target_valid = 0U;
    eth_udp_command.sequence_valid = 0U;
    eth_udp_command.queue_head = 0U;
    eth_udp_command.queue_tail = 0U;
    eth_udp_command.queue_count = 0U;
  }
  if (any == 0U)
  {
    eth_udp_command.active = 0U;
    dm_control_stats.last_cycle = 0U;
  }
  platform_exit_critical();
}
