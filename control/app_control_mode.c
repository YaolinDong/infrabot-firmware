/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_runtime.h"
#include "eth_state.h"
#include "control_services.h"
#include "platform.h"
#include "app_control_mode.h"
#include "app_state.h"
#include "app_motion.h"
#include "app_safety.h"
#include "app_gravity.h"
#include <math.h>

uint8_t AppControlMode_Profile(void)
{
  const uint8_t profile = eth_udp_command.motor_mode[0];
  if (profile != DM_ETH_MODE_MIT && profile != DM_ETH_MODE_POS_VEL) return 0U;
  for (unsigned i = 1; i < DM_GRIPPER_INDEX; ++i)
    if (eth_udp_command.motor_mode[i] != profile) return 0U;
  return eth_udp_command.motor_mode[DM_GRIPPER_INDEX] == DM_ETH_MODE_FORCE_POS ? profile : 0U;
}

uint8_t AppControlMode_ProfileReady(void)
{
  return dm_motor_runtime.profile_ready && AppControlMode_Profile() != 0U;
}

uint8_t AppControlMode_Telemetry(void)
{
  return 0x80U | (AppControlMode_ProfileReady() ? 0x40U : 0U) |
         (AppControlMode_Profile() << 4) | eth_udp_command.control_mode;
}

void AppControlMode_Hold(void)
{
  /* Latch ONCE at handover. Do not chase live feedback each tick.  Keep the
   * active command when the drive is following it: re-latching the measured
   * angle would drop a stationary arm by its gravity sag on every switch.
   * The arm only: the gripper keeps its own command (re-latched, a gripper
   * squeezing an object would let go of it). */
  eth_udp_command.sequence_valid = 0U;
  eth_udp_command.queue_head = eth_udp_command.queue_tail = eth_udp_command.queue_count = 0U;
  for (unsigned i = 0; i < DM_ARM_MOTOR_COUNT; ++i) {
    const float direction = DM_MOTOR_DIRECTION(i);
    const float hold = (dm_motor_runtime.control_enabled && dm_motor_runtime.enabled[i] &&
                        isfinite(dm_motor_runtime.command_position[i]) &&
                        fabsf(dm_motor_runtime.command_position[i] - dm_motors[i].position) <=
                            ARM_HOLD_KEEP_COMMAND_RAD) ?
                       dm_motor_runtime.command_position[i] : dm_motors[i].position;
    dm_motor_runtime.command_position[i] = hold;
    eth_udp_command.target_position[i] = (hold - dm_motor_runtime.zero_offset[i]) * direction;
    eth_udp_command.target_velocity[i] = eth_udp_command.target_torque[i] = 0.0f;
    dm_motor_runtime.override[i] = 1U;
    dm_motor_runtime.stopped[i] = 0U;
  }
  eth_udp_command.last_tick = platform_now_ms();
  eth_udp_command.active = dm_motor_runtime.control_enabled;
}

uint8_t AppControlMode_Switch(uint8_t mode)
{
  if (mode > DM_ETH_CONTROL_CARTESIAN_ADMITTANCE) return 3U;
  if (AppControlMode_IsLocalCompliant(mode) && AppSafety_Braking()) return 4U;
  if (AppGravity_Recovering()) return 4U;
  if (mode == DM_ETH_CONTROL_IDLE && AppGravity_Owned()) {
    AppSafety_Stop(INFRABOT_STOP_REQUESTED, INFRABOT_NO_JOINT, 0U);
    return 0U;
  }
  if (AppTasks_Enabling() || eth_udp_command.control_request || eth_maintenance.profile_pending) return 4U;
  if (mode == eth_udp_command.control_mode) {
    /* Retries never reset a target.  IDLE -> IDLE acknowledges a controlled
     * stop that is over (the arm stopped and holds, the PC is here): the
     * PC's resume of a stop that happened in IDLE. */
    if (mode == DM_ETH_CONTROL_IDLE && !AppSafety_Stopping()) AppSafety_Clear();
    return 0U;
  }
  if (mode != DM_ETH_CONTROL_IDLE && eth_udp_command.control_mode != DM_ETH_CONTROL_IDLE) return 6U;
  if (mode != DM_ETH_CONTROL_IDLE && !AppControlMode_ProfileReady()) return 6U;
  if (AppControlMode_IsLocalCompliant(mode)) {
    const uint8_t gravity_ready = AppGravity_CanStartMode(mode);
    if (gravity_ready) return gravity_ready;
  }
  /* The control modes move the arm: J1-J7 enabled; the gripper may be
   * disabled or absent (a different tool, identification runs). */
  if (mode != DM_ETH_CONTROL_IDLE) {
    if (!dm_motor_runtime.control_enabled) return 7U;
    for (unsigned i = 0; i < DM_ARM_MOTOR_COUNT; ++i)
      if (!dm_motor_runtime.enabled[i]) return 7U;
  }
  /* No discontinuous mode changes during motion. Stop/settle first. */
  for (unsigned i = 0; i < DM_ARM_MOTOR_COUNT; ++i) {
    if (!dm_motor_runtime.enabled[i]) continue;
    if (!dm_motors[i].feedback_count || platform_now_ms() - dm_motors[i].feedback_tick > 50U) return 7U;
    if (dm_motors[i].state != 1U || !isfinite(dm_motors[i].position) ||
        !isfinite(dm_motors[i].velocity)) return 7U;
    if (fabsf(dm_motors[i].velocity) > 0.05f) return 4U;
  }
  if (!AppMotion_ReleaseHold()) return 4U;
  platform_enter_critical();
  const uint8_t gravity = AppControlMode_IsLocalCompliant(mode);
  /* Gravity must capture the current MIT hold, including its load-bearing
   * feedforward. The generic hold clears torque/velocity and may rebase the
   * position, which would remove support before the gravity ramp starts. */
  if (!gravity) AppControlMode_Hold();
  AppSafety_Clear(); /* the PC has acknowledged a controlled stop */
  eth_udp_command.control_mode = mode;
  if (mode >= DM_ETH_CONTROL_RT_JOINT && mode <= DM_ETH_CONTROL_NRT_CARTESIAN)
    AppSafety_SetLocalHold(0U);
  if (gravity) AppGravity_Start(mode);
  if (mode == DM_ETH_CONTROL_IDLE) {
    /* The IDLE hold stays watched (the PC's 100 Hz heartbeat): without it
     * the arm makes a controlled stop (app_safety.c). */
    eth_udp_command.cartesian_target_valid = 0U;
    eth_udp_command.sequence_valid = 0U;
  }
  platform_exit_critical();
  return 0U;
}
