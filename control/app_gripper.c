/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_runtime.h"
#include "eth_state.h"
#include "platform.h"
#include "app_gripper.h"
#include "app_state.h"

uint8_t AppGripper_Command(float position, float speed, float force)
{
  const uint8_t i = DM_GRIPPER_INDEX;
  if (eth_udp_command.motor_mode[i] != DM_ETH_MODE_FORCE_POS) return 3U;
  if (!(position >= DM_GRIPPER_POSITION_MIN && position <= DM_GRIPPER_POSITION_MAX) ||
      !(speed > 0.0f && speed <= dm_motors[i].velocity_max) || !(force >= 0.0f && force <= 1.0f))
    return 3U;
  if (dm_motor_runtime.enabled[i] == 0U) return 7U;
  platform_enter_critical();
  /* A step to the new position: the drive itself keeps to the speed limit
   * (v_des), as for every position-velocity frame. */
  dm_motor_runtime.stopped[i] = 0U;
  eth_udp_command.target_position[i] = position;
  dm_motor_runtime.command_position[i] = dm_motor_runtime.zero_offset[i] + DM_MOTOR_DIRECTION(i) * position;
  eth_udp_command.velocity_limit[i] = speed;
  eth_udp_command.torque_limit_ratio[i] = force;
  platform_exit_critical();
  return 0U;
}
