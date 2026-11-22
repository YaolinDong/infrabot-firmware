/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_runtime.h"
#include "eth_state.h"
#include "control_services.h"
#include "robot_controller.h"

#include "app_control_mode.h"
#include "app_motion.h"
#include "app_state.h"
#include "app_safety.h"
#include "app_gravity.h"

static volatile uint8_t robot_status = ROBOT_STATUS_BOOTING;

const char *RobotController_StatusName(uint8_t status)
{
  static const char *const names[] = {
      "BOOTING", "PRECHECK", "PROFILE_REQUIRED", "DISABLED", "ENABLING",
      "IDLE_HOLD", "RT_JOINT", "RT_CARTESIAN", "NRT_JOINT", "NRT_CARTESIAN",
      "STOPPING", "FAULT", "RECOVERY", "ESTOP", "UNKNOWN", "GRAVITY_FREE", "GRAVITY_LOCK", "JOINT_IMP", "CART_IMP", "JOINT_ADM", "CART_ADM"};
  return status < (sizeof(names) / sizeof(names[0])) ? names[status] : "UNKNOWN";
}

uint8_t RobotController_OperationalStatus(void)
{
  RobotController_UpdateStatus();
  return robot_status;
}

uint8_t RobotController_IsReady(void)
{
  const uint8_t status = RobotController_OperationalStatus();
  return status == ROBOT_STATUS_IDLE_HOLD || status == ROBOT_STATUS_RT_JOINT ||
         status == ROBOT_STATUS_RT_CARTESIAN || status == ROBOT_STATUS_NRT_JOINT ||
         status == ROBOT_STATUS_NRT_CARTESIAN || status == ROBOT_STATUS_GRAVITY_FREE ||
         status == ROBOT_STATUS_GRAVITY_LOCK || status == ROBOT_STATUS_JOINT_IMPEDANCE ||
         status == ROBOT_STATUS_CARTESIAN_IMPEDANCE || status==ROBOT_STATUS_JOINT_ADMITTANCE || status==ROBOT_STATUS_CARTESIAN_ADMITTANCE;
}

uint8_t RobotController_IsBusy(void)
{
  return AppGravity_Owned() || (AppMotion_Owned() != 0U) || AppTasks_Enabling() != 0U ||
         eth_udp_command.control_mode == DM_ETH_CONTROL_RT_JOINT ||
         eth_udp_command.control_mode == DM_ETH_CONTROL_RT_CARTESIAN ||
         eth_udp_command.control_mode == DM_ETH_CONTROL_NRT_JOINT ||
         eth_udp_command.control_mode == DM_ETH_CONTROL_NRT_CARTESIAN;
}

uint8_t RobotController_IsFault(void)
{
  return AppFdcan_BusFault();
}

void RobotController_UpdateStatus(void)
{
  uint8_t status = ROBOT_STATUS_UNKNOWN;
  /* Bring-up re-probing an unpowered arm: unacknowledged frames raise the CAN
   * error counters, which is expected there, not a fault. */
  if (AppStartup_ArmWaiting(NULL)) status = ROBOT_STATUS_PRECHECK;
  else if (RobotController_IsFault()) status = ROBOT_STATUS_FAULT;
  else if (AppTasks_Enabling()) status = ROBOT_STATUS_ENABLING;
  else if (!dm_motor_runtime.control_enabled) status = ROBOT_STATUS_DISABLED;
  else if (AppGravity_Recovering()) status = ROBOT_STATUS_RECOVERY;
  else if (AppSafety_Stopping()) status = ROBOT_STATUS_STOPPING;
  else if (!AppControlMode_ProfileReady()) status = ROBOT_STATUS_PROFILE_REQUIRED;
  else if (AppMotion_Owned() && !AppMotion_Failed() &&
           eth_udp_command.control_mode == DM_ETH_CONTROL_NRT_CARTESIAN)
    status = ROBOT_STATUS_NRT_CARTESIAN;
  else if (AppMotion_Owned() && !AppMotion_Failed()) status = ROBOT_STATUS_NRT_JOINT;
  else if (eth_udp_command.control_mode == DM_ETH_CONTROL_RT_JOINT) status = ROBOT_STATUS_RT_JOINT;
  else if (eth_udp_command.control_mode == DM_ETH_CONTROL_NRT_JOINT) status = ROBOT_STATUS_NRT_JOINT;
  else if (eth_udp_command.control_mode == DM_ETH_CONTROL_RT_CARTESIAN) status = ROBOT_STATUS_RT_CARTESIAN;
  else if (eth_udp_command.control_mode == DM_ETH_CONTROL_NRT_CARTESIAN) status = ROBOT_STATUS_NRT_CARTESIAN;
  else if (eth_udp_command.control_mode == DM_ETH_CONTROL_JOINT_ADMITTANCE) status=ROBOT_STATUS_JOINT_ADMITTANCE;
  else if (eth_udp_command.control_mode == DM_ETH_CONTROL_CARTESIAN_ADMITTANCE) status=ROBOT_STATUS_CARTESIAN_ADMITTANCE;
  else if (eth_udp_command.control_mode == DM_ETH_CONTROL_JOINT_IMPEDANCE) status = ROBOT_STATUS_JOINT_IMPEDANCE;
  else if (eth_udp_command.control_mode == DM_ETH_CONTROL_CARTESIAN_IMPEDANCE) status = ROBOT_STATUS_CARTESIAN_IMPEDANCE;
  else if (eth_udp_command.control_mode == DM_ETH_CONTROL_GRAVITY_FREE) status = ROBOT_STATUS_GRAVITY_FREE;
  else if (eth_udp_command.control_mode == DM_ETH_CONTROL_GRAVITY_LOCK) status = ROBOT_STATUS_GRAVITY_LOCK;
  else if (dm_motor_runtime.control_enabled && eth_udp_command.control_mode == DM_ETH_CONTROL_IDLE) status = ROBOT_STATUS_IDLE_HOLD;
  robot_status = status;
}
