/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <stdint.h>
#include "infrabot/protocol.h"

/* Controller-level state model. Drive state and supervisory state are kept
 * separate so a powered drive cannot be mistaken for a robot ready for motion. */
typedef enum
{
  ROBOT_STATUS_BOOTING = 0,
  ROBOT_STATUS_PRECHECK,
  ROBOT_STATUS_PROFILE_REQUIRED,
  ROBOT_STATUS_DISABLED,
  ROBOT_STATUS_ENABLING,
  ROBOT_STATUS_IDLE_HOLD,
  ROBOT_STATUS_RT_JOINT,
  ROBOT_STATUS_RT_CARTESIAN,
  ROBOT_STATUS_NRT_JOINT,
  ROBOT_STATUS_NRT_CARTESIAN,
  ROBOT_STATUS_STOPPING,
  ROBOT_STATUS_FAULT,
  ROBOT_STATUS_RECOVERY,
  ROBOT_STATUS_ESTOP,
  ROBOT_STATUS_UNKNOWN,
  ROBOT_STATUS_GRAVITY_FREE,
  ROBOT_STATUS_GRAVITY_LOCK,
  ROBOT_STATUS_JOINT_IMPEDANCE,
  ROBOT_STATUS_CARTESIAN_IMPEDANCE,
  ROBOT_STATUS_JOINT_ADMITTANCE, ROBOT_STATUS_CARTESIAN_ADMITTANCE
} RobotOperationalStatus;
_Static_assert(ROBOT_STATUS_JOINT_ADMITTANCE==INFRABOT_STATUS_JOINT_ADMITTANCE &&
               ROBOT_STATUS_CARTESIAN_ADMITTANCE==INFRABOT_STATUS_CARTESIAN_ADMITTANCE &&
               ROBOT_STATUS_JOINT_IMPEDANCE == INFRABOT_STATUS_JOINT_IMPEDANCE &&
               ROBOT_STATUS_CARTESIAN_IMPEDANCE == INFRABOT_STATUS_CARTESIAN_IMPEDANCE &&
               ROBOT_STATUS_BOOTING == INFRABOT_STATUS_BOOTING &&
               ROBOT_STATUS_IDLE_HOLD == INFRABOT_STATUS_IDLE_HOLD &&
               ROBOT_STATUS_NRT_CARTESIAN == INFRABOT_STATUS_NRT_CARTESIAN &&
               ROBOT_STATUS_STOPPING == INFRABOT_STATUS_STOPPING &&
               ROBOT_STATUS_FAULT == INFRABOT_STATUS_FAULT &&
               ROBOT_STATUS_UNKNOWN == INFRABOT_STATUS_UNKNOWN, "controller status of the protocol");

uint8_t RobotController_OperationalStatus(void);
uint8_t RobotController_IsReady(void);
uint8_t RobotController_IsBusy(void);
uint8_t RobotController_IsFault(void);
const char *RobotController_StatusName(uint8_t status);
void RobotController_UpdateStatus(void);

#endif /* ROBOT_CONTROLLER_H */
