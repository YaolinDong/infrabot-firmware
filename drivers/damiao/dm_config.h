/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

/* Drive inventory, encoding ranges, direction and register identities. */
#include "infrabot/protocol.h"

#define DM_MOTOR_COUNT             8U
#define DM_ARM_MOTOR_COUNT         7U
#define DM_GRIPPER_INDEX           7U
#define DM_GRIPPER_CONTROL_MODE    4U
/* Gripper J8 travel, joint frame (rad), as measured on the gripper. */
#define DM_GRIPPER_POSITION_MIN    0.0f
#define DM_GRIPPER_POSITION_MAX    5.94f
#define DM_MIT_GRIPPER_TARGET_MIN   DM_GRIPPER_POSITION_MIN
#define DM_MIT_GRIPPER_TARGET_MAX   DM_GRIPPER_POSITION_MAX

/* Software-positive gripper motion direction relative to the motor encoder.
 * The installed M8/4310 is mechanically reversed, so keep the PC/UI
 * coordinate positive while commanding the motor in the negative direction. */
#define DM_GRIPPER_DIRECTION      (-1.0f)
/* Software-positive joint direction relative to each motor encoder. J2 and
 * J4 are mounted opposite to the encoder-positive direction on this arm. */
#define DM_JOINT2_DIRECTION       (-1.0f)
#define DM_JOINT4_DIRECTION       (-1.0f)
#define DM_MOTOR_DIRECTION(index) \
  (((index) == 1U) ? DM_JOINT2_DIRECTION : \
   (((index) == 3U) ? DM_JOINT4_DIRECTION : \
    (((index) == DM_GRIPPER_INDEX) ? DM_GRIPPER_DIRECTION : 1.0f)))
#define DM_8009P_POSITION_MAX      12.5f
#define DM_8009P_VELOCITY_MAX      45.0f
#define DM_8009P_TORQUE_MAX        54.0f
#define DM_4340P_POSITION_MAX      12.5f
#define DM_4340P_VELOCITY_MAX      10.0f
#define DM_4340P_TORQUE_MAX        28.0f
#define DM_4310_POSITION_MAX       12.5f
#define DM_4310_VELOCITY_MAX       30.0f
#define DM_4310_TORQUE_MAX         10.0f
/* Rated speeds from the Damiao manuals (J8009P 100 rpm, J4340P 36 rpm, J4310
 * 120 rpm; no-load at 24 V: 160, 52, 200 rpm).  The *_VELOCITY_MAX values
 * above are only the MIT encoding ranges.  Cartesian moves never command a
 * joint faster than its rated speed; above it the line slows down. */
#define DM_8009P_RATED_SPEED_RAD_S 10.47f
#define DM_4340P_RATED_SPEED_RAD_S 3.77f
#define DM_4310_RATED_SPEED_RAD_S  12.57f
#define DM_ARM_RATED_SPEED_RAD_S {DM_8009P_RATED_SPEED_RAD_S, DM_8009P_RATED_SPEED_RAD_S, \
                                  DM_4340P_RATED_SPEED_RAD_S, DM_4340P_RATED_SPEED_RAD_S, \
                                  DM_4310_RATED_SPEED_RAD_S, DM_4310_RATED_SPEED_RAD_S, \
                                  DM_4310_RATED_SPEED_RAD_S}
#define DM_ARM_GEAR_RATIO {9.0f, 9.0f, 40.0f, 40.0f, 10.0f, 10.0f, 10.0f}
#define DM_RID_KP_APR               INFRABOT_RID_KP_APR
#define DM_ARM_MOTOR_MASK            INFRABOT_ARM_MASK     /* J1-J7 */
#define DM_GRIPPER_MOTOR_MASK        INFRABOT_GRIPPER_MASK /* J8 */
#define DM_ALL_MOTOR_MASK            INFRABOT_ALL_MASK
/* A v4 maintenance/control packet uses 0xFF to address every motor. */
#define DM_ETH_MOTOR_ALL             INFRABOT_MOTOR_ALL
#define DM_ETH_REGISTER_U32          INFRABOT_REGISTER_U32
#define DM_ETH_REGISTER_F32          INFRABOT_REGISTER_F32

/* Damiao drive modes. */
#define DM_ETH_MODE_MIT              INFRABOT_DRIVE_MODE_MIT
#define DM_ETH_MODE_POS_VEL          INFRABOT_DRIVE_MODE_POS_VEL
#define DM_ETH_MODE_VEL              INFRABOT_DRIVE_MODE_VEL
#define DM_ETH_MODE_FORCE_POS        INFRABOT_DRIVE_MODE_FORCE_POS
/* All motors are preconfigured as CAN_BR=9: nominal 1 Mbps, data 5 Mbps. */
#define DM_8_MOTOR_FD_CONTROL       1U

/* Set to 1 for the isolated DM4310 ID 0x05 test. */
#define DM_SINGLE_4310_TEST          0U
#define DM_SINGLE_4310_ID            0x05U
#define DM_SINGLE_4310_MASTER_ID     0x15U
#define DM_RID_CTRL_MODE             INFRABOT_RID_CTRL_MODE
#define DM_RID_CAN_BR                INFRABOT_RID_CAN_BR
/* Parameter self-check of SET_ARM_MODE: wait per register read. */
#define DM_SELFCHECK_READ_TIMEOUT_MS 20U

_Static_assert(DM_MOTOR_COUNT == INFRABOT_MOTOR_COUNT, "motor count of the protocol");
_Static_assert(DM_ARM_MOTOR_COUNT == INFRABOT_ARM_AXES, "arm axes of the protocol");
_Static_assert(DM_GRIPPER_INDEX == INFRABOT_GRIPPER_INDEX, "gripper index of the protocol");
_Static_assert(DM_GRIPPER_CONTROL_MODE == INFRABOT_DRIVE_MODE_FORCE_POS, "gripper drive mode");
