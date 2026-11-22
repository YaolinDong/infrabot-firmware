/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

/* Control rates, motion profiles and supervisory limits. */
#include "infrabot/protocol.h"

#define DM_ARM_MAX_SPEED_RAD_S INFRABOT_ARM_MAX_JOINT_VELOCITY
/* Position-velocity drives follow their target with time constant gear
 * ratio / KP_APR (see ArmDriveLead).  The MCU reads KP_APR (register 27) from
 * J1-J7 when the position-velocity profile is applied and whenever it is read
 * or written; this, the value of InfraBot SDK's infrabot.tsv, is used until then
 * or when a drive does not answer. */
#define DM_POSVEL_KP_APR_DEFAULT    54.0f
#define DM_MIT_ARM_TARGET_MIN      -1.0f
#define DM_MIT_ARM_TARGET_MAX       1.0f
#define DM_MIT_TARGET_VELOCITY      0.0f
#define DM_MIT_KP_8009P             300.0f
#define DM_MIT_KD_8009P             2.5f
#define DM_MIT_KP_4340P             150.0f
#define DM_MIT_KD_4340P             2.5f
#define DM_MIT_KP_4310              40.0f
#define DM_MIT_KD_4310              0.8f
#define DM_MIT_KP_GRIPPER           3.0f
#define DM_MIT_KD_GRIPPER           0.8f
#define DM_MIT_TORQUE_FEEDFORWARD   0.0f
#define DM_GRIPPER_TARGET_VELOCITY  1.0f
#define DM_GRIPPER_TARGET_CURRENT   1.0f
#define DM_MIT_TARGET_SLEW_RAD_S    1.0f
#define DM_MIT_ENDPOINT_DWELL_MS    1000U
#define DM_CONTROL_HZ               1000U
#define DM_MOTOR_COMMAND_HZ         1000U
#define DM_ETH_COMMAND_HZ           1000U
/* A command stream (or the IDLE heartbeat) is considered live while packets
 * arrive within this interval.  On timeout the control task makes a
 * controlled stop (app_safety.c); it never falls back to a local motion
 * profile. */
#define DM_ETH_COMMAND_TIMEOUT_MS    100U
/* J1-J7 RT position changes obey the same 3.14 rad/s cap as the planners.
 * The motor VMAX constants above remain the protocol encoding ranges. */
#define DM_ETH_COMMAND_SLEW_RAD_S    DM_ARM_MAX_SPEED_RAD_S
/* Effective arm control layer reported in the DMTL mode byte. */
#define DM_ETH_CONTROL_IDLE          INFRABOT_CONTROL_IDLE
#define DM_ETH_CONTROL_RT_JOINT      INFRABOT_CONTROL_RT_JOINT
#define DM_ETH_CONTROL_RT_CARTESIAN  INFRABOT_CONTROL_RT_CARTESIAN
#define DM_ETH_CONTROL_NRT_JOINT     INFRABOT_CONTROL_NRT_JOINT
#define DM_ETH_CONTROL_NRT_CARTESIAN INFRABOT_CONTROL_NRT_CARTESIAN
#define DM_ETH_CONTROL_GRAVITY_FREE INFRABOT_CONTROL_GRAVITY_FREE
#define DM_ETH_CONTROL_GRAVITY_LOCK INFRABOT_CONTROL_GRAVITY_LOCK
#define DM_ETH_CONTROL_JOINT_IMPEDANCE INFRABOT_CONTROL_JOINT_IMPEDANCE
#define DM_ETH_CONTROL_CARTESIAN_IMPEDANCE INFRABOT_CONTROL_CARTESIAN_IMPEDANCE
#define DM_ETH_CONTROL_JOINT_ADMITTANCE INFRABOT_CONTROL_JOINT_ADMITTANCE
#define DM_ETH_CONTROL_CARTESIAN_ADMITTANCE INFRABOT_CONTROL_CARTESIAN_ADMITTANCE
/* Independent impedance defaults; bounded live RAM tuning via app_impedance. */
#define DM_IMPEDANCE_JOINT_K 8.0f
#define DM_IMPEDANCE_JOINT_D 0.8f
#define DM_IMPEDANCE_TRANSLATION_K 80.0f
#define DM_IMPEDANCE_TRANSLATION_D 12.0f
#define DM_IMPEDANCE_ROTATION_K 3.0f
#define DM_IMPEDANCE_ROTATION_D 0.6f
#define DM_IMPEDANCE_JOINT_DAMPING 0.3f
#define DM_IMPEDANCE_TORQUE_RAMP_S 0.2f

/* Local gravity v1: same default feel as the PC gravity controller. */
#define DM_GRAVITY_KD               0.8f
#define DM_GRAVITY_LOCK_KP          8.0f
#define DM_GRAVITY_RAMP_S           1.0f
#define DM_GRAVITY_STOP_RAMP_S      0.5f
#define DM_GRAVITY_BUDGET_US        250U
#define DM_GRAVITY_MAX_TEMP_C       80U
/* FREE/LOCK speed governor: the same 3.14 rad/s per joint as the motion
 * commands. Smoothly resisting from 50% of it, full brake at 100%. Slight
 * overspeed remains in gravity mode; only gross overspeed (125%) triggers
 * the emergency controlled stop. */
#define DM_GRAVITY_MAX_SPEED_RAD_S  {DM_ARM_MAX_SPEED_RAD_S, DM_ARM_MAX_SPEED_RAD_S, DM_ARM_MAX_SPEED_RAD_S, \
                                     DM_ARM_MAX_SPEED_RAD_S, DM_ARM_MAX_SPEED_RAD_S, DM_ARM_MAX_SPEED_RAD_S, \
                                     DM_ARM_MAX_SPEED_RAD_S}
#define DM_GRAVITY_SPEED_SOFT_RATIO 0.5f
#define DM_GRAVITY_SPEED_FULL_RATIO 1.0f
#define DM_GRAVITY_SPEED_TRIP_RATIO 1.25f
#define DM_GRAVITY_SPEED_BRAKE_RATIO 0.25f /* at most 25% of joint torque limit */
#define DM_GRAVITY_SPEED_BRAKE_RAMP_S 0.05f
/* Joint-limit brake: moving toward a joint limit faster than
 * sqrt(2 * DECEL * distance) is braked (dissipative only, never pushes),
 * reaching the full brake 0.5 rad/s above that speed. The brake is torque
 * limited, so a hand pushing hard can still reach the stop boundary. */
#define DM_GRAVITY_LIMIT_DECEL_RAD_S2   5.0f
#define DM_GRAVITY_LIMIT_BRAKE_SPAN     0.5f
#define DM_GRAVITY_LIMIT_BRAKE_RATIO    0.30f /* at most 30% of joint torque limit */
#define DM_SINGLE_4310_PERIOD_MS     10U
#define DM_SINGLE_4310_TARGET_MIN   (-0.5f)
#define DM_SINGLE_4310_TARGET_MAX    0.5f
#define DM_SINGLE_4310_KP            1.0f
#define DM_SINGLE_4310_KD            0.1f
#define DM_SINGLE_4310_DWELL_MS      500U
/* Controlled stop (app_safety.c, INFRABOT_STOP_* in protocol.h).  A stop
 * condition brakes the arm's joint command to rest at this deceleration (all
 * joints together, at most DM_STOP_MAX_BRAKE_S), then holds it enabled for
 * as long as it takes: the MCU never disables the arm by itself. */
#define DM_STOP_DECELERATION_RAD_S2  8.0f
#define DM_STOP_MAX_BRAKE_S          0.5f
/* An enabled arm drive silent this long is lost (FEEDBACK_LOST). */
#define DM_STOP_FEEDBACK_LOST_MS     150U
/* Bus fault and no feedback from any of J1-J7 this long: the arm has no
 * power (app_arm_power.h). Longer than one lost drive (above). */
#define DM_ARM_POWER_LOST_MS         200U
/* Torque supervisor: |measured torque| at or above the joint's limit for
 * DM_TORQUE_LIMIT_MS stops the arm (TORQUE_LIMIT).  0.9 x min(peak torque,
 * TMAX): J8009P 40 N.m, J4340P 40 (TMAX 28), J4310 12.5 (TMAX 10). */
#define DM_ARM_TORQUE_LIMIT_NM       {36.0f, 36.0f, 25.0f, 25.0f, 9.0f, 9.0f, 9.0f}
#define DM_TORQUE_LIMIT_MS           100U
