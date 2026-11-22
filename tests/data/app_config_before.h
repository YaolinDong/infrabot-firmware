/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Wire protocol values: infrabot/protocol.h, this repository's copy of the
 * protocol header the PC software keeps an identical copy of. */
#include "infrabot/protocol.h"

/* Application-level configuration.  Hardware initialization stays in main.c;
 * task/driver modules include this file for shared motor-test parameters. */
#define DM_MOTOR_COUNT             8U
#define DM_ARM_MOTOR_COUNT         7U
#define DM_GRIPPER_INDEX           7U
#define DM_GRIPPER_CONTROL_MODE    4U
/* Gripper J8 travel, joint frame (rad), as measured on the gripper. */
#define DM_GRIPPER_POSITION_MIN    0.0f
#define DM_GRIPPER_POSITION_MAX    5.94f
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
#define DM_ARM_MAX_SPEED_RAD_S INFRABOT_ARM_MAX_JOINT_VELOCITY
/* Position-velocity drives follow their target with time constant gear
 * ratio / KP_APR (see ArmDriveLead).  The MCU reads KP_APR (register 27) from
 * J1-J7 when the position-velocity profile is applied and whenever it is read
 * or written; this, the value of udp_test's infrabot.tsv, is used until then
 * or when a drive does not answer. */
#define DM_RID_KP_APR               INFRABOT_RID_KP_APR
#define DM_POSVEL_KP_APR_DEFAULT    54.0f
#define DM_MIT_ARM_TARGET_MIN      -1.0f
#define DM_MIT_ARM_TARGET_MAX       1.0f
#define DM_MIT_GRIPPER_TARGET_MIN   DM_GRIPPER_POSITION_MIN
#define DM_MIT_GRIPPER_TARGET_MAX   DM_GRIPPER_POSITION_MAX
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
#define DM_DUMP_PARAMS_ON_BOOT      0
#define DM_SAVE_PARAMS_ON_BOOT      0
#define DM_MOTOR_STARTUP_WAIT_MS    1000U
#define DM_CONTROL_HZ               1000U
#define DM_MOTOR_COMMAND_HZ         1000U
#define DM_CONTROL_STATS_MS         1000U
#define DM_CONFIG_SETTLE_MS         10U
#define DM_CLEAR_SETTLE_MS          10U
#define DM_ALL_STATE_SETTLE_MS      10U
#define DM_VERBOSE_STARTUP_TRACE    0U
#define DM_CAN_PREFLIGHT_GAP_MS     10U
#define DM_RX_TRACE_DEPTH           32U
#define DM_UART_TX_RING_SIZE        8192U
/* Words.  The NRT movep path (ControlTick -> AppMotion_Tick -> ArmMotion_Tick
 * -> Cartesian tracker -> IK iterate -> Jacobian) needs ~4.2 KB at -O0
 * (-fstack-usage); 8 KB leaves room for exception frames. */
#define DM_CONTROL_TASK_STACK       2048U
#define DM_STATUS_TASK_STACK        2048U
#define DM_CONTROL_TASK_PRIORITY    (configMAX_PRIORITIES - 1U)
#define DM_STATUS_TASK_PRIORITY     2U
#define DM_STARTUP_TASK_STACK       2048U
#define DM_STARTUP_TASK_PRIORITY    4U
#define DM_ETH_TASK_STACK           4096U
#define DM_ETH_TASK_PRIORITY        5U
/* MCU run log writer (app_log_task.c): 1 kHz, below control and Ethernet so
 * it only uses their idle time; above status/panel/LCD so it keeps up. */
#define DM_LOG_TASK_STACK           1024U
#define DM_LOG_TASK_PRIORITY        4U
/* RGB LCD (app_lcd.c).  Below the status task so drawing never delays
 * control, Ethernet or the UART status line. */
#define DM_LCD_ENABLE               1U
/* AUTO=0, force 4.3-inch=43, force 5-inch=5. Unknown/missing touch falls
 * back explicitly; never infer the display size from GT115x ID alone. */
#ifndef DM_LCD_PANEL
#define DM_LCD_PANEL 0U
#endif
#ifndef DM_LCD_FALLBACK_PANEL
#define DM_LCD_FALLBACK_PANEL 43U
#endif
/* 0: GL043056B0-40-N (new 4.3-inch); 1: old compatible-example timing. */
#ifndef DM_LCD_43_LEGACY_TIMING
#define DM_LCD_43_LEGACY_TIMING 0U
#endif
#define DM_LCD_TASK_STACK           1024U
#define DM_LCD_TASK_PRIORITY        1U
/* UI task uses fractional tick periods (33/33/34 ms at a 1 kHz tick). */
#define DM_LCD_FRAME_RATE_HZ        30U
/* Board status light and keys (app_panel.c): KEY1 = STOP, KEY2 held =
 * RESUME.  Above the status and LCD tasks so a key is never queued behind
 * printing or drawing, below control, Ethernet and bring-up. */
#define DM_PANEL_TASK_STACK         512U
#define DM_PANEL_TASK_PRIORITY      3U
#define DM_PANEL_PERIOD_MS          10U
#define DM_PANEL_RESUME_HOLD_MS     1000U

/* LAN8720A RMII + UDP joint-state telemetry.  A client registers by sending
 * any UDP datagram to DM_ETH_UDP_PORT; the board then streams packets at the
 * configured rate to that source endpoint.  Keep DHCP disabled for a direct
 * cable to a PC unless a DHCP server is known to be present on the switch. */
#define DM_ETH_USE_DHCP             0U
#define DM_ETH_IP_ADDR0             192U
#define DM_ETH_IP_ADDR1             168U
#define DM_ETH_IP_ADDR2             0U
#define DM_ETH_IP_ADDR3             50U
#define DM_ETH_NETMASK0             255U
#define DM_ETH_NETMASK1             255U
#define DM_ETH_NETMASK2             255U
#define DM_ETH_NETMASK3             0U
#define DM_ETH_GW_ADDR0             192U
#define DM_ETH_GW_ADDR1             168U
#define DM_ETH_GW_ADDR2             0U
#define DM_ETH_GW_ADDR3             1U
#define DM_ETH_UDP_PORT             INFRABOT_TELEMETRY_PORT
#define DM_ETH_COMMAND_UDP_PORT     INFRABOT_COMMAND_PORT
#define DM_ETH_COMMAND_HZ           1000U
#define DM_ETH_TELEMETRY_HZ         1000U
/* A command stream (or the IDLE heartbeat) is considered live while packets
 * arrive within this interval.  On timeout the control task makes a
 * controlled stop (app_safety.c); it never falls back to a local motion
 * profile. */
#define DM_ETH_COMMAND_TIMEOUT_MS    100U
/* J1-J7 RT position changes obey the same 3.14 rad/s cap as the planners.
 * The motor VMAX constants above remain the protocol encoding ranges. */
#define DM_ETH_COMMAND_SLEW_RAD_S    DM_ARM_MAX_SPEED_RAD_S

/* DMCP command codes (INFRABOT_CMD_*, documented in protocol.h). */
#define DM_ETH_CMD_MIT               INFRABOT_CMD_MIT
#define DM_ETH_CMD_ENABLE            INFRABOT_CMD_ENABLE
#define DM_ETH_CMD_DISABLE           INFRABOT_CMD_DISABLE
#define DM_ETH_CMD_SET_ZERO          INFRABOT_CMD_SET_ZERO
#define DM_ETH_CMD_SET_ZERO_ENABLE   INFRABOT_CMD_SET_ZERO_ENABLE
#define DM_ETH_CMD_STOP              INFRABOT_CMD_STOP
#define DM_ETH_CMD_CLEAR_ERROR       INFRABOT_CMD_CLEAR_ERROR
#define DM_ETH_CMD_REQUEST_FEEDBACK  INFRABOT_CMD_REQUEST_FEEDBACK
#define DM_ETH_CMD_ENSURE_MODE       INFRABOT_CMD_ENSURE_MODE
#define DM_ETH_CMD_SEND_MIT          INFRABOT_CMD_SEND_MIT
#define DM_ETH_CMD_SEND_POS_VEL      INFRABOT_CMD_SEND_POS_VEL
#define DM_ETH_CMD_SEND_VEL          INFRABOT_CMD_SEND_VEL
#define DM_ETH_CMD_SEND_FORCE_POS    INFRABOT_CMD_SEND_FORCE_POS
#define DM_ETH_CMD_SET_CAN_TIMEOUT   INFRABOT_CMD_SET_CAN_TIMEOUT
#define DM_ETH_CMD_STORE_PARAMETERS  INFRABOT_CMD_STORE_PARAMETERS
#define DM_ETH_CMD_GET_REGISTER      INFRABOT_CMD_GET_REGISTER
#define DM_ETH_CMD_WRITE_REGISTER    INFRABOT_CMD_WRITE_REGISTER
#define DM_ETH_CMD_GET_STATE         INFRABOT_CMD_GET_STATE
/* Batch-configure the arm profile (M1..M7 MIT or POS_VEL, M8 force-position),
 * then run the parameter self-check (app_damiao.c). */
#define DM_ETH_CMD_SET_ARM_MODE      INFRABOT_CMD_SET_ARM_MODE
/* The PC arm-control profile, stored in the MCU's reserved Flash sector (not
 * the motors' own parameter store). */
#define DM_ETH_CMD_STORE_CONTROL_PROFILE INFRABOT_CMD_STORE_CONTROL_PROFILE
/* Flexiv-style supervisory control mode, selected after the motor profile. */
#define DM_ETH_CMD_SET_CONTROL_MODE  INFRABOT_CMD_SET_CONTROL_MODE
#define DM_ETH_CMD_HOLD_HEARTBEAT    INFRABOT_CMD_HOLD_HEARTBEAT
/* Read-only: DMAR value = the motor's hold command in joint-frame mrad.  A PC
 * entering a motion mode seeds its target from it rather than from the
 * measured angle, which sits below the command by the gravity sag. */
#define DM_ETH_CMD_GET_HOLD_POSITION INFRABOT_CMD_GET_HOLD_POSITION
/* Enable/disable one drive group: the arm J1-J7 or the gripper J8.  Enabling
 * needs IDLE and stages only the group's drives that are not enabled yet;
 * disabling leaves the other group running. */
#define DM_ETH_CMD_ENABLE_ARM        INFRABOT_CMD_ENABLE_ARM
#define DM_ETH_CMD_ENABLE_GRIPPER    INFRABOT_CMD_ENABLE_GRIPPER
#define DM_ETH_CMD_DISABLE_ARM       INFRABOT_CMD_DISABLE_ARM
#define DM_ETH_CMD_DISABLE_GRIPPER   INFRABOT_CMD_DISABLE_GRIPPER
/* Gripper J8 in every control mode (position, speed limit, force). */
#define DM_ETH_CMD_GRIPPER           INFRABOT_CMD_GRIPPER
#define DM_ARM_MOTOR_MASK            INFRABOT_ARM_MASK     /* J1-J7 */
#define DM_GRIPPER_MOTOR_MASK        INFRABOT_GRIPPER_MASK /* J8 */
#define DM_ALL_MOTOR_MASK            INFRABOT_ALL_MASK

/* Why a command datagram was rejected; UART "CMD_INVALID=n(reason=r,detail=d)"
 * shows the last one.  detail is the packet length for FORMAT, else the
 * command code. */
#define DM_ETH_INVALID_FORMAT        INFRABOT_INVALID_FORMAT
#define DM_ETH_INVALID_JOINT_STREAM  INFRABOT_INVALID_JOINT_STREAM
#define DM_ETH_INVALID_STATE         INFRABOT_INVALID_STATE
#define DM_ETH_INVALID_MOTION_OWNED  INFRABOT_INVALID_MOTION_OWNED
#define DM_ETH_INVALID_COMMAND       INFRABOT_INVALID_COMMAND
#define DM_ETH_INVALID_CARTESIAN     INFRABOT_INVALID_CARTESIAN
#define DM_ETH_INVALID_LIMITS        INFRABOT_INVALID_LIMITS
#define DM_ETH_INVALID_MAINTENANCE   INFRABOT_INVALID_MAINTENANCE
#define DM_ETH_INVALID_TIMED         INFRABOT_INVALID_TIMED

/* A v4 maintenance/control packet uses 0xFF to address every motor. */
#define DM_ETH_MOTOR_ALL             INFRABOT_MOTOR_ALL
#define DM_ETH_REGISTER_U32          INFRABOT_REGISTER_U32
#define DM_ETH_REGISTER_F32          INFRABOT_REGISTER_F32

/* Damiao drive modes. */
#define DM_ETH_MODE_MIT              INFRABOT_DRIVE_MODE_MIT
#define DM_ETH_MODE_POS_VEL          INFRABOT_DRIVE_MODE_POS_VEL
#define DM_ETH_MODE_VEL              INFRABOT_DRIVE_MODE_VEL
#define DM_ETH_MODE_FORCE_POS        INFRABOT_DRIVE_MODE_FORCE_POS

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

/* All motors are preconfigured as CAN_BR=9: nominal 1 Mbps, data 5 Mbps. */
#define DM_8_MOTOR_FD_CONTROL       1U

/* Set to 1 for the isolated DM4310 ID 0x05 test. */
#define DM_SINGLE_4310_TEST          0U
#define DM_SINGLE_4310_ID            0x05U
#define DM_SINGLE_4310_MASTER_ID     0x15U
#define DM_SINGLE_4310_PERIOD_MS     10U
#define DM_SINGLE_4310_TARGET_MIN   (-0.5f)
#define DM_SINGLE_4310_TARGET_MAX    0.5f
#define DM_SINGLE_4310_KP            1.0f
#define DM_SINGLE_4310_KD            0.1f
#define DM_SINGLE_4310_DWELL_MS      500U

#define DM_RID_CTRL_MODE             INFRABOT_RID_CTRL_MODE
#define DM_RID_CAN_BR                INFRABOT_RID_CAN_BR

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

/* Parameter self-check of SET_ARM_MODE: wait per register read. */
#define DM_SELFCHECK_READ_TIMEOUT_MS 20U

_Static_assert(DM_MOTOR_COUNT == INFRABOT_MOTOR_COUNT, "motor count of the protocol");
_Static_assert(DM_ARM_MOTOR_COUNT == INFRABOT_ARM_AXES, "arm axes of the protocol");
_Static_assert(DM_GRIPPER_INDEX == INFRABOT_GRIPPER_INDEX, "gripper index of the protocol");
_Static_assert(DM_GRIPPER_CONTROL_MODE == INFRABOT_DRIVE_MODE_FORCE_POS, "gripper drive mode");

#endif /* APP_CONFIG_H */
