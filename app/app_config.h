/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Aggregate shared configuration for the firmware and the host tests. */
#include "infrabot/protocol.h"
#include "board_config.h"
#include "dm_config.h"
#include "control_config.h"

#define DM_DUMP_PARAMS_ON_BOOT      0
#define DM_SAVE_PARAMS_ON_BOOT      0
#define DM_MOTOR_STARTUP_WAIT_MS    1000U
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
#define DM_ETH_TELEMETRY_HZ         1000U
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

#endif /* APP_CONFIG_H */
