/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "bus_state.h"
#include "control_stats.h"
#include "dm_runtime.h"
#include "eth_state.h"
#include "uart_tx_state.h"
#include "dm_protocol.h"
#include "app_tasks.h"

#include "app_config.h"
#include "app_runtime.h"
#include "app_state.h"
#include "app_ethernet.h"
#include "dm_motor.h"
#include "app_uart.h"
#include "app_persist.h"
#include "app_damiao.h"
#include "app_motion.h"
#include "app_control_mode.h"
#include "app_enable.h"
#include "app_fdcan.h"
#include "app_safety.h"
#include "app_gravity.h"
#include "app_admittance.h"
#include "app_local_control.h"
#include "app_arm_power.h"
#include "robot_controller.h"
#include "infrabot_ik_mcu.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static ArmCartesianTracker app_rt_cartesian;
/* Speed limits in effect: the stream's, with decreases ramped. */
static ArmCartesianLimits app_rt_cartesian_limits;
static uint8_t app_rt_cartesian_seeded;
/* Position-velocity drives are sent their command plus the lag they will
 * have (ArmDriveLead). */
static ArmDriveLead app_drive_lead[DM_ARM_MOTOR_COUNT];
static float app_drive_target[DM_MOTOR_COUNT];

static uint64_t AppTasks_IKTimeUs(void *context)
{
  (void)context;
  return AppEthernet_GetTimeUs();
}

static void AppTasks_ResetRealtimeCartesian(void)
{
  app_rt_cartesian_seeded = 0U;
}

static uint32_t AppTasks_DwtNow(void)
{
  return DWT->CYCCNT;
}

static uint32_t AppTasks_CyclesToUs(uint32_t cycles)
{
  if (SystemCoreClock == 0U) return 0U;
  return (uint32_t)(((uint64_t)cycles * 1000000ULL) / SystemCoreClock);
}

static int AppTasks_MotorIndexById(uint16_t id)
{
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    if (dm_motors[i].id == id) return (int)i;
  }
  return -1;
}

static void AppTasks_ApplyLimit(uint8_t motor_id, uint8_t rid, float value)
{
  int index = AppTasks_MotorIndexById((uint16_t)motor_id);
  if ((index < 0) || (value <= 0.0f)) return;
  switch (rid)
  {
    case 21U: dm_motors[index].position_max = value; break;
    case 22U: dm_motors[index].velocity_max = value; break;
    case 23U: dm_motors[index].torque_max = value; break;
    default: break;
  }
}

uint8_t AppTasks_Enabling(void) { return AppEnable_InProgress(); }

static AppTasksStatusSnapshot app_status_snapshot;

void AppTasks_GetStatusSnapshot(AppTasksStatusSnapshot *snapshot)
{
  taskENTER_CRITICAL();
  *snapshot = app_status_snapshot;
  taskEXIT_CRITICAL();
}

/* Least free stack a task has had, in bytes; zero for a task that does not
 * exist (yet).  uxTaskGetStackHighWaterMark(NULL) would measure the caller. */
static uint32_t AppTasks_StackFree(TaskHandle_t task)
{
  if (task == NULL) return 0U;
  return (uint32_t)uxTaskGetStackHighWaterMark(task) * (uint32_t)sizeof(StackType_t);
}

/* Counters in a 16-bit telemetry field saturate. */
static uint16_t AppTasks_U16(uint32_t value)
{
  return value > 0xFFFFU ? 0xFFFFU : (uint16_t)value;
}

static void AppTasks_ProcessEthernetRequest(void)
{
  static uint32_t last_persistent_zero_tick;
  uint8_t request;
  uint32_t enable_group, disable_group;

  taskENTER_CRITICAL();
  request = eth_udp_command.control_request;
  eth_udp_command.control_request = 0U;
  enable_group = eth_udp_command.enable_request;
  eth_udp_command.enable_request = 0U;
  disable_group = eth_udp_command.disable_request;
  eth_udp_command.disable_request = 0U;
  taskEXIT_CRITICAL();

  if (request == DM_ETH_CMD_DISABLE)
  {
    /* Both groups; pending group requests are moot. */
    AppDamiao_Cancel();
    AppEnable_Disable(DM_ALL_MOTOR_MASK);
    return;
  }
  if (disable_group != 0U)
  {
    /* Only this group: the other one keeps running. */
    AppDamiao_CancelMotors(disable_group);
    AppEnable_Disable(disable_group);
    printf("DISABLE: %s\r\n", disable_group == DM_ARM_MOTOR_MASK ? "arm J1-J7" :
           disable_group == DM_GRIPPER_MOTOR_MASK ? "gripper J8" : "arm J1-J7 and gripper J8");
    enable_group &= ~disable_group; /* sent together: the disable wins */
  }

  if (eth_maintenance.waiting) return;
  /* The PC repeats an enable for loss tolerance.  Once the staged sequence
   * is running, a duplicate must not reset it back to the first drive. */
  if (AppEnable_InProgress() != 0U)
  {
    if (request == DM_ETH_CMD_ENABLE) request = 0U;
    enable_group = 0U;
  }
  if (enable_group != 0U)
  {
    if (!AppControlMode_ProfileReady() || eth_udp_command.control_mode != DM_ETH_CONTROL_IDLE)
      printf("ENABLE: rejected; apply motor profile while disabled and enter IDLE first\r\n");
    else if (AppEnable_Start(enable_group))
      AppSafety_Clear();
    return;
  }

  if ((request == DM_ETH_CMD_SET_ZERO) ||
      (request == DM_ETH_CMD_SET_ZERO_ENABLE))
  {
    const uint32_t now_tick = HAL_GetTick();
    if ((last_persistent_zero_tick != 0U) &&
        ((now_tick - last_persistent_zero_tick) < 250U))
    {
      /* The Qt action sends two identical control datagrams for loss
       * tolerance.  Do not erase eight motor flashes twice; the first action
       * already completed the zero and (for command 4) enabled the drives. */
      printf("ZERO: duplicate request ignored\r\n");
      return;
    }
    last_persistent_zero_tick = now_tick;
    /* The DM manuals define 0xFE as "Save Zero Position": the motor stores
     * the current output-shaft zero in its own firmware/flash.  Send it using
     * each motor's active receive ID (M8 is position/current mode).  Disable
     * first so the command is accepted deterministically even if the GUI
     * button was pressed while the arm was enabled. */
    AppEnable_Disable(DM_ALL_MOTOR_MASK); /* also aborts an enable sequence */
    /* The command FIFO is ordered, but give the drives time to enter Disable
     * before issuing the flash-backed 0xFE command.  This path is only used
     * by an operator action and is outside the 1 kHz motion stream. */
    vTaskDelay(pdMS_TO_TICKS(5U));
    uint8_t zero_tx_ok = 0U;
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
    {
      const uint16_t mode_id = AppDamiao_ModeId(eth_udp_command.motor_mode[i]);
      if (DM_Motor_SetZeroPosition(&hfdcan2, &dm_motors[i], mode_id) == HAL_OK)
        zero_tx_ok++;
      /* After the motor accepts 0xFE, its feedback coordinate is zero. */
      dm_motor_runtime.zero_offset[i] = 0.0f;
      dm_motor_runtime.command_position[i] = 0.0f;
      eth_udp_command.target_position[i] = 0.0f;
    }
    if (zero_tx_ok != DM_MOTOR_COUNT)
    {
      dm_motor_runtime.zero_valid = 0U;
      taskEXIT_CRITICAL();
      printf("ZERO: persistent-zero incomplete; motors remain disabled\r\n");
      return;
    }
    dm_motor_runtime.zero_valid = 1U;
    taskEXIT_CRITICAL();
    printf("ZERO: motor 0xFE persistent-zero frames tx=%u/%u\r\n",
           (unsigned)zero_tx_ok, (unsigned)DM_MOTOR_COUNT);
    /* Keep a small MCU-side marker for the PC status bit.  The actual angle
     * calibration now lives in each motor's own flash, not in this record. */
    (void)AppPersist_SaveZero();
  }

  if ((request == DM_ETH_CMD_ENABLE) ||
      (request == DM_ETH_CMD_SET_ZERO_ENABLE))
  {
    if (!AppControlMode_ProfileReady() || eth_udp_command.control_mode != DM_ETH_CONTROL_IDLE)
    {
      printf("ENABLE: rejected; apply motor profile while disabled and enter IDLE first\r\n");
      return;
    }
    /* Both groups: only the drives not enabled yet are staged, so a repeated
     * enable is not a new motion epoch. */
    if (request == DM_ETH_CMD_ENABLE)
    {
      if (AppEnable_Start(DM_ALL_MOTOR_MASK)) AppSafety_Clear();
      return;
    }
    if (request == DM_ETH_CMD_SET_ZERO_ENABLE)
    {
      vTaskDelay(pdMS_TO_TICKS(35U)); /* motor flash write max. 30 ms */
      if (AppEnable_Start(DM_ALL_MOTOR_MASK)) AppSafety_Clear();
      return;
    }
  }
}

void AppTasks_DrainTrace(void)
{
  while (dm_bus_stats.rx_trace_tail != dm_bus_stats.rx_trace_head)
  {
    DM_CanRxFrame_t frame;
    uint32_t index = dm_bus_stats.rx_trace_tail % DM_RX_TRACE_DEPTH;
    frame.id = dm_bus_stats.rx_trace[index].id;
    frame.length = dm_bus_stats.rx_trace[index].length;
    frame.fd_format = dm_bus_stats.rx_trace[index].fd_format;
    frame.brs = dm_bus_stats.rx_trace[index].brs;
    for (uint8_t i = 0U; i < 8U; i++) frame.data[i] = dm_bus_stats.rx_trace[index].data[i];
    dm_bus_stats.rx_trace_tail++;

    if (dm_bus_stats.startup_trace_enabled == 0U) continue;
    printf("CAN RX id=0x%03X dlc=%u format=%s brs=%u data=", (unsigned)frame.id,
           (unsigned)frame.length, (frame.fd_format != 0U) ? "FD" : "CLASSIC",
           (unsigned)frame.brs);
    for (uint8_t i = 0U; i < frame.length && i < 8U; i++)
      printf("%02X ", (unsigned)frame.data[i]);
    if ((frame.length >= 8U) && ((frame.data[2] == 0x33U) || (frame.data[2] == 0x55U)))
    {
      uint8_t rid = frame.data[3];
      if (AppDamiao_RegisterIsUint(rid) != 0U)
      {
        uint32_t value = DM_RegisterValue(frame.data);
        printf("-> id 0x%02X reg %-2u = %lu", (unsigned)frame.data[0],
               (unsigned)rid, (unsigned long)value);
      }
      else
      {
        float value;
        memcpy(&value, &frame.data[4], sizeof(value));
        AppTasks_ApplyLimit(frame.data[0], rid, value);
        printf("-> id 0x%02X reg %-2u = %ld/1000", (unsigned)frame.data[0],
               (unsigned)rid, (long)(value * 1000.0f));
      }
    }
    printf("\r\n");
  }
  if (dm_bus_stats.rx_trace_dropped != 0U)
  {
    if (dm_bus_stats.startup_trace_enabled != 0U)
      printf("CAN RX trace dropped=%lu\r\n", (unsigned long)dm_bus_stats.rx_trace_dropped);
    dm_bus_stats.rx_trace_dropped = 0U;
  }
}

const char *AppTasks_MotorStateName(uint8_t state)
{
  switch (state)
  {
    case 0x0U: return "DISABLED";
    case 0x1U: return "ENABLED";
    case 0x8U: return "OVERVOLTAGE";
    case 0x9U: return "UNDERVOLTAGE";
    case 0xAU: return "OVERCURRENT";
    case 0xBU: return "MOS_OVERTEMP";
    case 0xCU: return "COIL_OVERTEMP";
    case 0xDU: return "COMM_LOST";
    case 0xEU: return "OVERLOAD";
    default: return "UNKNOWN";
  }
}

/* newlib-nano has no float printf: millimetres with one decimal. */
static void AppTasks_PrintPointMm(const char *label, const float metres[3])
{
  printf(" %s=", label);
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    long tenths = lroundf(metres[i] * 10000.0f);
    printf("%s%s%ld.%ld", i ? "," : "", (tenths < 0) ? "-" : "",
           labs(tenths) / 10L, labs(tenths) % 10L);
  }
}

void AppTasks_ControlTick(void)
{
  AppArmPower_Watch();
  AppDamiao_Process();
  AppLocalControl_Process();
  AppTasks_ProcessEthernetRequest();
  AppGravity_ProcessRequest();
  const uint8_t enabling = AppEnable_Step();
  AppMotion_Tick(enabling);
  if (enabling != 0U)
  {
    AppEnable_HoldFrames();
    return;
  }
  if (dm_motor_runtime.control_enabled == 0U)
  {
    AppGravity_Tick();
    AppSafety_Tick(); /* ends a stop the disable has overtaken */
    return;
  }

  /* Apply the newest time-tagged command only at/after its MCU deadline.
   * The network callback only queues the target; this keeps execution on the
   * 1 kHz control task's deterministic clock. */
  if (eth_udp_command.sequence_valid != 0U)
  {
    const uint64_t now_us = AppEthernet_GetTimeUs();
    taskENTER_CRITICAL();
    while (eth_udp_command.queue_count != 0U)
    {
      const uint8_t slot = eth_udp_command.queue_tail;
      const uint64_t execute_us = eth_udp_command.queue_execute_us[slot];
      if (execute_us > now_us) break;
      for (uint8_t i = 0U; i < DM_ARM_MOTOR_COUNT; i++) /* J8: DM_ETH_CMD_GRIPPER */
        if (!dm_motor_runtime.override[i])
          eth_udp_command.target_position[i] = eth_udp_command.queue_position[slot][i];
      eth_udp_command.applied_sequence = eth_udp_command.queue_sequence[slot];
      eth_udp_command.applied_mcu_us = now_us;
      eth_udp_command.queue_tail =
          (uint8_t)((eth_udp_command.queue_tail + 1U) % DM_ETH_CMD_QUEUE_DEPTH);
      eth_udp_command.queue_count--;
    }
    taskEXIT_CRITICAL();
  }

  const uint32_t control_period_cycles = SystemCoreClock / DM_CONTROL_HZ;
  uint8_t external_command = 0U;
  if ((eth_udp_command.active != 0U) &&
      ((HAL_GetTick() - eth_udp_command.last_tick) <= DM_ETH_COMMAND_TIMEOUT_MS))
  {
    external_command = 1U;
  }
  else if (eth_udp_command.active != 0U && !AppGravity_Owned() && !AppSafety_LocalHold())
  {
    /* A missing PC stream (or IDLE heartbeat) is a stop condition: brake,
     * hold enabled. Never a local trajectory;
     * the PC must switch modes again. */
    AppSafety_Stop(INFRABOT_STOP_STREAM_LOST, INFRABOT_NO_JOINT, 0U);
  }
  uint32_t cycle_now = AppTasks_DwtNow();

  if (dm_control_stats.last_cycle != 0U)
  {
    uint32_t elapsed_cycles = cycle_now - dm_control_stats.last_cycle;
    uint32_t period_us = AppTasks_CyclesToUs(elapsed_cycles);
    if ((dm_control_stats.period_count == 0U) || (period_us < dm_control_stats.period_min_us))
      dm_control_stats.period_min_us = period_us;
    if ((dm_control_stats.period_count == 0U) || (period_us > dm_control_stats.period_max_us))
      dm_control_stats.period_max_us = period_us;
    dm_control_stats.period_sum_us += period_us;
    dm_control_stats.period_count++;
    if ((control_period_cycles != 0U) && (elapsed_cycles >= control_period_cycles))
    {
      uint32_t elapsed_ticks = elapsed_cycles / control_period_cycles;
      if (elapsed_ticks > 1U) dm_control_stats.missed += elapsed_ticks - 1U;
    }
  }
  dm_control_stats.last_cycle = cycle_now;
  dm_control_stats.cycles++;
  AppUart_Service();

  const uint8_t rt_cartesian = external_command != 0U && !AppMotion_Owned() &&
                               eth_udp_command.control_mode == DM_ETH_CONTROL_RT_CARTESIAN &&
                               eth_udp_command.cartesian_target_valid != 0U;
  if (!rt_cartesian) app_rt_cartesian_seeded = 0U;
  if (rt_cartesian)
  {
    ArmIkPose target;
    float measured[ARM_IK_AXES], hold[ARM_IK_AXES], drive_speed[ARM_IK_AXES];
    uint32_t feedback_age[ARM_IK_AXES];
    uint16_t wire_limits[4];
    uint32_t posvel_mask = 0U;
    const uint32_t now_ms = HAL_GetTick();
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < 4U; ++i) wire_limits[i] = eth_udp_command.cartesian_limits[i];
    for (uint8_t i = 0U; i < ARM_IK_AXES; ++i)
    {
      drive_speed[i] = fminf(eth_udp_command.velocity_limit[i], DM_ARM_MAX_SPEED_RAD_S);
      if (eth_udp_command.motor_mode[i] == DM_ETH_MODE_POS_VEL) posvel_mask |= 1UL << i;
    }
    target.position[0] = eth_udp_command.cartesian_pose[0];
    target.position[1] = eth_udp_command.cartesian_pose[1];
    target.position[2] = eth_udp_command.cartesian_pose[2];
    for (uint8_t row = 0U; row < 3U; ++row)
      for (uint8_t col = 0U; col < 3U; ++col)
        target.rotation[row][col] = eth_udp_command.cartesian_pose[3U + row * 3U + col];
    for (uint8_t i = 0U; i < ARM_IK_AXES; ++i)
    {
      const float direction = DM_MOTOR_DIRECTION(i);
      measured[i] = (dm_motors[i].position - dm_motor_runtime.zero_offset[i]) * direction;
      /* Signed: a frame received after now_ms was read is not ~49 days old. */
      const int32_t age = (int32_t)(now_ms - dm_motors[i].feedback_tick);
      feedback_age[i] = age > 0 ? (uint32_t)age : 0U;
      /* Seed once from the hold latched at mode entry.  Re-reading live
       * feedback every tick makes the reference chase the arm: zero position
       * stiffness, so it sags under gravity and never tracks the target. */
      hold[i] = (dm_motor_runtime.command_position[i] - dm_motor_runtime.zero_offset[i]) * direction;
    }
    taskEXIT_CRITICAL();
    /* Validated when the packet was accepted. */
    ArmCartesianLimits requested;
    (void)ArmMotion_CartesianLimitsFromWire(wire_limits, &requested);
    if (app_rt_cartesian_seeded != 0U)
      ArmMotion_RampCartesianLimits(&app_rt_cartesian_limits, &requested, 1.0f / (float)DM_CONTROL_HZ);
    else
      app_rt_cartesian_limits = requested;
    static const float rated[ARM_IK_AXES] = DM_ARM_RATED_SPEED_RAD_S;
    ArmMotion_CapJointLimits(&app_rt_cartesian_limits, rated, NULL);
    /* Position-velocity drives follow at most the v_des of the stream
     * (velocity_limit); faster commands leave them behind. */
    ArmMotion_CapDriveSpeeds(&app_rt_cartesian_limits, drive_speed, posvel_mask);
    if (app_rt_cartesian_seeded == 0U)
    {
      const float *lower = ArmIk_JointLower();
      const float *upper = ArmIk_JointUpper();
      for (uint8_t i = 0U; i < ARM_IK_AXES; ++i)
      {
        if (hold[i] < lower[i] && hold[i] >= lower[i] - ARM_IK_SEED_MARGIN_RAD) hold[i] = lower[i];
        else if (hold[i] > upper[i] && hold[i] <= upper[i] + ARM_IK_SEED_MARGIN_RAD) hold[i] = upper[i];
      }
      ArmCartesianTracker_Start(&app_rt_cartesian, hold, AppTasks_IKTimeUs, NULL);
      app_rt_cartesian_seeded = 1U;
    }
    const uint8_t can_fault = AppFdcan_BusFault();
    int8_t lagging = -1, silent = -1;
    for (uint8_t i = 0U; i < ARM_IK_AXES; ++i)
    {
      const float lag = fabsf(measured[i] - app_rt_cartesian.q[i]);
      if (lag > dm_control_stats.rt_max_lag) { dm_control_stats.rt_max_lag = lag; dm_control_stats.rt_max_lag_joint = i; }
      /* A MIT joint this far behind is blocked or stalled.  Position-velocity
       * joints are not checked (ARM_CARTESIAN_TRACKING_LIMIT_RAD); for every
       * joint, feedback missing as long as movep tolerates is a lost drive. */
      if (!(posvel_mask & (1UL << i)) && lag > ARM_CARTESIAN_TRACKING_LIMIT_RAD) lagging = (int8_t)i;
      if (feedback_age[i] > ARM_MOVE_FEEDBACK_MS + ARM_MOVE_FEEDBACK_FAIL_MS) silent = (int8_t)i;
    }
    const uint8_t fault = can_fault || lagging >= 0 || silent >= 0;
    uint8_t status = ARM_IK_CONVERGED;
    if (fault == 0U)
    {
      /* The pose copy above is deliberately explicit at the wire boundary.
       * The IK core validates the 3x3 rotation before it can affect CAN. */
      status = ArmCartesianTracker_Step(&app_rt_cartesian, &target, &app_rt_cartesian_limits,
                                        1.0f / (float)DM_CONTROL_HZ);
      dm_control_stats.rt_ik_last_status = status;
      dm_control_stats.rt_ik_last_elapsed_us = app_rt_cartesian.elapsed_us;
      if (app_rt_cartesian.elapsed_us > dm_control_stats.rt_ik_max_elapsed_us)
        dm_control_stats.rt_ik_max_elapsed_us = app_rt_cartesian.elapsed_us;
      dm_control_stats.rt_ik_last_position_error = app_rt_cartesian.position_error;
      if (status < 8U) dm_control_stats.rt_ik_status_count[status]++;
    }
    if (fault != 0U || status == ARM_IK_INVALID_INPUT || status == ARM_IK_QP_NUMERICAL_FAILURE)
    {
      /* A CAN fault, a motor not following or silent, or invalid data: a
       * controlled stop (app_safety.c), the arm brakes and holds.  Even a CAN
       * fault (bus-off or error passive now, not a past glitch): on a dead
       * bus a DISABLE would not reach the drives either. */
      eth_udp_command.cartesian_target_valid = 0U;
      if (can_fault)
      {
        printf("RT_CARTESIAN STOP: CAN bus fault (error passive/bus-off) events=%lu "
               "irq=0x%08lX code=0x%08lX\r\n", (unsigned long)dm_bus_stats.can_error_events,
               (unsigned long)dm_bus_stats.motor_error_status, (unsigned long)dm_bus_stats.motor_error_code);
        AppSafety_Stop(INFRABOT_STOP_DRIVE_FAULT, INFRABOT_NO_JOINT, 0U);
      }
      else if (silent >= 0)
      {
        printf("RT_CARTESIAN STOP: J%d no feedback for %lu ms\r\n", silent + 1,
               (unsigned long)feedback_age[silent]);
        AppSafety_Stop(INFRABOT_STOP_FEEDBACK_LOST, (uint8_t)silent, 0U);
      }
      else if (lagging >= 0)
      {
        printf("RT_CARTESIAN STOP: J%d not following cmd=%ld meas=%ld mrad\r\n", lagging + 1,
               (long)(app_rt_cartesian.q[lagging] * 1000.0f), (long)(measured[lagging] * 1000.0f));
        AppSafety_Stop(INFRABOT_STOP_NOT_FOLLOWING, (uint8_t)lagging, 0U);
      }
      else
      {
        printf("RT_CARTESIAN STOP: IK status %u (target invalid or numerical failure)\r\n",
               (unsigned)status);
        AppSafety_Stop(INFRABOT_STOP_INVALID_TARGET, INFRABOT_NO_JOINT, status);
      }
    }
    else
    {
      for (uint8_t i = 0U; i < ARM_IK_AXES; ++i)
      {
        const float next = app_rt_cartesian.q[i];
        dm_motor_runtime.command_position[i] = dm_motor_runtime.zero_offset[i] + DM_MOTOR_DIRECTION(i) * next;
        eth_udp_command.target_position[i] = next;
        eth_udp_command.target_velocity[i] = app_rt_cartesian.v[i];
        dm_motor_runtime.stopped[i] = 0U;
        dm_motor_runtime.override[i] = 1U;
      }
    }
  }
  else if (external_command != 0U && !AppMotion_Owned() &&
           eth_udp_command.control_mode == DM_ETH_CONTROL_RT_JOINT)
  {
    taskENTER_CRITICAL();
    /* The arm: the gripper follows its own command (DM_ETH_CMD_GRIPPER) in
     * every mode, its speed limited by the drive. */
    for (uint8_t i = 0U; i < DM_ARM_MOTOR_COUNT; i++)
    {
      if (!dm_motor_runtime.enabled[i] || dm_motor_runtime.stopped[i]) continue;
      const float direction = DM_MOTOR_DIRECTION(i);
      float target = dm_motor_runtime.zero_offset[i] + direction * eth_udp_command.target_position[i];
      const float min_target = -dm_motors[i].position_max;
      const float max_target = dm_motors[i].position_max;
      if (target > max_target) target = max_target;
      if (target < min_target) target = min_target;
      dm_motor_runtime.command_position[i] = ArmMotion_StreamPositionTarget(dm_motor_runtime.command_position[i],target,
                              DM_ETH_COMMAND_SLEW_RAD_S,1.0f/(float)DM_CONTROL_HZ);
    }
    taskEXIT_CRITICAL();
    dm_control_stats.endpoint_dwell_active = 0U;
    dm_control_stats.endpoint_dwell_ticks = 0U;
  }
  /* Torque, fault and feedback supervision; a controlled stop brakes and
   * holds the command computed above. */
  AppGravity_Tick();
  AppSafety_Tick();
  /* Position-velocity joints: send command + lag (ArmDriveLead), in every
   * mode, so the joint is where it is commanded instead of trailing it by
   * gear ratio / KP_APR seconds of motion.  RT Cartesian, movej, movep and a
   * braking stop publish their command's velocity; RT joint targets are
   * estimated. */
  {
    static const float gear_ratio[DM_ARM_MOTOR_COUNT] = DM_ARM_GEAR_RATIO;
    const uint8_t velocity_known = rt_cartesian || AppMotion_Owned() || AppSafety_Braking();
    const float *lower = ArmIk_JointLower();
    const float *upper = ArmIk_JointUpper();
    for (uint8_t i = 0U; i < DM_MOTOR_COUNT; ++i)
    {
      app_drive_target[i] = dm_motor_runtime.command_position[i];
      if (i >= DM_ARM_MOTOR_COUNT) continue;
      const float direction = DM_MOTOR_DIRECTION(i);
      const float q = (dm_motor_runtime.command_position[i] - dm_motor_runtime.zero_offset[i]) * direction;
      if (!dm_motor_runtime.enabled[i] || eth_udp_command.motor_mode[i] != DM_ETH_MODE_POS_VEL)
      {
        ArmDriveLead_Reset(&app_drive_lead[i], q);
        continue;
      }
      const float gain = dm_posvel_kp_apr[i];
      const float tau = gain >= 1.0f ? gear_ratio[i] / gain : 0.0f;
      /* RT joint targets move at up to the slew rate whatever v_des is. */
      const float speed = fminf(DM_ARM_MAX_SPEED_RAD_S,
                                fmaxf(eth_udp_command.velocity_limit[i], DM_ETH_COMMAND_SLEW_RAD_S));
      float velocity = ArmDriveLead_Velocity(&app_drive_lead[i], q, speed,
                                             1.0f / (float)DM_CONTROL_HZ);
      if (dm_motor_runtime.stopped[i]) velocity = 0.0f;
      else if (velocity_known) velocity = fminf(fmaxf(eth_udp_command.target_velocity[i], -speed), speed);
      const float measured = (dm_motors[i].position - dm_motor_runtime.zero_offset[i]) * direction;
      app_drive_target[i] = dm_motor_runtime.zero_offset[i] +
          direction * ArmDriveLead_Target(q, velocity, tau, measured, lower[i], upper[i]);
    }
  }
  uint32_t batch_start_cycle = AppTasks_DwtNow();
  uint32_t batch_ok = 0U;
  uint32_t batch_err = 0U;
  for (uint8_t n = 0U; n < DM_COMMANDS_PER_TICK; n++)
  {
    uint8_t i = dm_control_stats.tx_index;
    if (i >= DM_MOTOR_COUNT) i = 0U;
    dm_control_stats.tx_index = (uint8_t)((i + 1U) % DM_MOTOR_COUNT);
    if (!dm_motor_runtime.enabled[i]) continue;
    const float direction = DM_MOTOR_DIRECTION(i);
    const float speed_cap = i < DM_ARM_MOTOR_COUNT ?
        fminf(dm_motors[i].velocity_max,DM_ARM_MAX_SPEED_RAD_S) : dm_motors[i].velocity_max;
    const float velocity = dm_motor_runtime.stopped[i] ? 0.0f : direction *
        fminf(fmaxf(eth_udp_command.target_velocity[i],-speed_cap),speed_cap);
    const float velocity_limit = fminf(fmaxf(eth_udp_command.velocity_limit[i],0.f),speed_cap);
    const float torque = dm_motor_runtime.stopped[i] ? 0.0f : direction * eth_udp_command.target_torque[i];
    HAL_StatusTypeDef status;
    switch (eth_udp_command.motor_mode[i])
    {
      case DM_ETH_MODE_POS_VEL:
        status = DM_Motor_Position(&hfdcan2, &dm_motors[i],
                                   app_drive_target[i],
                                   velocity_limit);
        break;
      case DM_ETH_MODE_VEL:
        status = DM_Motor_Speed(&hfdcan2, &dm_motors[i],
                                velocity);
        break;
      case DM_ETH_MODE_FORCE_POS:
      {
        float ratio = eth_udp_command.torque_limit_ratio[i];
        if (ratio < 0.0f) ratio = 0.0f;
        if (ratio > 1.0f) ratio = 1.0f;
        status = DM_Motor_PositionCurrent(&hfdcan2, &dm_motors[i],
                                           dm_motor_runtime.command_position[i],
                                           velocity_limit,
                                           ratio);
        break;
      }
      case DM_ETH_MODE_MIT:
      default:
        status = DM_Motor_MIT(&hfdcan2, &dm_motors[i], dm_motor_runtime.command_position[i],
                              velocity,
                              eth_udp_command.target_kp[i], eth_udp_command.target_kd[i],
                              torque);
        break;
    }
    if (status == HAL_OK)
    {
      dm_control_stats.mit_ok++;
      batch_ok++;
    }
    else
    {
      dm_control_stats.mit_err++;
      batch_err++;
      if (AppMotion_Owned()) { AppMotion_TxFailed(); break; }
    }
    dm_control_stats.tx_index = (uint8_t)((i + 1U) % DM_MOTOR_COUNT);
  }
  uint32_t batch_us = AppTasks_CyclesToUs(AppTasks_DwtNow() - batch_start_cycle);
  if ((dm_control_stats.batch_count == 0U) || (batch_us < dm_control_stats.batch_min_us))
    dm_control_stats.batch_min_us = batch_us;
  if ((dm_control_stats.batch_count == 0U) || (batch_us > dm_control_stats.batch_max_us))
    dm_control_stats.batch_max_us = batch_us;
  dm_control_stats.batch_sum_us += batch_us;
  dm_control_stats.batch_count++;
  dm_control_stats.batch_frames_ok += batch_ok;
  dm_control_stats.batch_frames_err += batch_err;
}

void AppTasks_InitControlProfile(void)
{
  taskENTER_CRITICAL();
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
    dm_motor_runtime.command_position[i] = (dm_motors[i].feedback_count != 0U) ? dm_motors[i].position : 0.0f;
  dm_control_stats.target_positive = 0U;
  dm_control_stats.endpoint_dwell_active = 0U;
  dm_control_stats.endpoint_dwell_ticks = 0U;
  dm_control_stats.cycles = 0U;
  dm_control_stats.mit_ok = 0U;
  dm_control_stats.mit_err = 0U;
  dm_control_stats.missed = 0U;
  dm_control_stats.fd_tx = 0U;
  dm_control_stats.classic_tx = 0U;
  dm_control_stats.period_min_us = 0U;
  dm_control_stats.period_max_us = 0U;
  dm_control_stats.period_sum_us = 0U;
  dm_control_stats.period_count = 0U;
  dm_control_stats.batch_min_us = 0U;
  dm_control_stats.batch_max_us = 0U;
  dm_control_stats.batch_sum_us = 0U;
  dm_control_stats.batch_count = 0U;
  dm_control_stats.batch_frames_ok = 0U;
  dm_control_stats.batch_frames_err = 0U;
  dm_control_stats.last_cycle = 0U;
  dm_control_stats.tx_index = 0U;
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    eth_udp_command.target_position[i] =
        (dm_motor_runtime.command_position[i] - dm_motor_runtime.zero_offset[i]) * DM_MOTOR_DIRECTION(i);
    eth_udp_command.motor_mode[i] = (i == DM_GRIPPER_INDEX) ? DM_ETH_MODE_FORCE_POS : DM_ETH_MODE_MIT;
    dm_motor_runtime.enabled[i] = 0U;
    dm_motor_runtime.override[i] = 0U;
    dm_motor_runtime.stopped[i] = 0U;
    eth_udp_command.target_velocity[i] = (i == DM_GRIPPER_INDEX) ? DM_GRIPPER_TARGET_VELOCITY : DM_MIT_TARGET_VELOCITY;
    eth_udp_command.target_kp[i] = dm_motor_kp[i];
    eth_udp_command.target_kd[i] = dm_motor_kd[i];
    eth_udp_command.target_torque[i] = DM_MIT_TORQUE_FEEDFORWARD;
    eth_udp_command.velocity_limit[i] = (i == DM_GRIPPER_INDEX) ?
        DM_GRIPPER_TARGET_VELOCITY : DM_MIT_TARGET_SLEW_RAD_S;
    eth_udp_command.torque_limit_ratio[i] = (i == DM_GRIPPER_INDEX) ?
        DM_GRIPPER_TARGET_CURRENT : 1.0f;
  }
  eth_udp_command.last_sequence = 0U;
  eth_udp_command.applied_sequence = 0U;
  eth_udp_command.pending_sequence = 0U;
  eth_udp_command.pending_execute_us = 0U;
  eth_udp_command.queue_head = 0U;
  eth_udp_command.queue_tail = 0U;
  eth_udp_command.queue_count = 0U;
  eth_udp_command.last_rx_mcu_us = 0U;
  eth_udp_command.applied_mcu_us = 0U;
  eth_udp_command.last_tick = 0U;
  eth_udp_command.active = 0U;
  eth_udp_command.sequence_valid = 0U;
  eth_udp_command.invalid_count = 0U;
  eth_udp_command.cartesian_target_valid = 0U;
  for (uint8_t i = 0U; i < 12U; ++i) eth_udp_command.cartesian_pose[i] = 0.0f;
  eth_udp_command.cartesian_pose[3] = 1.0f;
  eth_udp_command.cartesian_pose[7] = 1.0f;
  eth_udp_command.cartesian_pose[11] = 1.0f;
  for (uint8_t i = 0U; i < 4U; ++i) eth_udp_command.cartesian_limits[i] = 0U;
  AppTasks_ResetRealtimeCartesian();
  eth_udp_command.control_mode = DM_ETH_CONTROL_IDLE;
  dm_motor_runtime.profile_ready = 0U; /* Explicit SET_ARM_MODE required each boot. */
  eth_maintenance.pending = 0U;
  eth_maintenance.waiting = 0U;
  eth_maintenance.profile_pending = 0U;
  eth_maintenance.profile_sequence = 0U;
  eth_maintenance.trace_head = dm_bus_stats.rx_trace_head;
  eth_maintenance.pending_tick = 0U;
  eth_maintenance.waiting_tick = 0U;
  eth_udp_command.control_request = 0U;
  dm_motor_runtime.zero_valid = 0U;
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
    dm_motor_runtime.zero_offset[i] = 0.0f;
  dm_motor_runtime.control_enabled = 0U;
  eth_udp_command.enable_request = 0U;
  eth_udp_command.disable_request = 0U;
  taskEXIT_CRITICAL();
}

static void AppTasks_ControlTask(void *argument)
{
  (void)argument;
  dm_control_stats.task_started = 1U;
  TickType_t wake_time = xTaskGetTickCount();
  const TickType_t period = (pdMS_TO_TICKS(1000U / DM_CONTROL_HZ) != 0U) ?
                            pdMS_TO_TICKS(1000U / DM_CONTROL_HZ) : 1U;
  for (;;)
  {
    vTaskDelayUntil(&wake_time, period);
    AppTasks_ControlTick();
  }
}

static void AppTasks_StatusTask(void *argument)
{
  (void)argument;
  TickType_t wake_time = xTaskGetTickCount();
  TickType_t last_stats_tick = wake_time;
  const TickType_t one_second = pdMS_TO_TICKS(DM_CONTROL_STATS_MS);
  uint32_t status_seconds = 0U;
  uint32_t previous_rx_frames = 0U, previous_uart_hal_errors = 0U, previous_can_state = 0U;
  uint32_t previous_can_events = 0U;
  uint32_t missed_total = 0U, can_tx_errors_total = 0U, command_invalid_total = 0U;
  uint8_t motor_state[DM_MOTOR_COUNT] = {0};

  for (;;)
  {
    vTaskDelayUntil(&wake_time, one_second);
    TickType_t now_tick = xTaskGetTickCount();
    TickType_t elapsed_ticks = now_tick - last_stats_tick;
    last_stats_tick = now_tick;
    AppTasks_DrainTrace();
    status_seconds++;

    uint32_t cycles, mit_ok, mit_err, missed, uart_dropped;
    uint32_t uart_hal_errors, uart_recoveries, uart_last_error, uart_last_state;
    uint32_t period_max_us;
    uint32_t udp_cmd_rx, udp_cmd_invalid, udp_cmd_interval_count, udp_cmd_interval_min_us;
    uint32_t udp_cmd_interval_max_us, udp_cmd_interval_sum_us, udp_cmd_jitter_max_us;
    uint32_t udp_tx_count, udp_tx_errors;
    taskENTER_CRITICAL();
    cycles = dm_control_stats.cycles; mit_ok = dm_control_stats.mit_ok; mit_err = dm_control_stats.mit_err;
    missed = dm_control_stats.missed;
    uart_dropped = uart_tx.dropped;
    uart_hal_errors = uart_tx.hal_errors;
    uart_recoveries = uart_tx.recoveries;
    uart_last_error = uart_tx.last_error;
    uart_last_state = uart_tx.last_state;
    period_max_us = dm_control_stats.period_max_us;
    udp_cmd_rx = eth_udp_command.rx_count;
    udp_cmd_invalid = eth_udp_command.invalid_count;
    udp_cmd_interval_count = eth_udp_command.interval_count;
    udp_cmd_interval_min_us = eth_udp_command.interval_min_us;
    udp_cmd_interval_max_us = eth_udp_command.interval_max_us;
    udp_cmd_interval_sum_us = eth_udp_command.interval_sum_us;
    udp_cmd_jitter_max_us = eth_udp_command.jitter_max_us;
    udp_tx_count = eth_udp_tx_stats.count;
    udp_tx_errors = eth_udp_tx_stats.errors;
    dm_control_stats.cycles = 0U; dm_control_stats.mit_ok = 0U; dm_control_stats.mit_err = 0U;
    dm_control_stats.missed = 0U; dm_control_stats.fd_tx = 0U; dm_control_stats.classic_tx = 0U;
    dm_control_stats.period_min_us = 0U; dm_control_stats.period_max_us = 0U;
    dm_control_stats.period_sum_us = 0U; dm_control_stats.period_count = 0U;
    dm_control_stats.batch_min_us = 0U; dm_control_stats.batch_max_us = 0U;
    dm_control_stats.batch_sum_us = 0U; dm_control_stats.batch_count = 0U;
    dm_control_stats.batch_frames_ok = 0U; dm_control_stats.batch_frames_err = 0U;
    eth_udp_command.rx_count = 0U;
    eth_udp_command.invalid_count = 0U;
    eth_udp_command.interval_count = 0U;
    eth_udp_command.interval_min_us = 0U;
    eth_udp_command.interval_max_us = 0U;
    eth_udp_command.interval_sum_us = 0U;
    eth_udp_command.last_interval_us = 0U;
    eth_udp_command.jitter_sum_us = 0U;
    eth_udp_command.jitter_max_us = 0U;
    if (udp_cmd_rx == 0U) eth_udp_command.last_cycle = 0U;
    eth_udp_tx_stats.count = 0U;
    eth_udp_tx_stats.interval_count = 0U;
    eth_udp_tx_stats.interval_min_us = 0U;
    eth_udp_tx_stats.interval_max_us = 0U;
    eth_udp_tx_stats.interval_sum_us = 0U;
    eth_udp_tx_stats.last_interval_us = 0U;
    eth_udp_tx_stats.jitter_sum_us = 0U;
    eth_udp_tx_stats.jitter_max_us = 0U;
    eth_udp_tx_stats.errors = 0U;
    if (udp_tx_count == 0U) eth_udp_tx_stats.last_cycle = 0U;
    uart_tx.dropped = 0U;
    taskEXIT_CRITICAL();

    {
      InfrabotLinkStats link = {0};
      link.command_rx = AppTasks_U16(udp_cmd_rx);
      link.command_invalid = AppTasks_U16(udp_cmd_invalid);
      link.command_interval_max_us = AppTasks_U16(udp_cmd_interval_max_us);
      link.command_jitter_max_us = AppTasks_U16(udp_cmd_jitter_max_us);
      link.control_missed = AppTasks_U16(missed);
      link.control_period_max_us = AppTasks_U16(period_max_us);
      link.can_tx_errors = AppTasks_U16(mit_err);
      link.telemetry_errors = AppTasks_U16(udp_tx_errors);
      link.invalid_reason = udp_cmd_invalid != 0U ? eth_udp_command.invalid_reason : 0U;
      AppEthernet_SetLinkStats(&link);
    }

    uint32_t actual_hz_x100 = (elapsed_ticks != 0U) ?
      (uint32_t)(((uint64_t)cycles * configTICK_RATE_HZ * 100ULL) / elapsed_ticks) : 0U;
    uint32_t udp_cmd_interval_avg_us = (udp_cmd_interval_count != 0U) ?
      (udp_cmd_interval_sum_us / udp_cmd_interval_count) : 0U;
    /* One line per second while the arm is enabled; at rest a 5 s heartbeat.
     * Any error counter forces the line out immediately. */
    const uint32_t rx_frames = dm_bus_stats.rx_frame_count;
    const uint32_t can_rx = rx_frames - previous_rx_frames;
    previous_rx_frames = rx_frames;
    const uint8_t anomaly = (mit_err | missed | udp_cmd_invalid | udp_tx_errors | uart_dropped) != 0U ||
                            uart_hal_errors != previous_uart_hal_errors;
    previous_uart_hal_errors = uart_hal_errors;
    uint32_t enabled_mask = 0U;
    for (uint8_t i = 0U; i < DM_MOTOR_COUNT; ++i)
      if (dm_motor_runtime.enabled[i]) enabled_mask |= 1UL << i;
    if (dm_motor_runtime.control_enabled || anomaly || status_seconds == 1U || (status_seconds % 5U) == 0U)
    {
      printf("STAT %s en=%02lX ctrl=%luHz miss=%lu period_max=%luus can_tx=%lu can_rx=%lu",
             RobotController_StatusName(RobotController_OperationalStatus()),
             (unsigned long)enabled_mask, (unsigned long)(actual_hz_x100 / 100U),
             (unsigned long)missed, (unsigned long)period_max_us,
             (unsigned long)mit_ok, (unsigned long)can_rx);
      if (mit_err != 0U) printf(" CAN_TX_ERR=%lu", (unsigned long)mit_err);
      printf(" cmd=%lu", (unsigned long)udp_cmd_rx);
      if (udp_cmd_rx != 0U)
        printf(" cmd_iv=%lu[%lu..%lu]us jit_max=%luus",
               (unsigned long)udp_cmd_interval_avg_us, (unsigned long)udp_cmd_interval_min_us,
               (unsigned long)udp_cmd_interval_max_us, (unsigned long)udp_cmd_jitter_max_us);
      if (udp_cmd_invalid != 0U)
        printf(" CMD_INVALID=%lu(reason=%u,detail=%u)", (unsigned long)udp_cmd_invalid,
               (unsigned)eth_udp_command.invalid_reason, (unsigned)eth_udp_command.invalid_length);
      printf(" tel=%lu", (unsigned long)udp_tx_count);
      if (udp_tx_errors != 0U) printf(" TEL_ERR=%lu", (unsigned long)udp_tx_errors);
      if (uart_dropped != 0U) printf(" UART_DROP=%lu", (unsigned long)uart_dropped);
      if (uart_hal_errors != 0U)
        printf(" uart_err=%lu recover=%lu last=0x%08lX/0x%08lX", (unsigned long)uart_hal_errors,
               (unsigned long)uart_recoveries, (unsigned long)uart_last_error,
               (unsigned long)uart_last_state);
      if (status_seconds == 1U) printf(" rtos=RUN ctrl_task=%u", (unsigned)dm_control_stats.task_started);
      printf("\r\n");
    }

    /* Motor faults are logged when they appear or clear, not on a timer. */
    for (uint8_t i = 0U; i < DM_MOTOR_COUNT; ++i)
    {
      const uint8_t state = dm_motors[i].state;
      if (state != motor_state[i] && (state >= 0x8U || motor_state[i] >= 0x8U))
        printf("MOTOR %s state %X->%X %s\r\n", dm_motor_names[i], (unsigned)motor_state[i],
               (unsigned)state, AppTasks_MotorStateName(state));
      motor_state[i] = state;
    }

    /* Joint-frame motor summary: position, tracking error (command -
     * measured, i.e. gravity sag/lag), torque, MOS temperature, state. */
    if ((dm_motor_runtime.control_enabled && (status_seconds % 5U) == 0U) || (status_seconds % 30U) == 0U)
    {
      printf("MOT q_mrad=");
      for (uint8_t i = 0U; i < DM_MOTOR_COUNT; ++i)
        printf("%s%ld", i ? "," : "", (long)((dm_motors[i].position - dm_motor_runtime.zero_offset[i]) *
                                             DM_MOTOR_DIRECTION(i) * 1000.0f));
      if (dm_motor_runtime.control_enabled)
      {
        printf(" err_mrad=");
        for (uint8_t i = 0U; i < DM_MOTOR_COUNT; ++i)
          printf("%s%ld", i ? "," : "", (long)((dm_motor_runtime.command_position[i] - dm_motors[i].position) *
                                               DM_MOTOR_DIRECTION(i) * 1000.0f));
      }
      printf(" tq_mNm=");
      for (uint8_t i = 0U; i < DM_MOTOR_COUNT; ++i)
        printf("%s%ld", i ? "," : "", (long)(dm_motors[i].torque * DM_MOTOR_DIRECTION(i) * 1000.0f));
      printf(" T=");
      for (uint8_t i = 0U; i < DM_MOTOR_COUNT; ++i)
        printf("%s%u", i ? "," : "", (unsigned)dm_motors[i].temperature_mos);
      printf(" st=");
      for (uint8_t i = 0U; i < DM_MOTOR_COUNT; ++i) printf("%X", (unsigned)dm_motors[i].state);
      printf("\r\n");
    }

    /* CAN bus errors: when the error state changes or new error events
     * arrived this second. */
    {
      const uint32_t error_events = dm_bus_stats.can_error_events;
      FDCAN_ProtocolStatusTypeDef protocol = {0};
      FDCAN_ErrorCountersTypeDef counters = {0};
      (void)HAL_FDCAN_GetProtocolStatus(&hfdcan2, &protocol);
      (void)HAL_FDCAN_GetErrorCounters(&hfdcan2, &counters);
      /* LEC 0 = no error, 7 = unchanged since the last read. */
      const uint8_t new_error = (protocol.LastErrorCode != 0U && protocol.LastErrorCode != 7U) ||
                                (protocol.DataLastErrorCode != 0U && protocol.DataLastErrorCode != 7U);
      const uint32_t can_state = counters.TxErrorCnt | (counters.RxErrorCnt << 8) |
                                 (protocol.ErrorPassive << 16) | (protocol.Warning << 17) |
                                 (protocol.BusOff << 18);
      AppTasksStatusSnapshot snapshot = {0};
      snapshot.seconds = status_seconds;
      snapshot.control_hz = actual_hz_x100 / 100U;
      snapshot.control_missed = missed;
      snapshot.period_max_us = period_max_us;
      snapshot.can_tx = mit_ok;
      snapshot.can_rx = can_rx;
      snapshot.can_tx_errors = mit_err;
      snapshot.command_rx = udp_cmd_rx;
      snapshot.command_invalid = udp_cmd_invalid;
      snapshot.telemetry_tx = udp_tx_count;
      snapshot.can_tec = (uint8_t)counters.TxErrorCnt;
      snapshot.can_rec = (uint8_t)counters.RxErrorCnt;
      snapshot.can_error_passive = (uint8_t)protocol.ErrorPassive;
      snapshot.can_bus_off = (uint8_t)protocol.BusOff;
      snapshot.can_error_events = error_events;
      missed_total += missed;
      can_tx_errors_total += mit_err;
      command_invalid_total += udp_cmd_invalid;
      snapshot.control_missed_total = missed_total;
      snapshot.can_tx_errors_total = can_tx_errors_total;
      snapshot.command_invalid_total = command_invalid_total;
      snapshot.heap_free_bytes = (uint32_t)xPortGetFreeHeapSize();
      snapshot.heap_min_free_bytes = (uint32_t)xPortGetMinimumEverFreeHeapSize();
      snapshot.stack_free_control_bytes = AppTasks_StackFree(dm_control_task_handle);
      snapshot.stack_free_status_bytes = AppTasks_StackFree(xTaskGetCurrentTaskHandle());
      snapshot.stack_free_ethernet_bytes = AppTasks_StackFree(dm_ethernet_task_handle);
      taskENTER_CRITICAL();
      app_status_snapshot = snapshot;
      taskEXIT_CRITICAL();
      if (new_error || can_state != previous_can_state || error_events != previous_can_events)
        printf("CAN tec=%lu rec=%lu passive=%lu warning=%lu busoff=%lu lec=%lu dlec=%lu "
               "events=%lu(+%lu) last_irq=0x%08lX last_code=0x%08lX\r\n",
               (unsigned long)counters.TxErrorCnt, (unsigned long)counters.RxErrorCnt,
               (unsigned long)protocol.ErrorPassive, (unsigned long)protocol.Warning,
               (unsigned long)protocol.BusOff, (unsigned long)protocol.LastErrorCode,
               (unsigned long)protocol.DataLastErrorCode, (unsigned long)error_events,
               (unsigned long)(error_events - previous_can_events),
               (unsigned long)dm_bus_stats.motor_error_status, (unsigned long)dm_bus_stats.motor_error_code);
      previous_can_state = can_state;
      previous_can_events = error_events;
    }

    AppMotion_PrintStatus();
    AppAdmittance_Print();
    AppGravity_PrintFault(); /* report a latched event even after recovery reached IDLE */
    if (AppGravity_Owned()) {
      AppGravityStatus g;
      AppGravityParameters p;
      AppGravity_Status(&g);
      AppGravity_GetParameters(&p);
      AppGravity_PrintImpedance(&g);
      printf("GRAV mode=%u recovery=%u us=%lu/%lu ticks=%lu local=1 "
             "kd_milli=%ld speed_mask=0x%02X\r\n",
             g.mode,g.recovering,(unsigned long)g.last_us,(unsigned long)g.max_us,(unsigned long)g.ticks,
             lroundf(p.applied[APP_GRAVITY_DAMPING]*1000.f),(unsigned)g.speed_limited_mask);
    }
    if (eth_udp_command.control_mode == DM_ETH_CONTROL_RT_CARTESIAN && app_rt_cartesian_seeded)
    {
      /* n = IK solves per status this second: converged, stepped, invalid,
       * QP iteration limit, infeasible, numerical, over budget.  vmax: speed
       * limits in effect; scale < 100 means a joint speed limit (rated speed,
       * joint-limit braking) slows the line; blk=1: holding, target not
       * followable (out of reach, singularity).  lag: the joint furthest
       * from its command this second (gravity sag, drive lag), mrad.
       * tgt/cmd/meas: target, commanded and measured tool position. */
      ArmIkPose command, actual;
      float measured[ARM_IK_AXES];
      ArmIk_Forward(app_rt_cartesian.q, &command);
      for (uint8_t i = 0U; i < ARM_IK_AXES; ++i)
        measured[i] = (dm_motors[i].position - dm_motor_runtime.zero_offset[i]) * DM_MOTOR_DIRECTION(i);
      ArmIk_Forward(measured, &actual);
      const float target[3] = {eth_udp_command.cartesian_pose[0], eth_udp_command.cartesian_pose[1],
                               eth_udp_command.cartesian_pose[2]};
      printf("RTIK n=%lu,%lu,%lu,%lu,%lu,%lu,%lu us=%lu/%lu vmax=%ldmm/s,%lddeg/s scale=%u blk=%u "
             "lag=J%u:%ld",
             (unsigned long)dm_control_stats.rt_ik_status_count[0], (unsigned long)dm_control_stats.rt_ik_status_count[1],
             (unsigned long)dm_control_stats.rt_ik_status_count[2], (unsigned long)dm_control_stats.rt_ik_status_count[3],
             (unsigned long)dm_control_stats.rt_ik_status_count[4], (unsigned long)dm_control_stats.rt_ik_status_count[5],
             (unsigned long)dm_control_stats.rt_ik_status_count[6], (unsigned long)dm_control_stats.rt_ik_last_elapsed_us,
             (unsigned long)dm_control_stats.rt_ik_max_elapsed_us,
             lroundf(app_rt_cartesian_limits.linear_velocity_max * 1000.0f),
             lroundf(app_rt_cartesian_limits.angular_velocity_max * 57.29578f),
             (unsigned)lroundf(app_rt_cartesian.scale * 100.0f), (unsigned)app_rt_cartesian.blocked,
             (unsigned)dm_control_stats.rt_max_lag_joint + 1U, lroundf(dm_control_stats.rt_max_lag * 1000.0f));
      AppTasks_PrintPointMm("tgt", target);
      AppTasks_PrintPointMm("cmd", command.position);
      AppTasks_PrintPointMm("meas", actual.position);
      printf("\r\n");
      memset((void *)dm_control_stats.rt_ik_status_count, 0, sizeof(dm_control_stats.rt_ik_status_count));
      dm_control_stats.rt_ik_max_elapsed_us = 0U;
      dm_control_stats.rt_max_lag = 0.0f;
    }
  }
}

BaseType_t AppTasks_Create(void)
{
  if (xTaskCreate(AppTasks_ControlTask, "dm_ctrl", DM_CONTROL_TASK_STACK, NULL,
                  DM_CONTROL_TASK_PRIORITY, &dm_control_task_handle) != pdPASS)
    return pdFAIL;
  if (xTaskCreate(AppTasks_StatusTask, "dm_stat", DM_STATUS_TASK_STACK, NULL,
                  DM_STATUS_TASK_PRIORITY, &dm_status_task_handle) != pdPASS)
    return pdFAIL;
  return pdPASS;
}
