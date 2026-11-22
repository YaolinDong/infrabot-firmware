/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_runtime.h"
#include "eth_state.h"
#include "app_damiao.h"
#include "dm_protocol.h"
#include "app_state.h"
#include "app_ethernet.h"
#include "app_persist.h"
#include "app_control_mode.h"
#include "app_tasks.h"
#include "app_gripper.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* DMAR: 0=verified/cache, 1=CAN queued (not a physical acknowledgement),
 * 2=CAN failure/timeout, 3=argument, 4=busy, 5=disable first,
 * 6=mode/readback mismatch, 7=no fresh feedback, 8=protected register. */
enum { IDLE, REGISTER_DELAY, REGISTER_WAIT, ENABLE_WAIT, ZERO_DELAY,
       ZERO_WAIT, STORE_DELAY };
static volatile uint8_t stage;
static uint8_t op, type;
static uint8_t local_operation, local_status;
static volatile uint8_t motor, rid;
static uint32_t seq, expected, started, feedback_before;
static volatile uint8_t register_ready;
static volatile uint32_t register_value;
static uint8_t register_check;
static uint8_t arm_mode;
static uint8_t arm_index;
static uint8_t gain_read_mask;
/* Wait per KP_APR read when applying the position-velocity profile. */
#define GAIN_READ_TIMEOUT_MS 20U
/* Parameter self-check, the last step of SET_ARM_MODE: every drive's CAN IDs,
 * mode and MIT ranges read back and compared with the firmware's table
 * (dm_motors[]: the MIT encoding and the telemetry scaling use it).  A
 * mismatch or a drive that does not answer leaves the profile not ready, so
 * the arm cannot be enabled. */
static const uint8_t selfcheck_rid[] = {INFRABOT_RID_MST_ID, INFRABOT_RID_ESC_ID,
                                        INFRABOT_RID_CTRL_MODE, INFRABOT_RID_PMAX,
                                        INFRABOT_RID_VMAX, INFRABOT_RID_TMAX};
#define SELFCHECK_REGISTERS ((uint8_t)(sizeof(selfcheck_rid) / sizeof(selfcheck_rid[0])))
static uint8_t selfchecking;   /* SET_ARM_MODE is in its self-check */
static uint8_t selfcheck_step; /* drive * SELFCHECK_REGISTERS + register */
static uint8_t selfcheck_failures, selfcheck_drives;
/* The result (INFRABOT_SELFCHECK_*) and its first failure, for telemetry. */
static volatile uint8_t selfcheck_result = INFRABOT_SELFCHECK_NOT_RUN;
static volatile uint8_t selfcheck_joint = INFRABOT_NO_JOINT, selfcheck_register;
static volatile uint32_t selfcheck_value, selfcheck_expected;

static uint8_t mode_allowed_for_motor(uint8_t index, uint8_t mode)
{
  /* The batch arm profile uses MIT/POS_VEL for M1..M7, but the low-rate
   * per-motor MotorBridge interface still exposes velocity mode for a
   * deliberately selected single joint.  M8 remains force-position only. */
  if (index >= DM_MOTOR_COUNT) return 0U;
  return (index == DM_GRIPPER_INDEX) ?
      (mode == DM_ETH_MODE_FORCE_POS) :
      (mode == DM_ETH_MODE_MIT || mode == DM_ETH_MODE_POS_VEL ||
       mode == DM_ETH_MODE_VEL);
}

static uint8_t arm_mode_for_motor(uint8_t index)
{
  return (index == DM_GRIPPER_INDEX) ? DM_ETH_MODE_FORCE_POS : arm_mode;
}

uint16_t AppDamiao_ModeId(uint8_t mode)
{
  return (mode >= 1U && mode <= 4U) ? (uint16_t)((mode - 1U) * 0x100U) : DM_MODE_MIT;
}

uint8_t AppDamiao_RegisterRx(const DM_CanRxFrame_t *frame)
{
  if (frame->length != 8U || (frame->data[2] != 0x33U && frame->data[2] != 0x55U))
    return 0U;
  /* A normal 8-byte motor feedback frame uses the same master arbitration ID
   * as a register reply, and its position low byte can legitimately equal
   * 0x33 or 0x55.  Only classify such a frame as a register response while a
   * maintenance transaction is actually waiting (or when it is addressed to
   * the broadcast register ID 0x7FF).  Otherwise it must continue through
   * DM_Motor_ProcessFeedback(); dropping it can make one joint appear stale
   * during a move. */
  if (!eth_maintenance.waiting && frame->id != 0x7FFU)
    return 0U;
  for (uint8_t i = 0; i < DM_MOTOR_COUNT; ++i)
  {
    /* Register replies use the feedback arbitration ID.  Some firmware
       revisions echo motor_id in data[0] and reserve data[1] (others echo the
       two-byte ID), so never compare data[0:2] as a little-endian u16. */
    if (frame->id != dm_motors[i].master_id &&
        frame->id != 0x7FFU &&
        (frame->data[0] & 0x0FU) != (dm_motors[i].id & 0x0FU)) continue;
    if (eth_maintenance.waiting && stage == REGISTER_WAIT && motor == i &&
        frame->data[2] == 0x33U && frame->data[3] == rid)
    {
      register_value = DM_RegisterValue(frame->data);
      __DMB();
      register_ready = 1U;
    }
    return 1U; /* A parameter response must never become a position sample. */
  }
  return 0U;
}

static void finish(uint8_t status, uint32_t value)
{
  if (local_operation) { local_status = status; local_operation = 0U; }
  else AppEthernet_QueueMaintenanceResult(seq, op, motor, status, type, rid, value);
  stage = IDLE;
  eth_maintenance.waiting = 0U;
}

static void update_enabled(void)
{
  dm_motor_runtime.control_enabled = 0U;
  for (uint8_t i = 0; i < DM_MOTOR_COUNT; ++i)
    if (dm_motor_runtime.enabled[i]) dm_motor_runtime.control_enabled = 1U;
}

void AppDamiao_Cancel(void)
{
  if (stage != IDLE) finish(4U, 0U);
  eth_maintenance.pending = 0U;
}

void AppDamiao_CancelMotors(uint32_t mask)
{
  /* The operation in flight on one of these drives (a single-drive ENABLE
   * would re-enable it after a group disable), and one still pending for
   * it; other drives' operations, e.g. a control mode switch, carry on. */
  if (stage != IDLE && motor < DM_MOTOR_COUNT && (mask & (1UL << motor)) != 0U) finish(4U, 0U);
  taskENTER_CRITICAL();
  const uint8_t target = eth_maintenance.motor_index;
  const uint8_t drop = eth_maintenance.pending != 0U && target < DM_MOTOR_COUNT &&
                       (mask & (1UL << target)) != 0U;
  const uint32_t sequence = eth_maintenance.sequence;
  const uint8_t command = eth_maintenance.command;
  if (drop) eth_maintenance.pending = 0U;
  taskEXIT_CRITICAL();
  if (drop) AppEthernet_QueueMaintenanceResult(sequence, command, target, 4U, 0U, 0U, 0U);
}

static float joint_position(uint8_t i)
{
  return (dm_motors[i].position - dm_motor_runtime.zero_offset[i]) *
         DM_MOTOR_DIRECTION(i);
}

static void hold(uint8_t i)
{
  dm_motor_runtime.command_position[i] = dm_motors[i].position;
  eth_udp_command.target_position[i] = joint_position(i);
  eth_udp_command.target_velocity[i] = 0.0f;
  eth_udp_command.target_torque[i] = 0.0f;
  dm_motor_runtime.override[i] = 1U;
}

/* Registers, their types and write ranges: the table of infrabot/protocol.h
 * (only this file holds a copy of it). */
uint8_t AppDamiao_RegisterIsUint(uint8_t rid)
{
  const InfrabotRegisterInfo *info = infrabot_register_info(rid);
  return info != NULL && info->type == INFRABOT_REGISTER_U32;
}

static void begin_read(void)
{
  taskENTER_CRITICAL();
  register_ready = 0U;
  stage = REGISTER_WAIT;
  HAL_StatusTypeDef result = DM_Motor_ReadRegister(&hfdcan2, &dm_motors[motor], rid);
  taskEXIT_CRITICAL();
  if (result != HAL_OK) finish(2U, 0U);
}

/* A position-velocity joint lags by gear ratio / KP_APR (ArmDriveLead), so
 * keep the value each drive reports. */
static void learn_position_gain(uint8_t index, uint32_t raw)
{
  float gain;
  memcpy(&gain, &raw, sizeof(gain));
  if (index < DM_MOTOR_COUNT && isfinite(gain) && gain >= 0.0f) dm_posvel_kp_apr[index] = gain;
}

/* The register value the firmware expects of drive i (raw u32/f32 bits). */
static uint32_t selfcheck_expected_raw(uint8_t i, uint8_t r)
{
  float value;
  switch (r)
  {
    case INFRABOT_RID_MST_ID: return dm_motors[i].master_id;
    case INFRABOT_RID_ESC_ID: return dm_motors[i].id;
    case INFRABOT_RID_CTRL_MODE: return arm_mode_for_motor(i);
    case INFRABOT_RID_PMAX: value = dm_motors[i].position_max; break;
    case INFRABOT_RID_VMAX: value = dm_motors[i].velocity_max; break;
    default: value = dm_motors[i].torque_max; break;
  }
  uint32_t raw;
  memcpy(&raw, &value, sizeof(raw));
  return raw;
}

static uint8_t selfcheck_matches(uint8_t r, uint32_t read, uint32_t expected)
{
  if (AppDamiao_RegisterIsUint(r)) return read == expected;
  float a, b;
  memcpy(&a, &read, sizeof(a));
  memcpy(&b, &expected, sizeof(b));
  return isfinite(a) && fabsf(a - b) <= 1.0e-3f * fmaxf(1.0f, fabsf(b));
}

/* newlib-nano has no float printf: f32 registers in thousandths. */
static void selfcheck_print_value(uint8_t r, uint32_t raw)
{
  if (AppDamiao_RegisterIsUint(r)) { printf("%lu (0x%lX)", (unsigned long)raw, (unsigned long)raw); return; }
  float value;
  memcpy(&value, &raw, sizeof(value));
  if (!isfinite(value)) { printf("non-finite"); return; }
  const long milli = lroundf(value * 1000.0f);
  printf("%s%ld.%03ld", milli < 0 ? "-" : "", labs(milli) / 1000L, labs(milli) % 1000L);
}

static void selfcheck_fail(uint8_t i, uint8_t r, uint8_t result, uint32_t read, uint32_t expected)
{
  const InfrabotRegisterInfo *info = infrabot_register_info(r);
  printf("SELFCHECK: J%u %s ", (unsigned)i + 1U, info != NULL ? info->name : "?");
  if (result == INFRABOT_SELFCHECK_NO_REPLY) printf("no reply");
  else { printf("read "); selfcheck_print_value(r, read); }
  printf(" expected ");
  selfcheck_print_value(r, expected);
  printf("\r\n");
  if (selfcheck_failures++ != 0U) return;
  selfcheck_result = result;
  selfcheck_joint = i;
  selfcheck_register = r;
  selfcheck_value = read;
  selfcheck_expected = expected;
}

static void finish_selfcheck(void)
{
  selfchecking = 0U;
  motor = DM_ETH_MOTOR_ALL;
  rid = DM_RID_CTRL_MODE;
  if (selfcheck_failures == 0U)
  {
    selfcheck_result = INFRABOT_SELFCHECK_OK;
    printf("SELFCHECK: OK, %u drives: MST_ID, ESC_ID, CTRL_MODE, PMAX/VMAX/TMAX as configured\r\n",
           (unsigned)selfcheck_drives);
    dm_motor_runtime.profile_ready = 1U;
    finish(0U, arm_mode);
    return;
  }
  printf("SELFCHECK: %u mismatch(es): motor profile not ready, enable refused; "
         "rewrite the drive parameters (motor template) or correct the firmware table\r\n",
         (unsigned)selfcheck_failures);
  dm_motor_runtime.profile_ready = 0U;
  finish(INFRABOT_REPLY_MISMATCH,
         INFRABOT_SELFCHECK_REPLY_VALUE(selfcheck_joint, selfcheck_register, selfcheck_result));
}

/* Reads the next register of the self-check, or ends it.  J8 is checked only
 * when it is connected (it has answered since boot). */
static void selfcheck_next(void)
{
  while (selfcheck_step < DM_MOTOR_COUNT * SELFCHECK_REGISTERS)
  {
    const uint8_t i = (uint8_t)(selfcheck_step / SELFCHECK_REGISTERS);
    if (i >= DM_ARM_MOTOR_COUNT && dm_motors[i].feedback_count == 0U)
    {
      selfcheck_step = (uint8_t)((i + 1U) * SELFCHECK_REGISTERS);
      continue;
    }
    if (selfcheck_step % SELFCHECK_REGISTERS == 0U) selfcheck_drives++;
    motor = i;
    rid = selfcheck_rid[selfcheck_step % SELFCHECK_REGISTERS];
    started = HAL_GetTick();
    taskENTER_CRITICAL();
    register_ready = 0U;
    stage = REGISTER_WAIT;
    const HAL_StatusTypeDef result = DM_Motor_ReadRegister(&hfdcan2, &dm_motors[motor], rid);
    taskEXIT_CRITICAL();
    if (result == HAL_OK) return;
    selfcheck_fail(i, rid, INFRABOT_SELFCHECK_NO_REPLY, 0U, selfcheck_expected_raw(i, rid));
    selfcheck_step++;
  }
  finish_selfcheck();
}

/* After the CTRL_MODE writes (and the KP_APR reads): the self-check. */
static void finish_arm_mode(void)
{
  if (arm_mode == DM_ETH_MODE_POS_VEL)
  {
    printf("ARM_MODE: position-velocity KP_APR");
    for (uint8_t i = 0U; i < DM_ARM_MOTOR_COUNT; ++i)
      printf("%c%ld%s", i ? ',' : '=', lroundf(dm_posvel_kp_apr[i]),
             (gain_read_mask & (1U << i)) ? "" : "(no reply)");
    printf("\r\n");
  }
  selfchecking = 1U;
  selfcheck_step = selfcheck_failures = selfcheck_drives = 0U;
  selfcheck_next();
}

void AppDamiao_ReportSelfCheck(InfrabotSafetyStatus *status)
{
  status->selfcheck = selfcheck_result;
  status->selfcheck_joint = selfcheck_joint;
  status->selfcheck_register = selfcheck_register;
  status->selfcheck_value = selfcheck_value;
  status->selfcheck_expected = selfcheck_expected;
}

/* After a position-velocity profile: read KP_APR of J1-J7 in turn.  A drive
 * that does not answer keeps its previous value; the profile still applies. */
static void read_next_gain(void)
{
  for (; arm_index < DM_ARM_MOTOR_COUNT; ++arm_index)
  {
    motor = arm_index;
    rid = DM_RID_KP_APR;
    started = HAL_GetTick();
    taskENTER_CRITICAL();
    register_ready = 0U;
    stage = REGISTER_WAIT;
    const HAL_StatusTypeDef result = DM_Motor_ReadRegister(&hfdcan2, &dm_motors[motor], rid);
    taskEXIT_CRITICAL();
    if (result == HAL_OK) return;
  }
  finish_arm_mode();
}

static void poll_operation(void)
{
  if (stage == IDLE) return;
  const uint32_t elapsed = HAL_GetTick() - started;
  if (elapsed > 500U) { finish(2U, 0U); return; }
  if (stage == REGISTER_DELAY && elapsed >= 10U)
  {
    if (op == DM_ETH_CMD_SET_ARM_MODE)
    {
      /* CTRL_MODE writes are serialized deliberately.  Some drive firmware
       * revisions acknowledge the write on CAN but do not emit a register
       * readback frame, so a readback-based batch would time out even though
       * the write succeeded. Keep the selected profile locally after each
       * accepted CAN enqueue; enable must not rewrite CTRL_MODE. This setup
       * acknowledgement is not independent physical mode verification. */
      eth_udp_command.motor_mode[motor] = arm_mode_for_motor(motor);
      arm_index++;
      if (arm_index < DM_MOTOR_COUNT)
      {
        motor = arm_index;
        expected = arm_mode_for_motor(motor);
        uint8_t bytes[4];
        memcpy(bytes, &expected, sizeof(expected));
        if (DM_Motor_WriteRegister(&hfdcan2, &dm_motors[motor],
                                   DM_RID_CTRL_MODE, bytes) != HAL_OK)
        {
          finish(2U, 0U);
          return;
        }
        started = HAL_GetTick();
        return;
      }
      if (arm_mode == DM_ETH_MODE_POS_VEL)
      {
        arm_index = 0U;
        gain_read_mask = 0U;
        read_next_gain();
        return;
      }
      finish_arm_mode();
      return;
    }
    begin_read();
    return;
  }
  if (op == DM_ETH_CMD_SET_ARM_MODE && stage == REGISTER_WAIT && selfchecking)
  {
    if (!register_ready && elapsed < DM_SELFCHECK_READ_TIMEOUT_MS) return;
    const uint32_t wanted = selfcheck_expected_raw(motor, rid);
    if (!register_ready)
      selfcheck_fail(motor, rid, INFRABOT_SELFCHECK_NO_REPLY, 0U, wanted);
    else if (!selfcheck_matches(rid, register_value, wanted))
      selfcheck_fail(motor, rid, INFRABOT_SELFCHECK_MISMATCH, register_value, wanted);
    selfcheck_step++;
    selfcheck_next();
    return;
  }
  if (op == DM_ETH_CMD_SET_ARM_MODE && stage == REGISTER_WAIT)
  {
    if (!register_ready && elapsed < GAIN_READ_TIMEOUT_MS) return;
    if (register_ready)
    {
      learn_position_gain(motor, register_value);
      gain_read_mask |= (uint8_t)(1U << motor);
    }
    arm_index++;
    read_next_gain();
    return;
  }
  if (stage == REGISTER_WAIT && register_ready)
  {
    const uint32_t value = register_value;
    if (register_check && value != expected) { finish(6U, value); return; }
    if (rid == 10U && value >= 1U && value <= 4U) {
      if (eth_udp_command.motor_mode[motor] != value) {
        dm_motor_runtime.profile_ready = 0U;
        if (dm_motor_runtime.control_enabled) {
          /* A diagnostic read must not silently switch the running CAN
           * encoder. Stop on a drive/profile mismatch; reconfigure disabled. */
          eth_udp_command.control_request = DM_ETH_CMD_DISABLE;
          finish(6U, value);
          return;
        }
      }
      eth_udp_command.motor_mode[motor] = (uint8_t)value;
    }
    if (rid == DM_RID_KP_APR) learn_position_gain(motor, value);

    finish(0U, value);
  }
  else if (stage == ENABLE_WAIT && dm_motors[motor].feedback_count != feedback_before)
  {
    if (dm_motors[motor].state > 1U) { finish(7U, dm_motors[motor].state); return; }
    hold(motor);
    /* Remain IDLE after enable; switching mode explicitly grants RT ownership. */
    dm_motor_runtime.override[motor] = 0U;
    dm_motor_runtime.stopped[motor] = 0U;
    /* CTRL_MODE was selected before enable; never rewrite it while arming. */
    if (DM_Motor_Enable(&hfdcan2, &dm_motors[motor],
                       AppDamiao_ModeId(eth_udp_command.motor_mode[motor])) != HAL_OK)
      { finish(2U, 0U); return; }
    dm_motor_runtime.enabled[motor] = 1U;
    update_enabled();
    eth_udp_command.last_tick = HAL_GetTick();
    eth_udp_command.active = 1U;
    finish(1U, 0U);
  }
  else if (stage == ZERO_DELAY && elapsed >= 35U)
  {
    feedback_before = dm_motors[motor].feedback_count;
    stage = ZERO_WAIT;
    if (DM_Motor_RequestFeedback(&hfdcan2, &dm_motors[motor]) != HAL_OK) finish(2U, 0U);
  }
  else if (stage == ZERO_WAIT && dm_motors[motor].feedback_count != feedback_before)
  {
    if (fabsf(dm_motors[motor].position) > 0.05f) { finish(6U, 0U); return; }
    dm_motor_runtime.zero_offset[motor] = 0.0f;
    hold(motor);
    dm_motor_runtime.zero_valid = 1U;
    /* Only reachable with ALL motors disabled; motor 0xFE persists the actual
     * zero. The MCU record preserves this calibration together with the
     * currently selected arm control profile. */
    (void)AppPersist_SaveZero();
    finish(0U, 0U);
  }
  else if (stage == STORE_DELAY && elapsed >= 35U)
    finish(1U, 0U); /* No motor flash-complete response in this protocol. */
}

static void AppDamiao_ProcessControlProfile(void)
{
  uint32_t profile_sequence;
  taskENTER_CRITICAL();
  if (eth_maintenance.profile_pending == 0U)
  {
    taskEXIT_CRITICAL();
    return;
  }
  profile_sequence = eth_maintenance.profile_sequence;
  eth_maintenance.profile_pending = 0U;
  taskEXIT_CRITICAL();

  if (stage != IDLE)
  {
    AppEthernet_QueueMaintenanceResult(profile_sequence,
                                        DM_ETH_CMD_STORE_CONTROL_PROFILE,
                                        DM_ETH_MOTOR_ALL, 4U, 0U, 0U, 0U);
    return;
  }
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; ++i)
  {
    if (dm_motor_runtime.enabled[i] != 0U)
    {
      AppEthernet_QueueMaintenanceResult(profile_sequence,
                                          DM_ETH_CMD_STORE_CONTROL_PROFILE,
                                          DM_ETH_MOTOR_ALL, 5U, 0U, 0U, 0U);
      return;
    }
    if (!isfinite(eth_udp_command.target_kp[i]) || eth_udp_command.target_kp[i] < 0.0f ||
        eth_udp_command.target_kp[i] > 500.0f ||
        !isfinite(eth_udp_command.target_kd[i]) || eth_udp_command.target_kd[i] < 0.0f ||
        eth_udp_command.target_kd[i] > 5.0f ||
        !isfinite(eth_udp_command.target_velocity[i]) ||
        fabsf(eth_udp_command.target_velocity[i]) > dm_motors[i].velocity_max ||
        !isfinite(eth_udp_command.target_torque[i]) ||
        fabsf(eth_udp_command.target_torque[i]) > dm_motors[i].torque_max ||
        !isfinite(eth_udp_command.velocity_limit[i]) ||
        eth_udp_command.velocity_limit[i] < 0.0f ||
        eth_udp_command.velocity_limit[i] > dm_motors[i].velocity_max ||
        !isfinite(eth_udp_command.torque_limit_ratio[i]) ||
        eth_udp_command.torque_limit_ratio[i] < 0.0f ||
        eth_udp_command.torque_limit_ratio[i] > 1.0f)
    {
      AppEthernet_QueueMaintenanceResult(profile_sequence,
                                          DM_ETH_CMD_STORE_CONTROL_PROFILE,
                                          DM_ETH_MOTOR_ALL, 3U, 0U, 0U, i + 1U);
      return;
    }
  }
  if (AppPersist_SaveControlProfile() == 0U)
  {
    AppEthernet_QueueMaintenanceResult(profile_sequence,
                                        DM_ETH_CMD_STORE_CONTROL_PROFILE,
                                        DM_ETH_MOTOR_ALL, 2U, 0U, 0U, 0U);
    return;
  }
  printf("CTRL_PROFILE: request saved sequence=%lu\r\n",
         (unsigned long)profile_sequence);
  AppEthernet_QueueMaintenanceResult(profile_sequence,
                                      DM_ETH_CMD_STORE_CONTROL_PROFILE,
                                      DM_ETH_MOTOR_ALL, 0U, 0U, 0U, 1U);
}

/* Shared by the UDP setup operation and the LCD enable sequence. */
static void begin_arm_profile(uint8_t mode)
{
    arm_mode = mode;
    dm_motor_runtime.profile_ready = 0U; /* Partial CAN failure must not authorize enable. */
    arm_index = 0U;
    selfchecking = 0U;
    selfcheck_result = INFRABOT_SELFCHECK_NOT_RUN;
    selfcheck_joint = INFRABOT_NO_JOINT;
    selfcheck_register = 0U;
    selfcheck_value = selfcheck_expected = 0U;
    /* Do not queue 32 neutral cyclic frames here.  The FDCAN TX FIFO is
     * intentionally small and those frames could consume all slots before
     * the first register write, making the batch report status=2.  All drives
     * are disabled at this point, so changing CTRL_MODE needs only the
     * serialized register transactions below; the 1 kHz task will emit the
     * correct mode frame after enable. */
    motor = arm_index;
    rid = DM_RID_CTRL_MODE;
    type = DM_ETH_REGISTER_U32;
    expected = arm_mode_for_motor(motor);
    register_check = 1U;
    uint8_t bytes[4];
    memcpy(bytes, &expected, sizeof(expected));
    if (DM_Motor_WriteRegister(&hfdcan2, &dm_motors[motor], rid, bytes) != HAL_OK)
      { finish(2U, 0U); return; }
    stage = REGISTER_DELAY;
}

uint8_t AppDamiao_StartLocalProfile(uint8_t mode)
{
  if (stage != IDLE || eth_maintenance.pending || eth_maintenance.waiting ||
      eth_maintenance.profile_pending || AppTasks_Enabling() || eth_udp_command.control_request)
    return INFRABOT_REPLY_BUSY;
  if (mode != DM_ETH_MODE_MIT && mode != DM_ETH_MODE_POS_VEL) return INFRABOT_REPLY_INVALID;
  if (dm_motor_runtime.control_enabled || eth_udp_command.control_mode != DM_ETH_CONTROL_IDLE) return INFRABOT_REPLY_DISABLE_FIRST;
  local_operation = 1U;
  local_status = INFRABOT_REPLY_BUSY;
  op = DM_ETH_CMD_SET_ARM_MODE;
  started = HAL_GetTick();
  eth_maintenance.waiting = 1U;
  begin_arm_profile(mode);
  return 0U; /* accepted; result is asynchronous */
}
uint8_t AppDamiao_LocalProfileStatus(void) { return local_status; }

void AppDamiao_Process(void)
{
  poll_operation();
  AppDamiao_ProcessControlProfile();
  if (!eth_maintenance.pending) return;
  /* DISABLE can cancel any in-flight operation. Other requests get BUSY at
   * ingress, so neither a register waiter nor a flash operation is replaced. */
  if (stage != IDLE && eth_maintenance.command != DM_ETH_CMD_DISABLE) return;
  if (stage != IDLE) finish(4U, 0U);
  taskENTER_CRITICAL();
  op = eth_maintenance.command;
  motor = eth_maintenance.motor_index;
  rid = eth_maintenance.register_id;
  type = eth_maintenance.value_type;
  seq = eth_maintenance.sequence;
  uint32_t value = eth_maintenance.value;
  const uint8_t mode = eth_maintenance.mode;
  const float pos = eth_maintenance.position_mrad / 1000.0f;
  const float vel = eth_maintenance.velocity_mrad_s / 1000.0f;
  const float kp = eth_maintenance.kp_milli / 1000.0f;
  const float kd = eth_maintenance.kd_milli / 1000.0f;
  const float torque = eth_maintenance.torque_mNm / 1000.0f;
  const float limit = eth_maintenance.velocity_limit_mrad_s / 1000.0f;
  const float ratio = eth_maintenance.torque_ratio_milli / 1000.0f;
  eth_maintenance.pending = 0U;
  eth_maintenance.waiting = 1U;
  taskEXIT_CRITICAL();
  started = HAL_GetTick();
  if ((AppTasks_Enabling() || eth_udp_command.control_request) &&
      op != DM_ETH_CMD_DISABLE && op != DM_ETH_CMD_GET_STATE)
    { finish(4U, 0U); return; }
  if (motor == DM_ETH_MOTOR_ALL && op != DM_ETH_CMD_DISABLE && op != DM_ETH_CMD_STOP &&
      op != DM_ETH_CMD_GET_STATE && op != DM_ETH_CMD_REQUEST_FEEDBACK &&
      op != DM_ETH_CMD_SET_ARM_MODE && op != DM_ETH_CMD_SET_CONTROL_MODE)
    { finish(3U, 0U); return; }
  if (op == DM_ETH_CMD_GRIPPER)
  {
    /* Any control mode, also while a move owns the arm. */
    const uint8_t status = motor == DM_GRIPPER_INDEX ? AppGripper_Command(pos, limit, ratio) : 3U;
    if (status == 0U) eth_udp_command.last_tick = HAL_GetTick();
    finish(status, 0U);
    return;
  }
  if (op == DM_ETH_CMD_GET_HOLD_POSITION)
  {
    const float joint = (dm_motor_runtime.command_position[motor] - dm_motor_runtime.zero_offset[motor]) *
                        DM_MOTOR_DIRECTION(motor);
    finish(dm_motor_runtime.enabled[motor] ? 0U : 7U, (uint32_t)(int32_t)lroundf(joint * 1000.0f));
    return;
  }

  if (op == DM_ETH_CMD_DISABLE || op == DM_ETH_CMD_STOP ||
      op == DM_ETH_CMD_REQUEST_FEEDBACK || op == DM_ETH_CMD_GET_STATE)
  {
    uint8_t status = 1U;
    uint32_t mask = 0U;
    for (uint8_t i = 0; i < DM_MOTOR_COUNT; ++i)
    {
      if (motor != DM_ETH_MOTOR_ALL && motor != i) continue;
      HAL_StatusTypeDef result = HAL_OK;
      if (op == DM_ETH_CMD_DISABLE)
      {
        dm_motor_runtime.enabled[i] = 0U;
        dm_motor_runtime.override[i] = 1U;
        result = DM_Motor_Disable(&hfdcan2, &dm_motors[i], AppDamiao_ModeId(eth_udp_command.motor_mode[i]));
      }
      else if (op == DM_ETH_CMD_STOP)
      {
        hold(i);
        dm_motor_runtime.stopped[i] = 1U;
      }
      else if (op == DM_ETH_CMD_REQUEST_FEEDBACK)
        result = DM_Motor_RequestFeedback(&hfdcan2, &dm_motors[i]);
      else
      {
        status = 0U; /* Cached state is carried in the next DMTL packet. */
        if (dm_motors[i].feedback_count && HAL_GetTick() - dm_motors[i].feedback_tick <= 100U)
          mask |= 1UL << i;
      }
      if (result != HAL_OK) status = 2U;
    }
    update_enabled();
    if (op == DM_ETH_CMD_DISABLE && !dm_motor_runtime.control_enabled) eth_udp_command.control_mode = DM_ETH_CONTROL_IDLE;
    finish(status, mask);
    return;
  }
  if (op == DM_ETH_CMD_SET_CONTROL_MODE)
  {
    if (motor != DM_ETH_MOTOR_ALL) { finish(3U, eth_udp_command.control_mode); return; }
    const uint8_t result = AppControlMode_Switch(mode);
    printf("CONTROL_MODE: requested=%u actual=%u status=%u profile=%u\r\n",
           mode, eth_udp_command.control_mode, result, AppControlMode_Profile());
    finish(result, eth_udp_command.control_mode);
    return;
  }
  if (op == DM_ETH_CMD_ENABLE)
  {
    if (eth_udp_command.control_mode != DM_ETH_CONTROL_IDLE || !AppControlMode_ProfileReady())
      { finish(6U, eth_udp_command.control_mode); return; }
    if (dm_motor_runtime.enabled[motor]) { finish(1U, 0U); return; }
    feedback_before = dm_motors[motor].feedback_count;
    stage = ENABLE_WAIT;
    if (DM_Motor_RequestFeedback(&hfdcan2, &dm_motors[motor]) != HAL_OK) finish(2U, 0U);
    return;
  }
  if (op == DM_ETH_CMD_CLEAR_ERROR)
  {
    if (dm_motor_runtime.enabled[motor]) { finish(5U, 0U); return; }
    finish(DM_Motor_ClearError(&hfdcan2, &dm_motors[motor],
           AppDamiao_ModeId(eth_udp_command.motor_mode[motor])) == HAL_OK ? 1U : 2U, 0U);
    return;
  }
  if (op >= DM_ETH_CMD_SEND_MIT && op <= DM_ETH_CMD_SEND_FORCE_POS)
  {
    if (eth_udp_command.control_mode != DM_ETH_CONTROL_RT_JOINT)
      { finish(6U, eth_udp_command.control_mode); return; }
    const uint8_t wanted = op - DM_ETH_CMD_SEND_MIT + 1U;
    if (!mode_allowed_for_motor(motor, wanted))
    {
      finish(3U, 0U);
      return;
    }
    if (eth_udp_command.motor_mode[motor] != wanted)
    {
      /* Return the mode currently selected by the MCU so the GUI/log can
       * distinguish a real CAN failure from a mode-selection mismatch. */
      finish(6U, eth_udp_command.motor_mode[motor]);
      return;
    }
    if (!dm_motor_runtime.enabled[motor]) { finish(7U, 0U); return; }
    const float pmin = motor == DM_GRIPPER_INDEX ? DM_GRIPPER_POSITION_MIN : -dm_motors[motor].position_max;
    const float pmax = motor == DM_GRIPPER_INDEX ? DM_GRIPPER_POSITION_MAX : dm_motors[motor].position_max;
    const float speed_cap = motor < DM_ARM_MOTOR_COUNT ?
        fminf(dm_motors[motor].velocity_max,DM_ARM_MAX_SPEED_RAD_S) : dm_motors[motor].velocity_max;
    if ((wanted != DM_ETH_MODE_VEL && (pos < pmin || pos > pmax)) ||
        fabsf(vel) > speed_cap || fabsf(torque) > dm_motors[motor].torque_max ||
        kp < 0 || kp > 500 || kd < 0 || kd > 5 || limit < 0 ||
        limit > speed_cap || ratio < 0 || ratio > 1)
      { finish(3U, 0U); return; }
    taskENTER_CRITICAL();
    dm_motor_runtime.override[motor] = 1U;
    dm_motor_runtime.stopped[motor] = 0U;
    eth_udp_command.target_position[motor] = pos;
    /* A v4 one-shot target owns this joint until another maintenance command
     * or an explicit PC stream takes ownership.  Do not wait for the legacy
     * endpoint trajectory to chase the target: convert the PC joint
     * coordinate to the encoder coordinate here and publish it directly to
     * the 1 kHz CAN owner. */
    if (wanted != DM_ETH_MODE_VEL)
    {
      const float direction = DM_MOTOR_DIRECTION(motor);
      float command_position = dm_motor_runtime.zero_offset[motor] + direction * pos;
      const float pmin = (motor == DM_GRIPPER_INDEX) ?
          (dm_motor_runtime.zero_offset[motor] + direction * DM_GRIPPER_POSITION_MIN) :
          -dm_motors[motor].position_max;
      const float pmax = (motor == DM_GRIPPER_INDEX) ?
          (dm_motor_runtime.zero_offset[motor] + direction * DM_GRIPPER_POSITION_MAX) :
          dm_motors[motor].position_max;
      const float lo = (pmin < pmax) ? pmin : pmax;
      const float hi = (pmin > pmax) ? pmin : pmax;
      if (command_position < lo) command_position = lo;
      if (command_position > hi) command_position = hi;
      dm_motor_runtime.command_position[motor] = command_position;
    }
    eth_udp_command.target_velocity[motor] = vel;
    if (wanted == DM_ETH_MODE_MIT)
    {
      eth_udp_command.target_kp[motor] = kp;
      eth_udp_command.target_kd[motor] = kd;
      eth_udp_command.target_torque[motor] = torque;
    }
    if (wanted == DM_ETH_MODE_POS_VEL || wanted == DM_ETH_MODE_FORCE_POS)
      eth_udp_command.velocity_limit[motor] = limit;
    if (wanted == DM_ETH_MODE_FORCE_POS) eth_udp_command.torque_limit_ratio[motor] = ratio;
    eth_udp_command.active = 1U;
    eth_udp_command.last_tick = HAL_GetTick();
    taskEXIT_CRITICAL();
    finish(1U, 0U);
    return;
  }
  /* All setup mutations, including individual ENSURE_MODE, require disabled
   * IDLE. A per-motor interface must not bypass the arm profile interlock. */
  if (op != DM_ETH_CMD_GET_REGISTER &&
      (dm_motor_runtime.control_enabled || eth_udp_command.control_mode != DM_ETH_CONTROL_IDLE))
    { finish(5U, 0U); return; }
  if (op == DM_ETH_CMD_SET_ZERO || op == DM_ETH_CMD_STORE_PARAMETERS)
  {
    /* The high nibble cached in DM_Motor_t::state comes from the drive's
     * feedback/error field; it is not the MCU-side enable latch.  In
     * particular, a valid disabled drive may report a non-zero code here.
     * Use dm_motor_runtime.enabled[] for the safety gate.  Zeroing still needs a
     * fresh position sample, but saving parameters is a write-only flash
     * command: after the preceding J1 transactions a disabled drive may no
     * longer have a feedback sample newer than 500 ms. */
    if (motor >= DM_MOTOR_COUNT)
      { finish(3U, 0U); return; }
    if (dm_motor_runtime.enabled[motor] != 0U)
      { finish(5U, 0U); return; }
    if (op == DM_ETH_CMD_SET_ZERO &&
        (!dm_motors[motor].feedback_count ||
         HAL_GetTick() - dm_motors[motor].feedback_tick > 500U))
      { finish(7U, 0U); return; }
    HAL_StatusTypeDef result = op == DM_ETH_CMD_SET_ZERO ?
      DM_Motor_SetZeroPosition(&hfdcan2, &dm_motors[motor], AppDamiao_ModeId(eth_udp_command.motor_mode[motor])) :
      DM_Motor_SaveParameters(&hfdcan2, &dm_motors[motor]);
    stage = op == DM_ETH_CMD_SET_ZERO ? ZERO_DELAY : STORE_DELAY;
    if (result != HAL_OK) finish(2U, 0U);
    return;
  }
  if (op == DM_ETH_CMD_ENSURE_MODE)
  {
    if (motor >= DM_MOTOR_COUNT || mode < DM_ETH_MODE_MIT || mode > DM_ETH_MODE_FORCE_POS)
      { finish(3U, 0U); return; }
    if (!mode_allowed_for_motor(motor, mode))
      { finish(3U, 0U); return; }

    /* Clear the other cyclic command channels before changing CTRL_MODE.
     * This mirrors the Damiao host implementation and prevents a stale
     * command from being interpreted during the mode transition. */
    const float hold_position = dm_motors[motor].position;
    (void)DM_Motor_MIT(&hfdcan2, &dm_motors[motor], hold_position, 0.0f, 0.0f, 0.0f, 0.0f);
    (void)DM_Motor_Position(&hfdcan2, &dm_motors[motor], hold_position, 0.0f);
    (void)DM_Motor_Speed(&hfdcan2, &dm_motors[motor], 0.0f);
    (void)DM_Motor_PositionCurrent(&hfdcan2, &dm_motors[motor], hold_position, 0.0f, 0.0f);
    rid = 10U;
    type = 0U;
    value = mode;
  }
  if (op == DM_ETH_CMD_SET_ARM_MODE)
  {
    if (motor != DM_ETH_MOTOR_ALL ||
        (mode != DM_ETH_MODE_MIT && mode != DM_ETH_MODE_POS_VEL))
      { finish(3U, 0U); return; }
    /* Mode changes are a setup operation.  Require all drives disabled so a
     * batch cannot leave a running joint with a half-switched profile. */
    if (dm_motor_runtime.control_enabled || eth_udp_command.control_mode != DM_ETH_CONTROL_IDLE)
      { finish(5U, 0U); return; }
    begin_arm_profile(mode);
    return;
  }
  if (op == DM_ETH_CMD_SET_CAN_TIMEOUT)
  {
    if (!value || value > 0xFFFFFFFFU / 20U) { finish(3U, 0U); return; }
    rid = 9U; type = 0U; value *= 20U;
  }
  if (infrabot_register_info(rid) == NULL || type != (AppDamiao_RegisterIsUint(rid) ? 0U : 1U))
    { finish(3U, 0U); return; }
  register_check = op != DM_ETH_CMD_GET_REGISTER;
  if (!register_check) { begin_read(); return; }
  if (!infrabot_register_write_allowed(rid, value)) { finish(8U, 0U); return; }
  if (rid == DM_RID_CTRL_MODE &&
      (motor >= DM_MOTOR_COUNT || !mode_allowed_for_motor(motor, (uint8_t)value)))
    { finish(8U, 0U); return; }
  if (rid == DM_RID_CTRL_MODE) dm_motor_runtime.profile_ready = 0U;
  expected = value;
  uint8_t bytes[4]; memcpy(bytes, &value, 4);
  if (DM_Motor_WriteRegister(&hfdcan2, &dm_motors[motor], rid, bytes) != HAL_OK)
    { finish(2U, 0U); return; }
  stage = REGISTER_DELAY; /* readback after the drive applies the write */
}
