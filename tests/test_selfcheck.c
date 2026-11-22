/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* The parameter self-check of SET_ARM_MODE (app_damiao.c): mocked drives
 * answer register reads from their register file; no HAL, network or RTOS. */
#include "app_damiao.h"
#include "app_state.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

MockMotor dm_motors[8];

FDCAN_HandleTypeDef hfdcan2;
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }

/* The other modules. */
uint8_t AppTasks_Enabling(void) { return 0U; }
uint8_t AppControlMode_Profile(void) { return eth_udp_command.motor_mode[0]; }
uint8_t AppControlMode_ProfileReady(void) { return dm_motor_runtime.profile_ready; }
uint8_t AppControlMode_Switch(uint8_t mode) { (void)mode; return 0U; }
uint8_t AppGripper_Command(float position, float speed, float force)
{ (void)position; (void)speed; (void)force; return 0U; }
uint8_t AppPersist_SaveZero(void) { return 1U; }
uint8_t AppPersist_SaveControlProfile(void) { return 1U; }
HAL_StatusTypeDef DM_Motor_Enable(FDCAN_HandleTypeDef *h, const DM_Motor_t *m, uint16_t mode)
{ (void)h; (void)m; (void)mode; return HAL_OK; }
HAL_StatusTypeDef DM_Motor_Disable(FDCAN_HandleTypeDef *h, const DM_Motor_t *m, uint16_t mode)
{ (void)h; (void)m; (void)mode; return HAL_OK; }
HAL_StatusTypeDef DM_Motor_ClearError(FDCAN_HandleTypeDef *h, const DM_Motor_t *m, uint16_t mode)
{ (void)h; (void)m; (void)mode; return HAL_OK; }
HAL_StatusTypeDef DM_Motor_RequestFeedback(FDCAN_HandleTypeDef *h, const DM_Motor_t *m)
{ (void)h; (void)m; return HAL_OK; }
HAL_StatusTypeDef DM_Motor_MIT(FDCAN_HandleTypeDef *h, const DM_Motor_t *m, float p, float v, float kp,
                               float kd, float t)
{ (void)h; (void)m; (void)p; (void)v; (void)kp; (void)kd; (void)t; return HAL_OK; }
HAL_StatusTypeDef DM_Motor_Position(FDCAN_HandleTypeDef *h, const DM_Motor_t *m, float p, float v)
{ (void)h; (void)m; (void)p; (void)v; return HAL_OK; }
HAL_StatusTypeDef DM_Motor_Speed(FDCAN_HandleTypeDef *h, const DM_Motor_t *m, float v)
{ (void)h; (void)m; (void)v; return HAL_OK; }
HAL_StatusTypeDef DM_Motor_PositionCurrent(FDCAN_HandleTypeDef *h, const DM_Motor_t *m, float p, float v,
                                           float c)
{ (void)h; (void)m; (void)p; (void)v; (void)c; return HAL_OK; }
HAL_StatusTypeDef DM_Motor_SetZeroPosition(FDCAN_HandleTypeDef *h, const DM_Motor_t *m, uint16_t mode)
{ (void)h; (void)m; (void)mode; return HAL_OK; }
HAL_StatusTypeDef DM_Motor_SaveParameters(FDCAN_HandleTypeDef *h, const DM_Motor_t *m)
{ (void)h; (void)m; return HAL_OK; }

/* The drives: a register file each; a connected drive answers a read on the
 * next tick, except the register it is muted on; ctrl_mode_stuck ignores a
 * CTRL_MODE write. */
static uint32_t file[8][96];
static uint8_t connected[8], muted[8], ctrl_mode_stuck[8];
static int pending_motor = -1, reads[8];
static uint8_t pending_rid;
HAL_StatusTypeDef DM_Motor_ReadRegister(FDCAN_HandleTypeDef *h, const DM_Motor_t *m, uint8_t rid)
{
  (void)h;
  pending_motor = (int)(m - dm_motors);
  pending_rid = rid;
  reads[pending_motor]++;
  return HAL_OK;
}
HAL_StatusTypeDef DM_Motor_WriteRegister(FDCAN_HandleTypeDef *h, const DM_Motor_t *m, uint8_t rid,
                                         const uint8_t value[4])
{
  (void)h;
  const int i = (int)(m - dm_motors);
  if (!(rid == INFRABOT_RID_CTRL_MODE && ctrl_mode_stuck[i])) memcpy(&file[i][rid], value, 4);
  return HAL_OK;
}

static int replies;
static uint8_t reply_status;
static uint32_t reply_value, reply_sequence;
void AppEthernet_QueueMaintenanceResult(uint32_t sequence, uint8_t command, uint8_t motor_index,
                                        uint8_t status, uint8_t value_type, uint8_t register_id,
                                        uint32_t value)
{
  (void)motor_index; (void)value_type; (void)register_id;
  assert(command == DM_ETH_CMD_SET_ARM_MODE);
  replies++;
  reply_status = status;
  reply_value = value;
  reply_sequence = sequence;
}

static uint32_t bits(float value)
{
  uint32_t raw;
  memcpy(&raw, &value, sizeof(raw));
  return raw;
}

static const float vmax[8] = {45, 45, 10, 10, 30, 30, 30, 30};
static const float tmax[8] = {54, 54, 28, 28, 10, 10, 10, 10};

/* Eight connected drives whose registers match the firmware's table. */
static void reset(void)
{
  tick = 1000;
  memset(file, 0, sizeof(file));
  for (unsigned i = 0; i < 8; ++i) {
    dm_motors[i] = (MockMotor){.position_max = 12.5f, .velocity_max = vmax[i], .torque_max = tmax[i],
                               .feedback_count = 1, .feedback_tick = tick, .id = (uint16_t)(i + 1),
                               .master_id = (uint16_t)(0x11 + i)};
    file[i][INFRABOT_RID_MST_ID] = 0x11 + i;
    file[i][INFRABOT_RID_ESC_ID] = i + 1;
    file[i][INFRABOT_RID_CTRL_MODE] = 1;
    file[i][INFRABOT_RID_PMAX] = bits(12.5f);
    file[i][INFRABOT_RID_VMAX] = bits(vmax[i]);
    file[i][INFRABOT_RID_TMAX] = bits(tmax[i]);
    file[i][INFRABOT_RID_KP_APR] = bits(54.0f);
    connected[i] = 1;
    muted[i] = ctrl_mode_stuck[i] = 0;
    reads[i] = 0;
    dm_motor_runtime.enabled[i] = 0;
    dm_posvel_kp_apr[i] = 0.0f;
  }
  dm_motor_runtime.control_enabled = 0;
  eth_udp_command.control_mode = DM_ETH_CONTROL_IDLE;
  dm_motor_runtime.profile_ready = 0;
}

/* One control tick: the read sent last tick is answered (the CAN ISR), then
 * the maintenance step runs. */
static void control_tick(void)
{
  ++tick;
  if (pending_motor >= 0) {
    const int i = pending_motor;
    pending_motor = -1;
    if (connected[i] && muted[i] != pending_rid) {
      DM_CanRxFrame_t frame = {0};
      frame.id = dm_motors[i].master_id;
      frame.length = 8;
      frame.data[0] = (uint8_t)(dm_motors[i].id & 0x0FU);
      frame.data[2] = 0x33;
      frame.data[3] = pending_rid;
      memcpy(&frame.data[4], &file[i][pending_rid], 4);
      assert(AppDamiao_RegisterRx(&frame));
    }
  }
  AppDamiao_Process();
}

/* SET_ARM_MODE until its DMAR; returns the milliseconds it took. */
static unsigned set_arm_mode(uint8_t mode, uint32_t sequence)
{
  eth_maintenance.command = DM_ETH_CMD_SET_ARM_MODE;
  eth_maintenance.motor_index = DM_ETH_MOTOR_ALL;
  eth_maintenance.mode = mode;
  eth_maintenance.value_type = 0;
  eth_maintenance.register_id = 0;
  eth_maintenance.sequence = sequence;
  eth_maintenance.pending = 1;
  replies = 0;
  unsigned n = 0;
  while (replies == 0) {
    control_tick();
    assert(++n < 3000);
  }
  assert(replies == 1 && reply_sequence == sequence && !eth_maintenance.waiting);
  return n;
}

static InfrabotSafetyStatus report(void)
{
  InfrabotSafetyStatus s = {0};
  AppDamiao_ReportSelfCheck(&s);
  return s;
}

int main(void)
{
  /* Cold LCD setup uses the identical self-check, with no fake DMAR sent
   * to a PC and no dependency on a gripper being present. */
  reset(); replies=0; connected[7]=0; dm_motors[7].feedback_count=0;
  assert(AppDamiao_StartLocalProfile(DM_ETH_MODE_MIT)==0);
  assert(AppDamiao_StartLocalProfile(DM_ETH_MODE_MIT)==4);
  for (unsigned n=0;eth_maintenance.waiting;++n) { assert(n<3000); control_tick(); }
  assert(AppDamiao_LocalProfileStatus()==0 && dm_motor_runtime.profile_ready && replies==0);
  assert(report().selfcheck==INFRABOT_SELFCHECK_OK && reads[7]==0);
  reset(); file[2][INFRABOT_RID_VMAX]=bits(8.f);
  assert(AppDamiao_StartLocalProfile(DM_ETH_MODE_MIT)==0);
  while(eth_maintenance.waiting) control_tick();
  assert(AppDamiao_LocalProfileStatus()==6 && !dm_motor_runtime.profile_ready && replies==0);
  reset();
  assert(AppDamiao_StartLocalProfile(DM_ETH_MODE_MIT)==0);
  AppDamiao_Cancel();
  for (unsigned n=0;n<500;++n) control_tick();
  assert(!dm_motor_runtime.profile_ready && AppDamiao_LocalProfileStatus()==4 && replies==0);

  /* Every drive as configured: ready, in a few hundred milliseconds. */
  reset();
  const unsigned ms = set_arm_mode(DM_ETH_MODE_MIT, 41);
  assert(reply_status == 0 && reply_value == DM_ETH_MODE_MIT && dm_motor_runtime.profile_ready);
  assert(ms < 500);
  InfrabotSafetyStatus s = report();
  assert(s.selfcheck == INFRABOT_SELFCHECK_OK && s.selfcheck_joint == INFRABOT_NO_JOINT);
  for (unsigned i = 0; i < 8; ++i) assert(reads[i] == 6 && file[i][INFRABOT_RID_CTRL_MODE] == (i == 7 ? 4U : 1U));

  /* J8 not connected (another tool, none): not checked. */
  reset();
  connected[7] = 0;
  dm_motors[7].feedback_count = 0;
  set_arm_mode(DM_ETH_MODE_MIT, 42);
  assert(reply_status == 0 && dm_motor_runtime.profile_ready && reads[7] == 0);
  assert(report().selfcheck == INFRABOT_SELFCHECK_OK);

  /* J3 VMAX 8 where the firmware encodes with 10: refused, the first
   * mismatch reported (DMAR value and telemetry). */
  reset();
  file[2][INFRABOT_RID_VMAX] = bits(8.0f);
  file[6][INFRABOT_RID_TMAX] = bits(7.0f); /* a second one is only logged */
  set_arm_mode(DM_ETH_MODE_MIT, 43);
  assert(reply_status == INFRABOT_REPLY_MISMATCH && !dm_motor_runtime.profile_ready);
  assert(reply_value == INFRABOT_SELFCHECK_REPLY_VALUE(2, INFRABOT_RID_VMAX, INFRABOT_SELFCHECK_MISMATCH));
  s = report();
  assert(s.selfcheck == INFRABOT_SELFCHECK_MISMATCH && s.selfcheck_joint == 2 &&
         s.selfcheck_register == INFRABOT_RID_VMAX && s.selfcheck_value == bits(8.0f) &&
         s.selfcheck_expected == bits(10.0f));
  for (unsigned i = 0; i < 8; ++i) assert(reads[i] == 6); /* all checked */

  /* J5 silent on TMAX: no reply is a failure too. */
  reset();
  muted[4] = INFRABOT_RID_TMAX;
  set_arm_mode(DM_ETH_MODE_MIT, 44);
  assert(reply_status == INFRABOT_REPLY_MISMATCH && !dm_motor_runtime.profile_ready);
  assert(reply_value == INFRABOT_SELFCHECK_REPLY_VALUE(4, INFRABOT_RID_TMAX, INFRABOT_SELFCHECK_NO_REPLY));
  assert(report().selfcheck == INFRABOT_SELFCHECK_NO_REPLY && report().selfcheck_joint == 4);

  /* A wrong feedback ID and a mode write that did not take. */
  reset();
  file[1][INFRABOT_RID_MST_ID] = 0x22;
  set_arm_mode(DM_ETH_MODE_MIT, 45);
  assert(reply_value == INFRABOT_SELFCHECK_REPLY_VALUE(1, INFRABOT_RID_MST_ID, INFRABOT_SELFCHECK_MISMATCH));
  reset();
  ctrl_mode_stuck[3] = 1;
  set_arm_mode(DM_ETH_MODE_POS_VEL, 46);
  assert(reply_status == INFRABOT_REPLY_MISMATCH);
  assert(reply_value == INFRABOT_SELFCHECK_REPLY_VALUE(3, INFRABOT_RID_CTRL_MODE, INFRABOT_SELFCHECK_MISMATCH));
  assert(report().selfcheck_value == 1U && report().selfcheck_expected == DM_ETH_MODE_POS_VEL);

  /* Position-velocity: KP_APR learnt first, then the check with mode 2 (J8
   * force-position); a float within 1e-3 matches. */
  reset();
  file[0][INFRABOT_RID_PMAX] = bits(12.5001f);
  file[2][INFRABOT_RID_KP_APR] = bits(60.0f);
  set_arm_mode(DM_ETH_MODE_POS_VEL, 47);
  assert(reply_status == 0 && reply_value == DM_ETH_MODE_POS_VEL && dm_motor_runtime.profile_ready);
  assert(dm_posvel_kp_apr[2] == 60.0f && dm_posvel_kp_apr[0] == 54.0f);
  for (unsigned i = 0; i < 8; ++i)
    assert(reads[i] == (i < 7 ? 7 : 6) && file[i][INFRABOT_RID_CTRL_MODE] == (i == 7 ? 4U : 2U));
  assert(report().selfcheck == INFRABOT_SELFCHECK_OK);

  /* Fixed, applied again: ready. */
  reset();
  file[2][INFRABOT_RID_VMAX] = bits(8.0f);
  set_arm_mode(DM_ETH_MODE_MIT, 48);
  assert(!dm_motor_runtime.profile_ready);
  file[2][INFRABOT_RID_VMAX] = bits(10.0f);
  set_arm_mode(DM_ETH_MODE_MIT, 49);
  assert(reply_status == 0 && dm_motor_runtime.profile_ready && report().selfcheck == INFRABOT_SELFCHECK_OK);
  assert(report().selfcheck_joint == INFRABOT_NO_JOINT);

  puts("selfcheck: all match, absent gripper, VMAX mismatch, no reply, IDs, stuck mode, "
       "position-velocity with KP_APR, tolerance, retry passed");
  return 0;
}
