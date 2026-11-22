/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_runtime.h"
#include "eth_state.h"
#include "control_services.h"
#include "platform.h"
#include "app_local_control.h"
#include "app_state.h"
#include "app_enable.h"
#include "app_control_mode.h"
#include "app_safety.h"
#include "app_gravity.h"
#include "app_motion.h"
#include <math.h>

_Static_assert(APP_LOCAL_ACTIONS<=16U,"local request bitmask capacity");
static volatile uint16_t requests;
static AppLocalControlStatus status;
static uint8_t preparing, enabling;
uint8_t AppLocalControl_SetupPending(void){return preparing || enabling;}

void AppLocalControl_Request(uint8_t action)
{
  if (action >= APP_LOCAL_ACTIONS) return;
  platform_enter_critical(); requests |= (uint16_t)(1U << action); platform_exit_critical();
}
void AppLocalControl_Status(AppLocalControlStatus *out)
{
  if (!out) return;
  platform_enter_critical(); *out = status; out->pending_actions=requests; platform_exit_critical();
}
static void result(uint8_t code, uint8_t success)
{
  status.result = code;
  status.state = code ? APP_LOCAL_REJECTED : success;
}
static uint8_t arm_enabled(void)
{
  for (unsigned i=0; i<DM_ARM_MOTOR_COUNT; ++i)
    if (!dm_motor_runtime.enabled[i]) return 0U;
  return 1U;
}
static void cancel_setup(void)
{
  if (preparing) AppDamiao_Cancel();
  preparing = enabling = 0U;
  AppEnable_CancelAndHold();
  AppGravity_CancelRequest();
}
static void cancel_arm_start_requests(void)
{
  /* A queued PC enable must not undo a STOP/DISABLE pressed this tick. */
  platform_enter_critical();
  eth_udp_command.enable_request &= ~DM_ARM_MOTOR_MASK;
  if (eth_udp_command.control_request==DM_ETH_CMD_ENABLE ||
      eth_udp_command.control_request==DM_ETH_CMD_SET_ZERO_ENABLE)
    eth_udp_command.control_request=0U;
  platform_exit_critical();
  AppDamiao_CancelMotors(DM_ARM_MOTOR_MASK);
}
static uint8_t resume(void)
{
  InfrabotSafetyStatus stop = {0};
  AppSafety_Report(&stop);
  if (AppSafety_Braking() || AppGravity_Recovering() || AppEnable_InProgress() ||
      eth_maintenance.pending || eth_maintenance.waiting || eth_maintenance.profile_pending)
    return INFRABOT_REPLY_BUSY;
  if (eth_udp_command.control_mode != DM_ETH_CONTROL_IDLE) return INFRABOT_REPLY_MISMATCH;
  if (!dm_motor_runtime.zero_valid || !arm_enabled() || !AppControlMode_ProfileReady() || AppFdcan_BusFault())
    return INFRABOT_REPLY_NOT_READY;
  /* An explicit resume acknowledges communication loss or an operator stop.
   * Hardware/limit/invalid-target faults require inspection and re-enable. */
  if (!AppSafety_Resumable(stop.stop_reason)) return INFRABOT_REPLY_NOT_READY;
  static const float torque_limit[DM_ARM_MOTOR_COUNT] = DM_ARM_TORQUE_LIMIT_NM;
  for (unsigned i=0; i<DM_ARM_MOTOR_COUNT; ++i) {
    if (!dm_motors[i].feedback_count || (int32_t)(platform_now_ms()-dm_motors[i].feedback_tick)>50 ||
        dm_motors[i].state!=1U || !isfinite(dm_motors[i].position) ||
        !isfinite(dm_motors[i].velocity) || !isfinite(dm_motors[i].torque) ||
        fabsf(dm_motors[i].torque)>=torque_limit[i] ||
        dm_motors[i].temperature_mos>=DM_GRAVITY_MAX_TEMP_C ||
        dm_motors[i].temperature_coil>=DM_GRAVITY_MAX_TEMP_C) return INFRABOT_REPLY_NOT_READY;
    if (fabsf(dm_motors[i].velocity)>.05f) return INFRABOT_REPLY_BUSY;
  }
  if (AppControlMode_IsLocalCompliant(stop.stop_mode))
    return AppControlMode_Switch(stop.stop_mode); /* normal one-second ramp */
  /* A screen cannot restart a PC's interrupted trajectory or command
   * producer. Acknowledge into IDLE, preserving the actual holding torque. */
  if (!AppMotion_ReleaseHold()) return INFRABOT_REPLY_BUSY;
  AppSafety_SetLocalHold(1U);
  AppSafety_Clear();
  eth_udp_command.active=0U;
  return 0U;
}
static void enable_arm(void)
{
  if (!dm_motor_runtime.zero_valid || AppFdcan_BusFault()) { result(7U,0U); return; }
  if (eth_udp_command.control_mode!=DM_ETH_CONTROL_IDLE) { result(6U,0U); return; }
  if (AppGravity_Owned() || AppSafety_Braking() || AppEnable_InProgress() ||
      eth_maintenance.pending || eth_maintenance.waiting || eth_maintenance.profile_pending) {
    result(4U,0U); return;
  }
  if (arm_enabled()) { result(0U,APP_LOCAL_ENABLED); return; }
  if (!AppControlMode_ProfileReady()) {
    const uint8_t code=AppDamiao_StartLocalProfile(DM_ETH_MODE_MIT);
    if (code) { result(code,0U); return; }
    preparing=1U; result(0U,APP_LOCAL_PROFILE); return;
  }
  AppSafety_SetLocalHold(1U);
  if (!AppEnable_Start(DM_ARM_MOTOR_MASK)) { result(4U,0U); return; }
  AppSafety_Clear();
  enabling=1U; result(0U,APP_LOCAL_ENABLING);
}
void AppLocalControl_Process(void)
{
  platform_enter_critical(); const uint16_t req=requests; requests=0U; platform_exit_critical();
  /* External disable also cancels a queued LCD enable. Stop/disable win over
   * all starting actions received in the same tick. */
  if (eth_udp_command.control_request==DM_ETH_CMD_DISABLE ||
      (eth_udp_command.disable_request & DM_ARM_MOTOR_MASK)) {
    cancel_setup(); return;
  }
  if (req & (1U<<APP_LOCAL_DISABLE)) {
    cancel_setup(); cancel_arm_start_requests();
    AppMotion_RequestRelease();
    AppEnable_Disable(DM_ARM_MOTOR_MASK); /* explicit LCD operator action only */
    status.action=APP_LOCAL_DISABLE; result(0U,APP_LOCAL_DISABLED); return;
  }
  if ((req & (1U<<APP_LOCAL_STOP)) || AppSafety_StopRequested()) {
    cancel_setup(); cancel_arm_start_requests();
    if (req & (1U<<APP_LOCAL_STOP)) {
      AppSafety_SetLocalHold(1U);
      AppSafety_Stop(INFRABOT_STOP_REQUESTED,INFRABOT_NO_JOINT,0U);
    }
    status.action=APP_LOCAL_STOP; result(0U,APP_LOCAL_STOPPED);
    return;
  }
  if (preparing) {
    const uint8_t code=AppDamiao_LocalProfileStatus();
    if (code==INFRABOT_REPLY_BUSY && eth_maintenance.waiting) return;
    preparing=0U;
    if (code || !AppControlMode_ProfileReady()) { result(code ? code : 6U,0U); return; }
    enable_arm(); return;
  }
  if (enabling) {
    if (AppEnable_InProgress()) return;
    enabling=0U; result(arm_enabled() ? 0U : 7U,APP_LOCAL_ENABLED);
  }
  if (req & (1U<<APP_LOCAL_ENABLE)) { status.action=APP_LOCAL_ENABLE; enable_arm(); }
  else if (req & (1U<<APP_LOCAL_RESUME)) {
    status.action=APP_LOCAL_RESUME; result(resume(),APP_LOCAL_RESUMED);
  } else if(req & ((1U<<APP_LOCAL_JOINT_ADM)|(1U<<APP_LOCAL_CART_ADM))){
    const uint8_t cart=(req&(1U<<APP_LOCAL_CART_ADM))!=0;
    status.action=cart?APP_LOCAL_CART_ADM:APP_LOCAL_JOINT_ADM;
    result(AppControlMode_Switch(cart?DM_ETH_CONTROL_CARTESIAN_ADMITTANCE:DM_ETH_CONTROL_JOINT_ADMITTANCE),APP_LOCAL_RESUMED);
  } else if (req & ((1U<<APP_LOCAL_JOINT_IMP)|(1U<<APP_LOCAL_CART_IMP))) {
    const uint8_t cart=(req & (1U<<APP_LOCAL_CART_IMP))!=0U;
    status.action=cart?APP_LOCAL_CART_IMP:APP_LOCAL_JOINT_IMP;
    result(AppControlMode_Switch(cart?DM_ETH_CONTROL_CARTESIAN_IMPEDANCE:DM_ETH_CONTROL_JOINT_IMPEDANCE),APP_LOCAL_RESUMED);
  } else if (req & ((1U<<APP_LOCAL_FREE)|(1U<<APP_LOCAL_LOCK))) {
    const uint8_t lock=(req & (1U<<APP_LOCAL_LOCK))!=0U;
    status.action=lock ? APP_LOCAL_LOCK : APP_LOCAL_FREE;
    result(AppControlMode_Switch(lock ? DM_ETH_CONTROL_GRAVITY_LOCK : DM_ETH_CONTROL_GRAVITY_FREE),APP_LOCAL_RESUMED);
  }
}
const char *AppLocalControl_Message(const AppLocalControlStatus *s)
{
  if (s->result==4U) return "Busy: wait for stop/setup or let the arm settle";
  if (s->result==5U) return "Profile setup requires disabled drives";
  if (s->result==6U) return "STOP first; compliance requires verified MIT profile";
  if (s->result==7U) return "Not ready: check zero, feedback, drive/stop fault";
  if (s->result) return "Setup failed: check CAN/self-check log";
  switch (s->state) {
    case APP_LOCAL_PROFILE: return "Applying MIT profile and checking drive parameters...";
    case APP_LOCAL_ENABLING: return "Enabling J1-J7 in stages...";
    case APP_LOCAL_ENABLED: return "Arm enabled; J8 is controlled separately";
    case APP_LOCAL_DISABLED: return "Arm disabled: holding torque removed";
    case APP_LOCAL_STOPPED: return "STOP requested: brake, restore gains, hold enabled";
    case APP_LOCAL_RESUMED: return "Resumed: local compliance or IDLE ready for PC motion";
    default: return "J1-J7 controls; DISABLE removes holding torque";
  }
}
