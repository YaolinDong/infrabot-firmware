/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_runtime.h"
#include "eth_state.h"
#include "main.h"
#include "update.h"
#include "app_state.h"
#include "platform.h"
#include "app_lcd.h"
#include "app_enable.h"
#include "app_local_control.h"
#include "app_motion.h"
#include "app_gravity.h"
#include "app_persist.h"
/* Recheck atomically immediately before reset. No update command removes
 * drive torque. Reject outstanding setup, maintenance and persistent writes. */
uint32_t fw_running_base(void){
  extern const uint32_t g_pfnVectors[];
  return (uint32_t)(uintptr_t)g_pfnVectors;
}
int fw_is_trial(void){
  return FW_MAILBOX[0]==FW_ATTEMPT && FW_MAILBOX[1]==~FW_ATTEMPT &&
         FW_MAILBOX[2]==fw_running_base()-1024;
}
int fw_app_busy_locked(void)
{
  AppLocalControlStatus local;
  AppLocalControl_Status(&local);
  int busy=dm_motor_runtime.control_enabled || eth_udp_command.control_mode!=DM_ETH_CONTROL_IDLE ||
    AppEnable_InProgress() || AppGravity_Owned() || AppMotion_Owned() ||
    local.pending_actions || AppLocalControl_SetupPending() || AppPersist_Busy() ||
    eth_udp_command.enable_request || eth_udp_command.disable_request || eth_udp_command.control_request ||
    eth_maintenance.pending || eth_maintenance.waiting || eth_maintenance.profile_pending;
  for(unsigned i=0;i<DM_MOTOR_COUNT;i++)
    busy|=dm_motor_runtime.enabled[i] || (dm_motors[i].feedback_count && dm_motors[i].state==1U);
  return busy;
}
int fw_enter_if_disabled(uint32_t reason)
{
  platform_enter_critical();
  if(!fw_app_busy_locked()) {
    uint32_t meta=fw_running_base()-1024U;
    uint32_t generation=*(const uint32_t *)(uintptr_t)(meta+68U);
    fw_mail(reason,meta,generation);
    __DSB();NVIC_SystemReset();
  }
  platform_exit_critical();
  return FW_BUSY;
}

int fw_trial_ready(void)
{
  return HAL_GetTick()>=3000U && (!DM_LCD_ENABLE || AppLcd_Ready());
}

/* Run on the LwIP owner's ordinary poll loop even with the RMII cable absent.
 * A debugger never manufactures a GOOD marker. Use the same health and
 * disabled-state gates, generation check and bootloader commit as the SDK. */
void fw_swd_poll(void)
{
  static uint8_t published;
  if(!published) {
    if(!fw_trial_ready())return;
    const uint32_t vector=fw_running_base();
    FW_MAILBOX[5]=0;
    FW_MAILBOX[6]=vector;
    FW_MAILBOX[7]=*(const uint32_t *)(uintptr_t)(vector-1024U+68U);
    __DSB();
    FW_MAILBOX[4]=fw_is_trial()?FW_SWD_TRIAL_READY:FW_SWD_NORMAL_READY;
    published=1;
  } else if(FW_MAILBOX[5]==FW_SWD_ACCEPT) {
    FW_MAILBOX[5]=0;
    if(fw_is_trial() && fw_trial_ready())(void)fw_enter_if_disabled(FW_CONFIRM_REQ);
  }
}
