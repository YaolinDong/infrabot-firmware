/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Verify real application gate inputs independently of the simulated socket board. */
#include "app_state.h"
#include "app_local_control.h"
#include <assert.h>
#include <stdio.h>
#include <sys/mman.h>
#include "update.h"
volatile uint32_t host_mail[24];
static unsigned now,resets;
uint32_t HAL_GetTick(void){return now;}
uint8_t AppLcd_Ready(void){return 1;}
void NVIC_SystemReset(void){resets++;}

MockMotor dm_motors[8];
static unsigned state;
uint8_t AppEnable_InProgress(void){return state&1;}
uint8_t AppGravity_Owned(void){return !!(state&2);}
uint8_t AppMotion_Owned(void){return !!(state&4);}
uint8_t AppLocalControl_SetupPending(void){return !!(state&8);}
uint8_t AppPersist_Busy(void){return !!(state&16);}
void AppLocalControl_Status(AppLocalControlStatus *out){*out=(AppLocalControlStatus){.pending_actions=(state&32)?1:0};}
int fw_app_busy_locked(void);
int main(void){
 assert(!fw_app_busy_locked());
 for(state=1;state<=32;state*=2)assert(fw_app_busy_locked());
 state=0;
 volatile uint8_t *flags[]={&dm_motor_runtime.control_enabled,&eth_udp_command.control_mode,&eth_udp_command.control_request,
 &eth_maintenance.pending,&eth_maintenance.waiting,&eth_maintenance.profile_pending};
 for(unsigned i=0;i<sizeof(flags)/sizeof(*flags);i++){*flags[i]=1;assert(fw_app_busy_locked());*flags[i]=0;}
 eth_udp_command.enable_request=128;assert(fw_app_busy_locked());eth_udp_command.enable_request=0;
 eth_udp_command.disable_request=1;assert(fw_app_busy_locked());eth_udp_command.disable_request=0;
 for(unsigned i=0;i<8;i++){
   dm_motor_runtime.enabled[i]=1;assert(fw_app_busy_locked());dm_motor_runtime.enabled[i]=0;
   dm_motors[i].feedback_count=1;dm_motors[i].state=1;assert(fw_app_busy_locked());
   dm_motors[i].state=0;
 }
 assert(!fw_app_busy_locked());
 assert(mmap((void*)FW_META_ADDR,4096,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED_NOREPLACE,-1,0)!=(void*)-1);
 *(uint32_t*)(FW_META_ADDR+68)=9;
 fw_mail(FW_ATTEMPT,FW_META_ADDR,9);host_mail[5]=FW_SWD_ACCEPT;
 now=2999;fw_swd_poll();assert(!resets && !host_mail[4]);
 now=3000;fw_swd_poll();assert(!resets && host_mail[4]==FW_SWD_TRIAL_READY && !host_mail[5]);
 assert(host_mail[6]==FW_APP_ADDR && host_mail[7]==9);
 dm_motor_runtime.control_enabled=1;host_mail[5]=FW_SWD_ACCEPT;fw_swd_poll();assert(!resets);
 dm_motor_runtime.control_enabled=0;host_mail[5]=FW_SWD_ACCEPT;fw_swd_poll();
 assert(resets==1 && host_mail[0]==FW_CONFIRM_REQ && host_mail[2]==FW_META_ADDR && host_mail[3]==9);
 puts("Application reset gate: all drives, group requests, mode ownership, setup and persistence passed");
}
