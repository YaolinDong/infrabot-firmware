/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "update.h"
#include "consent.h"
#include <assert.h>
#include <stdint.h>
static uint32_t time_ms;
static int busy,touch=1;
uint32_t HAL_GetTick(void){return time_ms;}
uint8_t AppTouch_Ready(void){return touch;}
int fw_app_busy_locked(void){return busy;}
int main(void){
 uint8_t a[32]={1},b[32]={2};FwPrompt q;
 time_ms=UINT32_MAX-100;
 assert(fw_consent_poll(1,3,FW_ENTER,a)==FW_PENDING);
 assert(fw_consent_view(&q) && q.id==1);
 fw_consent_decide(2,1);
 assert(fw_consent_poll(2,3,FW_ENTER,a)==FW_BUSY);
 assert(fw_consent_poll(1,3,FW_ENTER,b)==FW_BUSY);
 fw_consent_decide(1,0);assert(fw_consent_poll(1,3,FW_ENTER,a)==FW_DENIED);
 assert(fw_consent_poll(2,3,FW_ENTER,b)==FW_PENDING);
 time_ms+=60000;fw_consent_decide(2,1);assert(fw_consent_poll(2,3,FW_ENTER,b)==FW_EXPIRED);
 assert(!fw_consent_view(&q));
 assert(fw_consent_poll(3,3,FW_ENTER,a)==FW_PENDING);fw_consent_decide(3,1);
 busy=1;assert(fw_consent_poll(3,3,FW_ENTER,a)==FW_DENIED);
 busy=0;touch=0;assert(fw_consent_poll(4,3,FW_ENTER,a)==FW_BUSY);
 touch=1;assert(fw_consent_poll(4,3,FW_ENTER,a)==FW_PENDING);fw_consent_decide(4,1);
 assert(fw_consent_poll(4,4,FW_ENTER,a)==FW_BUSY && fw_consent_poll(4,3,FW_REVERT,a)==FW_BUSY);
 assert(fw_consent_poll(4,3,FW_ENTER,a)==FW_OK && fw_consent_poll(4,3,FW_ENTER,a)==FW_DENIED);
}
