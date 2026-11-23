/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Network owner polls; LCD is the only producer of an allow/deny decision. */
#include "update.h"
#include "consent.h"
#include "main.h"
#include "app_state.h"
#include "platform.h"
#include "app_touch.h"
#include <string.h>
extern int fw_app_busy_locked(void);
static FwPrompt prompt;
static unsigned state;
static uint32_t start;
enum {NONE, ASK, ALLOW, DENY, EXPIRED, USED};
static void refresh(void){
 if(state==ASK || state==ALLOW){
   if(HAL_GetTick()-start>=60000U)state=EXPIRED;
   else if(fw_app_busy_locked() || !AppTouch_Ready())state=DENY;
 }
}
/* Printable ASCII only, so a requester cannot draw control characters. */
static void keep_version(char out[FW_VERSION_LEN],const char *in){
 unsigned n=0;
 if(in)while(n+1U<FW_VERSION_LEN && in[n]>=0x20 && in[n]<0x7f){out[n]=in[n];n++;}
 memset(out+n,0,FW_VERSION_LEN-n);
}
int fw_consent_poll(uint64_t id,uint32_t ip,unsigned op,const uint8_t tag[32],const char *version){
 int result=FW_BUSY;
 platform_enter_critical();
 refresh();
 if(id && (id!=prompt.id || state==NONE)){
   if(state!=ASK && state!=ALLOW && !fw_app_busy_locked() && AppTouch_Ready()){
     prompt.id=id;prompt.ip=ip;prompt.op=op;memcpy(prompt.tag,tag,32);keep_version(prompt.version,version);
     start=HAL_GetTick();state=ASK;
   }
 }
 if(id && prompt.id==id && prompt.ip==ip && prompt.op==op && !memcmp(tag,prompt.tag,32)){
   if(state==ASK)result=FW_PENDING;
   if(state==ALLOW){state=USED;result=FW_OK;}
   else if(state==DENY || state==USED)result=FW_DENIED;
   if(state==EXPIRED)result=FW_EXPIRED;
 }
 platform_exit_critical();return result;
}
int fw_consent_reserved(void){
 platform_enter_critical();refresh();int reserved=state==ASK || state==ALLOW;
 platform_exit_critical();return reserved;
}
int fw_consent_view(FwPrompt *out){
 platform_enter_critical();refresh();int visible=state==ASK;
 if(visible)*out=prompt;
 platform_exit_critical();return visible;
}
void fw_consent_decide(uint64_t id,int allow){
 platform_enter_critical();refresh();
 if(state==ASK && prompt.id==id)state=allow?ALLOW:DENY;
 platform_exit_critical();
}
