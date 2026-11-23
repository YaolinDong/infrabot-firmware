/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* IFU2. Session/peer correlation is not publisher authentication. The
 * immutable loader checks an Ed25519 signature before touching flash. */
#include "update.h"
#include "main.h"
#include "lwip/udp.h"
#include <string.h>
#ifdef FW_LOADER_BUILD
#include "app_sdram.h"
#include "slots.h"
static uint8_t *const stage=(uint8_t *)APP_SDRAM_BASE;
static FwManifest manifest;
static uint32_t cursor;
static uint8_t receiving;
int fw_install(const FwManifest *,const uint8_t *);
#define ROLE FW_LOADER
#else
#include "consent.h"
int fw_enter_if_disabled(uint32_t);
int fw_trial_ready(void);
#define ROLE (fw_is_trial()?FW_TRIAL:FW_APPLICATION)
#endif
static struct udp_pcb *pcb;
static ip_addr_t peer;
static uint16_t peer_port;
static uint64_t session;
static uint32_t seen;
#ifndef FW_LOADER_BUILD
static uint32_t hello_seq;
#endif
static union {FwHeader align;uint8_t bytes[sizeof(FwHeader)+FW_CHUNK];} packet;
uint64_t fw_challenge(void);

static void reply(FwHeader h,unsigned status,uint64_t cookie,uint32_t offset,const void *data,unsigned size,
                  const ip_addr_t *ip,uint16_t port){
 uint8_t buf[sizeof(h)+32];h.op|=0x80;h.status=status;h.role=ROLE;
 h.offset=offset;h.session=cookie;memcpy(buf,&h,sizeof(h));if(size)memcpy(buf+sizeof(h),data,size);
 struct pbuf *p=pbuf_alloc(PBUF_TRANSPORT,sizeof(h)+size,PBUF_RAM);
 if(p){if(pbuf_take(p,buf,p->tot_len)==ERR_OK)udp_sendto(pcb,p,ip,port);pbuf_free(p);}
}
static void receive(void *arg,struct udp_pcb *listener,struct pbuf *p,const ip_addr_t *ip,uint16_t port){
 (void)arg;(void)listener;
 if(!p)return;
 ip_addr_t sender=*ip;
 unsigned n=p->tot_len;
 if(n<sizeof(FwHeader)||n>sizeof(packet.bytes)){pbuf_free(p);return;}
 unsigned got=pbuf_copy_partial(p,packet.bytes,n,0);pbuf_free(p);
 FwHeader h;memcpy(&h,packet.bytes,sizeof(h));
 if(got!=n || memcmp(h.magic,"IFU2",4)||h.version!=2||h.role||h.status)return;
 uint8_t *body=packet.bytes+sizeof(h);unsigned len=n-sizeof(h);
 uint32_t now=HAL_GetTick(), addr=ip4_addr_get_u32(ip_2_ip4(&sender));
 int owner=session && port==peer_port && ip_addr_cmp(&peer,&sender);
 int ticket=FW_TICKET->cookie && h.session==FW_TICKET->cookie && addr==FW_TICKET->ip && port==FW_TICKET->port;
 unsigned status=FW_INVALID;uint32_t offset=0;
 if(h.op==FW_HELLO && !len){
   /* Offset 1 is a read-only probe: never steal, open or return a session. */
   uint64_t answer=0;
#ifdef FW_LOADER_BUILD
   offset=fw_destination();
   if(h.offset==1)status=FW_OK;
   else if(ticket && offset){
     if(!owner){receiving=0;cursor=0;}
     session=h.session;peer=sender;peer_port=port;seen=now;answer=session;status=FW_OK;
   }else status=FW_DENIED;
   reply(h,status,answer,offset,0,0,&sender,port);
#else
   offset=fw_running_base();
   if(h.offset==1)status=FW_OK;
   else if(fw_is_trial()){
     status=ticket?FW_OK:FW_DENIED;
     if(ticket){session=h.session;peer=sender;peer_port=port;seen=now;answer=session;}
   }else{
     if(fw_consent_reserved() && (!owner || h.seq!=hello_seq))status=FW_BUSY;
     else{
       if(!owner||hello_seq!=h.seq||now-seen>=10000U)session=fw_challenge();
       peer=sender;peer_port=port;hello_seq=h.seq;seen=now;answer=session;status=session?FW_OK:FW_RNG;
     }
   }
   reply(h,status,answer,offset,(void*)(uintptr_t)(offset-1024+32U),32,&sender,port);
#endif
   return;
 }
 if(!owner||h.session!=session||now-seen>=10000U)status=FW_SESSION;
 else{
   seen=now;
#ifdef FW_LOADER_BUILD
   if(!ticket)status=FW_DENIED;
   else if(h.op==FW_BEGIN && len==sizeof(manifest) && !h.offset){
     FwManifest m;memcpy(&m,body,sizeof(m));
     if(fw_manifest_ok(&m) && m.base==fw_destination() && !memcmp(m.tag,(const void*)FW_TICKET->tag,32)){
       if(!receiving){manifest=m;cursor=0;receiving=1;}
       if(!memcmp(&manifest,&m,sizeof(m)))status=FW_OK;
     }
   }else if(h.op==FW_DATA && receiving && len && h.offset<=manifest.length && len<=manifest.length-h.offset){
     if(h.offset==cursor){memcpy(stage+cursor,body,len);cursor+=len;fw_progress();status=FW_OK;}
     else status=h.offset<cursor && len<=cursor-h.offset && !memcmp(stage+h.offset,body,len)?FW_OK:FW_OFFSET;
   }else if(h.op==FW_COMMIT && !len && receiving && cursor==manifest.length && h.offset==cursor){
     receiving=0;status=!fw_image_ok(&manifest,stage)?FW_AUTH:fw_install(&manifest,stage)?FW_OK:FW_FLASH;
     if(status==FW_OK)fw_mail(FW_INSTALLED_REQ,0,0);else FW_TICKET->cookie=0;
     reply(h,status,session,cursor,0,0,&sender,port);HAL_Delay(30);NVIC_SystemReset();return;
   }
   offset=cursor;
#else
   if(!h.offset && !fw_is_trial() && ((h.op==FW_ENTER && len>=32 && len<32U+FW_VERSION_LEN)||(h.op==FW_REVERT && !len))){
     const uint8_t empty[32]={0};const uint8_t *tag=len?body:empty;
     char version[FW_VERSION_LEN]={0};
     if(len>32)memcpy(version,body+32,len-32);
     status=fw_consent_poll(session,addr,h.op,tag,version);
     if(status==FW_OK){
       FW_TICKET->ip=addr;FW_TICKET->port=port;memcpy((void*)FW_TICKET->tag,tag,32);
       FW_TICKET->cookie=session;
       status=fw_enter_if_disabled(h.op==FW_ENTER?FW_REQUEST_MAGIC:FW_REVERT_REQ);
       if(status!=FW_OK)FW_TICKET->cookie=0; /* reset was refused */
     }
   }else if(h.op==FW_CONFIRM && !len && !h.offset && fw_is_trial() && ticket)
     status=fw_trial_ready()?fw_enter_if_disabled(FW_CONFIRM_REQ):FW_BUSY;
#endif
 }
 reply(h,status,h.session,offset,0,0,&sender,port);
}
int fw_service_start(void){
 session=0;peer_port=0;
#ifdef FW_LOADER_BUILD
 receiving=0;cursor=0;
#endif
 pcb=udp_new();if(!pcb)return 0;
 if(udp_bind(pcb,IP_ADDR_ANY,FW_PORT)!=ERR_OK){udp_remove(pcb);pcb=NULL;return 0;}
 udp_recv(pcb,receive,NULL);return 1;
}
