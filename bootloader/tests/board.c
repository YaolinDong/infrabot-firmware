/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* POSIX adapter for the real service, installer and boot slot selector. */
#include "main.h"
#include "lwip/udp.h"
#include "slots.h"
#include "consent.h"
#include <arpa/inet.h>
#include <assert.h>
#include <poll.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
unsigned char host_stage[FW_CAPACITY];
static recv_fn receiver;static struct udp_pcb pcb;
static int fd,role=1,running=0,reset,drop=1,deny,unhealthy;
static uint32_t deadline;
static uint64_t challenge=42;
static int rng_fail;
void map_flash(void);void load_factory(const char *);int simulate_reset(uint32_t,uint32_t,uint32_t,int*);
uint64_t fw_challenge(void){return role==rng_fail?0:++challenge;}
static int choice;
int fw_app_busy_locked(void){return deny;}
uint8_t AppTouch_Ready(void){return 1;}

int fw_is_trial(void){return role==3;}
int fw_trial_ready(void){return !unhealthy;}
uint32_t fw_running_base(void){return fw_slots[running].base+1024;}
uint32_t HAL_GetTick(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec*1000+t.tv_nsec/1000000;}
void fw_progress(void){deadline=HAL_GetTick()+4500;}
void HAL_Delay(unsigned ms){(void)ms;}
void NVIC_SystemReset(void){reset=1;}
int fw_enter_if_disabled(uint32_t reason){
 if(deny)return FW_BUSY;
 fw_mail(reason,fw_slots[running].base,fw_slots[running].gen);reset=1;return FW_OK;
}
struct udp_pcb *udp_new(void){return &pcb;}
int udp_bind(struct udp_pcb*p,const ip_addr_t*i,unsigned port){(void)p;(void)i;assert(port==FW_PORT);return 0;}
void udp_recv(struct udp_pcb*p,recv_fn fn,void*a){(void)p;(void)a;receiver=fn;}
void udp_remove(struct udp_pcb*p){(void)p;}
int udp_sendto(struct udp_pcb*p,struct pbuf*b,const ip_addr_t*i,uint16_t port){
 (void)p;FwHeader h;memcpy(&h,b->bytes,sizeof(h));
 if(h.op==(128|FW_DATA)&&drop && h.offset==1024){drop=0;return 0;}
 if(h.op==(128|FW_COMMIT)&&!h.status)return 0; /* lost final ACK */
 struct sockaddr_in a={.sin_family=AF_INET,.sin_port=htons(port),.sin_addr.s_addr=*i};
 return sendto(fd,b->bytes,b->tot_len,0,(void*)&a,sizeof(a))>=0?0:-1;
}
int loader_service_start(void);int application_service_start(void);
int main(int argc,char **argv){
 assert(argc==3);deny=!strcmp(argv[2],"busy");unhealthy=!strcmp(argv[2],"bad-trial");
 choice=!strcmp(argv[2],"denied")?-1:!strcmp(argv[2],"timeout")?0:1;
 if(!strcmp(argv[2],"rng-app"))rng_fail=1;
 if(!strcmp(argv[2],"rng-loader"))rng_fail=role=2;
 map_flash();load_factory(argv[1]);
 fd=socket(AF_INET,SOCK_DGRAM,0);assert(fd>=0);
 struct sockaddr_in a={.sin_family=AF_INET,.sin_addr.s_addr=htonl(INADDR_LOOPBACK)};
 assert(bind(fd,(void*)&a,sizeof(a))==0);socklen_t alen=sizeof(a);getsockname(fd,(void*)&a,&alen);
 printf("%u\n",ntohs(a.sin_port));fflush(stdout);
 for(;;){
   if(role==2)assert(loader_service_start());else assert(application_service_start());
   fw_progress();
   while(!reset){
     struct pollfd pf={fd,POLLIN,0};
     if(poll(&pf,1,25)>0){
       struct pbuf *p=pbuf_alloc(0,0,0);ssize_t n=recvfrom(fd,p->bytes,sizeof(p->bytes),0,(void*)&a,&alen);
       assert(n>=0);p->tot_len=n;ip_addr_t ip=a.sin_addr.s_addr;receiver(0,&pcb,p,&ip,ntohs(a.sin_port));
     }
     FwPrompt prompt;
     if(role==1 && choice && fw_consent_view(&prompt))fw_consent_decide(prompt.id,choice==1);
     if(role!=1 && (int32_t)(HAL_GetTick()-deadline)>=0)reset=1;
   }
   if(host_mail[0]!=FW_REQUEST_MAGIC && host_mail[0]!=FW_INSTALLED_REQ)FW_TICKET->cookie=0;
   int trial=0;running=simulate_reset(host_mail[0],host_mail[2],host_mail[3],&trial);
   memset((void*)host_mail,0,32);reset=0;role=running<0?2:trial?3:1;
 }
}
