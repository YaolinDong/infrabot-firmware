/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
typedef uint32_t ip_addr_t;
struct udp_pcb { int unused; };
struct pbuf {uint16_t tot_len;unsigned char bytes[1400];};
#define ERR_OK 0
#define IP_ADDR_ANY NULL
#define PBUF_TRANSPORT 0
#define PBUF_RAM 0
#define ip_addr_cmp(a,b) (*(a)==*(b))
typedef void (*recv_fn)(void*,struct udp_pcb*,struct pbuf*,const ip_addr_t*,uint16_t);
struct udp_pcb *udp_new(void);
int udp_bind(struct udp_pcb*,const ip_addr_t*,unsigned);
void udp_recv(struct udp_pcb*,recv_fn,void*);
void udp_remove(struct udp_pcb*);
int udp_sendto(struct udp_pcb*,struct pbuf*,const ip_addr_t*,uint16_t);
static inline struct pbuf *pbuf_alloc(int layer,unsigned n,int kind){(void)layer;(void)kind;
 struct pbuf *p=calloc(1,sizeof(*p));if(p)p->tot_len=n;return p;}
static inline int pbuf_take(struct pbuf *p,const void *v,unsigned n){memcpy(p->bytes,v,n);return 0;}
static inline void pbuf_free(struct pbuf*p){free(p);}
static inline unsigned pbuf_copy_partial(struct pbuf*p,void*d,unsigned n,unsigned off){memcpy(d,p->bytes+off,n);return n;}

#define ip_2_ip4(x) (x)
#define ip4_addr_get_u32(x) (*(x))
