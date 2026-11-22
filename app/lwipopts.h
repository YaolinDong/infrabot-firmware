/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#include "app_config.h"

/* Raw-api/no-sys LwIP.  All LwIP calls are serialized in AppEthernetTask;
 * the 1 kHz motor task never enters the network stack. */
#define NO_SYS                  1
#define SYS_LIGHTWEIGHT_PROT    0
#define MEM_ALIGNMENT           4
/* mem.c places allocator sentinels beyond MEM_SIZE.  Keep 32 bytes inside the
 * 16-KB MPU window for those headers when LWIP_RAM_HEAP_POINTER is external. */
#define LWIP_HEAP_REGION_SIZE   (16U * 1024U)
#define LWIP_HEAP_OVERHEAD      32U
#define MEM_SIZE                (LWIP_HEAP_REGION_SIZE - LWIP_HEAP_OVERHEAD)
extern unsigned char lwip_heap[];
#define LWIP_RAM_HEAP_POINTER  ((void *)lwip_heap)
#define MEMP_NUM_PBUF           32
#define MEMP_NUM_UDP_PCB        4
#define PBUF_POOL_SIZE          16
#define PBUF_POOL_BUFSIZE       1536
#define LWIP_RAW                1
#define LWIP_SUPPORT_CUSTOM_PBUF 1
#define LWIP_UDP                1
#define LWIP_TCP                0
#define LWIP_DHCP               DM_ETH_USE_DHCP
#define LWIP_AUTOIP             0
#define LWIP_DNS                0
#define LWIP_IPV6               0
#define LWIP_ICMP               1
#define LWIP_ARP                1
#define LWIP_ETHERNET           1
#define LWIP_NETIF_LINK_CALLBACK 1
#define LWIP_NETCONN             0
#define LWIP_SOCKET              0
#define LWIP_STATS               0
#define LWIP_TIMEVAL_PRIVATE     0
#define ETHARP_SUPPORT_STATIC_ENTRIES 1
#define CHECKSUM_GEN_IP          1
#define CHECKSUM_GEN_UDP         1
#define CHECKSUM_CHECK_IP        1
#define CHECKSUM_CHECK_UDP       1
#define TCPIP_MBOX_SIZE          1
#define DEFAULT_THREAD_STACKSIZE 512
#define LWIP_TIMERS              1
#define IP_DEFAULT_TTL           64

#endif
