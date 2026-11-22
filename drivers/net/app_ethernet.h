/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_ETHERNET_H
#define APP_ETHERNET_H

#include "FreeRTOS.h"
#include <stdint.h>
#include "infrabot/protocol.h"
#include "app_persist.h"

typedef struct
{
  AppNetConfig address;   /* what the netif uses now */
  uint8_t mac[6];
  uint8_t link_up;
  uint8_t from_flash;     /* address came from the LCD setting, not the default */
  uint8_t pending;        /* a requested change is not applied yet */
  uint8_t client_valid;   /* a PC has registered for telemetry */
  uint8_t client_ip[4];
  uint16_t client_port;
} AppEthernetInfo;

/* Creates the low-priority Ethernet/LwIP task. */
BaseType_t AppEthernet_CreateTask(void);

/* Wakes the LwIP owner task when the Ethernet DMA receives a frame. */
void AppEthernet_NotifyRxFromISR(void);

/* Monotonic MCU time based on the DWT cycle counter, extended across wrap. */
uint64_t AppEthernet_GetTimeUs(void);

/* Called by the control task when a low-rate CAN maintenance operation has
 * completed.  The Ethernet task sends the result to the last registered PC
 * telemetry endpoint. */
void AppEthernet_QueueMaintenanceResult(uint32_t sequence,
                                        uint8_t command,
                                        uint8_t motor_index,
                                        uint8_t status,
                                        uint8_t value_type,
                                        uint8_t register_id,
                                        uint32_t value);

/* 1 while the PHY reports link up (for display only). */
uint8_t AppEthernet_LinkUp(void);
/* Current address, MAC, link and registered PC, for display (unlocked reads
 * of 32-bit fields). */
void AppEthernet_GetInfo(AppEthernetInfo *info);
/* Switches the static address; the Ethernet task applies it within a few
 * milliseconds and drops the registered PC, which must connect to the new
 * address.  Does not save it (AppPersist_SaveNetwork). */
void AppEthernet_RequestAddress(const AppNetConfig *config);
/* The status task's once-a-second summary of the command link, sent in every
 * DMTL v3 packet until the next one. */
void AppEthernet_SetLinkStats(const InfrabotLinkStats *stats);
/* A DMTL v3 snapshot of now with sequence 0 (the run log numbers its own). */
void AppEthernet_FillTelemetry(InfrabotTelemetryV3 *packet);

#endif
