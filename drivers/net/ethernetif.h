/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef ETHERNETIF_H
#define ETHERNETIF_H

#include "lwip/err.h"
#include "lwip/netif.h"

extern ETH_HandleTypeDef EthHandle;
extern volatile uint32_t ethernetif_rx_packets;
extern volatile uint32_t ethernetif_rx_alloc_errors;
extern volatile uint32_t ethernetif_tx_packets;
extern volatile uint32_t ethernetif_tx_errors;

err_t ethernetif_init(struct netif *netif);
void ethernetif_input(struct netif *netif);
void ethernet_link_check_state(struct netif *netif);

/* Returns negotiated PHY rate and duplex mode when the link is valid. */
uint8_t ethernetif_get_link_info(uint32_t *speed_mbps, uint8_t *full_duplex);

uint8_t ethernetif_get_phy_diagnostics(uint32_t *phy_addr,
                                       uint32_t *phy_id1,
                                       uint32_t *phy_id2,
                                       uint32_t *bcr,
                                       uint32_t *bsr,
                                       uint32_t *phyc_sr);

#endif
