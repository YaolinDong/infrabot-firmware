/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef MOCK_APP_ETHERNET_H
#define MOCK_APP_ETHERNET_H
/* The Ethernet task for host tests: maintenance results only. */
#include <stdint.h>
uint64_t AppEthernet_GetTimeUs(void);
void AppEthernet_QueueMaintenanceResult(uint32_t sequence, uint8_t command, uint8_t motor_index,
                                        uint8_t status, uint8_t value_type, uint8_t register_id,
                                        uint32_t value);
#endif
