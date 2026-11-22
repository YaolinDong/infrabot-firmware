/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <stdint.h>
#include "app_config.h"

typedef struct {
    volatile uint32_t dropped;
    volatile uint32_t hal_errors;
    volatile uint32_t recoveries;
    volatile uint32_t last_error;
    volatile uint32_t last_state;
    uint8_t ring[DM_UART_TX_RING_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint8_t busy;
    volatile uint16_t active_len;
    volatile uint32_t start_tick;
} UartTx;

extern UartTx uart_tx;
