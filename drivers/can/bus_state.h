/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <stdint.h>
#include "app_config.h"
#include "can_types.h"

typedef struct {
    /* Accumulated error events; the live bus state is queried separately. */
    volatile uint32_t can_error_events;
    volatile uint32_t motor_error_status;
    volatile uint32_t motor_error_code;
    volatile uint32_t rx_frame_count;
    volatile uint32_t feedback_match_count;
    volatile DM_CanRxFrame_t rx_trace[DM_RX_TRACE_DEPTH];
    volatile uint32_t rx_trace_head;
    uint32_t rx_trace_tail;
    volatile uint32_t rx_trace_dropped;
    volatile uint32_t feedback_fd_count;
    volatile uint32_t feedback_classic_count;
    volatile uint8_t feedback_enabled;
    volatile uint8_t startup_trace_enabled;
} DmBusStats;

extern DmBusStats dm_bus_stats;
