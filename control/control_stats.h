/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <stdint.h>
#include "app_config.h"

typedef struct {
    volatile uint8_t task_started;
    volatile uint8_t target_positive;
    volatile uint8_t endpoint_dwell_active;
    volatile uint32_t endpoint_dwell_ticks;
    volatile uint32_t cycles;
    volatile uint32_t mit_ok;
    volatile uint32_t mit_err;
    volatile uint32_t missed;
    volatile uint32_t last_cycle;
    volatile uint8_t tx_index;
    volatile uint32_t fd_tx;
    volatile uint32_t classic_tx;
    volatile uint32_t period_min_us;
    volatile uint32_t period_max_us;
    volatile uint32_t period_sum_us;
    volatile uint32_t period_count;
    volatile uint32_t batch_min_us;
    volatile uint32_t batch_max_us;
    volatile uint32_t batch_sum_us;
    volatile uint32_t batch_count;
    volatile uint32_t batch_frames_ok;
    volatile uint32_t batch_frames_err;
    /* RTIK diagnostic snapshot; status_count is indexed by ArmIkStatus. */
    volatile uint8_t rt_ik_last_status;
    volatile uint32_t rt_ik_last_elapsed_us;
    volatile uint32_t rt_ik_max_elapsed_us;
    volatile float rt_ik_last_position_error;
    volatile uint32_t rt_ik_status_count[8];
    /* Largest joint tracking lag since the last RTIK report. */
    volatile float rt_max_lag;
    volatile uint8_t rt_max_lag_joint;
} DmControlStats;

extern DmControlStats dm_control_stats;
