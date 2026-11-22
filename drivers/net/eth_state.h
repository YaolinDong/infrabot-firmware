/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <stdint.h>
#include "app_config.h"

#define DM_ETH_CMD_QUEUE_DEPTH 16U

typedef struct {
    volatile uint8_t control_request;
    /* Independent J1–J7/J8 group masks, consumed by the control task. */
    volatile uint8_t enable_request;
    volatile uint8_t disable_request;
    /* Command timing counters, maintained by the lwIP owner. */
    volatile uint32_t rx_count;
    volatile uint32_t interval_count;
    volatile uint32_t interval_min_us;
    volatile uint32_t interval_max_us;
    volatile uint32_t interval_sum_us;
    volatile uint32_t last_interval_us;
    volatile uint32_t jitter_sum_us;
    volatile uint32_t jitter_max_us;
    volatile uint32_t last_cycle;
    volatile float target_position[DM_MOTOR_COUNT];
    volatile float pending_position[DM_MOTOR_COUNT];
    /* Base-frame XYZ (metres), then row-major 3x3 orientation. */
    volatile float cartesian_pose[12];
    /* Wire speed limits (zero selects default). */
    volatile uint16_t cartesian_limits[4];
    volatile uint8_t cartesian_target_valid;
    volatile uint8_t motor_mode[DM_MOTOR_COUNT];
    /* Supervisory mode is independent of the per-drive motor profile. */
    volatile uint8_t control_mode;
    volatile float target_velocity[DM_MOTOR_COUNT];
    volatile float target_kp[DM_MOTOR_COUNT];
    volatile float target_kd[DM_MOTOR_COUNT];
    volatile float target_torque[DM_MOTOR_COUNT];
    volatile float velocity_limit[DM_MOTOR_COUNT];
    volatile float torque_limit_ratio[DM_MOTOR_COUNT];
    volatile float queue_position[DM_ETH_CMD_QUEUE_DEPTH][DM_MOTOR_COUNT];
    volatile uint32_t queue_sequence[DM_ETH_CMD_QUEUE_DEPTH];
    volatile uint64_t queue_execute_us[DM_ETH_CMD_QUEUE_DEPTH];
    volatile uint8_t queue_head;
    volatile uint8_t queue_tail;
    volatile uint8_t queue_count;
    volatile uint32_t last_sequence;
    volatile uint32_t applied_sequence;
    volatile uint32_t pending_sequence;
    volatile uint64_t pending_execute_us;
    volatile uint64_t last_rx_mcu_us;
    volatile uint64_t applied_mcu_us;
    volatile uint32_t last_tick;
    volatile uint8_t active;
    volatile uint8_t sequence_valid;
    volatile uint32_t invalid_count;
    /* Last rejected datagram: DM_ETH_INVALID_* and length in bytes. */
    volatile uint8_t invalid_reason;
    volatile uint16_t invalid_length;
} EthUdpCommand;

extern EthUdpCommand eth_udp_command;

typedef struct {
    /* Ethernet callback publishes requests; the control task performs CAN I/O. */
    volatile uint8_t pending;
    volatile uint8_t command;
    volatile uint8_t motor_index;
    volatile uint8_t mode;
    volatile uint8_t value_type;
    volatile uint8_t register_id;
    volatile uint32_t value;
    volatile int32_t position_mrad;
    volatile int32_t velocity_mrad_s;
    volatile int32_t kp_milli;
    volatile int32_t kd_milli;
    volatile int32_t torque_mNm;
    volatile int32_t velocity_limit_mrad_s;
    volatile int32_t torque_ratio_milli;
    volatile uint32_t sequence;
    volatile uint32_t pending_tick;
    volatile uint32_t trace_head;
    volatile uint8_t waiting;
    volatile uint8_t waiting_motor_index;
    volatile uint8_t waiting_register_id;
    volatile uint8_t waiting_value_type;
    volatile uint8_t waiting_command;
    volatile uint32_t waiting_sequence;
    volatile uint32_t waiting_tick;
    /* Pending MCU-side profile validation and Flash save. */
    volatile uint8_t profile_pending;
    volatile uint32_t profile_sequence;
} EthMaintenance;

extern EthMaintenance eth_maintenance;

typedef struct {
    volatile uint32_t count;
    volatile uint32_t interval_count;
    volatile uint32_t interval_min_us;
    volatile uint32_t interval_max_us;
    volatile uint32_t interval_sum_us;
    volatile uint32_t last_interval_us;
    volatile uint32_t jitter_sum_us;
    volatile uint32_t jitter_max_us;
    volatile uint32_t errors;
    volatile uint32_t last_cycle;
} EthUdpTxStats;

extern EthUdpTxStats eth_udp_tx_stats;
