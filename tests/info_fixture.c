/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Deterministic motor state for host DMIF tests and the loopback responder. */
#include "app_state.h"
#include "dm_runtime.h"
#include "eth_state.h"
#include "app_control_mode.h"
#include "robot_controller.h"
#include "arm_gravity.h"

DM_Motor_t dm_motors[DM_MOTOR_COUNT];
uint8_t RobotController_OperationalStatus(void) { return 9; }
uint8_t AppControlMode_Profile(void) { return 1; }
uint8_t AppControlMode_ProfileReady(void) { return 1; }
const char *ArmGravity_ModelName(void) { return "H7C bare v1"; }

void info_fixture_init(void)
{
    eth_udp_command.control_mode = 4;
    for (unsigned i = 0; i < DM_MOTOR_COUNT; ++i) {
        dm_motor_runtime.enabled[i] = (0xa5U >> i) & 1U;
        dm_motors[i].position_max = 12.5f + i;
        dm_motors[i].velocity_max = 45.f - i;
        dm_motors[i].torque_max = 54.f - i * 2;
    }
}
