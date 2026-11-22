/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_runtime.h"

DmMotorRuntime dm_motor_runtime;

volatile float dm_posvel_kp_apr[DM_MOTOR_COUNT] = {
  DM_POSVEL_KP_APR_DEFAULT, DM_POSVEL_KP_APR_DEFAULT, DM_POSVEL_KP_APR_DEFAULT,
  DM_POSVEL_KP_APR_DEFAULT, DM_POSVEL_KP_APR_DEFAULT, DM_POSVEL_KP_APR_DEFAULT,
  DM_POSVEL_KP_APR_DEFAULT, DM_POSVEL_KP_APR_DEFAULT
};
