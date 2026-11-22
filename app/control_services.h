/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <stdint.h>

/* Task-owner queries and requests available to the control state machines. */
uint8_t AppTasks_Enabling(void);
/* 1 while J1-J7 do not all answer: bring-up waiting for arm power (arm
 * powered after the MCU), or the arm lost power at runtime (app_arm_power.h);
 * *responding = arm motors that answered the last probe. */
uint8_t AppStartup_ArmWaiting(uint8_t *responding);
/* 1 while the bus is unusable now: bus-off, or error passive (TEC or REC
 * >= 128).  A single protocol error is not a fault; the counters recover on
 * their own and dm_bus_stats.can_error_events records it.  Side-effect free for the
 * status task's PSR/LEC reporting (reads ECR and CCCR only). */
uint8_t AppFdcan_BusFault(void);
void AppDamiao_Cancel(void);
/* A group disable: cancels maintenance on the drives in mask only. */
void AppDamiao_CancelMotors(uint32_t mask);
/* Control-task LCD setup: runs the same asynchronous profile/self-check,
 * without sending a fabricated UDP acknowledgement. */
uint8_t AppDamiao_StartLocalProfile(uint8_t mode);
uint8_t AppDamiao_LocalProfileStatus(void);
uint16_t AppDamiao_ModeId(uint8_t mode);
