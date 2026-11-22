/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include <stdint.h>
#include "infrabot/protocol.h"

/* Controlled stop of the arm (INFRABOT_STOP_*, see protocol.h).  On a stop
 * condition the arm does not lose power: it leaves its control mode for IDLE
 * (the PC must switch modes again), its joint command brakes to rest and it
 * holds enabled.  A PC keeping it alive (heartbeat, any accepted command)
 * takes it over as a normal IDLE hold; without one it keeps holding, however
 * long: only a DISABLE from a PC removes power.  The module also supervises
 * the enabled arm drives: torque at its limit, drive faults, lost feedback.
 *
 * The control task owns the stop; other tasks only request one
 * (AppSafety_RequestStop).  Telemetry and status readers take the state
 * inside a critical section. */

/* Every control tick, after this tick's command is computed and before it is
 * sent: takes a requested stop, supervises the drives, brakes, holds. */
void AppSafety_Tick(void);
/* Starts a controlled stop now (control task).  joint: the joint involved,
 * INFRABOT_NO_JOINT if none; a NOT_FOLLOWING or TORQUE_LIMIT joint is held
 * where it is instead of where it was commanded.  During a stop a new reason
 * replaces the latched one; a pending DISABLE wins over a stop. */
void AppSafety_Stop(uint8_t reason, uint8_t joint, uint8_t detail);
/* From any task: a controlled stop at the next control tick (the PC's STOP). */
void AppSafety_RequestStop(uint8_t reason);
uint8_t AppSafety_StopRequested(void);
/* The latched stop (reason, mode to resume) is acknowledged: a successful
 * mode switch (a resume re-enters the interrupted mode; IDLE -> IDLE only
 * acknowledges) or an enable. */
void AppSafety_Clear(void);
/* LCD/local gravity owns an IDLE hold without a PC heartbeat. This never
 * exempts an external RT stream or NRT trajectory from its watchdog. */
void AppSafety_SetLocalHold(uint8_t enabled);
uint8_t AppSafety_LocalHold(void);
/* 1 while braking: the stop owns the joint command and its velocity. */
uint8_t AppSafety_Braking(void);
/* 1 while braking or holding without a PC (controller status STOPPING). */
uint8_t AppSafety_Stopping(void);
/* Joints whose measured torque is at the supervisor limit (bit i = J(i+1)). */
uint8_t AppSafety_TorqueLimitMask(void);
/* Fills the stop_* fields of the telemetry block. */
void AppSafety_Report(InfrabotSafetyStatus *status);
/* No latched stop, or one an operator RESUME may acknowledge: communication
 * loss or an operator stop.  Hardware, limit and target faults need
 * inspection and a re-enable. */
static inline uint8_t AppSafety_Resumable(uint8_t reason)
{
  return reason == INFRABOT_STOP_NONE || reason == INFRABOT_STOP_REQUESTED ||
         reason == INFRABOT_STOP_STREAM_LOST || reason == INFRABOT_STOP_MOVE_LOST;
}

#endif /* APP_SAFETY_H */
