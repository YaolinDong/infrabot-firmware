/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_ENABLE_H
#define APP_ENABLE_H

#include <stdint.h>

/* Staged enable and disable of drive groups (DM_ARM_MOTOR_MASK J1-J7,
 * DM_GRIPPER_MOTOR_MASK J8, or both).  Called from the 1 kHz control task
 * only; AppEnable_InProgress() may be read from any task. */

/* Starts enabling the drives in mask that are not enabled yet: first each
 * must answer a feedback request, then one at a time clear error, fresh
 * feedback, hold frame, enable, hold frame.  Drives that are already enabled
 * keep holding their command.  Returns 0 when there is nothing to enable or
 * a sequence is already running. */
uint8_t AppEnable_Start(uint32_t mask);
/* One control tick of the sequence: non-zero while normal output must wait,
 * during which AppEnable_HoldFrames() commands the enabled drives instead.  A
 * drive that does not answer (not connected: nothing gets enabled) or accept
 * a frame fails the sequence, which then disables every drive it was
 * bringing up: never a partial group. */
uint8_t AppEnable_Step(void);
void AppEnable_HoldFrames(void);
uint8_t AppEnable_InProgress(void);
/* Explicit STOP during staged enable: cancel remaining steps and hold the
 * drives already powered. Never withdraw torque from an enabled drive. */
void AppEnable_CancelAndHold(void);
/* Disables the drives in mask now, aborting a running sequence together with
 * the drives it was bringing up.  Without all of J1-J7 the control mode
 * returns to IDLE; with no drive left the command watchdog stops. */
void AppEnable_Disable(uint32_t mask);

#endif /* APP_ENABLE_H */
