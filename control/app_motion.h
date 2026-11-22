/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_MOTION_H
#define APP_MOTION_H
#include "arm_motion.h"

/* The DMOV request / DMOS reply (protocol.h).  For op=ARM_MOVEJ
 * command.joint is used; for op=ARM_MOVEP command.cartesian. */
typedef InfrabotMoveRequest AppMotionRequest;
typedef InfrabotMoveReply AppMotionReply;

/* Ethernet owns publish/take; the control task alone owns the planner. */
uint8_t AppMotion_Publish(const AppMotionRequest *request, uint32_t ip, uint16_t port);
uint8_t AppMotion_TakeReply(AppMotionReply *reply, uint32_t *ip, uint16_t *port);
uint8_t AppMotion_Owned(void);
uint8_t AppMotion_ReleaseHold(void); /* Only a completed hold can relinquish ownership. */
void AppMotion_RequestCancel(void);
/* Ends a move as the arm alone is disabled: no full DISABLE (gripper). */
void AppMotion_RequestRelease(void);
void AppMotion_Tick(uint8_t enabling);
/* A CAN frame of the move could not be sent: it fails (controlled stop). */
void AppMotion_TxFailed(void);
/* A controlled stop ends a move owning the arm (control task): ARM_CANCEL
 * releases it, another reason fails it (latched for DMOS).  No-op for a move
 * that has already failed. */
void AppMotion_Abort(uint8_t reason);
/* A failed move is still latched (owned, FAILED). */
uint8_t AppMotion_Failed(void);
void AppMotion_PrintStatus(void); /* Low-priority status task only. */
#endif
