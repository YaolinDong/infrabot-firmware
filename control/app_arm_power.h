/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_ARM_POWER_H
#define APP_ARM_POWER_H

#include <stdint.h>

/* The arm loses power while the MCU stays on (switched off first, E-stop,
 * cable).  Nobody ACKs on the bus then: the controller goes error passive or
 * bus-off and never recovers by itself, so the status would read FAULT even
 * after the arm is powered again.
 *
 * Every control tick: when the bus is faulted and none of J1-J7 has answered
 * for DM_ARM_POWER_LOST_MS, the arm has no power.  Enabled drives are marked
 * disabled (they are off already, and nothing would reach them), then once a
 * second the CAN controller is re-initialized and J1-J7 are asked for
 * feedback.  When all seven answer two probes in a row the arm is back: the
 * controller is reset once more, and since the drives rebooted with their
 * stored CTRL_MODE the profile is cleared, so ENABLE applies it and the
 * self-check again.  Never enables anything. */
void AppArmPower_Watch(void);
/* 1 while waiting for the arm to come back; *responding = arm drives that
 * answered the last probe. */
uint8_t AppArmPower_Waiting(uint8_t *responding);

#endif /* APP_ARM_POWER_H */
