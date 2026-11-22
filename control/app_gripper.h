/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_GRIPPER_H
#define APP_GRIPPER_H

#include <stdint.h>

/* Gripper J8 command (DM_ETH_CMD_GRIPPER), the same in every control mode:
 * the arm streams do not drive J8.  Force-position: the drive moves to
 * position (joint frame, rad) at up to speed (rad/s); stalled on an object it
 * pushes with up to force, its current limit as a fraction of the maximum
 * phase current (0-1).  Returns the DMAR status: 0 applied, 3 out of range
 * or J8 not in force-position mode, 7 gripper not enabled. */
uint8_t AppGripper_Command(float position, float speed, float force);

#endif /* APP_GRIPPER_H */
