/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include "infrabot/protocol.h"

/* Parameter service (DMPR -> DMPA): the tunable gravity, impedance and
 * admittance settings by id. Pure request/reply logic over the three
 * parameter modules; the Ethernet task only moves the datagrams. */

/* Number of parameters the service lists (INFO positions 0 .. count - 1). */
uint16_t AppParam_Count(void);

/* Handles one DMPR datagram. Returns the wire length of *reply, or zero when
 * the datagram is not a valid DMPR (wrong magic, version or CRC), which the
 * caller treats like any other unknown datagram. may_write is 1 when the
 * sender controls the arm: SET and SAVE from anyone else answer
 * INFRABOT_PARAM_BUSY. */
uint16_t AppParam_Handle(const InfrabotParamRequest *request, uint8_t may_write,
                         InfrabotParamReply *reply);
