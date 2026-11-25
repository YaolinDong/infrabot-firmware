/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include "infrabot/protocol.h"

/* Diagnostic snapshot (DMDR -> DMDA). Pure request/reply logic: the caller
 * collects the measurements into *body (header and CRC fields ignored) and
 * this module validates the request and frames the reply. */

/* Returns the wire length of *reply, or zero when the datagram is not a valid
 * DMDR (wrong magic, version, reserved field or CRC), which the caller treats
 * like any other unknown datagram. */
uint16_t AppDiag_Reply(const InfrabotDiagRequest *request, const InfrabotDiagReply *body,
                       InfrabotDiagReply *reply);
