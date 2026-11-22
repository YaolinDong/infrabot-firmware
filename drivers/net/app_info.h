/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include "infrabot/protocol.h"

/* Returns the wire length, or zero for a malformed or unsupported request. */
uint16_t AppInfo_Reply(const InfrabotInfoRequest *request, InfrabotInfoReplyV2 *reply);
