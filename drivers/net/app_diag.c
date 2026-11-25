/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_diag.h"
#include <stddef.h>
#include <string.h>

uint16_t AppDiag_Reply(const InfrabotDiagRequest *request, const InfrabotDiagReply *body,
                       InfrabotDiagReply *reply)
{
  if (request->magic != INFRABOT_MAGIC_DIAG_REQUEST || request->version != INFRABOT_DIAG_V1 ||
      request->reserved != 0U ||
      request->crc32 != infrabot_crc32(request, offsetof(InfrabotDiagRequest, crc32))) return 0U;
  *reply = *body;
  reply->magic = INFRABOT_MAGIC_DIAG_REPLY;
  reply->version = INFRABOT_DIAG_V1;
  reply->reserved0 = 0U;
  reply->request_id = request->request_id;
  reply->reserved1 = 0U;
  reply->reserved2 = 0U;
  reply->crc32 = infrabot_crc32(reply, offsetof(InfrabotDiagReply, crc32));
  return (uint16_t)sizeof(*reply);
}
