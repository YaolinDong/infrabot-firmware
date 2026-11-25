/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Diagnostic snapshot: DMDR validation and DMDA framing. */
#include "app_diag.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static InfrabotDiagRequest request(uint32_t id)
{
  InfrabotDiagRequest r;
  memset(&r, 0, sizeof(r));
  r.magic = INFRABOT_MAGIC_DIAG_REQUEST;
  r.version = INFRABOT_DIAG_V1;
  r.request_id = id;
  r.crc32 = infrabot_crc32(&r, offsetof(InfrabotDiagRequest, crc32));
  return r;
}

int main(void)
{
  InfrabotDiagReply body, reply;
  memset(&body, 0x5A, sizeof(body)); /* header, reserved and CRC fields must be overwritten */
  body.uptime_ms = 123456U;
  body.control_hz = 1000U;
  body.heap_min_free_bytes = 40000U;
  body.stack_free_control_bytes = 812U;
  body.drive_state[3] = INFRABOT_DRIVE_OVERCURRENT;

  InfrabotDiagRequest r = request(0xA1B2C3D4U);
  memset(&reply, 0, sizeof(reply));
  assert(AppDiag_Reply(&r, &body, &reply) == sizeof(reply));
  assert(reply.magic == INFRABOT_MAGIC_DIAG_REPLY && reply.version == INFRABOT_DIAG_V1);
  assert(reply.request_id == 0xA1B2C3D4U);
  assert(reply.reserved0 == 0U && reply.reserved1 == 0U && reply.reserved2 == 0U);
  assert(reply.uptime_ms == 123456U && reply.control_hz == 1000U && reply.heap_min_free_bytes == 40000U);
  assert(reply.stack_free_control_bytes == 812U && reply.drive_state[3] == INFRABOT_DRIVE_OVERCURRENT);
  assert(reply.crc32 == infrabot_crc32(&reply, offsetof(InfrabotDiagReply, crc32)));

  /* Not a DMDR: the caller passes the datagram on. */
  r = request(1U); ++r.crc32;
  assert(AppDiag_Reply(&r, &body, &reply) == 0U);
  r = request(1U); r.version = 2U; r.crc32 = infrabot_crc32(&r, offsetof(InfrabotDiagRequest, crc32));
  assert(AppDiag_Reply(&r, &body, &reply) == 0U);
  r = request(1U); r.reserved = 1U; r.crc32 = infrabot_crc32(&r, offsetof(InfrabotDiagRequest, crc32));
  assert(AppDiag_Reply(&r, &body, &reply) == 0U);
  r = request(1U); r.magic = INFRABOT_MAGIC_PARAM_REQUEST; r.crc32 = infrabot_crc32(&r, offsetof(InfrabotDiagRequest, crc32));
  assert(AppDiag_Reply(&r, &body, &reply) == 0U);

  puts("DMDR/DMDA: framing, echo, CRC and rejection checks passed");
  return 0;
}
