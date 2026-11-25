/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Parameter service: DMPR requests against the real gravity, impedance and
 * admittance parameter modules. */
#include "app_param.h"
#include "app_admittance.h"
#include "app_gravity.h"
#include "app_impedance.h"
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static InfrabotParamRequest request(uint8_t op, uint16_t id, float value)
{
  InfrabotParamRequest r;
  memset(&r, 0, sizeof(r));
  r.magic = INFRABOT_MAGIC_PARAM_REQUEST;
  r.version = INFRABOT_PARAM_V1;
  r.op = op;
  r.request_id = 0x01020304U + op;
  r.param_id = id;
  r.value = value;
  r.crc32 = infrabot_crc32(&r, offsetof(InfrabotParamRequest, crc32));
  return r;
}

static InfrabotParamReply call(InfrabotParamRequest r, uint8_t may_write)
{
  InfrabotParamReply reply;
  memset(&reply, 0xA5, sizeof(reply));
  assert(AppParam_Handle(&r, may_write, &reply) == sizeof(reply));
  assert(reply.magic == INFRABOT_MAGIC_PARAM_REPLY && reply.version == INFRABOT_PARAM_V1);
  assert(reply.op == r.op && reply.request_id == r.request_id);
  assert(reply.param_count == AppParam_Count());
  assert(reply.crc32 == infrabot_crc32(&reply, offsetof(InfrabotParamReply, crc32)));
  return reply;
}

int main(void)
{
  const uint16_t count = AppParam_Count();
  uint16_t ids[64];
  assert(count == 22U && count <= 64U);
  /* Every listed parameter describes itself and starts at its default. */
  for (uint16_t i = 0U; i < count; ++i)
  {
    const InfrabotParamReply info = call(request(INFRABOT_PARAM_OP_INFO, i, 0.f), 0U);
    assert(info.result == INFRABOT_PARAM_OK);
    assert(info.name[0] != 0 && info.name[INFRABOT_PARAM_NAME_LENGTH - 1U] == 0);
    assert(info.unit[INFRABOT_PARAM_UNIT_LENGTH - 1U] == 0);
    assert(info.minimum <= info.default_value && info.default_value <= info.maximum);
    assert(info.requested == info.default_value && info.applied == info.default_value);
    assert((info.flags & INFRABOT_PARAM_FLAG_WRITABLE) != 0U);
    assert(info.group == INFRABOT_PARAM_ID_GROUP(info.param_id));
    if (strcmp(info.name, "adm.bias_learning") == 0)
      assert((info.flags & INFRABOT_PARAM_FLAG_BOOLEAN) != 0U && info.rate == 0.f);
    else
      assert(info.rate > 0.f && (info.flags & INFRABOT_PARAM_FLAG_BOOLEAN) == 0U);
    for (uint16_t j = 0U; j < i; ++j) assert(ids[j] != info.param_id);
    ids[i] = info.param_id;
    const InfrabotParamReply got = call(request(INFRABOT_PARAM_OP_GET, info.param_id, 0.f), 0U);
    assert(got.result == INFRABOT_PARAM_OK && got.param_id == info.param_id);
    assert(strcmp(got.name, info.name) == 0);
  }

  InfrabotParamReply r = call(request(INFRABOT_PARAM_OP_INFO, count, 0.f), 0U);
  assert(r.result == INFRABOT_PARAM_UNKNOWN && r.param_id == 0U && r.name[0] == 0);
  r = call(request(INFRABOT_PARAM_OP_GET, INFRABOT_PARAM_ID(9U, 0U), 0.f), 0U);
  assert(r.result == INFRABOT_PARAM_UNKNOWN);

  /* SET stores the requested value; the applied value only moves with the control task. */
  const uint16_t joint_k = INFRABOT_PARAM_ID(INFRABOT_PARAM_GROUP_IMPEDANCE, APP_IMP_JOINT_K);
  r = call(request(INFRABOT_PARAM_OP_GET, joint_k, 0.f), 0U);
  const float lo = r.minimum, hi = r.maximum, initial = r.default_value;
  r = call(request(INFRABOT_PARAM_OP_SET, joint_k, 12.f), 1U);
  assert(r.result == INFRABOT_PARAM_OK && r.requested == 12.f && r.applied == initial);
  AppImpedanceParameters ip;
  AppImpedance_GetParameters(&ip);
  assert(ip.requested[APP_IMP_JOINT_K] == 12.f);

  /* Out of range or not finite: refused, nothing changes. */
  const float bad[] = {lo - .01f, hi + .01f, NAN, INFINITY, -INFINITY};
  for (unsigned i = 0U; i < sizeof(bad) / sizeof(bad[0]); ++i)
  {
    r = call(request(INFRABOT_PARAM_OP_SET, joint_k, bad[i]), 1U);
    assert(r.result == INFRABOT_PARAM_RANGE && r.requested == 0.f && r.name[0] == 0);
    r = call(request(INFRABOT_PARAM_OP_GET, joint_k, 0.f), 0U);
    assert(r.result == INFRABOT_PARAM_OK && r.requested == 12.f);
  }
  r = call(request(INFRABOT_PARAM_OP_SET, joint_k, lo), 1U);
  assert(r.result == INFRABOT_PARAM_OK && r.requested == lo);
  r = call(request(INFRABOT_PARAM_OP_SET, joint_k, hi), 1U);
  assert(r.result == INFRABOT_PARAM_OK && r.requested == hi);

  /* A PC that does not control the arm may read but not write. */
  r = call(request(INFRABOT_PARAM_OP_SET, joint_k, 10.f), 0U);
  assert(r.result == INFRABOT_PARAM_BUSY);
  r = call(request(INFRABOT_PARAM_OP_GET, joint_k, 0.f), 0U);
  assert(r.result == INFRABOT_PARAM_OK && r.requested == hi);

  /* The other two modules. */
  AppGravityParameters gp;
  r = call(request(INFRABOT_PARAM_OP_SET, INFRABOT_PARAM_ID(INFRABOT_PARAM_GROUP_GRAVITY, APP_GRAVITY_DAMPING), 1.5f), 1U);
  assert(r.result == INFRABOT_PARAM_OK);
  AppGravity_GetParameters(&gp);
  assert(gp.value[APP_GRAVITY_DAMPING] == 1.5f);
  AppAdmittanceParameters ap;
  r = call(request(INFRABOT_PARAM_OP_SET, INFRABOT_PARAM_ID(INFRABOT_PARAM_GROUP_ADMITTANCE, APP_ADM_ZETA), 1.2f), 1U);
  assert(r.result == INFRABOT_PARAM_OK);
  AppAdmittance_GetParameters(&ap);
  assert(ap.requested[APP_ADM_ZETA] == 1.2f);

  /* Bias learning is a switch that takes effect at once. */
  const uint16_t bias = INFRABOT_PARAM_ID(INFRABOT_PARAM_GROUP_ADMITTANCE, APP_ADM_PARAMETERS);
  r = call(request(INFRABOT_PARAM_OP_GET, bias, 0.f), 0U);
  assert((r.flags & INFRABOT_PARAM_FLAG_BOOLEAN) != 0U && r.requested == 1.f);
  r = call(request(INFRABOT_PARAM_OP_SET, bias, 0.f), 1U);
  assert(r.result == INFRABOT_PARAM_OK && r.requested == 0.f && r.applied == 0.f && AppAdmittance_BiasLearning() == 0U);
  r = call(request(INFRABOT_PARAM_OP_SET, bias, .5f), 1U);
  assert(r.result == INFRABOT_PARAM_RANGE && AppAdmittance_BiasLearning() == 0U);
  r = call(request(INFRABOT_PARAM_OP_SET, bias, 2.f), 1U);
  assert(r.result == INFRABOT_PARAM_RANGE);
  r = call(request(INFRABOT_PARAM_OP_SET, bias, 1.f), 1U);
  assert(r.result == INFRABOT_PARAM_OK && AppAdmittance_BiasLearning() == 1U);

  /* SAVE is reserved. */
  r = call(request(INFRABOT_PARAM_OP_SAVE, joint_k, 0.f), 1U);
  assert(r.result == INFRABOT_PARAM_UNSUPPORTED);
  r = call(request(INFRABOT_PARAM_OP_SAVE, joint_k, 0.f), 0U);
  assert(r.result == INFRABOT_PARAM_BUSY);

  /* Malformed requests: an unknown op or a reserved field is an error reply;
   * a bad magic, version or CRC is not a DMPR at all. */
  r = call(request(9U, joint_k, 0.f), 1U);
  assert(r.result == INFRABOT_PARAM_INVALID);
  InfrabotParamRequest q = request(INFRABOT_PARAM_OP_GET, joint_k, 0.f);
  InfrabotParamReply scratch;
  q.reserved1 = 1U;
  q.crc32 = infrabot_crc32(&q, offsetof(InfrabotParamRequest, crc32));
  r = call(q, 1U);
  assert(r.result == INFRABOT_PARAM_INVALID);
  q = request(INFRABOT_PARAM_OP_GET, joint_k, 0.f);
  ++q.crc32;
  assert(AppParam_Handle(&q, 1U, &scratch) == 0U);
  q = request(INFRABOT_PARAM_OP_GET, joint_k, 0.f);
  q.magic ^= 1U;
  q.crc32 = infrabot_crc32(&q, offsetof(InfrabotParamRequest, crc32));
  assert(AppParam_Handle(&q, 1U, &scratch) == 0U);
  q = request(INFRABOT_PARAM_OP_GET, joint_k, 0.f);
  q.version = 2U;
  q.crc32 = infrabot_crc32(&q, offsetof(InfrabotParamRequest, crc32));
  assert(AppParam_Handle(&q, 1U, &scratch) == 0U);

  puts("DMPR: parameter list, GET/SET ranges, ownership, switch and malformed request checks passed");
  return 0;
}
