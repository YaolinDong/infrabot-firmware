/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_param.h"
#include "app_admittance.h"
#include "app_gravity.h"
#include "app_impedance.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define PARAM_RW INFRABOT_PARAM_FLAG_WRITABLE
#define PARAM_BOOL (INFRABOT_PARAM_FLAG_WRITABLE | INFRABOT_PARAM_FLAG_BOOLEAN)
/* Admittance index past the slewed gains: the bias learning switch. */
#define ADM_BIAS_LEARNING APP_ADM_PARAMETERS

typedef struct {
  uint8_t group, index, flags;
  char name[INFRABOT_PARAM_NAME_LENGTH];
  char unit[INFRABOT_PARAM_UNIT_LENGTH]; /* at most 7 characters: always NUL-terminated */
} ParamEntry;

typedef struct {
  float requested, applied, minimum, maximum, rate, default_value;
} ParamValues;

/* INFO list order. Ids are group << 8 | index, so they stay fixed when
 * parameters are appended to a group. */
static const ParamEntry table[] = {
  {INFRABOT_PARAM_GROUP_GRAVITY, APP_GRAVITY_DAMPING, PARAM_RW, "gravity.damping", "Nms/rad"},
  {INFRABOT_PARAM_GROUP_GRAVITY, APP_GRAVITY_STIFFNESS, PARAM_RW, "gravity.lock_kp", "Nm/rad"},
  {INFRABOT_PARAM_GROUP_IMPEDANCE, APP_IMP_JOINT_K, PARAM_RW, "imp.joint_k", "Nm/rad"},
  {INFRABOT_PARAM_GROUP_IMPEDANCE, APP_IMP_JOINT_D, PARAM_RW, "imp.joint_d", "Nms/rad"},
  {INFRABOT_PARAM_GROUP_IMPEDANCE, APP_IMP_TRANSLATION_K, PARAM_RW, "imp.linear_k", "N/m"},
  {INFRABOT_PARAM_GROUP_IMPEDANCE, APP_IMP_TRANSLATION_D, PARAM_RW, "imp.linear_d", "Ns/m"},
  {INFRABOT_PARAM_GROUP_IMPEDANCE, APP_IMP_ROTATION_K, PARAM_RW, "imp.rotation_k", "Nm/rad"},
  {INFRABOT_PARAM_GROUP_IMPEDANCE, APP_IMP_ROTATION_D, PARAM_RW, "imp.rotation_d", "Nms/rad"},
  {INFRABOT_PARAM_GROUP_IMPEDANCE, APP_IMP_JOINT_DAMPING, PARAM_RW, "imp.joint_damping", "Nms/rad"},
  {INFRABOT_PARAM_GROUP_ADMITTANCE, APP_ADM_JOINT_K, PARAM_RW, "adm.joint_k", "Nm/rad"},
  {INFRABOT_PARAM_GROUP_ADMITTANCE, APP_ADM_JOINT_MASS, PARAM_RW, "adm.joint_mass", "x"},
  {INFRABOT_PARAM_GROUP_ADMITTANCE, APP_ADM_LINEAR_K, PARAM_RW, "adm.linear_k", "N/m"},
  {INFRABOT_PARAM_GROUP_ADMITTANCE, APP_ADM_ROTATION_K, PARAM_RW, "adm.rotation_k", "Nm/rad"},
  {INFRABOT_PARAM_GROUP_ADMITTANCE, APP_ADM_MASS, PARAM_RW, "adm.mass", "kg"},
  {INFRABOT_PARAM_GROUP_ADMITTANCE, APP_ADM_ZETA, PARAM_RW, "adm.zeta", ""},
  {INFRABOT_PARAM_GROUP_ADMITTANCE, APP_ADM_STATIC_RATIO, PARAM_RW, "adm.static_ratio", "x"},
  {INFRABOT_PARAM_GROUP_ADMITTANCE, APP_ADM_FRICTION, PARAM_RW, "adm.friction_beta", ""},
  {INFRABOT_PARAM_GROUP_ADMITTANCE, APP_ADM_DRIVE_GAIN, PARAM_RW, "adm.drive_gain", "x"},
  {INFRABOT_PARAM_GROUP_ADMITTANCE, APP_ADM_NULL_K, PARAM_RW, "adm.null_k", "Nm/rad"},
  {INFRABOT_PARAM_GROUP_ADMITTANCE, APP_ADM_NULL_D, PARAM_RW, "adm.null_d", "Nms/rad"},
  {INFRABOT_PARAM_GROUP_ADMITTANCE, APP_ADM_RETURN_SPEED, PARAM_RW, "adm.return_speed", "mm/s"},
  {INFRABOT_PARAM_GROUP_ADMITTANCE, ADM_BIAS_LEARNING, PARAM_BOOL, "adm.bias_learning", ""},
};
#define PARAM_COUNT (sizeof(table) / sizeof(table[0]))
_Static_assert(PARAM_COUNT == (size_t)(APP_GRAVITY_PARAMETERS + APP_IMP_PARAMETERS + APP_ADM_PARAMETERS + 1),
               "every parameter of the three modules is listed");

uint16_t AppParam_Count(void) { return (uint16_t)PARAM_COUNT; }

static const ParamEntry *find(uint16_t id)
{
  for (size_t i = 0U; i < PARAM_COUNT; ++i)
    if (INFRABOT_PARAM_ID(table[i].group, table[i].index) == id) return &table[i];
  return NULL;
}

static void read_values(const ParamEntry *entry, ParamValues *out)
{
  const uint8_t i = entry->index;
  switch (entry->group)
  {
    case INFRABOT_PARAM_GROUP_GRAVITY:
    {
      AppGravityParameters p;
      AppGravity_GetParameters(&p);
      *out = (ParamValues){p.value[i], p.applied[i], AppGravity_ParameterMin(i), AppGravity_ParameterMax(i),
                           AppGravity_ParameterRate(i), AppGravity_ParameterDefault(i)};
      break;
    }
    case INFRABOT_PARAM_GROUP_IMPEDANCE:
    {
      AppImpedanceParameters p;
      AppImpedance_GetParameters(&p);
      *out = (ParamValues){p.requested[i], p.applied[i], AppImpedance_ParameterMin(i), AppImpedance_ParameterMax(i),
                           AppImpedance_ParameterRate(i), AppImpedance_ParameterDefault(i)};
      break;
    }
    default:
    {
      if (i == ADM_BIAS_LEARNING)
      {
        /* A switch, not a gain: it takes effect at once, so nothing slews. */
        const float on = AppAdmittance_BiasLearning() ? 1.f : 0.f;
        *out = (ParamValues){on, on, 0.f, 1.f, 0.f, 1.f};
        break;
      }
      AppAdmittanceParameters p;
      AppAdmittance_GetParameters(&p);
      *out = (ParamValues){p.requested[i], p.applied[i], AppAdmittance_ParameterMin(i), AppAdmittance_ParameterMax(i),
                           AppAdmittance_ParameterRate(i), AppAdmittance_ParameterDefault(i)};
      break;
    }
  }
}

static uint8_t write_value(const ParamEntry *entry, float value)
{
  switch (entry->group)
  {
    case INFRABOT_PARAM_GROUP_GRAVITY: return AppGravity_SetParameter(entry->index, value);
    case INFRABOT_PARAM_GROUP_IMPEDANCE: return AppImpedance_SetParameter(entry->index, value);
    default:
      if (entry->index == ADM_BIAS_LEARNING)
      {
        AppAdmittance_SetBiasLearning(value != 0.f);
        return 1U;
      }
      return AppAdmittance_SetParameter(entry->index, value);
  }
}

/* Runs the request and fills the descriptor of the addressed parameter on
 * success. Returns an INFRABOT_PARAM_* result. */
static uint8_t execute(const InfrabotParamRequest *request, uint8_t may_write, InfrabotParamReply *out)
{
  if (request->op > INFRABOT_PARAM_OP_SAVE || request->reserved0 != 0U || request->reserved1 != 0U)
    return INFRABOT_PARAM_INVALID;
  if ((request->op == INFRABOT_PARAM_OP_SET || request->op == INFRABOT_PARAM_OP_SAVE) && !may_write)
    return INFRABOT_PARAM_BUSY;
  if (request->op == INFRABOT_PARAM_OP_SAVE) return INFRABOT_PARAM_UNSUPPORTED;

  const ParamEntry *entry = request->op == INFRABOT_PARAM_OP_INFO
                                ? (request->param_id < PARAM_COUNT ? &table[request->param_id] : NULL)
                                : find(request->param_id);
  if (entry == NULL) return INFRABOT_PARAM_UNKNOWN;

  ParamValues v;
  read_values(entry, &v);
  if (request->op == INFRABOT_PARAM_OP_SET)
  {
    const float value = request->value;
    const uint8_t boolean = (entry->flags & INFRABOT_PARAM_FLAG_BOOLEAN) != 0U;
    if ((entry->flags & INFRABOT_PARAM_FLAG_WRITABLE) == 0U) return INFRABOT_PARAM_READ_ONLY;
    if (!isfinite(value) || value < v.minimum || value > v.maximum ||
        (boolean && value != 0.f && value != 1.f) || !write_value(entry, value))
      return INFRABOT_PARAM_RANGE;
    read_values(entry, &v); /* the reply shows the state after the write */
  }

  out->param_id = INFRABOT_PARAM_ID(entry->group, entry->index);
  out->requested = v.requested;
  out->applied = v.applied;
  out->minimum = v.minimum;
  out->maximum = v.maximum;
  out->default_value = v.default_value;
  out->rate = v.rate;
  out->flags = entry->flags;
  out->group = entry->group;
  /* The reply is zeroed first, so the last byte of each field stays NUL. */
  memcpy(out->name, entry->name, sizeof(out->name) - 1U);
  memcpy(out->unit, entry->unit, sizeof(out->unit) - 1U);
  return INFRABOT_PARAM_OK;
}

uint16_t AppParam_Handle(const InfrabotParamRequest *request, uint8_t may_write, InfrabotParamReply *reply)
{
  if (request->magic != INFRABOT_MAGIC_PARAM_REQUEST || request->version != INFRABOT_PARAM_V1 ||
      request->crc32 != infrabot_crc32(request, offsetof(InfrabotParamRequest, crc32)))
    return 0U;

  memset(reply, 0, sizeof(*reply));
  reply->magic = INFRABOT_MAGIC_PARAM_REPLY;
  reply->version = INFRABOT_PARAM_V1;
  reply->op = request->op;
  reply->request_id = request->request_id;
  reply->param_count = AppParam_Count();
  reply->result = execute(request, may_write, reply);
  reply->crc32 = infrabot_crc32(reply, offsetof(InfrabotParamReply, crc32));
  return (uint16_t)sizeof(*reply);
}
