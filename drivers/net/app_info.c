/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_info.h"
#include "app_build_info.h"
#include "app_control_mode.h"
#include "app_state.h"
#include "arm_gravity.h"
#include "dm_runtime.h"
#include "eth_state.h"
#include "robot_controller.h"

static void copy_label(char *out, size_t capacity, const char *label)
{
  const size_t n = strlen(label);
  memcpy(out, label, n < capacity ? n : capacity - 1U);
}

uint16_t AppInfo_Reply(const InfrabotInfoRequest *request, InfrabotInfoReplyV2 *reply)
{
  if (request->magic != INFRABOT_MAGIC_INFO_REQUEST ||
      (request->version != INFRABOT_INFO_V2 &&
       !(request->version == INFRABOT_INFO_V1 && INFRABOT_PROTOCOL_MAJOR == 1U)) ||
      request->crc32 != infrabot_crc32(request, offsetof(InfrabotInfoRequest, crc32))) return 0U;

  memset(reply, 0, sizeof(*reply));
  InfrabotInfoReply *info = &reply->info;
  info->magic = INFRABOT_MAGIC_INFO_REPLY;
  info->version = request->version;
  info->status = 0U;
  info->controller_status = RobotController_OperationalStatus();
  info->control_mode = eth_udp_command.control_mode;
  info->motor_profile = AppControlMode_Profile();
  info->profile_ready = AppControlMode_ProfileReady();
  info->enabled_mask = 0U;
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; ++i)
  {
    if (dm_motor_runtime.enabled[i] != 0U) info->enabled_mask |= (uint8_t)(1U << i);
    info->position_min[i] = (i == DM_GRIPPER_INDEX) ? DM_GRIPPER_POSITION_MIN : -dm_motors[i].position_max;
    info->position_max[i] = (i == DM_GRIPPER_INDEX) ? DM_GRIPPER_POSITION_MAX : dm_motors[i].position_max;
    info->velocity_max[i] = dm_motors[i].velocity_max;
    info->torque_max[i] = dm_motors[i].torque_max;
  }
  info->axes = DM_ARM_MOTOR_COUNT;
  info->motor_count = DM_MOTOR_COUNT;
  info->sequence = request->sequence;
  info->capabilities = INFRABOT_CAP_JOINT_RT | INFRABOT_CAP_JOINT_NRT |
                          INFRABOT_CAP_MAINTENANCE | INFRABOT_CAP_PERSISTENCE |
                          INFRABOT_CAP_CARTESIAN_NRT | INFRABOT_CAP_CARTESIAN_RT |
                          INFRABOT_CAP_GROUPS | INFRABOT_CAP_TELEMETRY_V3 |
                          INFRABOT_CAP_CONTROLLED_STOP | INFRABOT_CAP_SELFCHECK |
                          INFRABOT_CAP_RESUME | INFRABOT_CAP_LOCAL_GRAVITY | INFRABOT_CAP_ADMITTANCE |
                          INFRABOT_CAP_RUN_LOG;
  info->crc32 = infrabot_crc32(info, offsetof(InfrabotInfoReply, crc32));
  if (request->version == INFRABOT_INFO_V1) return sizeof(*info);

  reply->protocol_major = INFRABOT_PROTOCOL_MAJOR;
  reply->protocol_minor = INFRABOT_PROTOCOL_MINOR;
  copy_label(reply->firmware_version, sizeof(reply->firmware_version), APP_FIRMWARE_VERSION);
  copy_label(reply->board, sizeof(reply->board), "fire-h743-v2");
  copy_label(reply->arm_model, sizeof(reply->arm_model), ArmGravity_ModelName());
  reply->crc32 = infrabot_crc32(reply, offsetof(InfrabotInfoReplyV2, crc32));
  return sizeof(*reply);
}
