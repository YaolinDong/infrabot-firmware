/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_runtime.h"
#include "eth_state.h"
#include "platform.h"
#include "app_motion.h"
#include "app_state.h"
#include "app_safety.h"
#include <math.h>
#include <stdio.h>
#include <string.h>


static ArmMotion motion;
static AppMotionRequest inbox;
static AppMotionReply outbox;
static uint32_t request_ip, reply_ip, owner_ip;
static uint16_t request_port, reply_port, owner_port;
static volatile uint8_t pending, reply_pending, cancel_requested;
static uint32_t rx_count, mailbox_drops;
/* This move's failure has started (or came from) a controlled stop. */
static uint8_t stop_reported;

static uint64_t AppMotion_TimeUs(void *context)
{
  (void)context;
  return platform_now_us();
}

uint8_t AppMotion_Owned(void)
{
  /* All callers run in tasks. Prevent a legacy packet racing a queued start. */
  platform_enter_critical();
  const uint8_t owned = motion.owned || (pending &&
                         (inbox.op == ARM_MOVEJ || inbox.op == ARM_MOVEP));
  platform_exit_critical();
  return owned;
}

void AppMotion_RequestCancel(void) { cancel_requested = 1; }

/* 2: the arm is being disabled on its own (DISABLE_ARM).  End the move
 * without the full DISABLE a cancel requests, so an enabled gripper keeps
 * holding; a full cancel already requested stays one. */
void AppMotion_RequestRelease(void) { if (!cancel_requested) cancel_requested = 2; }

/* A completed hold, or a failure the controlled stop has handled: a mode
 * switch acknowledges it. */
uint8_t AppMotion_ReleaseHold(void)
{
  if (pending || cancel_requested ||
      (motion.owned && motion.state != ARM_DONE && motion.state != ARM_FAILED)) return 0U;
  if (motion.owned) ArmMotion_Cancel(&motion);
  return 1U;
}

uint8_t AppMotion_Failed(void)
{
  return motion.owned && motion.state == ARM_FAILED;
}

void AppMotion_Abort(uint8_t reason)
{
  if (!motion.owned || motion.state == ARM_FAILED) return;
  stop_reported = 1U;
  if (reason == ARM_CANCEL) ArmMotion_Cancel(&motion);
  else ArmMotion_Fail(&motion, reason);
}

uint8_t AppMotion_Publish(const AppMotionRequest *request, uint32_t ip, uint16_t port)
{
  platform_enter_critical();
  if (pending || reply_pending) { mailbox_drops++; platform_exit_critical(); return 0; }
  rx_count++;
  inbox = *request;
  request_ip = ip;
  request_port = port;
  pending = 1;
  platform_exit_critical();
  return 1;
}

uint8_t AppMotion_TakeReply(AppMotionReply *reply, uint32_t *ip, uint16_t *port)
{
  platform_enter_critical();
  if (!reply_pending) { platform_exit_critical(); return 0; }
  *reply = outbox;
  *ip = reply_ip;
  *port = reply_port;
  reply_pending = 0;
  platform_exit_critical();
  return 1;
}

static void disable_output(void)
{
  /* DISABLE during a move: no more motion frames this tick; the staged
   * control owner disables the drives next tick.  Disable is not a
   * safety-rated brake / E-stop. */
  dm_motor_runtime.control_enabled = 0;
  eth_udp_command.control_mode = DM_ETH_CONTROL_IDLE;
  eth_udp_command.active = 0;
  eth_udp_command.sequence_valid = 0;
  eth_udp_command.queue_head = eth_udp_command.queue_tail = eth_udp_command.queue_count = 0;
  eth_udp_command.control_request = DM_ETH_CMD_DISABLE;
}

/* The next tick turns the failure into a controlled stop. */
void AppMotion_TxFailed(void)
{
  if (motion.owned) ArmMotion_Fail(&motion, ARM_TX_ERROR);
}

/* A failed move: the controlled stop that matches its reason.  A Cartesian
 * MOTOR_FAULT with every drive healthy is the tracking check: a joint left
 * behind its command. */
static void stop_after_failure(const ArmMoveFeedback *fb)
{
  uint8_t joint = INFRABOT_NO_JOINT;
  switch (motion.reason)
  {
    case ARM_COMM_TIMEOUT:
      AppSafety_Stop(INFRABOT_STOP_MOVE_LOST, INFRABOT_NO_JOINT, 0U);
      return;
    case ARM_FEEDBACK_STALE:
      for (uint8_t i = 0U; i < ARM_AXES && joint == INFRABOT_NO_JOINT; ++i)
        if (!(fb->fresh_mask & (1UL << i))) joint = i;
      AppSafety_Stop(INFRABOT_STOP_FEEDBACK_LOST, joint, 0U);
      return;
    case ARM_MOTOR_FAULT:
      for (uint8_t i = 0U; i < ARM_AXES && joint == INFRABOT_NO_JOINT; ++i)
        if (!(fb->healthy_mask & (1UL << i))) joint = i;
      if (joint != INFRABOT_NO_JOINT)
      {
        AppSafety_Stop(INFRABOT_STOP_DRIVE_FAULT, joint, dm_motors[joint].state);
        return;
      }
      {
        float lag = 0.0f;
        for (uint8_t i = 0U; i < ARM_AXES; ++i)
          if (fabsf(fb->position[i] - motion.position[i]) > lag)
          {
            lag = fabsf(fb->position[i] - motion.position[i]);
            joint = i;
          }
      }
      AppSafety_Stop(INFRABOT_STOP_NOT_FOLLOWING, joint, 0U);
      return;
    default:
      AppSafety_Stop(INFRABOT_STOP_MOVE_FAILED, INFRABOT_NO_JOINT, motion.reason);
      return;
  }
}

void AppMotion_PrintStatus(void)
{
  static const char *const names[] = {"IDLE", "RUNNING", "SETTLING", "DONE", "FAILED", "CANCELLED"};
  uint32_t id, arrived, fresh, enabled, healthy, age, count, drops;
  uint8_t state, reason, owned;
  float error;
  platform_enter_critical();
  id = motion.id; state = motion.state; reason = motion.reason; owned = motion.owned;
  arrived = motion.arrived_mask; error = motion.max_error;
  fresh = enabled = healthy = 0U;
  for (unsigned i = 0U; i < ARM_AXES; ++i) {
    if (dm_motors[i].feedback_count && platform_now_ms() - dm_motors[i].feedback_tick <= ARM_MOVE_FEEDBACK_MS)
      fresh |= 1U << i;
    if (dm_motor_runtime.enabled[i]) enabled |= 1U << i;
    if (dm_motors[i].state == 1U) healthy |= 1U << i;
  }
  age = platform_now_ms() - motion.last_heartbeat_ms;
  count = rx_count; drops = mailbox_drops; rx_count = mailbox_drops = 0;
  platform_exit_critical();
  /* Every second while a move owns the arm; afterwards only once per
   * state change, so a finished session does not repeat forever. */
  static uint32_t printed_id;
  static uint8_t printed_state = 0xFFU, printed_reason = 0xFFU;
  const uint8_t changed = id != printed_id || state != printed_state || reason != printed_reason;
  printed_id = id; printed_state = state; printed_reason = reason;
  if (id && (owned || changed || drops))
    printf("MOVE id=%lu state=%s reason=%u owned=%u rx=%lu mailbox_drop=%lu "
                 "arrived=0x%02lX fresh=0x%02lX enabled=0x%02lX healthy=0x%02lX "
                 "error_mrad=%ld heartbeat_age_ms=%lu\r\n",
                 (unsigned long)id, names[state], reason, owned, (unsigned long)count,
                 (unsigned long)drops, (unsigned long)arrived, (unsigned long)fresh,
                 (unsigned long)enabled, (unsigned long)healthy,
                 (long)(error * 1000.0f),
                 (unsigned long)age);
}

void AppMotion_Tick(uint8_t enabling)
{
  if (!motion.owned && !pending && !cancel_requested) return;
  const uint32_t now = platform_now_ms();
  ArmMoveFeedback fb = {0};
  platform_enter_critical();
  fb.ready = !enabling && dm_motor_runtime.control_enabled &&
             (!eth_maintenance.pending || eth_maintenance.command == DM_ETH_CMD_SET_CONTROL_MODE) &&
             !eth_maintenance.waiting && !eth_udp_command.control_request;
  if ((eth_udp_command.control_mode != DM_ETH_CONTROL_NRT_JOINT &&
       eth_udp_command.control_mode != DM_ETH_CONTROL_NRT_CARTESIAN) || !dm_motor_runtime.profile_ready ||
      eth_udp_command.motor_mode[DM_GRIPPER_INDEX] != DM_ETH_MODE_FORCE_POS)
    fb.ready = 0;
  const uint8_t mode = eth_udp_command.motor_mode[0];
  if (mode != DM_ETH_MODE_MIT && mode != DM_ETH_MODE_POS_VEL) fb.ready = 0;
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    const float direction = DM_MOTOR_DIRECTION(i);
    fb.position[i] = (dm_motors[i].position - dm_motor_runtime.zero_offset[i]) * direction;
    fb.velocity[i] = dm_motors[i].velocity * direction;
    fb.command[i] = (dm_motor_runtime.command_position[i] - dm_motor_runtime.zero_offset[i]) * direction;
    fb.lower[i] = -dm_motors[i].position_max;
    fb.upper[i] = dm_motors[i].position_max;
    fb.velocity_max[i] = fminf(dm_motors[i].velocity_max, DM_ARM_MAX_SPEED_RAD_S);
    fb.torque_max[i] = dm_motors[i].torque_max;
    if (dm_motor_runtime.enabled[i]) fb.enabled_mask |= 1U << i;
    if (dm_motors[i].feedback_count && now - dm_motors[i].feedback_tick <= ARM_MOVE_FEEDBACK_MS)
      fb.fresh_mask |= 1U << i;
    if (dm_motors[i].state == 1U) fb.healthy_mask |= 1U << i;
    if (eth_udp_command.motor_mode[i] != mode) fb.ready = 0;
  }
  fb.command_valid = dm_motor_runtime.control_enabled;
  for (unsigned i = 0; i < ARM_AXES; ++i)
    if (eth_udp_command.motor_mode[i] == DM_ETH_MODE_POS_VEL) fb.posvel_mask |= 1UL << i;
  static const float rated[ARM_AXES] = DM_ARM_RATED_SPEED_RAD_S;
  memcpy(fb.rated_speed, rated, sizeof(fb.rated_speed));
  fb.time_source = (ArmIkTimeSource)AppMotion_TimeUs;
  fb.time_source_context = NULL;
  platform_exit_critical();

  if (cancel_requested) {
    /* 1: DISABLE (the drives go off), 2: the arm alone disabled. */
    const uint8_t full = cancel_requested == 1;
    cancel_requested = 0;
    if (motion.owned) {
      ArmMotion_Cancel(&motion);
      if (full) disable_output();
    }
    /* A DISABLE arriving after a queued MOVE must win, never start it. */
    fb.ready = 0;
  }
  /* Disabled without a cancel (a register read found a drive in another
   * control mode, app_damiao.c): the latched failure ends with the drives. */
  if (motion.owned && motion.state == ARM_FAILED && !dm_motor_runtime.control_enabled) ArmMotion_Cancel(&motion);
  /* Test the watchdog BEFORE accepting a late heartbeat: late packets must
   * not hide a communication outage or revive a failed trajectory. */
  ArmMotion_Tick(&motion, &fb, now);
  uint8_t result = ARM_OK, responded = 0;
  AppMotionRequest request;
  uint32_t ip = 0;
  uint16_t port = 0;
  platform_enter_critical();
  if (pending) {
    request = inbox; ip = request_ip; port = request_port;
    pending = 0; responded = 1;
  }
  platform_exit_critical();
  if (responded) {
    uint8_t command[sizeof(ArmMoveCommand)];
    memcpy(command, &request.command, sizeof(command));
    const uint8_t was_owned = motion.owned, was_failed = motion.state == ARM_FAILED;
    if (motion.owned && request.op != ARM_QUERY && (ip != owner_ip || port != owner_port))
      result = ARM_BAD_SESSION;
    else if ((eth_udp_command.control_mode == DM_ETH_CONTROL_NRT_CARTESIAN && request.op != ARM_MOVEP) ||
             (eth_udp_command.control_mode == DM_ETH_CONTROL_NRT_JOINT && request.op == ARM_MOVEP))
      result = ARM_UNSUPPORTED;
    else
      result = ArmMotion_Request(&motion, request.op, request.session, request.motion_id,
                                  command, &fb, now);
    if (!was_owned && motion.owned && result == ARM_OK) { owner_ip = ip; owner_port = port; }
    /* The PC cancelled the move: a controlled stop, no disable (after a
     * failure the stop has already happened). */
    if (was_owned && !motion.owned && !was_failed)
      AppSafety_Stop(INFRABOT_STOP_REQUESTED, INFRABOT_NO_JOINT, 0U);
  }
  if (motion.owned && motion.state == ARM_FAILED) {
    /* Keep the failure latched for the PC (DMOS) until it cancels, switches
     * mode or disables; the arm makes a controlled stop instead of losing
     * power. */
    if (!stop_reported) { stop_reported = 1U; stop_after_failure(&fb); }
  } else if (motion.owned) {
    /* This task publishes exactly one interpolated setpoint per control tick;
     * no 10 ms network callback can block the 1 kHz CAN loop. */
    eth_udp_command.active = 1;
    eth_udp_command.last_tick = motion.last_heartbeat_ms;
    eth_udp_command.sequence_valid = 0;
    eth_udp_command.queue_head = eth_udp_command.queue_tail = eth_udp_command.queue_count = 0;
    for (unsigned i = 0; i < ARM_AXES; ++i) {
      dm_motor_runtime.command_position[i] = dm_motor_runtime.zero_offset[i] +
                               DM_MOTOR_DIRECTION(i) * motion.position[i];
      eth_udp_command.target_position[i] = motion.cartesian ? motion.position[i] :
                                      motion.command.target[i];
      eth_udp_command.target_velocity[i] = motion.velocity[i];
      eth_udp_command.target_kp[i] = motion.cartesian ? motion.cartesian_command.kp[i] :
                                 motion.command.kp[i];
      eth_udp_command.target_kd[i] = motion.cartesian ? motion.cartesian_command.kd[i] :
                                 motion.command.kd[i];
      eth_udp_command.target_torque[i] = motion.cartesian ? 0.0f : motion.command.torque[i];
      eth_udp_command.velocity_limit[i] = motion.cartesian ?
                                     motion.cartesian_command.velocity_max[i] :
                                     motion.command.velocity_max[i];
      dm_motor_runtime.stopped[i] = 0;
      dm_motor_runtime.override[i] = 1;
    }
    /* J8 retains its existing force/position command; not part of arrival. */
  }
  if (motion.state != ARM_FAILED) stop_reported = 0U;
  if (responded) {
    AppMotionReply reply = {0};
    reply.magic = INFRABOT_MAGIC_MOVE_REPLY;
    reply.version = INFRABOT_MOVE_V1;
    reply.result = result;
    reply.state = motion.state;
    reply.session = request.session;
    reply.request_id = request.motion_id;
    reply.active_id = motion.id;
    reply.elapsed_ms = motion.id ? now - motion.started_ms : 0;
    reply.duration_ms = motion.duration_ms;
    reply.arrived_mask = motion.arrived_mask;
    reply.enabled_mask = fb.enabled_mask;
    reply.fresh_mask = fb.fresh_mask;
    reply.max_error_rad = motion.max_error;
    reply.reason = motion.reason;
    platform_enter_critical();
    outbox = reply; reply_ip = ip; reply_port = port; reply_pending = 1;
    platform_exit_critical();
  }
}
