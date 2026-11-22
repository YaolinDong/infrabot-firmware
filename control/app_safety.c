/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_runtime.h"
#include "eth_state.h"
#include "platform.h"
#include "app_safety.h"
#include "app_gravity.h"
#include "app_state.h"
#include "app_motion.h"
#include "infrabot_ik_mcu.h"
#include <math.h>
#include <stdio.h>

#define ARM DM_ARM_MOTOR_COUNT
#define DT (1.0f / (float)DM_CONTROL_HZ)
#define TORQUE_LIMIT_TICKS ((DM_TORQUE_LIMIT_MS * DM_CONTROL_HZ) / 1000U)

/* The stop: phase, latched reason / joint / detail (INFRABOT_STOP_*). */
static volatile uint8_t phase = INFRABOT_STOP_PHASE_NONE;
static volatile uint8_t reason = INFRABOT_STOP_NONE;
static volatile uint8_t joint = INFRABOT_NO_JOINT;
static volatile uint8_t detail;
static volatile uint8_t requested; /* AppSafety_RequestStop, taken by the tick */
static uint8_t local_hold;
void AppSafety_SetLocalHold(uint8_t enabled) { local_hold=enabled!=0U; }
uint8_t AppSafety_LocalHold(void) { return local_hold && eth_udp_command.control_mode==DM_ETH_CONTROL_IDLE; }
/* The control mode the latched stop interrupted: the PC resumes it. */
static volatile uint8_t stop_mode = DM_ETH_CONTROL_IDLE;
static uint32_t hold_started_ms;   /* HOLDING since (for the log) */
/* Braking: every joint decelerates uniformly from start_v to rest in
 * brake_time, so they stop together (the joint-space direction is kept). */
static float brake_time, brake_elapsed;
static float start_q[ARM], start_v[ARM];
static uint8_t held_mask; /* blocked joints, held at their measured angle */
/* Command velocity of each joint over the last ARM_DRIVE_LEAD_TICKS ticks:
 * the speed a stop brakes from. */
static ArmDriveLead window[ARM];
static float velocity[ARM];
/* Supervisor state: torque-limit counters and edge latches (bit i = joint i). */
static uint16_t torque_ticks[ARM];
static volatile uint8_t torque_mask;
static uint8_t fault_latched, lost_latched;

static const char *reason_name(uint8_t r)
{
  static const char *const names[] = {
      "none", "stream lost", "move keepalive lost", "move failed", "requested",
      "joint not following", "invalid target", "feedback lost", "drive fault",
      "torque limit"};
  return r < sizeof(names) / sizeof(names[0]) ? names[r] : "unknown";
}

static float command_q(uint8_t i)
{
  return (dm_motor_runtime.command_position[i] - dm_motor_runtime.zero_offset[i]) * DM_MOTOR_DIRECTION(i);
}

static float measured_q(uint8_t i)
{
  return (dm_motors[i].position - dm_motor_runtime.zero_offset[i]) * DM_MOTOR_DIRECTION(i);
}

static void set_command(uint8_t i, float q, float v)
{
  dm_motor_runtime.command_position[i] = dm_motor_runtime.zero_offset[i] + DM_MOTOR_DIRECTION(i) * q;
  eth_udp_command.target_position[i] = q;
  eth_udp_command.target_velocity[i] = v;
  dm_motor_runtime.override[i] = 1U;
}

/* A joint the stop moves: enabled and not held by a v4 STOP. */
static uint8_t driven(uint8_t i)
{
  return dm_motor_runtime.enabled[i] != 0U && dm_motor_runtime.stopped[i] == 0U;
}

/* The PC keeps the watchdog fresh (a heartbeat or any accepted command); a
 * lost stream or keepalive has let it expire. */
static uint8_t pc_alive(uint32_t now)
{
  if (AppSafety_LocalHold()) return 1U;
  return (int32_t)(now - eth_udp_command.last_tick) <= (int32_t)DM_ETH_COMMAND_TIMEOUT_MS;
}

/* What a stop makes of a running movej/movep (its DMOS reason). */
static uint8_t move_reason(uint8_t why)
{
  switch (why)
  {
    case INFRABOT_STOP_REQUESTED: return ARM_CANCEL;
    case INFRABOT_STOP_FEEDBACK_LOST: return ARM_FEEDBACK_STALE;
    case INFRABOT_STOP_STREAM_LOST:
    case INFRABOT_STOP_MOVE_LOST: return ARM_COMM_TIMEOUT;
    default: return ARM_MOTOR_FAULT;
  }
}

static void print_stop(const char *what)
{
  if (joint < ARM)
    printf("STOP: %s J%u detail=%u %s\r\n", reason_name(reason), (unsigned)joint + 1U,
           (unsigned)detail, what);
  else
    printf("STOP: %s detail=%u %s\r\n", reason_name(reason), (unsigned)detail, what);
}

void AppSafety_Stop(uint8_t why, uint8_t which, uint8_t what)
{
  if (!dm_motor_runtime.control_enabled || eth_udp_command.control_request == DM_ETH_CMD_DISABLE) return;
  const uint8_t active = phase == INFRABOT_STOP_PHASE_BRAKING ||
                         phase == INFRABOT_STOP_PHASE_HOLDING;
  /* The mode to resume.  A stop while an earlier one is still latched (not
   * acknowledged) happens in IDLE: the earlier stop's mode stays the one the
   * PC resumes. */
  if (reason == INFRABOT_STOP_NONE) stop_mode = eth_udp_command.control_mode;
  AppGravity_OnStop(why);
  platform_enter_critical();
  /* Leave the control mode: the streams and movej/movep are refused until
   * the PC switches modes again. */
  eth_udp_command.control_mode = DM_ETH_CONTROL_IDLE;
  eth_udp_command.cartesian_target_valid = 0U;
  eth_udp_command.active = 0U;
  eth_udp_command.sequence_valid = 0U;
  eth_udp_command.queue_head = eth_udp_command.queue_tail = eth_udp_command.queue_count = 0U;
  platform_exit_critical();
  /* A lost stream during a stop is the stop's own silence, not news; the
   * PC repeats its STOP for loss tolerance. */
  if (active && (why == INFRABOT_STOP_STREAM_LOST || (why == reason && which == joint))) return;
  AppMotion_Abort(move_reason(why));
  reason = why;
  joint = which;
  detail = what;
  if (!active) held_mask = 0U;
  if (which < ARM && (why == INFRABOT_STOP_NOT_FOLLOWING || why == INFRABOT_STOP_TORQUE_LIMIT))
    held_mask |= (uint8_t)(1U << which);
  float fastest = 0.0f;
  for (uint8_t i = 0U; i < ARM; ++i)
  {
    start_q[i] = command_q(i);
    start_v[i] = 0.0f;
    if (!driven(i)) continue;
    if (held_mask & (1U << i))
    {
      /* Blocked: pushing on toward the command would only press harder. */
      start_q[i] = measured_q(i);
      set_command(i, start_q[i], 0.0f);
      continue;
    }
    start_v[i] = isfinite(velocity[i]) ? velocity[i] : 0.0f;
    fastest = fmaxf(fastest, fabsf(start_v[i]));
  }
  brake_time = fminf(fastest / DM_STOP_DECELERATION_RAD_S2, DM_STOP_MAX_BRAKE_S);
  brake_elapsed = 0.0f;
  phase = INFRABOT_STOP_PHASE_BRAKING;
  print_stop("braking");
}

void AppSafety_RequestStop(uint8_t why)
{
  requested = why;
}
uint8_t AppSafety_StopRequested(void) { return requested != 0U; }

void AppSafety_Clear(void)
{
  /* Overtaken while braking: hold where the brake got to. */
  if (phase == INFRABOT_STOP_PHASE_BRAKING)
    for (uint8_t i = 0U; i < ARM; ++i)
      if (driven(i)) eth_udp_command.target_velocity[i] = 0.0f;
  phase = INFRABOT_STOP_PHASE_NONE;
  reason = INFRABOT_STOP_NONE;
  joint = INFRABOT_NO_JOINT;
  detail = 0U;
  stop_mode = DM_ETH_CONTROL_IDLE;
  held_mask = 0U;
}

uint8_t AppSafety_Braking(void)
{
  return phase == INFRABOT_STOP_PHASE_BRAKING;
}

uint8_t AppSafety_Stopping(void)
{
  return phase == INFRABOT_STOP_PHASE_BRAKING || phase == INFRABOT_STOP_PHASE_HOLDING;
}

uint8_t AppSafety_TorqueLimitMask(void)
{
  return torque_mask;
}

void AppSafety_Report(InfrabotSafetyStatus *status)
{
  status->stop_reason = reason;
  status->stop_phase = phase;
  status->stop_joint = joint;
  status->stop_detail = detail;
  status->stop_mode = reason != INFRABOT_STOP_NONE ? stop_mode : DM_ETH_CONTROL_IDLE;
}

/* Drive faults, lost feedback and torque at the limit of the enabled arm
 * drives (the gripper squeezes on purpose and is not supervised).  Faults and
 * lost feedback stop on their onset; the torque limit after TORQUE_LIMIT_TICKS
 * consecutive ticks, then again if it persists (the joint, held where it is,
 * yields to a push). */
static void supervise(uint32_t now)
{
  static const float limit[ARM] = DM_ARM_TORQUE_LIMIT_NM;
  for (uint8_t i = 0U; i < ARM; ++i)
  {
    const uint8_t bit = (uint8_t)(1U << i);
    if (!dm_motor_runtime.enabled[i])
    {
      torque_ticks[i] = 0U;
      torque_mask &= (uint8_t)~bit;
      fault_latched &= (uint8_t)~bit;
      lost_latched &= (uint8_t)~bit;
      continue;
    }
    const uint8_t state = dm_motors[i].state;
    if (INFRABOT_DRIVE_IS_FAULT(state))
    {
      if (!(fault_latched & bit))
      {
        fault_latched |= bit;
        AppSafety_Stop(INFRABOT_STOP_DRIVE_FAULT, i, state);
      }
    }
    else fault_latched &= (uint8_t)~bit;
    /* Signed: a frame received after now was read is fresh. */
    if (dm_motors[i].feedback_count == 0U ||
        (int32_t)(now - dm_motors[i].feedback_tick) > (int32_t)DM_STOP_FEEDBACK_LOST_MS)
    {
      if (!(lost_latched & bit))
      {
        lost_latched |= bit;
        AppSafety_Stop(INFRABOT_STOP_FEEDBACK_LOST, i, 0U);
      }
    }
    else lost_latched &= (uint8_t)~bit;
    if (fabsf(dm_motors[i].torque) >= limit[i])
    {
      torque_mask |= bit;
      if (++torque_ticks[i] >= TORQUE_LIMIT_TICKS)
      {
        torque_ticks[i] = 0U;
        AppSafety_Stop(INFRABOT_STOP_TORQUE_LIMIT, i, 0U);
      }
    }
    else
    {
      torque_ticks[i] = 0U;
      torque_mask &= (uint8_t)~bit;
    }
  }
}

static void brake(uint32_t now)
{
  const float *lower = ArmIk_JointLower();
  const float *upper = ArmIk_JointUpper();
  brake_elapsed += DT;
  const float t = fminf(brake_elapsed, brake_time);
  for (uint8_t i = 0U; i < ARM; ++i)
  {
    if (!driven(i) || (held_mask & (1U << i))) continue;
    float q = start_q[i], v = 0.0f;
    if (brake_time > 0.0f)
    {
      v = start_v[i] * (1.0f - t / brake_time);
      q = start_q[i] + start_v[i] * (t - 0.5f * t * t / brake_time);
    }
    /* Never further past a joint limit than the command already was. */
    const float lo = fminf(lower[i], start_q[i]), hi = fmaxf(upper[i], start_q[i]);
    if (q < lo) { q = lo; v = 0.0f; }
    else if (q > hi) { q = hi; v = 0.0f; }
    set_command(i, q, v);
  }
  if (brake_elapsed < brake_time) return;
  if (pc_alive(now))
  {
    phase = INFRABOT_STOP_PHASE_NONE;
    eth_udp_command.active = 1U;
    print_stop("stopped, IDLE hold");
    return;
  }
  phase = INFRABOT_STOP_PHASE_HOLDING;
  hold_started_ms = now;
  printf("STOP: holding, no PC (stays enabled)\r\n");
}

/* Stopped with no PC: hold, however long, until one is back.  Never
 * disables; only a DISABLE from a PC removes power. */
static void hold(uint32_t now)
{
  if (!pc_alive(now)) return;
  phase = INFRABOT_STOP_PHASE_NONE;
  eth_udp_command.active = 1U;
  printf("STOP: %s after %lu s, IDLE hold\r\n", AppSafety_LocalHold() ? "local control" : "PC back",
         (unsigned long)((now - hold_started_ms) / 1000U));
}

void AppSafety_Tick(void)
{
  const uint32_t now = platform_now_ms();
  const uint8_t why = requested;
  if (why != 0U)
  {
    requested = 0U;
    AppSafety_Stop(why, INFRABOT_NO_JOINT, 0U);
  }
  if (!dm_motor_runtime.control_enabled)
  {
    /* Disabled (a DISABLE from a PC): the stop is over. */
    if (AppSafety_Stopping()) phase = INFRABOT_STOP_PHASE_NONE;
    torque_mask = fault_latched = lost_latched = 0U;
    for (uint8_t i = 0U; i < ARM; ++i) torque_ticks[i] = 0U;
    return;
  }
  if (eth_udp_command.control_request != DM_ETH_CMD_DISABLE) supervise(now);
  if (phase == INFRABOT_STOP_PHASE_BRAKING) brake(now);
  else if (phase == INFRABOT_STOP_PHASE_HOLDING) hold(now);
  for (uint8_t i = 0U; i < ARM; ++i)
    velocity[i] = ArmDriveLead_Velocity(&window[i], command_q(i), dm_motors[i].velocity_max, DT);
}
