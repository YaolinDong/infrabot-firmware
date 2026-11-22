/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_motion.h"
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static ArmMoveFeedback feedback(void)
{
  ArmMoveFeedback f = {0};
  f.ready = 1;
  f.enabled_mask = f.fresh_mask = f.healthy_mask = ARM_MASK;
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    f.lower[i] = -2; f.upper[i] = 2; f.velocity_max[i] = 10; f.torque_max[i] = 10;
  }
  return f;
}

static ArmMoveCommand command(void)
{
  ArmMoveCommand c = {0};
  c.timeout_ms = 10000; c.dwell_ms = 100;
  c.position_tolerance = 0.01f; c.velocity_tolerance = 0.05f;
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    c.target[i] = i & 1 ? -0.2f : 0.1f;
    c.velocity_max[i] = 0.1f; c.acceleration_max[i] = 0.2f;
    c.kp[i] = 30; c.kd[i] = 0.8f;
  }
  return c;
}

static void test_profile_and_blocking(void)
{
  ArmMotion m = {0};
  ArmMoveFeedback f = feedback();
  ArmMoveCommand c = command();
  assert(ArmMotion_Request(&m, ARM_MOVEJ, 42, 1, &c, &f, 0) == ARM_OK);
  uint32_t duration = m.duration_ms;
  assert(duration >= 3750);
  for (uint32_t t = 1; t <= duration; ++t) {
    float last_v[ARM_AXES]; memcpy(last_v, m.velocity, sizeof(last_v));
    if (t % 10 == 0) {
      assert(ArmMotion_Request(&m, ARM_MOVEJ, 42, 1, &c, &f, t) == ARM_OK);
      assert(m.started_ms == 0); /* 100 Hz repeats cannot restart the segment. */
      assert(ArmMotion_Request(&m, ARM_MOVEJ, 42, 2, &c, &f, t) == ARM_BUSY);
    }
    ArmMotion_Tick(&m, &f, t);
    for (unsigned i = 0; i < ARM_AXES; ++i) {
      assert(fabsf(m.velocity[i]) <= c.velocity_max[i] + 1e-6f);
      assert(fabsf(m.velocity[i] - last_v[i]) * 1000 <= c.acceleration_max[i] + 1e-3f);
      assert(m.position[i] >= fminf(0, c.target[i]) - 1e-6f);
      assert(m.position[i] <= fmaxf(0, c.target[i]) + 1e-6f);
      f.position[i] = m.position[i]; f.velocity[i] = m.velocity[i];
    }
    assert(m.state != ARM_DONE); /* Even arrival near endpoint requires dwell. */
  }
  /* Six joints arrived is NOT whole-arm completion. */
  f.position[6] += 0.1f;
  for (uint32_t t = duration + 1; t <= duration + 200; ++t) {
    if (t % 10 == 0) ArmMotion_Request(&m, ARM_KEEPALIVE, 42, 1, &c, &f, t);
    ArmMotion_Tick(&m, &f, t);
    assert(m.state == ARM_SETTLING);
  }
  f.position[6] = c.target[6];
  memset(f.velocity, 0, sizeof(f.velocity));
  for (uint32_t t = duration + 201; t <= duration + 250; ++t) {
    ArmMotion_Request(&m, ARM_KEEPALIVE, 42, 1, &c, &f, t);
    ArmMotion_Tick(&m, &f, t);
  }
  f.velocity[0] = 0.2f; /* Brief disturbance resets dwell. */
  ArmMotion_Tick(&m, &f, duration + 251);
  f.velocity[0] = 0;
  for (uint32_t t = duration + 252; t <= duration + 352; ++t) {
    ArmMotion_Request(&m, ARM_KEEPALIVE, 42, 1, &c, &f, t);
    ArmMotion_Tick(&m, &f, t);
    if (t < duration + 352) assert(m.state != ARM_DONE);
  }
  assert(m.state == ARM_DONE && m.arrived_mask == ARM_MASK);
  assert(ArmMotion_Request(&m, ARM_MOVEJ, 42, 1, &c, &f, duration + 353) == ARM_OK);
  assert(m.state == ARM_DONE); /* Lost completion ACK -> idempotent replay. */
  c.target[0] = 0.2f;
  assert(ArmMotion_Request(&m, ARM_MOVEJ, 42, 1, &c, &f, duration + 353) == ARM_INVALID);
  assert(ArmMotion_Request(&m, ARM_MOVEJ, 42, 2, &c, &f, duration + 353) == ARM_OK);
  assert(ArmMotion_Request(&m, ARM_MOVEJ, 42, 1, &c, &f, duration + 354) == ARM_STALE_ID);
  assert(ArmMotion_Request(&m, ARM_MOVEJ, 43, 3, &c, &f, duration + 354) == ARM_BAD_SESSION);
}

static void test_failures(void)
{
  ArmMoveFeedback f = feedback();
  ArmMoveCommand c = command();
  for (unsigned scenario = 0; scenario < 6; ++scenario) {
    ArmMotion m = {0};
    f = feedback();
    assert(ArmMotion_Request(&m, ARM_MOVEJ, 42, 1, &c, &f, 0) == ARM_OK);
    uint8_t expected = ARM_OK;
    switch (scenario) {
      case 0: expected = ARM_COMM_TIMEOUT; ArmMotion_Tick(&m, &f, 101); break;
      case 1:
        expected = ARM_FEEDBACK_STALE;
        f.fresh_mask &= ~8U;
        /* A transient stale sample is held, not failed immediately. */
        for (uint32_t t = 1; t <= ARM_MOVE_FEEDBACK_FAIL_MS + 1U; ++t) {
          if (t % 10U == 0U) ArmMotion_Request(&m, ARM_KEEPALIVE, 42, 1, &c, &f, t);
          ArmMotion_Tick(&m, &f, t);
        }
        break;
      case 2: expected = ARM_MOTOR_FAULT; f.healthy_mask &= ~8U; ArmMotion_Tick(&m, &f, 1); break;
      case 3: expected = ARM_NOT_READY; f.enabled_mask &= ~8U; ArmMotion_Tick(&m, &f, 1); break;
      case 4: expected = ARM_TX_ERROR; ArmMotion_Fail(&m, expected); break;
      case 5:
        expected = ARM_MOTION_TIMEOUT;
        for (uint32_t t = 1; t <= c.timeout_ms; ++t) {
          if (t % 10 == 0) ArmMotion_Request(&m, ARM_KEEPALIVE, 42, 1, &c, &f, t);
          ArmMotion_Tick(&m, &f, t); /* Feedback never reaches target. */
        }
        break;
    }
    assert(m.state == ARM_FAILED && m.reason == expected);
    f = feedback();
    assert(ArmMotion_Request(&m, ARM_MOVEJ, 42, 2, &c, &f, 10001) == expected);
    assert(ArmMotion_Request(&m, ARM_KEEPALIVE, 42, 1, &c, &f, 10001) == expected);
    assert(ArmMotion_Request(&m, ARM_CANCEL_MOVE, 42, 1, &c, &f, 10002) == ARM_OK);
    assert(!m.owned && m.state == ARM_CANCELLED);
    assert(ArmMotion_Request(&m, ARM_MOVEJ, 42, 2, &c, &f, 10003) == ARM_BAD_SESSION);
    assert(ArmMotion_Request(&m, ARM_CANCEL_MOVE, 42, 1, &c, &f, 10003) == ARM_OK);
    assert(ArmMotion_Request(&m, ARM_MOVEJ, 43, 1, &c, &f, 10003) == ARM_OK);
  }
}

static void test_validation_and_clock_wrap(void)
{
  ArmMoveFeedback f = feedback();
  ArmMoveCommand c = command(), invalid;
  ArmMotion m = {0};
  ArmMoveCartesianCommand p = {0};
  ArmIkPose home;
  const float q_home[ARM_IK_AXES] = {0};
  ArmIk_Forward(q_home, &home);
  p.timeout_ms = 10000; p.dwell_ms = 100;
  p.position_tolerance = 0.01f; p.velocity_tolerance = 0.05f;
  p.pose[0] = home.position[0]; p.pose[1] = home.position[1]; p.pose[2] = home.position[2];
  for (unsigned row = 0; row < 3; ++row)
    for (unsigned col = 0; col < 3; ++col)
      p.pose[3 + row * 3 + col] = home.rotation[row][col];
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    p.velocity_max[i] = 1.0f; p.acceleration_max[i] = 2.0f;
    p.kp[i] = 30.0f; p.kd[i] = 0.8f;
  }
  assert(ArmMotion_Request(&m, ARM_MOVEP, 42, 1, &p, &f, 0) == ARM_OK && m.owned);
  ArmMotion_Cancel(&m);
  memset(&m, 0, sizeof(m));
  assert(ArmMotion_Request(&m, ARM_MOVEJ, 0, 1, &c, &f, 0) == ARM_INVALID);
  assert(ArmMotion_Request(&m, ARM_MOVEJ, 42, 0, &c, &f, 0) == ARM_INVALID);
  for (unsigned n = 0; n < 10; ++n) {
    invalid = c;
    switch (n) {
      case 0: invalid.target[0] = NAN; break;
      case 1: invalid.target[0] = 3; break;
      case 2: invalid.velocity_max[0] = 0; break;
      case 3: invalid.acceleration_max[0] = INFINITY; break;
      case 4: invalid.kd[0] = 6; break;
      case 5: invalid.kp[0] = 0; break;
      case 6: invalid.torque[0] = 11; break;
      case 7: invalid.position_tolerance = -1; break;
      case 8: invalid.timeout_ms = 100; break;
      case 9: invalid.dwell_ms = 0; break;
    }
    assert(ArmMotion_Request(&m, ARM_MOVEJ, 42, 1, &invalid, &f, 0) == ARM_INVALID && !m.owned);
  }
  f.velocity[0] = 1;
  assert(ArmMotion_Request(&m, ARM_MOVEJ, 42, 1, &c, &f, 0) == ARM_NOT_READY);
  f = feedback();
  uint32_t start = UINT32_MAX - 50U;
  assert(ArmMotion_Request(&m, ARM_MOVEJ, 42, 1, &c, &f, start) == ARM_OK);
  for (uint32_t t = 1; t < 200; ++t) {
    ArmMotion_Request(&m, ARM_KEEPALIVE, 42, 1, &c, &f, start + t);
    ArmMotion_Tick(&m, &f, start + t);
  }
  assert(m.state == ARM_RUNNING);
  ArmMotion_Request(&m, ARM_QUERY, 42, 1, &c, &f, start + 400);
  ArmMotion_Tick(&m, &f, start + 400);
  assert(m.reason == ARM_COMM_TIMEOUT); /* QUERY does not feed watchdog. */
}

static ArmMoveCartesianCommand cartesian_at(const float q[ARM_AXES])
{
  ArmMoveCartesianCommand p = {0};
  ArmIkPose pose;
  ArmIk_Forward(q, &pose);
  p.timeout_ms = 10000; p.dwell_ms = 100;
  p.position_tolerance = 0.01f; p.velocity_tolerance = 0.05f;
  for (unsigned i = 0; i < 3; ++i) p.pose[i] = pose.position[i];
  for (unsigned row = 0; row < 3; ++row)
    for (unsigned col = 0; col < 3; ++col) p.pose[3 + row * 3 + col] = pose.rotation[row][col];
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    p.velocity_max[i] = 0.1f; p.acceleration_max[i] = 0.2f;
    p.kp[i] = 30.0f; p.kd[i] = 0.8f;
  }
  return p;
}

static void test_start_from_command(void)
{
  /* A stationary arm sags below its command; motions continue from the
   * command instead of dropping onto the sagged feedback. */
  ArmMoveFeedback f = feedback();
  const float q[ARM_AXES] = {0.1f, 0.4f, 0.1f, 0.6f, 0.1f, 0.1f, 0.1f};
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    f.command[i] = q[i];
    f.position[i] = q[i] - 0.02f;
  }
  f.command_valid = 1;
  ArmMoveCommand c = command();
  c.timeout_ms = 60000;
  ArmMotion m = {0};
  assert(ArmMotion_Request(&m, ARM_MOVEJ, 7, 1, &c, &f, 0) == ARM_OK);
  for (unsigned i = 0; i < ARM_AXES; ++i) assert(m.start[i] == q[i]);
  ArmMotion_Cancel(&m);
  memset(&m, 0, sizeof(m));
  f.command[2] = q[2] + 0.5f; /* Blocked joint: far from feedback, measured is used. */
  assert(ArmMotion_Request(&m, ARM_MOVEJ, 7, 1, &c, &f, 0) == ARM_OK);
  assert(m.start[2] == f.position[2] && m.start[3] == q[3]);
  ArmMotion_Cancel(&m);
  memset(&m, 0, sizeof(m));
  f.command[2] = q[2];
  f.command_valid = 0;
  assert(ArmMotion_Request(&m, ARM_MOVEJ, 7, 1, &c, &f, 0) == ARM_OK);
  assert(m.start[0] == f.position[0]);
}

static void test_cartesian_integrates_on_command(void)
{
  /* Target = FK(command) while the feedback sags: the command must stay put
   * (the old feedback-seeded step dragged it onto the sagged pose), and a
   * lagging arm only sees rate-limited setpoints. */
  ArmMoveFeedback f = feedback();
  const float q[ARM_AXES] = {0.1f, 0.4f, 0.1f, 0.6f, 0.1f, 0.1f, 0.1f};
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    f.command[i] = q[i];
    f.position[i] = q[i] - 0.02f;
  }
  f.command_valid = 1;
  ArmMoveCartesianCommand p = cartesian_at(q);
  ArmMotion m = {0};
  assert(ArmMotion_Request(&m, ARM_MOVEP, 9, 1, &p, &f, 0) == ARM_OK);
  for (uint32_t t = 1; t <= 200; ++t) {
    ArmMotion_Request(&m, ARM_KEEPALIVE, 9, 1, &p, &f, t);
    ArmMotion_Tick(&m, &f, t);
  }
  assert(m.state != ARM_FAILED);
  for (unsigned i = 0; i < ARM_AXES; ++i) assert(fabsf(m.position[i] - q[i]) < 1e-3f);
  ArmMotion_Cancel(&m);

  /* A 5 cm move with frozen feedback: setpoints respect 0.1 rad/s and
   * 0.2 rad/s^2 from the start, never jumping toward the feedback. */
  memset(&m, 0, sizeof(m));
  p.pose[0] += 0.05f;
  for (unsigned i = 0; i < ARM_AXES; ++i) f.position[i] = q[i];
  assert(ArmMotion_Request(&m, ARM_MOVEP, 10, 1, &p, &f, 0) == ARM_OK);
  float previous[ARM_AXES] = {0};
  for (uint32_t t = 1; t <= 300; ++t) {
    ArmMotion_Request(&m, ARM_KEEPALIVE, 10, 1, &p, &f, t);
    ArmMotion_Tick(&m, &f, t);
    for (unsigned i = 0; i < ARM_AXES; ++i) {
      assert(fabsf(m.velocity[i]) <= 0.1f + 1e-4f);
      assert(fabsf(m.velocity[i] - previous[i]) <= 0.2f * 0.001f + 1e-4f);
      previous[i] = m.velocity[i];
    }
  }
  assert(m.state == ARM_RUNNING);
  float moved = 0.0f;
  for (unsigned i = 0; i < ARM_AXES; ++i) moved = fmaxf(moved, fabsf(m.position[i] - q[i]));
  assert(moved > 0.001f);
  /* A position-velocity joint that far behind is not checked (its drive runs
   * its own loop); a MIT joint not following beyond the tracking window
   * fails the motion. */
  f.position[0] = m.position[0] - 0.3f;
  f.posvel_mask = 1U;
  ArmMotion_Request(&m, ARM_KEEPALIVE, 10, 1, &p, &f, 301);
  ArmMotion_Tick(&m, &f, 301);
  assert(m.state == ARM_RUNNING);
  f.posvel_mask = 0U;
  f.position[0] = m.position[0] - 0.3f;
  ArmMotion_Request(&m, ARM_KEEPALIVE, 10, 1, &p, &f, 302);
  ArmMotion_Tick(&m, &f, 302);
  assert(m.state == ARM_FAILED && m.reason == ARM_MOTOR_FAULT);
}

static void test_follow_joints(void)
{
  float vmax[ARM_AXES], amax[ARM_AXES], goal[ARM_AXES], q[ARM_AXES], v[ARM_AXES];
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    vmax[i] = 0.5f; amax[i] = 2.0f; goal[i] = 0.05f * (float)(i + 1); q[i] = 0.0f; v[i] = 0.0f;
  }
  /* From rest: limits hold, never past the goal, stops exactly on it. */
  for (unsigned k = 0; k < 2000; ++k) {
    float previous[ARM_AXES];
    memcpy(previous, v, sizeof(previous));
    ArmMotion_FollowJoints(q, v, goal, NULL, vmax, amax, 0.001f);
    for (unsigned i = 0; i < ARM_AXES; ++i) {
      assert(fabsf(v[i]) <= vmax[i] + 1e-6f);
      assert(fabsf(v[i] - previous[i]) <= amax[i] * 0.001f + 1e-6f);
      /* Float rounding can land ~1e-7 rad past the goal; its correction is
       * ~1e-4 rad/s, far below a real reversal. */
      assert(q[i] <= goal[i] + 1e-6f && v[i] >= -1e-3f);
    }
  }
  for (unsigned i = 0; i < ARM_AXES; ++i) assert(fabsf(q[i] - goal[i]) < 1e-5f && fabsf(v[i]) < 1e-5f);

  /* Goal jumps behind a joint moving at full speed: it has to overshoot,
   * but brakes at the limit and comes back once, without swinging. */
  q[0] = 0.0f; v[0] = 0.5f; goal[0] = 0.01f;
  unsigned reversals = 0;
  for (unsigned k = 0; k < 3000; ++k) {
    const float previous = v[0];
    ArmMotion_FollowJoints(q, v, goal, NULL, vmax, amax, 0.001f);
    assert(fabsf(v[0] - previous) <= amax[0] * 0.001f + 1e-6f);
    if (v[0] * previous < 0.0f && fabsf(previous) > 0.005f) ++reversals;
  }
  assert(reversals <= 1 && fabsf(q[0] - goal[0]) < 1e-5f && fabsf(v[0]) < 1e-5f);

  /* No acceleration limit: plain velocity-limited step. */
  amax[1] = 0.0f; q[1] = 0.0f; v[1] = 0.0f; goal[1] = 0.0002f;
  ArmMotion_FollowJoints(q, v, goal, NULL, vmax, amax, 0.001f);
  assert(fabsf(q[1] - 0.0002f) < 1e-7f);
}

static void test_cartesian_no_overshoot(void)
{
  /* Regression for the side-to-side sway: a one-step IK under velocity and
   * acceleration limits is greedy and swings around the target.  With an
   * ideally tracking arm the move must arrive without overshoot or any
   * joint reversing direction, then complete. */
  ArmMoveFeedback f = feedback();
  const float q[ARM_AXES] = {0.1f, 0.4f, 0.1f, 0.6f, 0.1f, 0.1f, 0.1f};
  memcpy(f.position, q, sizeof(f.position));
  memcpy(f.command, q, sizeof(f.command));
  f.command_valid = 1;
  ArmMoveCartesianCommand p = cartesian_at(q);
  const float start_x = p.pose[0];
  p.pose[0] += 0.015f;
  p.position_tolerance = 0.0005f;
  for (unsigned i = 0; i < ARM_AXES; ++i) { p.velocity_max[i] = 0.5f; p.acceleration_max[i] = 2.0f; }
  ArmMotion m = {0};
  assert(ArmMotion_Request(&m, ARM_MOVEP, 11, 1, &p, &f, 0) == ARM_OK);
  float previous[ARM_AXES] = {0}, max_x = start_x;
  uint32_t t = 1;
  for (; t < 5000 && m.state != ARM_DONE; ++t) {
    ArmMotion_Request(&m, ARM_KEEPALIVE, 11, 1, &p, &f, t);
    ArmMotion_Tick(&m, &f, t);
    assert(m.state != ARM_FAILED);
    for (unsigned i = 0; i < ARM_AXES; ++i) {
      assert(!(m.velocity[i] * previous[i] < 0.0f && fabsf(previous[i]) > 0.005f));
      previous[i] = m.velocity[i];
    }
    memcpy(f.position, m.position, sizeof(f.position));
    memcpy(f.velocity, m.velocity, sizeof(f.velocity));
    ArmIkPose pose;
    ArmIk_Forward(m.position, &pose);
    max_x = fmaxf(max_x, pose.position[0]);
  }
  assert(m.state == ARM_DONE);
  assert(max_x <= p.pose[0] + 0.0002f);
}

static void test_follow_moving_goal(void)
{
  /* A goal moving within the limits is tracked exactly (no braking lag,
   * which would differ per joint and bend the tool path). */
  float vmax[ARM_AXES], amax[ARM_AXES], goal[ARM_AXES] = {0}, gv[ARM_AXES] = {0};
  float q[ARM_AXES] = {0}, v[ARM_AXES] = {0};
  for (unsigned i = 0; i < ARM_AXES; ++i) { vmax[i] = 0.5f; amax[i] = 2.0f; }
  for (unsigned k = 0; k < 1500; ++k) {
    for (unsigned i = 0; i < ARM_AXES; ++i) {
      const float target_v = 0.1f * (float)(i + 1) * 0.5f;
      gv[i] = fminf(gv[i] + 1.0f * 0.001f, target_v); /* 1 rad/s^2 ramp */
      goal[i] += gv[i] * 0.001f;
    }
    ArmMotion_FollowJoints(q, v, goal, gv, vmax, amax, 0.001f);
    for (unsigned i = 0; i < ARM_AXES; ++i) assert(fabsf(q[i] - goal[i]) < 1e-5f);
  }
}

/* The RT defaults with the arm's rated speed caps (J3/J4 3.77 rad/s). */
static const float rated_speed[ARM_AXES] = {10.47f, 10.47f, 3.77f, 3.77f, 12.57f, 12.57f, 12.57f};
static ArmCartesianLimits wire_limits(uint16_t linear, uint16_t angular, uint16_t linear_acceleration,
                                      uint16_t angular_acceleration)
{
  const uint16_t wire[4] = {linear, angular, linear_acceleration, angular_acceleration};
  ArmCartesianLimits limits;
  assert(ArmMotion_CartesianLimitsFromWire(wire, &limits));
  ArmMotion_CapJointLimits(&limits, rated_speed, NULL);
  return limits;
}

static ArmCartesianLimits rt_limits(void) { return wire_limits(0, 0, 0, 0); }

/* Beyond the wire maxima: tracker robustness at speeds the arm may be given
 * again later (the IK residual fix only showed above ~0.4 m/s). */
static ArmCartesianLimits stress_limits(float linear, float angular, float linear_acceleration,
                                        float angular_acceleration)
{
  ArmCartesianLimits limits;
  ArmMotion_CartesianLimitsFromValues(linear, angular, linear_acceleration, angular_acceleration, &limits);
  ArmMotion_CapJointLimits(&limits, rated_speed, NULL);
  return limits;
}

static void test_tracker_straight_line(void)
{
  /* Regression for the curved path: an X step must move the tool along X
   * only.  The old IK-to-target + per-joint follower left the line by
   * ~10 mm here. */
  const float q0[ARM_AXES] = {0.1f, 0.4f, 0.1f, 0.6f, 0.1f, 0.1f, 0.1f};
  const ArmCartesianLimits limits = rt_limits();
  ArmCartesianTracker t;
  ArmCartesianTracker_Start(&t, q0, NULL, NULL);
  t.ik_options.time_budget_us = 0U;
  ArmIkPose start, target;
  ArmIk_Forward(q0, &start);
  target = start;
  target.position[0] += 0.05f;
  float previous[ARM_AXES] = {0}, worst = 0.0f;
  for (unsigned k = 0; k < 3000; ++k) {
    const uint8_t status = ArmCartesianTracker_Step(&t, &target, &limits, 0.001f);
    assert(status != ARM_IK_INVALID_INPUT && status != ARM_IK_QP_NUMERICAL_FAILURE);
    for (unsigned i = 0; i < ARM_AXES; ++i) {
      assert(fabsf(t.v[i]) <= limits.joint_velocity_max[i] + 1e-4f);
      assert(fabsf(t.v[i] - previous[i]) <= limits.joint_acceleration_max[i] * 0.001f + 1e-4f);
      previous[i] = t.v[i];
    }
    ArmIkPose pose;
    ArmIk_Forward(t.q, &pose);
    const float dy = pose.position[1] - start.position[1], dz = pose.position[2] - start.position[2];
    worst = fmaxf(worst, sqrtf(dy * dy + dz * dz));
    assert(pose.position[0] <= target.position[0] + 1e-4f); /* no overshoot along X */
  }
  ArmIkPose end;
  ArmIk_Forward(t.q, &end);
  assert(worst < 5e-5f);
  assert(fabsf(end.position[0] - target.position[0]) < 1e-5f);
}

static void test_tracker_joint_limited_line(void)
{
  /* Cartesian limits faster than the joints allow: only scaling every joint
   * by one common factor keeps the line (clamping each joint on its own
   * bent this path by ~3.6 mm). */
  const float q0[ARM_AXES] = {0.1f, 0.4f, 0.1f, 0.6f, 0.1f, 0.1f, 0.1f};
  ArmCartesianLimits limits = wire_limits(100, 349, 400, 1396); /* joint caps ~0.6 rad/s */
  limits.linear_velocity_max = 0.5f;
  limits.angular_velocity_max = 1.5f;
  ArmCartesianTracker t;
  ArmCartesianTracker_Start(&t, q0, NULL, NULL);
  t.ik_options.time_budget_us = 0U;
  ArmIkPose start, target;
  ArmIk_Forward(q0, &start);
  target = start;
  target.position[0] += 0.08f;
  float worst = 0.0f, slowest = 1.0f;
  for (unsigned k = 0; k < 3000; ++k) {
    ArmCartesianTracker_Step(&t, &target, &limits, 0.001f);
    slowest = fminf(slowest, t.scale);
    for (unsigned i = 0; i < ARM_AXES; ++i) assert(fabsf(t.v[i]) <= limits.joint_velocity_max[i] + 1e-4f);
    ArmIkPose pose;
    ArmIk_Forward(t.q, &pose);
    const float dy = pose.position[1] - start.position[1], dz = pose.position[2] - start.position[2];
    worst = fmaxf(worst, sqrtf(dy * dy + dz * dz));
  }
  assert(slowest < 1.0f); /* the joint speed limit did bind */
  assert(worst < 5e-5f);
}

static void test_tracker_pure_rotation(void)
{
  /* A 30 deg rotation about base Z keeps the tool point in place. */
  const float q0[ARM_AXES] = {0.1f, 0.4f, 0.1f, 0.6f, 0.1f, 0.1f, 0.1f};
  const ArmCartesianLimits limits = rt_limits();
  ArmCartesianTracker t;
  ArmCartesianTracker_Start(&t, q0, NULL, NULL);
  t.ik_options.time_budget_us = 0U;
  ArmIkPose start, target;
  ArmIk_Forward(q0, &start);
  target = start;
  const float w[3] = {0.0f, 0.0f, 0.5236f};
  float r[3][3];
  ArmIk_RotationExp(w, r);
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      target.rotation[i][j] = r[i][0] * start.rotation[0][j] + r[i][1] * start.rotation[1][j] +
                              r[i][2] * start.rotation[2][j];
  float worst = 0.0f;
  for (unsigned k = 0; k < 4000; ++k) {
    ArmCartesianTracker_Step(&t, &target, &limits, 0.001f);
    ArmIkPose pose;
    ArmIk_Forward(t.q, &pose);
    float d = 0.0f;
    for (unsigned i = 0; i < 3; ++i) d += (pose.position[i] - start.position[i]) * (pose.position[i] - start.position[i]);
    worst = fmaxf(worst, sqrtf(d));
  }
  ArmIkPose end, difference;
  ArmIk_Forward(t.q, &end);
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      difference.rotation[i][j] = target.rotation[i][0] * end.rotation[j][0] +
                                  target.rotation[i][1] * end.rotation[j][1] +
                                  target.rotation[i][2] * end.rotation[j][2];
  float e[3];
  ArmIk_RotationLog(&difference, e);
  assert(worst < 5e-5f);
  assert(sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]) < 1e-3f);
}

static void test_tracker_unreachable_target(void)
{
  /* Out of reach, at the default and the top speed: the arm stops at the
   * edge near the line, without running a joint into its limit, and holds
   * there (no creep along the edge) while the target stays. */
  const float q0[ARM_AXES] = {0.1f, 0.4f, 0.1f, 0.6f, 0.1f, 0.1f, 0.1f};
  const float *lower = ArmIk_JointLower(), *upper = ArmIk_JointUpper();
  for (unsigned c = 0; c < 3; ++c) {
    const ArmCartesianLimits limits = c == 2 ? stress_limits(1.5f, 3.142f, 5.0f, 15.0f) :
                                      c == 1 ? stress_limits(1.5f, 3.142f, 2.0f, 6.0f) : rt_limits();
    ArmCartesianTracker t;
    ArmCartesianTracker_Start(&t, q0, NULL, NULL);
    t.ik_options.time_budget_us = 0U;
    ArmIkPose start, target;
    ArmIk_Forward(q0, &start);
    target = start;
    target.position[0] += 0.6f;
    float held_q[ARM_AXES] = {0}, previous[ARM_AXES] = {0};
    for (unsigned k = 0; k < 8000; ++k) {
      const uint8_t status = ArmCartesianTracker_Step(&t, &target, &limits, 0.001f);
      assert(status != ARM_IK_INVALID_INPUT && status != ARM_IK_QP_NUMERICAL_FAILURE);
      for (unsigned i = 0; i < ARM_AXES; ++i) {
        /* Braking near a limit may exceed the cap (it must stop in time),
         * but no joint is cut off on its limit at speed. */
        assert(fabsf(t.v[i] - previous[i]) <= 2.0f * limits.joint_acceleration_max[i] * 0.001f + 1e-4f);
        assert(t.q[i] >= lower[i] && t.q[i] <= upper[i]);
        previous[i] = t.v[i];
      }
      if (k == 7000) memcpy(held_q, t.q, sizeof(held_q));
    }
    assert(t.blocked);
    for (unsigned i = 0; i < ARM_AXES; ++i) assert(t.v[i] == 0.0f && t.q[i] == held_q[i]);
    ArmIkPose reach;
    ArmIk_Forward(t.q, &reach);
    assert(reach.position[0] - start.position[0] > 0.5f);
    assert(fabsf(reach.position[1] - start.position[1]) < 0.003f &&
           fabsf(reach.position[2] - start.position[2]) < 0.003f);
  }
}

static void test_tracker_near_edge_round_trip(void)
{
  /* Reachable targets right up to the edge (the GUI only sends reachable
   * ones) go out and come back on the line at both speed settings. */
  const float q0[ARM_AXES] = {0.1f, 0.4f, 0.1f, 0.6f, 0.1f, 0.1f, 0.1f};
  /* With the H7C flange TCP the fixed-orientation +X line from this
   * seed ends at about 0.513 m. Keep the round-trip cases inside it. */
  const float out[] = {0.45f, 0.505f, 0.510f};
  for (unsigned c = 0; c < 3; ++c)
    for (unsigned n = 0; n < 3; ++n) {
      const ArmCartesianLimits limits = c == 2 ? stress_limits(1.5f, 3.142f, 5.0f, 15.0f) :
                                        c == 1 ? stress_limits(1.5f, 3.142f, 2.0f, 6.0f) : rt_limits();
      ArmCartesianTracker t;
      ArmCartesianTracker_Start(&t, q0, NULL, NULL);
      t.ik_options.time_budget_us = 0U;
      ArmIkPose start, target;
      ArmIk_Forward(q0, &start);
      target = start;
      target.position[0] += out[n];
      for (unsigned k = 0; k < 7000; ++k) ArmCartesianTracker_Step(&t, &target, &limits, 0.001f);
      ArmIkPose pose;
      ArmIk_Forward(t.q, &pose);
      assert(pose.position[0] - start.position[0] > out[n] - 0.001f);
      target = start;
      for (unsigned k = 0; k < 8000; ++k) ArmCartesianTracker_Step(&t, &target, &limits, 0.001f);
      ArmIk_Forward(t.q, &pose);
      for (unsigned i = 0; i < 3; ++i) assert(fabsf(pose.position[i] - start.position[i]) < 1e-4f);
    }
}

static void test_tracker_top_speed(void)
{
  /* 1.5 m/s and 180 deg/s at 2 and at the maximum 5 m/s^2: straight, every
   * joint within its rated speed and acceleration cap; J3/J4 (rated
   * 3.77 rad/s) slow the line where needed. */
  const float q0[ARM_AXES] = {0.3f, 1.0f, -0.2f, 1.2f, 0.2f, 0.3f, 0.1f};
  const float moves[3][3] = {{0.2f, 0.0f, 0.0f}, {0.0f, 0.2f, 0.0f}, {0.0f, 0.0f, -0.25f}};
  for (unsigned n = 0; n < 6; ++n) {
    const unsigned m = n % 3;
    const ArmCartesianLimits limits = n < 3 ? stress_limits(1.5f, 3.142f, 2.0f, 6.0f) :
                                              stress_limits(1.5f, 3.142f, 5.0f, 15.0f);
    ArmCartesianTracker t;
    ArmCartesianTracker_Start(&t, q0, NULL, NULL);
    t.ik_options.time_budget_us = 0U;
    ArmIkPose start, target;
    ArmIk_Forward(q0, &start);
    target = start;
    for (unsigned i = 0; i < 3; ++i) target.position[i] += moves[m][i];
    const float length = sqrtf(moves[m][0] * moves[m][0] + moves[m][1] * moves[m][1] + moves[m][2] * moves[m][2]);
    float previous[ARM_AXES] = {0}, worst = 0.0f;
    for (unsigned k = 0; k < 3000; ++k) {
      ArmCartesianTracker_Step(&t, &target, &limits, 0.001f);
      assert(!t.blocked);
      for (unsigned i = 0; i < ARM_AXES; ++i) {
        assert(fabsf(t.v[i]) <= rated_speed[i] + 1e-3f);
        assert(fabsf(t.v[i] - previous[i]) <= limits.joint_acceleration_max[i] * 0.001f + 1e-4f);
        previous[i] = t.v[i];
      }
      ArmIkPose pose;
      ArmIk_Forward(t.q, &pose);
      float d[3], along = 0.0f;
      for (unsigned i = 0; i < 3; ++i) { d[i] = pose.position[i] - start.position[i]; along += d[i] * moves[m][i] / length; }
      float side = 0.0f;
      for (unsigned i = 0; i < 3; ++i) { const float e = d[i] - along * moves[m][i] / length; side += e * e; }
      worst = fmaxf(worst, sqrtf(side));
    }
    ArmIkPose end;
    ArmIk_Forward(t.q, &end);
    assert(worst < 5e-5f);
    for (unsigned i = 0; i < 3; ++i) assert(fabsf(end.position[i] - target.position[i]) < 1e-5f);
  }
}

static void test_tracker_rotation_into_joint_limit(void)
{
  /* A large tool rotation at top speed that runs a wrist joint into its
   * URDF limit: the braking curve slows the whole motion so the joint stops
   * at the limit within (twice, while braking) its acceleration cap, instead
   * of being cut off there at speed (40-90x the cap without it). */
  const float q0[ARM_AXES] = {0.1f, 0.4f, 0.1f, 0.6f, 0.1f, 0.1f, 0.1f};
  const ArmCartesianLimits limits = stress_limits(1.5f, 3.142f, 2.0f, 6.0f);
  for (unsigned axis = 0; axis < 2; ++axis) {
    ArmCartesianTracker t;
    ArmCartesianTracker_Start(&t, q0, NULL, NULL);
    t.ik_options.time_budget_us = 0U;
    ArmIkPose start, target;
    ArmIk_Forward(q0, &start);
    target = start;
    float w[3] = {0.0f, 0.0f, 0.0f}, r[3][3];
    w[axis] = 3.0f;
    ArmIk_RotationExp(w, r);
    for (unsigned i = 0; i < 3; ++i)
      for (unsigned j = 0; j < 3; ++j)
        target.rotation[i][j] = r[i][0] * start.rotation[0][j] + r[i][1] * start.rotation[1][j] +
                                r[i][2] * start.rotation[2][j];
    float previous[ARM_AXES] = {0};
    for (unsigned k = 0; k < 5000; ++k) {
      ArmCartesianTracker_Step(&t, &target, &limits, 0.001f);
      for (unsigned i = 0; i < ARM_AXES; ++i) {
        assert(fabsf(t.v[i] - previous[i]) <= 2.0f * limits.joint_acceleration_max[i] * 0.001f + 1e-4f);
        previous[i] = t.v[i];
      }
      ArmIkPose pose;
      ArmIk_Forward(t.q, &pose);
      for (unsigned i = 0; i < 3; ++i) assert(fabsf(pose.position[i] - start.position[i]) < 0.002f);
    }
  }
}

static void test_ik_reports_final_residual(void)
{
  /* An iteration-capped solve reports the residual of the q it returns (the
   * host solver's behaviour), not the error from before its last step. */
  const float q0[ARM_IK_AXES] = {0.1f, 0.4f, 0.1f, 0.6f, 0.1f, 0.1f, 0.1f};
  ArmIkPose target;
  ArmIk_Forward(q0, &target);
  target.position[0] += 0.001f;
  ArmIkOptions options;
  ArmMotion_TrackingIkOptions(&options);
  options.time_budget_us = 0U;
  options.tolerance = 1.0e-12f;
  const ArmIkResult r = ArmIk_Solve(&target, q0, &options, NULL);
  assert(r.status == ARM_IK_MAX_ITERATIONS && r.iterations == 1U);
  assert(r.position_error < 0.0002f);
  ArmIkPose reached;
  ArmIk_Forward(r.q, &reached);
  const float dx = target.position[0] - reached.position[0], dy = target.position[1] - reached.position[1],
              dz = target.position[2] - reached.position[2];
  assert(fabsf(sqrtf(dx * dx + dy * dy + dz * dz) - r.position_error) < 1e-6f);
}

/* Peak linear and angular reference accelerations of a move. */
static void measure_reference_acceleration(const float q0[ARM_AXES], const ArmIkPose *target,
                                           const ArmCartesianLimits *limits, float *linear,
                                           float *angular, ArmIkPose *end)
{
  ArmCartesianTracker t;
  ArmCartesianTracker_Start(&t, q0, NULL, NULL);
  t.ik_options.time_budget_us = 0U;
  float previous[6] = {0};
  *linear = *angular = 0.0f;
  for (unsigned k = 0; k < 3000; ++k) {
    ArmCartesianTracker_Step(&t, target, limits, 0.001f);
    float l = 0.0f, a = 0.0f;
    for (unsigned i = 0; i < 3; ++i) {
      l += (t.reference_velocity[i] - previous[i]) * (t.reference_velocity[i] - previous[i]);
      a += (t.reference_velocity[i + 3] - previous[i + 3]) * (t.reference_velocity[i + 3] - previous[i + 3]);
    }
    *linear = fmaxf(*linear, sqrtf(l) / 0.001f);
    *angular = fmaxf(*angular, sqrtf(a) / 0.001f);
    memcpy(previous, t.reference_velocity, sizeof(previous));
  }
  ArmIk_Forward(t.q, end);
}

static void test_tracker_direction_acceleration(void)
{
  /* Speed ratio 0.5 m/s : 180 deg/s differs from the acceleration ratio
   * 2 m/s^2 : 360 deg/s^2.  Each part must reach its own limit: one scalar
   * for both translated at 1 m/s^2 only. */
  const float q0[ARM_AXES] = {0.1f, 0.4f, 0.1f, 0.6f, 0.1f, 0.1f, 0.1f};
  const ArmCartesianLimits limits = wire_limits(500, 3142, 2000, 6283);
  ArmIkPose start, target, end;
  ArmIk_Forward(q0, &start);
  float linear, angular;
  target = start;
  target.position[0] += 0.3f;
  measure_reference_acceleration(q0, &target, &limits, &linear, &angular, &end);
  /* The angular part is float noise at arrival (~1e-5 rad/s for a 1 ms tick). */
  assert(linear > 1.98f && linear < 2.0f + 1e-3f && angular < 0.1f);
  assert(fabsf(end.position[0] - target.position[0]) < 1e-5f);
  const float w[3] = {0.0f, 0.0f, 1.2f}, mixed[3] = {0.0f, 0.0f, 0.785f};
  for (unsigned c = 0; c < 2; ++c) {
    float r[3][3];
    ArmIk_RotationExp(c ? mixed : w, r);
    target = start;
    if (c) target.position[0] += 0.1f;
    for (unsigned i = 0; i < 3; ++i)
      for (unsigned j = 0; j < 3; ++j)
        target.rotation[i][j] = r[i][0] * start.rotation[0][j] + r[i][1] * start.rotation[1][j] +
                                r[i][2] * start.rotation[2][j];
    measure_reference_acceleration(q0, &target, &limits, &linear, &angular, &end);
    /* Pure rotation reaches its limit; a combined move keeps both within. */
    assert(angular < 6.283f + 1e-2f && linear < 2.0f + 1e-3f);
    if (!c) assert(angular > 6.2f);
    for (unsigned i = 0; i < 3; ++i) assert(fabsf(end.position[i] - target.position[i]) < 1e-4f);
  }
}

/* DM position-velocity drive as measured on the arm: the joint follows its
 * target with a first-order lag of gear ratio / KP_APR (54) seconds, at most
 * v_des.  At 0.1 rad/s J2/J4/J7 trailed by 17/73/18 mrad. */
static const float drive_tau[ARM_AXES] = {9.0f / 54, 9.0f / 54, 40.0f / 54, 40.0f / 54,
                                          10.0f / 54, 10.0f / 54, 10.0f / 54};
static void posvel_drive(float *position, float target, float tau, float v_des, float dt)
{
  *position += fminf(fmaxf((target - *position) / tau, -v_des), v_des) * dt;
}

/* The target the MCU sends a position-velocity drive for command q moving at
 * velocity (as RT Cartesian, movej and movep publish it): with the lead
 * (lead = 1) or the command as is (lead = 0). */
static float drive_target(unsigned i, float q, float velocity, float measured, int lead)
{
  return lead ? ArmDriveLead_Target(q, velocity, drive_tau[i], measured, ArmIk_JointLower()[i],
                                    ArmIk_JointUpper()[i]) : q;
}

static void test_drive_lead(void)
{
  const float tau = 0.5f, dt = 0.001f;
  ArmDriveLead l;
  ArmDriveLead_Reset(&l, 0.1f);
  /* At rest: 0.  A ramp at 0.3 rad/s reads 0.3 from the first difference on. */
  assert(ArmDriveLead_Velocity(&l, 0.1f, 1.0f, dt) == 0.0f);
  float q = 0.1f, v = 0.0f;
  for (unsigned k = 0; k < 30; ++k) {
    q += 0.3f * dt;
    v = ArmDriveLead_Velocity(&l, q, 1.0f, dt);
    assert(fabsf(v - 0.3f) < 1e-3f);
  }
  /* A re-latched command (a jump, not motion) restarts the window at 0. */
  assert(ArmDriveLead_Velocity(&l, q + 0.05f, 1.0f, dt) == 0.0f);
  /* The RT joint slew: 0.5 rad/s for 2 ms of every 10 ms target = 0.1 rad/s;
   * the window reads the steady speed, not the bursts. */
  ArmDriveLead_Reset(&l, 0.0f);
  q = 0.0f;
  for (unsigned k = 0; k < 200; ++k) {
    if (k % 10 < 2) q += 0.5f * dt;
    v = ArmDriveLead_Velocity(&l, q, 1.0f, dt);
    if (k >= 10) assert(fabsf(v - 0.1f) < 1e-4f);
  }
  /* The target: q + tau * v, also past a limit while the joint is inside it;
   * a joint at or beyond a limit is not pushed further out. */
  assert(fabsf(ArmDriveLead_Target(0.1f, -0.4f, tau, 0.1f, 0.0f, 1.0f) + 0.1f) < 1e-6f);
  assert(ArmDriveLead_Target(0.1f, -0.4f, tau, -0.001f, 0.0f, 1.0f) == 0.0f);
  assert(fabsf(ArmDriveLead_Target(0.1f, 0.4f, tau, -0.001f, 0.0f, 1.0f) - 0.3f) < 1e-6f);
  assert(ArmDriveLead_Target(0.95f, 0.4f, tau, 1.0f, 0.0f, 1.0f) == 1.0f);
  assert(ArmDriveLead_Target(-0.05f, -0.4f, tau, -0.05f, 0.0f, 1.0f) == -0.05f);
  /* tau <= 0 or no valid velocity: the command. */
  assert(ArmDriveLead_Target(0.1f, 0.4f, 0.0f, 0.1f, 0.0f, 1.0f) == 0.1f);
  assert(ArmDriveLead_Target(0.1f, NAN, tau, 0.1f, 0.0f, 1.0f) == 0.1f);
  /* RT joint: the PC steps J4's target 1 mrad every 10 ms (0.1 rad/s), the
   * MCU slews at 0.5 rad/s; with the windowed lead the drive tracks within a
   * millimetre-sized error (73 mrad measured without). */
  float command = 0.5f, joint = 0.5f, goal = 0.5f, worst = 0.0f;
  ArmDriveLead_Reset(&l, command);
  for (unsigned k = 0; k < 6000; ++k) {
    if (k % 10 == 0 && k < 3000) goal += 0.001f;
    command += fminf(fmaxf(goal - command, -0.0005f), 0.0005f);
    v = ArmDriveLead_Velocity(&l, command, 3.0f, dt);
    posvel_drive(&joint, ArmDriveLead_Target(command, v, drive_tau[3], joint, 0.0f, 3.14f),
                 drive_tau[3], 3.0f, dt);
    worst = fmaxf(worst, fabsf(command - joint));
  }
  assert(worst < 0.003f);
}

static void test_cap_drive_speeds(void)
{
  ArmCartesianLimits l = rt_limits();
  const float drive[ARM_AXES] = {1.0f, 1.0f, 2.0f, 0.0f, 1.0f, 1.0f, 1.0f};
  const float before = l.joint_velocity_max[4];
  ArmMotion_CapDriveSpeeds(&l, drive, 0x0fU);
  assert(fabsf(l.joint_velocity_max[0] - ARM_POSVEL_SPEED_FRACTION * 1.0f) < 1e-6f);
  assert(fabsf(l.joint_velocity_max[2] - ARM_POSVEL_SPEED_FRACTION * 2.0f) < 1e-6f);
  assert(l.joint_velocity_max[3] <= 1.0e-3f + 1e-9f); /* v_des 0: the drive does not move */
  assert(l.joint_velocity_max[4] == before);          /* not position-velocity */
}

/* RT Cartesian from q0 to the tool position goal with position-velocity
 * drives (v_des 3 rad/s) whose time constants are drive_tau * scale.  The
 * largest joint lag, tool distance from its command and joint excursion
 * beyond a URDF limit, and the ticks at which the command and the tool came
 * within 1 and 2 mm of the goal (0 = never). */
typedef struct { float lag, off_command, beyond; unsigned commanded, arrived; } DriveRun;
static DriveRun drive_run(const float q0[ARM_AXES], const float goal[3], int lead, float scale)
{
  const float v_des[ARM_AXES] = {3, 3, 3, 3, 3, 3, 3};
  ArmCartesianLimits limits = rt_limits();
  ArmMotion_CapDriveSpeeds(&limits, v_des, 0x7fU);
  ArmCartesianTracker t;
  ArmCartesianTracker_Start(&t, q0, NULL, NULL);
  t.ik_options.time_budget_us = 0U;
  ArmIkPose target;
  ArmIk_Forward(q0, &target);
  memcpy(target.position, goal, sizeof(target.position));
  float measured[ARM_AXES];
  memcpy(measured, q0, sizeof(measured));
  DriveRun r = {0};
  for (unsigned k = 1; k <= 4000; ++k) {
    ArmCartesianTracker_Step(&t, &target, &limits, 0.001f);
    for (unsigned i = 0; i < ARM_AXES; ++i) {
      const float sent = drive_target(i, t.q[i], t.v[i], measured[i], lead);
      posvel_drive(&measured[i], sent, drive_tau[i] * scale, v_des[i], 0.001f);
      r.lag = fmaxf(r.lag, fabsf(t.q[i] - measured[i]));
      r.beyond = fmaxf(r.beyond, fmaxf(ArmIk_JointLower()[i] - measured[i],
                                       measured[i] - ArmIk_JointUpper()[i]));
    }
    ArmIkPose command, actual;
    ArmIk_Forward(t.q, &command);
    ArmIk_Forward(measured, &actual);
    float off = 0.0f, to_goal = 0.0f, command_to_goal = 0.0f;
    for (unsigned i = 0; i < 3; ++i) {
      off += powf(actual.position[i] - command.position[i], 2.0f);
      to_goal += powf(actual.position[i] - goal[i], 2.0f);
      command_to_goal += powf(command.position[i] - goal[i], 2.0f);
    }
    r.off_command = fmaxf(r.off_command, sqrtf(off));
    if (!r.commanded && sqrtf(command_to_goal) < 1e-3f) r.commanded = k;
    if (!r.arrived && sqrtf(to_goal) < 2e-3f) r.arrived = k;
  }
  return r;
}

static void test_tracker_position_velocity_drives(void)
{
  /* Full speed from the slow-home pose.  Sent as is, J3/J4 (0.74 s) trail
   * by up to ~0.4 rad and the tool arrives seconds after its command; with
   * the lead it is where it is commanded. */
  const float home[ARM_AXES] = {0};
  ArmIkPose p;
  ArmIk_Forward(home, &p);
  const float up[3] = {p.position[0], p.position[1], p.position[2] + 0.15f};
  const float out[3] = {p.position[0] + 0.15f, p.position[1], p.position[2]};
  for (unsigned c = 0; c < 2; ++c) {
    const DriveRun plain = drive_run(home, c ? out : up, 0, 1.0f);
    const DriveRun led = drive_run(home, c ? out : up, 1, 1.0f);
    assert(plain.commanded && led.commanded);
    assert(plain.lag > 0.3f && plain.off_command > 0.02f &&
           (!plain.arrived || plain.arrived > plain.commanded + 1500U));
    assert(led.lag < 0.01f && led.off_command < 0.002f && led.arrived &&
           led.arrived < led.commanded + 100U);
  }
  /* Approach the lower limits from the field-run joint pose. Specify the
   * goal relative to flange home so changing the tool does not put it
   * outside the workspace (the original goal was ~40 mm above home). */
  const float high[ARM_AXES] = {0.001f, 0.374f, 0.0f, 0.800f, 0.0f, 0.0f, -0.368f};
  ArmIk_Forward(high, &p);
  ArmIkPose home_pose;
  ArmIk_Forward(home, &home_pose);
  const float down[3] = {p.position[0], p.position[1], home_pose.position[2]+0.040f};
  const DriveRun field = drive_run(high, down, 1, 1.0f);
  assert(field.lag < 0.01f && field.off_command < 0.002f && field.arrived &&
         field.arrived < field.commanded + 100U);
  /* Into the limit (a target below home): a drive 20 % faster than modelled
   * runs ahead of its command, but stops at the limit (120 mrad past it with
   * the lead alone). */
  ArmIk_Forward(home, &p);
  const float below_home[3] = {p.position[0], p.position[1], p.position[2] - 0.05f};
  for (unsigned s = 0; s < 3; ++s) assert(drive_run(high, below_home, 1, 0.8f + 0.2f * s).beyond < 0.003f);
}

static void test_movep_position_velocity_drives(void)
{
  /* movep with position-velocity drives arrives with the lead; without it a
   * position-velocity joint far behind its command is not a fault (the drive
   * runs its own loop), while a MIT one is (test above). */
  for (int lead = 1; lead >= 0; --lead) {
    ArmMoveFeedback f = feedback();
    const float q[ARM_AXES] = {0.002f, 0.025f, 0.001f, 0.036f, 0.010f, 0.004f, -0.325f};
    memcpy(f.position, q, sizeof(f.position));
    memcpy(f.command, q, sizeof(f.command));
    f.command_valid = 1;
    f.posvel_mask = 0x7fU;
    ArmMoveCartesianCommand p = cartesian_at(q);
    p.pose[2] += 0.112f;
    p.position_tolerance = 0.001f;
    for (unsigned i = 0; i < ARM_AXES; ++i) { p.velocity_max[i] = 3.0f; p.acceleration_max[i] = 100.0f; }
    ArmMotion m = {0};
    assert(ArmMotion_Request(&m, ARM_MOVEP, 40, 1, &p, &f, 0) == ARM_OK);
    for (unsigned i = 0; i < ARM_AXES; ++i)
      assert(m.cartesian_limits.joint_velocity_max[i] <= ARM_POSVEL_SPEED_FRACTION * 3.0f + 1e-6f);
    float worst = 0.0f;
    uint32_t t = 1;
    for (; t < 9000 && m.state != ARM_DONE; ++t) {
      ArmMotion_Request(&m, ARM_KEEPALIVE, 40, 1, &p, &f, t);
      ArmMotion_Tick(&m, &f, t);
      assert(m.state != ARM_FAILED);
      for (unsigned i = 0; i < ARM_AXES; ++i) {
        const float before = f.position[i];
        posvel_drive(&f.position[i], drive_target(i, m.position[i], m.velocity[i], f.position[i], lead),
                     drive_tau[i], 3.0f, 0.001f);
        f.velocity[i] = (f.position[i] - before) / 0.001f;
        worst = fmaxf(worst, fabsf(m.position[i] - f.position[i]));
      }
    }
    assert(m.state == ARM_DONE);
    if (lead) assert(worst < 0.02f && t < 1000U);
    else assert(worst > ARM_CARTESIAN_TRACKING_LIMIT_RAD);
  }
}

static void test_cartesian_limits_from_wire(void)
{
  ArmCartesianLimits l;
  const uint16_t zero[4] = {0, 0, 0, 0};
  assert(ArmMotion_CartesianLimitsFromWire(zero, &l));
  assert(fabsf(l.linear_velocity_max - ARM_CARTESIAN_DEFAULT_LINEAR_VELOCITY) < 1e-6f);
  assert(fabsf(l.angular_acceleration_max - ARM_CARTESIAN_DEFAULT_ANGULAR_ACCELERATION) < 1e-6f);
  const uint16_t fast[4] = {250, 1000, 1000, 3000};
  assert(ArmMotion_CartesianLimitsFromWire(fast, &l));
  assert(fabsf(l.linear_velocity_max - 0.25f) < 1e-6f && fabsf(l.angular_velocity_max - 1.0f) < 1e-6f);
  assert(fabsf(l.linear_acceleration_max - 1.0f) < 1e-6f && fabsf(l.angular_acceleration_max - 3.0f) < 1e-6f);
  /* Joint caps from the same ratios: max(0.25 / 0.2, 1.0 / 0.6); acceleration
   * with the 3x margin: 3 * max(1.0 / 0.2, 3.0 / 0.6). */
  for (unsigned i = 0; i < ARM_AXES; ++i)
    assert(fabsf(l.joint_velocity_max[i] - 1.0f / 0.6f) < 1e-4f && fabsf(l.joint_acceleration_max[i] - 15.0f) < 1e-4f);
  const uint16_t over[4] = {501, 0, 0, 0}, over_angular[4] = {0, 3143, 0, 0};
  assert(!ArmMotion_CartesianLimitsFromWire(over, &l));
  assert(fabsf(l.linear_velocity_max - ARM_CARTESIAN_DEFAULT_LINEAR_VELOCITY) < 1e-6f);
  assert(!ArmMotion_CartesianLimitsFromWire(over_angular, &l));
  /* The maxima (= defaults): 0.5 m/s, 180 deg/s (3142 mrad/s), 2 m/s^2,
   * 360 deg/s^2 (6283 mrad/s^2). */
  const uint16_t max[4] = {500, 3142, 2000, 6283};
  assert(ArmMotion_CartesianLimitsFromWire(max, &l));
  assert(fabsf(l.linear_velocity_max - 0.5f) < 1e-6f && fabsf(l.angular_velocity_max - 3.142f) < 1e-5f);
  assert(fabsf(l.linear_acceleration_max - 2.0f) < 1e-6f && fabsf(l.angular_acceleration_max - 6.283f) < 1e-5f);
  for (unsigned i = 0; i < ARM_AXES; ++i)
    assert(fabsf(l.joint_velocity_max[i] - INFRABOT_ARM_MAX_JOINT_VELOCITY) < 1e-6f &&
           fabsf(l.joint_acceleration_max[i] - 3.0f * 6.283f / 0.6f) < 1e-3f);
  const uint16_t over_linear_acceleration[4] = {0, 0, 2001, 0}, over_angular_acceleration[4] = {0, 0, 0, 6284};
  assert(!ArmMotion_CartesianLimitsFromWire(over_linear_acceleration, &l));
  assert(!ArmMotion_CartesianLimitsFromWire(over_angular_acceleration, &l));
  assert(ArmMotion_CartesianLimitsFromWire(max, &l));
  /* Rated speeds cap individual joints only. */
  ArmMotion_CapJointLimits(&l, rated_speed, NULL);
  assert(fabsf(l.joint_velocity_max[2] - INFRABOT_ARM_MAX_JOINT_VELOCITY) < 1e-6f &&
         fabsf(l.joint_velocity_max[0] - INFRABOT_ARM_MAX_JOINT_VELOCITY) < 1e-6f);
  assert(fabsf(l.linear_velocity_max - 0.5f) < 1e-6f);

  /* Decreases ramp at the acceleration limit, increases apply at once. */
  ArmCartesianLimits applied, slow;
  assert(ArmMotion_CartesianLimitsFromWire(fast, &applied));
  const uint16_t slow_wire[4] = {50, 200, 1000, 3000};
  assert(ArmMotion_CartesianLimitsFromWire(slow_wire, &slow));
  ArmMotion_RampCartesianLimits(&applied, &slow, 0.001f);
  assert(fabsf(applied.linear_velocity_max - (0.25f - 1.0f * 0.001f)) < 1e-6f);
  assert(fabsf(applied.angular_velocity_max - (1.0f - 3.0f * 0.001f)) < 1e-6f);
  for (unsigned k = 0; k < 1000; ++k) ArmMotion_RampCartesianLimits(&applied, &slow, 0.001f);
  assert(fabsf(applied.linear_velocity_max - 0.05f) < 1e-6f && fabsf(applied.angular_velocity_max - 0.2f) < 1e-6f);
  ArmCartesianLimits faster;
  assert(ArmMotion_CartesianLimitsFromWire(fast, &faster));
  ArmMotion_RampCartesianLimits(&applied, &faster, 0.001f);
  assert(fabsf(applied.linear_velocity_max - 0.25f) < 1e-6f);
}

/* Runs an X move, optionally lowering the speed setting mid-way as an RT
 * stream would; returns the time to arrive (ms) and the worst Y/Z deviation. */
static unsigned run_speed_case(const uint16_t wire[4], const uint16_t *slower_at_150ms, float *deviation)
{
  const float q0[ARM_AXES] = {0.1f, 0.4f, 0.1f, 0.6f, 0.1f, 0.1f, 0.1f};
  ArmCartesianLimits requested, applied;
  assert(ArmMotion_CartesianLimitsFromWire(wire, &requested));
  applied = requested;
  ArmCartesianTracker t;
  ArmCartesianTracker_Start(&t, q0, NULL, NULL);
  t.ik_options.time_budget_us = 0U;
  ArmIkPose start, target;
  ArmIk_Forward(q0, &start);
  target = start;
  target.position[0] += 0.05f;
  float previous[ARM_AXES] = {0};
  unsigned arrived = 0U;
  *deviation = 0.0f;
  for (unsigned k = 0; k < 4000; ++k) {
    if (slower_at_150ms && k == 150) assert(ArmMotion_CartesianLimitsFromWire(slower_at_150ms, &requested));
    ArmMotion_RampCartesianLimits(&applied, &requested, 0.001f);
    const uint8_t status = ArmCartesianTracker_Step(&t, &target, &applied, 0.001f);
    assert(status != ARM_IK_INVALID_INPUT && status != ARM_IK_QP_NUMERICAL_FAILURE);
    for (unsigned i = 0; i < ARM_AXES; ++i) {
      assert(fabsf(t.v[i]) <= applied.joint_velocity_max[i] + 1e-4f);
      assert(fabsf(t.v[i] - previous[i]) <= applied.joint_acceleration_max[i] * 0.001f + 1e-4f);
      previous[i] = t.v[i];
    }
    ArmIkPose pose;
    ArmIk_Forward(t.q, &pose);
    const float dy = pose.position[1] - start.position[1], dz = pose.position[2] - start.position[2];
    *deviation = fmaxf(*deviation, sqrtf(dy * dy + dz * dz));
    if (!arrived && fabsf(pose.position[0] - target.position[0]) < 1e-5f) arrived = k;
  }
  assert(arrived);
  return arrived;
}

static void test_cartesian_speed_setting(void)
{
  /* A faster setting really is faster and stays on the line; lowering it
   * mid-move slows the tool down smoothly, still on the line. */
  const uint16_t standard[4] = {100, 349, 400, 1396}, fast[4] = {300, 1000, 1200, 4000};
  const uint16_t slow[4] = {30, 100, 1200, 4000};
  float deviation;
  const unsigned standard_ms = run_speed_case(standard, NULL, &deviation);
  assert(deviation < 5e-5f);
  const unsigned fast_ms = run_speed_case(fast, NULL, &deviation);
  assert(deviation < 5e-5f);
  assert(fast_ms < 450U && fast_ms * 3U < standard_ms * 2U);
  const unsigned slowed_ms = run_speed_case(fast, slow, &deviation);
  assert(deviation < 5e-5f && slowed_ms > fast_ms);
}

static void test_movep_speed_setting(void)
{
  /* movep takes the tool speed from cartesian_limits; per-joint caps still
   * tighten it, and an out-of-range speed is rejected. */
  ArmMoveFeedback f = feedback();
  const float q[ARM_AXES] = {0.1f, 0.4f, 0.1f, 0.6f, 0.1f, 0.1f, 0.1f};
  memcpy(f.position, q, sizeof(f.position));
  memcpy(f.command, q, sizeof(f.command));
  f.command_valid = 1;
  ArmMoveCartesianCommand p = cartesian_at(q);
  p.pose[0] += 0.03f;
  p.position_tolerance = 0.0005f;
  for (unsigned i = 0; i < ARM_AXES; ++i) { p.velocity_max[i] = 2.5f; p.acceleration_max[i] = 10.0f; }
  unsigned done_ms[2] = {0, 0};
  for (unsigned c = 0; c < 2; ++c) {
    const uint16_t fast[4] = {300, 1000, 1200, 4000}, standard[4] = {100, 349, 400, 1396};
    memcpy(p.cartesian_limits, c ? fast : standard, sizeof(p.cartesian_limits));
    memcpy(f.position, q, sizeof(f.position));
    memset(f.velocity, 0, sizeof(f.velocity));
    ArmMotion m = {0};
    assert(ArmMotion_Request(&m, ARM_MOVEP, 20 + c, 1, &p, &f, 0) == ARM_OK);
    uint32_t t = 1;
    for (; t < 5000 && m.state != ARM_DONE; ++t) {
      ArmMotion_Request(&m, ARM_KEEPALIVE, 20 + c, 1, &p, &f, t);
      ArmMotion_Tick(&m, &f, t);
      assert(m.state != ARM_FAILED);
      memcpy(f.position, m.position, sizeof(f.position));
      memcpy(f.velocity, m.velocity, sizeof(f.velocity));
    }
    assert(m.state == ARM_DONE);
    done_ms[c] = t;
  }
  assert(done_ms[1] * 3U < done_ms[0] * 2U);
  /* Per-joint caps and the motors' rated speeds cap joints, not the tool
   * speed setting: the common step scaling slows the line where they bind. */
  ArmMotion m = {0};
  for (unsigned i = 0; i < ARM_AXES; ++i) { p.velocity_max[i] = 0.1f; p.acceleration_max[i] = 0.2f; }
  memcpy(f.rated_speed, rated_speed, sizeof(f.rated_speed));
  f.rated_speed[0] = 0.05f;
  memcpy(f.position, q, sizeof(f.position));
  memset(f.velocity, 0, sizeof(f.velocity));
  assert(ArmMotion_Request(&m, ARM_MOVEP, 30, 1, &p, &f, 0) == ARM_OK);
  assert(fabsf(m.cartesian_limits.linear_velocity_max - 0.3f) < 1e-6f);
  assert(fabsf(m.cartesian_limits.joint_velocity_max[0] - 0.05f) < 1e-6f);
  assert(fabsf(m.cartesian_limits.joint_velocity_max[1] - 0.1f) < 1e-6f);
  assert(fabsf(m.cartesian_limits.joint_acceleration_max[3] - 0.2f) < 1e-6f);
  assert(fabsf(m.cartesian_limits.linear_acceleration_max - ARM_CARTESIAN_LEVER_M * 0.2f) < 1e-6f);
  assert(fabsf(m.cartesian_limits.angular_acceleration_max - ARM_CARTESIAN_ANGULAR_RATIO * 0.2f) < 1e-6f);
  ArmMotion_Cancel(&m);
  memset(&m, 0, sizeof(m));
  p.cartesian_limits[0] = 501;
  assert(ArmMotion_Request(&m, ARM_MOVEP, 31, 1, &p, &f, 0) == ARM_INVALID && !m.owned);
}

int main(void)
{
  assert(offsetof(AppMotionRequest, command) == 20);
  assert(offsetof(AppMotionRequest, crc32) == 204);
  assert(offsetof(AppMotionReply, reason) == 48);
  test_profile_and_blocking();
  test_failures();
  test_validation_and_clock_wrap();
  test_start_from_command();
  test_cartesian_integrates_on_command();
  test_follow_joints();
  test_cartesian_no_overshoot();
  test_follow_moving_goal();
  test_tracker_straight_line();
  test_tracker_joint_limited_line();
  test_tracker_pure_rotation();
  test_tracker_unreachable_target();
  test_tracker_near_edge_round_trip();
  test_tracker_top_speed();
  test_tracker_rotation_into_joint_limit();
  test_ik_reports_final_residual();
  test_tracker_direction_acceleration();
  test_drive_lead();
  test_cap_drive_speeds();
  test_tracker_position_velocity_drives();
  test_movep_position_velocity_drives();
  test_cartesian_limits_from_wire();
  test_cartesian_speed_setting();
  test_movep_speed_setting();
  puts("arm_motion: profile, serial completion, retry, limits, faults, wrap, command-start, Cartesian integration, follower, no-overshoot, straight-line, joint-limited line, rotation, reach, edge, top-speed, joint-limit rotation, IK-residual, direction-acceleration, drive-lead, position-velocity drive and speed-setting tests passed");
}
