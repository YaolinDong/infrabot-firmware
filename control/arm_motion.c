/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "arm_motion.h"
#include <math.h>
float ArmMotion_StreamPositionTarget(float current,float target,float rate,float dt)
{
  if (!isfinite(current) || !isfinite(target)) return current;
  if (rate <= 0.0f) return target;
  const float step=rate*dt, delta=target-current;
  if (delta>step) return current+step;
  if (delta<-step) return current-step;
  return target;
}
#include <string.h>

void ArmMotion_Fail(ArmMotion *m, uint8_t reason)
{
  if (!m->owned || m->state == ARM_FAILED) return;
  m->state = ARM_FAILED;
  m->reason = reason;
  m->dwelling = 0;
  m->feedback_stale = 0;
  memset(m->velocity, 0, sizeof(m->velocity));
}

void ArmMotion_Cancel(ArmMotion *m)
{
  if (!m->owned) return;
  m->closed_session = m->session;
  m->owned = 0;
  m->state = ARM_CANCELLED;
  m->reason = ARM_CANCEL;
  m->dwelling = 0;
  m->feedback_stale = 0;
  m->cartesian = 0U;
  memset(m->velocity, 0, sizeof(m->velocity));
}

void ArmMotion_TrackingIkOptions(ArmIkOptions *options)
{
  ArmIk_DefaultOptions(options);
  /* One iteration per 1 kHz tick from the joint command (a differential IK
   * step): cheap and deterministic; a residual is finished next tick. */
  options->max_iterations = 1U;
  options->tolerance = 1.0e-5f;
  options->max_step_norm = 0.20f;
  options->max_joint_step = 0.12f;
  /* A full Gauss-Newton step: the per-tick steps are tiny, so linearization
   * is exact enough, and the 0.8 of the host's full solve would leave the
   * command lagging the moving reference by ~1/3 of a step per tick. */
  options->step_size = 1.0f;
  /* Drift the redundant elbow toward the neutral posture while moving (at
   * most 2e-5 rad per tick, 0.02 rad/s) instead of swinging it when motion
   * starts. */
  options->posture_step_max = 2.0e-5f;
  /* Leaves the rest of the 1 ms tick for CAN; an expired solve returns its
   * last accepted iterate and the next tick continues from it. */
  options->time_budget_us = 300U;
}

static float clampf(float value, float lower, float upper)
{
  return fminf(fmaxf(value, lower), upper);
}

void ArmMotion_FollowJoints(float q[ARM_AXES], float v[ARM_AXES], const float goal[ARM_AXES],
                            const float goal_velocity[ARM_AXES],
                            const float velocity_max[ARM_AXES],
                            const float acceleration_max[ARM_AXES], float dt)
{
  /* A one-step IK under the same limits is greedy: it brakes only once it is
   * on the goal, overshoots and swings around it indefinitely. */
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    const float speed = velocity_max[i];
    const float feed = goal_velocity ? clampf(goal_velocity[i], -speed, speed) : 0.0f;
    /* What is left after the goal's own motion this tick; zero while tracking. */
    const float error = goal[i] - q[i] - feed * dt;
    const float a = acceleration_max[i];
    float next;
    if (a > 0.0f) {
      /* Highest closing speed that still stops exactly on the goal with
       * steps of a*dt (discrete form of sqrt(2 a |error|)). */
      const float dv = a * dt;
      const float brake = sqrtf(0.25f * dv * dv + 2.0f * a * fabsf(error)) - 0.5f * dv;
      next = clampf(feed + clampf(error / dt, -brake, brake), -speed, speed);
      next = clampf(next, v[i] - dv, v[i] + dv);
    } else {
      next = clampf(feed + error / dt, -speed, speed);
    }
    v[i] = next;
    q[i] += next * dt;
  }
}

static void orthonormalize(float r[3][3])
{
  /* Gram-Schmidt on the columns; keeps the IK's orthonormality check happy
   * after many small rotations. */
  float x[3] = {r[0][0], r[1][0], r[2][0]}, y[3] = {r[0][1], r[1][1], r[2][1]};
  const float xn = sqrtf(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
  for (unsigned i = 0; i < 3U; ++i) x[i] /= xn;
  const float d = x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
  for (unsigned i = 0; i < 3U; ++i) y[i] -= d * x[i];
  const float yn = sqrtf(y[0] * y[0] + y[1] * y[1] + y[2] * y[2]);
  for (unsigned i = 0; i < 3U; ++i) y[i] /= yn;
  const float z[3] = {x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]};
  for (unsigned i = 0; i < 3U; ++i) { r[i][0] = x[i]; r[i][1] = y[i]; r[i][2] = z[i]; }
}

/* Largest acceleration along the 6-D direction x (position / length, then
 * rotation vector) whose linear and angular parts each stay within their own
 * limit.  One scalar for both (the tighter of the two) applied the angular
 * limit to a pure translation whenever the speed and acceleration ratios
 * differ: 0.5 m/s, 180 deg/s, 2 m/s^2, 360 deg/s^2 translated at 1 m/s^2. */
static float direction_acceleration(const float x[6], float length,
                                    const ArmCartesianLimits *limits)
{
  float linear = 0.0f, angular = 0.0f;
  for (unsigned i = 0; i < 3U; ++i) { linear += x[i] * x[i]; angular += x[i + 3U] * x[i + 3U]; }
  const float norm = sqrtf(linear + angular);
  if (norm <= 1.0e-12f)
    return fminf(limits->linear_acceleration_max / length, limits->angular_acceleration_max);
  linear = sqrtf(linear) / norm * length; /* m/s^2 per unit along x */
  angular = sqrtf(angular) / norm;        /* rad/s^2 per unit along x */
  float acceleration = 1.0e30f;
  if (linear > 1.0e-9f) acceleration = fminf(acceleration, limits->linear_acceleration_max / linear);
  if (angular > 1.0e-9f) acceleration = fminf(acceleration, limits->angular_acceleration_max / angular);
  return acceleration;
}

/* Left-multiplies rotation by exp(vector): a base-frame rotation. */
static void rotate_base(float rotation[3][3], const float vector[3])
{
  float delta[3][3], out[3][3];
  ArmIk_RotationExp(vector, delta);
  for (unsigned i = 0; i < 3U; ++i)
    for (unsigned j = 0; j < 3U; ++j)
      out[i][j] = delta[i][0] * rotation[0][j] + delta[i][1] * rotation[1][j] +
                  delta[i][2] * rotation[2][j];
  memcpy(rotation, out, sizeof(out));
  orthonormalize(rotation);
}

static void joint_caps_from_cartesian(ArmCartesianLimits *limits)
{
  /* The same ratio for every joint; acceleration with margin. */
  const float velocity = fminf(INFRABOT_ARM_MAX_JOINT_VELOCITY,
                               fmaxf(limits->linear_velocity_max / ARM_CARTESIAN_LEVER_M,
                                     limits->angular_velocity_max / ARM_CARTESIAN_ANGULAR_RATIO));
  const float acceleration = ARM_CARTESIAN_JOINT_ACCELERATION_MARGIN *
                             fmaxf(limits->linear_acceleration_max / ARM_CARTESIAN_LEVER_M,
                                   limits->angular_acceleration_max / ARM_CARTESIAN_ANGULAR_RATIO);
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    limits->joint_velocity_max[i] = velocity;
    limits->joint_acceleration_max[i] = acceleration;
  }
}

uint8_t ArmMotion_CartesianLimitsFromWire(const uint16_t wire[4], ArmCartesianLimits *limits)
{
  static const float defaults[4] = {
      ARM_CARTESIAN_DEFAULT_LINEAR_VELOCITY, ARM_CARTESIAN_DEFAULT_ANGULAR_VELOCITY,
      ARM_CARTESIAN_DEFAULT_LINEAR_ACCELERATION, ARM_CARTESIAN_DEFAULT_ANGULAR_ACCELERATION};
  static const uint16_t maxima[4] = {
      ARM_CARTESIAN_MAX_LINEAR_VELOCITY_WIRE, ARM_CARTESIAN_MAX_ANGULAR_VELOCITY_WIRE,
      ARM_CARTESIAN_MAX_LINEAR_ACCELERATION_WIRE, ARM_CARTESIAN_MAX_ANGULAR_ACCELERATION_WIRE};
  float value[4];
  uint8_t valid = 1U;
  for (unsigned i = 0; i < 4U; ++i) {
    value[i] = wire[i] ? (float)wire[i] * 0.001f : defaults[i];
    if (wire[i] > maxima[i]) { value[i] = defaults[i]; valid = 0U; }
  }
  ArmMotion_CartesianLimitsFromValues(value[0], value[1], value[2], value[3], limits);
  return valid;
}

void ArmMotion_CartesianLimitsFromValues(float linear_velocity, float angular_velocity,
                                         float linear_acceleration, float angular_acceleration,
                                         ArmCartesianLimits *limits)
{
  limits->linear_velocity_max = linear_velocity;
  limits->angular_velocity_max = angular_velocity;
  limits->linear_acceleration_max = linear_acceleration;
  limits->angular_acceleration_max = angular_acceleration;
  joint_caps_from_cartesian(limits);
}

void ArmMotion_CapDriveSpeeds(ArmCartesianLimits *limits, const float drive_speed[ARM_AXES],
                              uint32_t posvel_mask)
{
  float cap[ARM_AXES];
  for (unsigned i = 0; i < ARM_AXES; ++i)
    /* A drive sent v_des = 0 does not move: keep the joint (nearly) still. */
    cap[i] = (posvel_mask & (1UL << i)) ?
             fmaxf(ARM_POSVEL_SPEED_FRACTION * drive_speed[i], 1.0e-3f) : 0.0f;
  ArmMotion_CapJointLimits(limits, cap, NULL);
}

void ArmDriveLead_Reset(ArmDriveLead *lead, float q)
{
  for (unsigned i = 0; i < ARM_DRIVE_LEAD_TICKS; ++i) lead->past[i] = q;
  lead->next = 0U;
  lead->count = 0U;
}

float ArmDriveLead_Velocity(ArmDriveLead *lead, float q, float speed_max, float dt)
{
  const float last = lead->past[(lead->next + ARM_DRIVE_LEAD_TICKS - 1U) % ARM_DRIVE_LEAD_TICKS];
  if (!(fabsf(q - last) <= 2.0f * speed_max * dt)) ArmDriveLead_Reset(lead, q);
  /* Over the window, so the RT joint slew's 2 ms bursts per 10 ms target
   * read as the target's steady speed. */
  float velocity = 0.0f;
  if (lead->count > 0U) {
    const float oldest = lead->past[(lead->next + ARM_DRIVE_LEAD_TICKS - lead->count) %
                                    ARM_DRIVE_LEAD_TICKS];
    velocity = clampf((q - oldest) / ((float)lead->count * dt), -speed_max, speed_max);
  }
  lead->past[lead->next] = q;
  lead->next = (uint8_t)((lead->next + 1U) % ARM_DRIVE_LEAD_TICKS);
  if (lead->count < ARM_DRIVE_LEAD_TICKS) lead->count++;
  return velocity;
}

float ArmDriveLead_Target(float q, float velocity, float tau, float measured,
                          float lower, float upper)
{
  if (!(tau > 0.0f) || !isfinite(velocity)) return q;
  float target = q + tau * velocity;
  if (measured <= lower) target = fmaxf(target, fminf(q, lower));
  if (measured >= upper) target = fminf(target, fmaxf(q, upper));
  return target;
}

void ArmMotion_RampCartesianLimits(ArmCartesianLimits *applied,
                                   const ArmCartesianLimits *requested, float dt)
{
  const float linear = fmaxf(requested->linear_velocity_max,
                             applied->linear_velocity_max - requested->linear_acceleration_max * dt);
  const float angular = fmaxf(requested->angular_velocity_max,
                              applied->angular_velocity_max - requested->angular_acceleration_max * dt);
  *applied = *requested;
  applied->linear_velocity_max = linear;
  applied->angular_velocity_max = angular;
  joint_caps_from_cartesian(applied);
}

void ArmMotion_CapJointLimits(ArmCartesianLimits *limits, const float velocity[ARM_AXES],
                              const float acceleration[ARM_AXES])
{
  float slowest = 1.0e30f;
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    if (velocity && velocity[i] > 0.0f)
      limits->joint_velocity_max[i] = fminf(limits->joint_velocity_max[i], velocity[i]);
    if (acceleration && acceleration[i] > 0.0f) {
      limits->joint_acceleration_max[i] = fminf(limits->joint_acceleration_max[i], acceleration[i]);
      slowest = fminf(slowest, acceleration[i]);
    }
  }
  /* Speed caps need nothing more: the common step scaling slows the whole
   * line.  An acceleration cap must also slow the tool's acceleration, by the
   * same ratios: a joint held back by it lags on its own, the IK residual
   * grows and the line blocks. */
  limits->linear_acceleration_max = fminf(limits->linear_acceleration_max,
                                          ARM_CARTESIAN_LEVER_M * slowest);
  limits->angular_acceleration_max = fminf(limits->angular_acceleration_max,
                                           ARM_CARTESIAN_ANGULAR_RATIO * slowest);
}

/* Stops the joints: every joint brakes by the same factor, as hard as the
 * joint accelerations allow, so the tool keeps its direction instead of
 * curving off as each joint brakes on its own; a joint about to reach its
 * URDF limit brakes harder.  The reference follows the arm (pose and
 * velocity), so it can neither run away while blocked nor jump when
 * following resumes. */
static void brake(ArmCartesianTracker *t, const ArmCartesianLimits *limits, float dt)
{
  const float *lower = ArmIk_JointLower();
  const float *upper = ArmIk_JointUpper();
  float factor = 0.0f, ceiling = 1.0f;
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    const float speed = fabsf(t->v[i]);
    const float a = limits->joint_acceleration_max[i];
    if (speed <= 1.0e-9f) continue;
    factor = fmaxf(factor, a > 0.0f ? 1.0f - a * dt / speed : 0.0f);
    if (a > 0.0f) {
      const float room = fmaxf(t->v[i] > 0.0f ? upper[i] - t->q[i] : t->q[i] - lower[i], 0.0f);
      const float dv = a * dt;
      ceiling = fminf(ceiling, (sqrtf(0.25f * dv * dv + 2.0f * a * room) - 0.5f * dv) / speed);
    }
  }
  factor = fminf(fmaxf(factor, 0.0f), fmaxf(ceiling, 0.0f));
  ArmIkPose before;
  ArmIk_Forward(t->q, &before);
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    t->v[i] *= factor;
    t->q[i] = clampf(t->q[i] + t->v[i] * dt, lower[i], upper[i]);
    t->goal[i] = t->q[i];
  }
  ArmIk_Forward(t->q, &t->reference);
  ArmIkPose turn; /* rotation over this tick, base frame */
  for (unsigned i = 0; i < 3U; ++i) {
    t->reference_velocity[i] = (t->reference.position[i] - before.position[i]) / dt;
    for (unsigned j = 0; j < 3U; ++j)
      turn.rotation[i][j] = t->reference.rotation[i][0] * before.rotation[j][0] +
                            t->reference.rotation[i][1] * before.rotation[j][1] +
                            t->reference.rotation[i][2] * before.rotation[j][2];
  }
  ArmIk_RotationLog(&turn, &t->reference_velocity[3]);
  for (unsigned i = 3U; i < 6U; ++i) t->reference_velocity[i] /= dt;
  t->scale = 0.0f;
}

static uint8_t same_target(const ArmIkPose *a, const ArmIkPose *b)
{
  float d = 0.0f;
  ArmIkPose turn;
  for (unsigned i = 0; i < 3U; ++i) {
    d += (a->position[i] - b->position[i]) * (a->position[i] - b->position[i]);
    for (unsigned j = 0; j < 3U; ++j)
      turn.rotation[i][j] = a->rotation[i][0] * b->rotation[j][0] + a->rotation[i][1] * b->rotation[j][1] +
                            a->rotation[i][2] * b->rotation[j][2];
  }
  float w[3];
  ArmIk_RotationLog(&turn, w);
  return d <= ARM_CARTESIAN_TARGET_CHANGE_M * ARM_CARTESIAN_TARGET_CHANGE_M &&
         w[0] * w[0] + w[1] * w[1] + w[2] * w[2] <=
             ARM_CARTESIAN_TARGET_CHANGE_RAD * ARM_CARTESIAN_TARGET_CHANGE_RAD;
}

void ArmCartesianTracker_Start(ArmCartesianTracker *t, const float q[ARM_AXES],
                               ArmIkTimeSource time_source, void *time_source_context)
{
  memset(t, 0, sizeof(*t));
  memcpy(t->goal, q, sizeof(t->goal));
  memcpy(t->q, q, sizeof(t->q));
  ArmIk_Forward(q, &t->reference);
  ArmMotion_TrackingIkOptions(&t->ik_options);
  t->ik_options.time_source = time_source;
  t->ik_options.time_source_context = time_source_context;
  t->scale = 1.0f;
  t->status = ARM_IK_CONVERGED;
}

uint8_t ArmCartesianTracker_Step(ArmCartesianTracker *t, const ArmIkPose *target,
                                 const ArmCartesianLimits *limits, float dt)
{
  if (t->blocked) {
    if (same_target(target, &t->blocked_target)) { brake(t, limits, dt); return t->status; }
    t->blocked = 0U;
  }
  /* 1. Propose the next reference.  Position (scaled by length) and the
   * rotation vector form one 6-D vector in which both speed limits are
   * equal, so they start, brake and arrive together, on a straight line and
   * a fixed rotation axis.  Accelerations are per direction (above). */
  /* Governor: slow down as last tick's residual approaches its limits. */
  const float residual = fmaxf(t->position_error / ARM_CARTESIAN_FOLLOW_POSITION_M,
                               t->orientation_error / ARM_CARTESIAN_FOLLOW_ORIENTATION_RAD);
  const float governor =
      clampf((1.0f - residual) / (1.0f - ARM_CARTESIAN_SLOWDOWN_FRACTION), 0.0f, 1.0f);
  const float length = limits->linear_velocity_max / limits->angular_velocity_max;
  const float crawl = fminf(ARM_CARTESIAN_CRAWL_LINEAR_VELOCITY / length, limits->angular_velocity_max);
  const float speed_max = fmaxf(limits->angular_velocity_max * governor, crawl);
  float error[6], velocity[6];
  ArmIkPose difference; /* rotation from the reference to the target */
  for (unsigned i = 0; i < 3U; ++i) {
    error[i] = (target->position[i] - t->reference.position[i]) / length;
    velocity[i] = t->reference_velocity[i] / length;
    velocity[i + 3U] = t->reference_velocity[i + 3U];
    for (unsigned j = 0; j < 3U; ++j)
      difference.rotation[i][j] = target->rotation[i][0] * t->reference.rotation[j][0] +
                                  target->rotation[i][1] * t->reference.rotation[j][1] +
                                  target->rotation[i][2] * t->reference.rotation[j][2];
  }
  ArmIk_RotationLog(&difference, &error[3]);
  float distance = 0.0f;
  for (unsigned i = 0; i < 6U; ++i) distance += error[i] * error[i];
  distance = sqrtf(distance);
  float change[6], change_norm = 0.0f;
  const float braking = direction_acceleration(error, length, limits);
  const float dv = braking * dt;
  /* Discrete braking speed: stops exactly on the target with steps of dv. */
  const float speed = distance > 1.0e-7f ?
      fminf(fminf(speed_max, sqrtf(0.25f * dv * dv + 2.0f * braking * distance) - 0.5f * dv),
            distance / dt) : 0.0f;
  for (unsigned i = 0; i < 6U; ++i) {
    change[i] = (distance > 1.0e-7f ? error[i] / distance * speed : 0.0f) - velocity[i];
    change_norm += change[i] * change[i];
  }
  change_norm = sqrtf(change_norm);
  const float step = direction_acceleration(change, length, limits) * dt;
  const float limit = change_norm > step ? step / change_norm : 1.0f;
  float step_position[3], step_rotation[3];
  for (unsigned i = 0; i < 6U; ++i) velocity[i] += change[i] * limit;
  for (unsigned i = 0; i < 3U; ++i) {
    step_position[i] = velocity[i] * length * dt;
    step_rotation[i] = velocity[i + 3U] * dt;
  }
  ArmIkPose proposal = t->reference;
  for (unsigned i = 0; i < 3U; ++i) proposal.position[i] += step_position[i];
  rotate_base(proposal.rotation, step_rotation);

  /* 2. IK of the proposal from the joint command.  A joint lagging behind
   * (acceleration limit near a singularity) is then corrected toward the
   * line in Cartesian space instead of catching up on its own, and the
   * speed scaling below holds the reference back until it has.  While the
   * reference moves, always step: the convergence tolerance would skip
   * ticks whose motion is below it and move the joints in bursts.  At rest,
   * step only above the tolerance, so the elbow does not drift by itself. */
  ArmIkOptions options = t->ik_options;
  float moving = 0.0f;
  for (unsigned i = 0; i < 6U; ++i) moving += velocity[i] * velocity[i];
  if (moving > 0.0f) options.tolerance = 1.0e-12f;
  const ArmIkResult ik = ArmIk_Solve(&proposal, t->q, &options, NULL);
  t->status = ik.status;
  t->elapsed_us = ik.elapsed_us;
  t->position_error = ik.position_error;
  t->orientation_error = ik.orientation_error;
  if (ik.status == ARM_IK_INVALID_INPUT || ik.status == ARM_IK_QP_NUMERICAL_FAILURE)
    return ik.status;
  const uint8_t usable = (ik.status == ARM_IK_CONVERGED || ik.status == ARM_IK_MAX_ITERATIONS ||
                          ik.status == ARM_IK_TIME_BUDGET_EXCEEDED) &&
                         ik.position_error <= ARM_CARTESIAN_FOLLOW_POSITION_M &&
                         ik.orientation_error <= ARM_CARTESIAN_FOLLOW_ORIENTATION_RAD;

  /* 3. Each joint's speed cap: its limit, and near its URDF limit a braking
   * curve on which it can still stop there at its acceleration limit.  One
   * common scale for every joint keeps the tool on the line while none
   * exceeds its cap, so the whole line slows down (and stops short of the
   * joint limit) instead of one joint being cut off.  (Bounding the scale by
   * joint acceleration too stalls the arm: one joint reversing its tiny
   * speed collapses it.)  Acceleration is shaped by Cartesian limits derived
   * from the joint ones with margin; the follower below enforces it. */
  const float *lower = ArmIk_JointLower();
  const float *upper = ArmIk_JointUpper();
  float scale = 1.0f, delta[ARM_AXES] = {0.0f};
  for (unsigned i = 0; usable && i < ARM_AXES; ++i) {
    delta[i] = ik.q[i] - t->q[i];
    const float d = fabsf(delta[i]);
    if (d <= 1.0e-9f) continue;
    float cap = limits->joint_velocity_max[i];
    const float a = limits->joint_acceleration_max[i];
    if (a > 0.0f) {
      const float room = fmaxf(delta[i] > 0.0f ? upper[i] - t->q[i] : t->q[i] - lower[i], 0.0f);
      const float dv = a * dt;
      cap = fminf(cap, sqrtf(0.25f * dv * dv + 2.0f * a * room) - 0.5f * dv);
    }
    scale = fminf(scale, cap * dt / d);
  }
  if (!usable) {
    /* Target beyond reach (or the line leaves it), singularity or a failed
     * QP: stop, and hold until the target moves. */
    t->blocked = 1U;
    t->blocked_target = *target;
    brake(t, limits, dt);
    return ik.status;
  }
  t->scale = scale;
  for (unsigned i = 0; i < 3U; ++i) {
    t->reference.position[i] += scale * step_position[i];
    step_rotation[i] *= scale;
    t->reference_velocity[i] = scale * velocity[i] * length;
    t->reference_velocity[i + 3U] = scale * velocity[i + 3U];
  }
  rotate_base(t->reference.rotation, step_rotation);
  float goal_velocity[ARM_AXES];
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    t->goal[i] = t->q[i] + scale * delta[i];
    goal_velocity[i] = scale * delta[i] / dt;
  }
  /* 4. Joint acceleration limit: exact while the scaled step is feasible.
   * The braking curves keep a joint off its URDF limit; the clamp is only a
   * last guard (the next solve is seeded here and would be rejected). */
  ArmMotion_FollowJoints(t->q, t->v, t->goal, goal_velocity, limits->joint_velocity_max,
                         limits->joint_acceleration_max, dt);
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    if (t->q[i] < lower[i] || t->q[i] > upper[i]) {
      t->q[i] = clampf(t->q[i], lower[i], upper[i]);
      t->v[i] = 0.0f;
    }
  }
  return ik.status;
}

static uint8_t health(const ArmMoveFeedback *fb)
{
  if (!fb->ready || (fb->enabled_mask & ARM_MASK) != ARM_MASK) return ARM_NOT_READY;
  if ((fb->fresh_mask & ARM_MASK) != ARM_MASK) return ARM_FEEDBACK_STALE;
  if ((fb->healthy_mask & ARM_MASK) != ARM_MASK) return ARM_MOTOR_FAULT;
  for (unsigned i = 0; i < ARM_AXES; ++i)
    if (!isfinite(fb->position[i]) || !isfinite(fb->velocity[i])) return ARM_MOTOR_FAULT;
  return ARM_OK;
}

static float start_reference(const ArmMoveFeedback *fb,unsigned i)
{
  return fb->command_valid && isfinite(fb->command[i]) &&
         fabsf(fb->command[i]-fb->position[i])<=ARM_HOLD_KEEP_COMMAND_RAD ? fb->command[i] : fb->position[i];
}

static uint8_t validate_joint(const ArmMoveCommand *c, const ArmMoveFeedback *fb, uint32_t *duration)
{
  if (!c || c->timeout_ms > 600000U || c->dwell_ms < 20U || c->dwell_ms > 5000U ||
      !isfinite(c->position_tolerance) || c->position_tolerance < 0.001f || c->position_tolerance > 0.05f ||
      !isfinite(c->velocity_tolerance) || c->velocity_tolerance < 0.01f || c->velocity_tolerance > 0.1f)
    return ARM_INVALID;
  float seconds = 0.01f;
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    if (!isfinite(c->target[i]) || !isfinite(c->velocity_max[i]) || !isfinite(c->acceleration_max[i]) ||
        !isfinite(c->kp[i]) || !isfinite(c->kd[i]) || !isfinite(c->torque[i]) ||
        !isfinite(fb->lower[i]) || !isfinite(fb->upper[i]) || !isfinite(fb->velocity_max[i]) ||
        !isfinite(fb->torque_max[i]) || fb->lower[i] >= fb->upper[i] ||
        c->target[i] < fb->lower[i] || c->target[i] > fb->upper[i] ||
        c->velocity_max[i] <= 0 || c->velocity_max[i] > fb->velocity_max[i] ||
        c->acceleration_max[i] <= 0 || c->acceleration_max[i] > 10.0f ||
        c->kp[i] <= 0 || c->kp[i] > 500 || c->kd[i] < 0 || c->kd[i] > 5 ||
        fabsf(c->torque[i]) > fb->torque_max[i]) return ARM_INVALID;
    /* Start each segment at rest. No blending or replacement of an in-flight target. */
    if (fabsf(fb->velocity[i]) > c->velocity_tolerance ||
        fb->position[i] < fb->lower[i] || fb->position[i] > fb->upper[i]) return ARM_NOT_READY;
    /* Retiming must use the same latched hold reference as execution;
     * using sagged feedback can underestimate distance and exceed the cap. */
    float distance = fabsf(c->target[i] - start_reference(fb,i));
    /* Quintic smoothstep maxima: ds/du = 1.875; |d2s/du2| = 10/sqrt(3). */
    seconds = fmaxf(seconds, 1.875f * distance / c->velocity_max[i]);
    seconds = fmaxf(seconds, sqrtf(5.774f * distance / c->acceleration_max[i]));
  }
  if (!isfinite(seconds) || seconds > 599.0f) return ARM_INVALID;
  *duration = (uint32_t)ceilf(seconds * 1000.0f);
  return c->timeout_ms > *duration + c->dwell_ms ? ARM_OK : ARM_INVALID;
}

static uint8_t validate_cartesian(const ArmMoveCartesianCommand *c,
                                  const ArmMoveFeedback *fb, uint32_t *duration)
{
  if (!c || c->timeout_ms == 0U || c->timeout_ms > 600000U ||
      c->dwell_ms < 20U || c->dwell_ms > 5000U ||
      !isfinite(c->position_tolerance) || c->position_tolerance < 0.0005f ||
      c->position_tolerance > 0.05f || !isfinite(c->velocity_tolerance) ||
      c->velocity_tolerance < 0.01f || c->velocity_tolerance > 0.5f)
    return ARM_INVALID;
  ArmIkPose pose;
  pose.position[0] = c->pose[0];
  pose.position[1] = c->pose[1];
  pose.position[2] = c->pose[2];
  for (unsigned row = 0U; row < 3U; ++row)
    for (unsigned col = 0U; col < 3U; ++col)
      pose.rotation[row][col] = c->pose[3U + row * 3U + col];
  /* The solver performs the definitive orthonormality check.  This early
   * check rejects NaNs and malformed wire payloads before taking ownership. */
  for (unsigned i = 0U; i < 3U; ++i)
  {
    if (!isfinite(pose.position[i])) return ARM_INVALID;
    for (unsigned j = 0U; j < 3U; ++j)
      if (!isfinite(pose.rotation[i][j])) return ARM_INVALID;
  }
  if (!fb || !duration || c->timeout_ms <= c->dwell_ms) return ARM_INVALID;
  for (unsigned i = 0U; i < ARM_AXES; ++i)
  {
    if (!isfinite(c->velocity_max[i]) || !isfinite(c->acceleration_max[i]) ||
        !isfinite(c->kp[i]) || !isfinite(c->kd[i]) ||
        !isfinite(fb->position[i]) || !isfinite(fb->velocity[i]) ||
        c->velocity_max[i] <= 0.0f || c->velocity_max[i] > fb->velocity_max[i] ||
        c->acceleration_max[i] <= 0.0f || c->acceleration_max[i] > 100.0f ||
        c->kp[i] <= 0.0f || c->kp[i] > 500.0f ||
        c->kd[i] < 0.0f || c->kd[i] > 5.0f)
      return ARM_INVALID;
  }
  ArmCartesianLimits limits;
  if (!ArmMotion_CartesianLimitsFromWire(c->cartesian_limits, &limits)) return ARM_INVALID;
  *duration = c->timeout_ms - c->dwell_ms;
  return ARM_OK;
}

static float orientation_error(const ArmIkPose *a, const ArmIkPose *b)
{
  float trace = 0.0f;
  for (unsigned row = 0U; row < 3U; ++row)
    for (unsigned col = 0U; col < 3U; ++col) trace += a->rotation[row][col] * b->rotation[row][col];
  return acosf(fminf(1.0f, fmaxf(-1.0f, 0.5f * (trace - 1.0f))));
}

uint8_t ArmMotion_Request(ArmMotion *m, uint8_t op, uint64_t session, uint32_t id,
                         const void *command, const ArmMoveFeedback *fb, uint32_t now)
{
  if (op == ARM_QUERY) return ARM_OK; /* Read-only; never feeds the watchdog. */
  if (!session) return ARM_INVALID;
  if (m->owned && session != m->session) return ARM_BAD_SESSION;
  if (op == ARM_CANCEL_MOVE) {
    if (session == m->closed_session && !m->owned) return ARM_OK; /* Retry lost cancel ACK. */
    if (!m->owned) return ARM_BAD_SESSION;
    ArmMotion_Cancel(m);
    return ARM_OK;
  }
  if (session == m->closed_session) return ARM_BAD_SESSION;
  if (op == ARM_KEEPALIVE) {
    if (!m->owned || id != m->id) return ARM_STALE_ID;
    if (m->state == ARM_FAILED) return m->reason;
    m->last_heartbeat_ms = now;
    return ARM_OK;
  }
  if ((op != ARM_MOVEJ && op != ARM_MOVEP) || !id || !command) return ARM_INVALID;
  const uint8_t cartesian = op == ARM_MOVEP;
  if (m->owned) {
    if (m->state == ARM_FAILED) return m->reason;
    if (id == m->id) {
      const void *active_command = m->cartesian ? (const void *)&m->cartesian_command :
                                    (const void *)&m->command;
      if (m->cartesian != cartesian ||
          memcmp(command, active_command, sizeof(ArmMoveCommand))) return ARM_INVALID;
      m->last_heartbeat_ms = now; /* An identical 100 Hz retry does NOT restart the profile. */
      return ARM_OK;
    }
    if (id < m->id) return ARM_STALE_ID;
    if (m->state != ARM_DONE) return ARM_BUSY;
  }
  uint8_t result = health(fb);
  if (result != ARM_OK) return result;
  uint32_t duration;
  if (cartesian)
    result = validate_cartesian((const ArmMoveCartesianCommand *)command, fb, &duration);
  else
    result = validate_joint((const ArmMoveCommand *)command, fb, &duration);
  if (result != ARM_OK) return result;
  m->cartesian = cartesian;
  if (cartesian)
  {
    m->cartesian_command = *(const ArmMoveCartesianCommand *)command;
    /* Tool speed from cartesian_limits.  The command's per-joint caps and
     * the motors' rated speeds only cap joints: the common step scaling then
     * slows the whole line where one would be exceeded. */
    (void)ArmMotion_CartesianLimitsFromWire(m->cartesian_command.cartesian_limits,
                                            &m->cartesian_limits);
    ArmMotion_CapJointLimits(&m->cartesian_limits, m->cartesian_command.velocity_max,
                             m->cartesian_command.acceleration_max);
    ArmMotion_CapJointLimits(&m->cartesian_limits, fb->rated_speed, NULL);
    /* The move sends velocity_max as v_des to position-velocity drives. */
    ArmMotion_CapDriveSpeeds(&m->cartesian_limits, m->cartesian_command.velocity_max,
                             fb->posvel_mask);
  }
  else
    m->command = *(const ArmMoveCommand *)command;
  for (unsigned i = 0; i < ARM_AXES; ++i)
    m->start[i] = start_reference(fb,i);
  if (cartesian)
  {
    const float *lower = ArmIk_JointLower();
    const float *upper = ArmIk_JointUpper();
    for (unsigned i = 0; i < ARM_AXES; ++i)
    {
      if (m->start[i] < lower[i] && m->start[i] >= lower[i] - ARM_IK_SEED_MARGIN_RAD)
        m->start[i] = lower[i];
      else if (m->start[i] > upper[i] && m->start[i] <= upper[i] + ARM_IK_SEED_MARGIN_RAD)
        m->start[i] = upper[i];
    }
  }
  memcpy(m->position, m->start, sizeof(m->position));
  memset(m->velocity, 0, sizeof(m->velocity));
  if (cartesian)
    ArmCartesianTracker_Start(&m->tracker, m->start, fb->time_source, fb->time_source_context);
  m->session = session;
  m->id = id;
  m->started_ms = m->last_heartbeat_ms = now;
  m->duration_ms = duration;
  m->state = ARM_RUNNING;
  m->reason = ARM_OK;
  m->owned = 1;
  m->dwelling = 0;
  m->feedback_stale = 0;
  m->feedback_stale_since_ms = 0;
  m->arrived_mask = 0;
  m->max_error = 0;
  return ARM_OK;
}

void ArmMotion_Tick(ArmMotion *m, const ArmMoveFeedback *fb, uint32_t now)
{
  if (!m->owned || m->state == ARM_FAILED) return;
  if (now - m->last_heartbeat_ms > ARM_MOVE_HEARTBEAT_MS) {
    ArmMotion_Fail(m, ARM_COMM_TIMEOUT); return;
  }
  uint8_t result = health(fb);
  if (result == ARM_FEEDBACK_STALE) {
    /* Hold the last published setpoint during a short CAN feedback gap.  A
     * continuous 100 ms gap is still a hard
     * fault, so a disconnected motor cannot remain commanded indefinitely. */
    if (!m->feedback_stale) {
      m->feedback_stale = 1U;
      m->feedback_stale_since_ms = now;
    } else if (now - m->feedback_stale_since_ms >= ARM_MOVE_FEEDBACK_FAIL_MS) {
      ArmMotion_Fail(m, ARM_FEEDBACK_STALE);
    }
    return;
  }
  if (m->feedback_stale) {
    /* Pause the profile for the gap.  Without this compensation, the next
     * fresh sample would advance the quintic by the whole gap and create a
     * discontinuous setpoint jump. */
    const uint32_t paused_ms = now - m->feedback_stale_since_ms;
    m->started_ms += paused_ms;
    if (m->dwelling) m->dwell_started_ms += paused_ms;
    m->feedback_stale = 0U;
  }
  if (result != ARM_OK) { ArmMotion_Fail(m, result); return; }
  uint32_t elapsed = now - m->started_ms;
  const uint32_t timeout_ms = m->cartesian ? m->cartesian_command.timeout_ms :
                              m->command.timeout_ms;
  if (m->state != ARM_DONE && elapsed >= timeout_ms) {
    ArmMotion_Fail(m, ARM_MOTION_TIMEOUT); return;
  }
  if (m->cartesian)
  {
    if (m->state == ARM_DONE) return;
    ArmIkPose target;
    target.position[0] = m->cartesian_command.pose[0];
    target.position[1] = m->cartesian_command.pose[1];
    target.position[2] = m->cartesian_command.pose[2];
    for (unsigned row = 0U; row < 3U; ++row)
      for (unsigned col = 0U; col < 3U; ++col)
        target.rotation[row][col] = m->cartesian_command.pose[3U + row * 3U + col];
    m->tracker.ik_options.time_source = fb->time_source;
    m->tracker.ik_options.time_source_context = fb->time_source_context;
    for (unsigned i = 0U; i < ARM_AXES; ++i)
      if (!(fb->posvel_mask & (1UL << i)) &&
          fabsf(fb->position[i] - m->position[i]) > ARM_CARTESIAN_TRACKING_LIMIT_RAD)
        { ArmMotion_Fail(m, ARM_MOTOR_FAULT); return; }
    /* Straight line to the target; the tracker never seeds from feedback
     * (that chases the gravity sag) and holds on a failed QP (singularity,
     * joint limit); the timeout bounds a target that is never reached. */
    const uint8_t status = ArmCartesianTracker_Step(&m->tracker, &target, &m->cartesian_limits,
                                                    ARM_MOVE_PERIOD_S);
    if (status == ARM_IK_INVALID_INPUT || status == ARM_IK_QP_NUMERICAL_FAILURE)
    {
      ArmMotion_Fail(m, ARM_INVALID);
      return;
    }
    memcpy(m->position, m->tracker.q, sizeof(m->position));
    memcpy(m->velocity, m->tracker.v, sizeof(m->velocity));
    /* Arrival is judged on the measured pose, not the command. */
    ArmIkPose actual;
    ArmIk_Forward(fb->position, &actual);
    const float dx = target.position[0] - actual.position[0];
    const float dy = target.position[1] - actual.position[1];
    const float dz = target.position[2] - actual.position[2];
    m->arrived_mask = 0U;
    m->max_error = fmaxf(sqrtf(dx * dx + dy * dy + dz * dz),
                         orientation_error(&target, &actual));
    for (unsigned i = 0U; i < ARM_AXES; ++i)
      if (m->max_error <= m->cartesian_command.position_tolerance &&
          fabsf(fb->velocity[i]) <= m->cartesian_command.velocity_tolerance)
        m->arrived_mask |= 1UL << i;
    if (m->arrived_mask != ARM_MASK) { m->dwelling = 0U; m->state = ARM_RUNNING; return; }
    if (m->state != ARM_SETTLING) { m->state = ARM_SETTLING; m->dwell_started_ms = now; m->dwelling = 1U; }
    if (now - m->dwell_started_ms >= m->cartesian_command.dwell_ms) m->state = ARM_DONE;
    return;
  }

  float u = fminf((float)elapsed / m->duration_ms, 1.0f);
  float s = u*u*u*(10.0f + u*(-15.0f + 6.0f*u));
  float ds = 30.0f*u*u*(1.0f-u)*(1.0f-u) * 1000.0f / m->duration_ms;
  m->arrived_mask = 0;
  m->max_error = 0;
  for (unsigned i = 0; i < ARM_AXES; ++i) {
    m->position[i] = m->start[i] + (m->command.target[i] - m->start[i]) * s;
    m->velocity[i] = (m->command.target[i] - m->start[i]) * ds;
    float error = fabsf(m->command.target[i] - fb->position[i]);
    m->max_error = fmaxf(m->max_error, error);
    if (error <= m->command.position_tolerance && fabsf(fb->velocity[i]) <= m->command.velocity_tolerance)
      m->arrived_mask |= 1U << i;
  }
  if (m->state == ARM_DONE) return; /* Latch completion for lost-ACK retries. */
  if (elapsed < m->duration_ms) return;
  m->state = ARM_SETTLING;
  if (m->arrived_mask != ARM_MASK) { m->dwelling = 0; return; }
  if (!m->dwelling) { m->dwell_started_ms = now; m->dwelling = 1; }
  if (now - m->dwell_started_ms >= m->command.dwell_ms) m->state = ARM_DONE;
}
