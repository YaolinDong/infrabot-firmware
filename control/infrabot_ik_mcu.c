/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "infrabot_ik_mcu.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define ARM_IK_PI 3.14159265358979323846f
#define ARM_IK_SQRT7 2.6457513110645905905f
#define ARM_IK_QP_MAX_ITERATIONS 64U
#define ARM_IK_EPSILON 1.0e-10f

typedef struct { float x, y, z; } Vec3;
typedef struct { float m[3][3]; } Mat3;
typedef struct { Vec3 position; Mat3 rotation; } Pose;
typedef struct { float linear[3][ARM_IK_AXES]; float angular[3][ARM_IK_AXES]; } Jacobian;

typedef struct {
  Vec3 xyz;
  Vec3 rpy;
  Vec3 axis;
  float lower;
  float upper;
  float velocity;
} JointSpec;

#include "arm_kinematics_model.inc"

static const int8_t k_motor_direction[ARM_IK_AXES] = {1, -1, 1, -1, 1, 1, 1};

static float clampf_local(float value, float lower, float upper)
{
  return value < lower ? lower : (value > upper ? upper : value);
}

static float dot3(Vec3 a, Vec3 b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

static Vec3 add3(Vec3 a, Vec3 b)
{
  Vec3 out = {a.x + b.x, a.y + b.y, a.z + b.z};
  return out;
}

static Vec3 sub3(Vec3 a, Vec3 b)
{
  Vec3 out = {a.x - b.x, a.y - b.y, a.z - b.z};
  return out;
}

static Vec3 scale3(Vec3 value, float scale)
{
  Vec3 out = {value.x * scale, value.y * scale, value.z * scale};
  return out;
}

static Vec3 cross3(Vec3 a, Vec3 b)
{
  Vec3 out = {a.y * b.z - a.z * b.y,
              a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.x};
  return out;
}

static float norm3(Vec3 value)
{
  return sqrtf(dot3(value, value));
}

static Vec3 normalize3(Vec3 value)
{
  const float length = norm3(value);
  if (!isfinite(length) || length <= 1.0e-7f) {
    Vec3 zero = {0.0f, 0.0f, 0.0f};
    return zero;
  }
  return scale3(value, 1.0f / length);
}

static Mat3 identity3(void)
{
  Mat3 out = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
  return out;
}

static Mat3 transpose3(Mat3 value)
{
  Mat3 out;
  for (uint8_t row = 0U; row < 3U; ++row)
    for (uint8_t col = 0U; col < 3U; ++col)
      out.m[row][col] = value.m[col][row];
  return out;
}

static Mat3 multiply3(Mat3 a, Mat3 b)
{
  Mat3 out = {{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}};
  for (uint8_t row = 0U; row < 3U; ++row)
    for (uint8_t col = 0U; col < 3U; ++col)
      for (uint8_t k = 0U; k < 3U; ++k)
        out.m[row][col] += a.m[row][k] * b.m[k][col];
  return out;
}

static Vec3 multiply_vec3(Mat3 matrix, Vec3 value)
{
  Vec3 out = {
    matrix.m[0][0] * value.x + matrix.m[0][1] * value.y + matrix.m[0][2] * value.z,
    matrix.m[1][0] * value.x + matrix.m[1][1] * value.y + matrix.m[1][2] * value.z,
    matrix.m[2][0] * value.x + matrix.m[2][1] * value.y + matrix.m[2][2] * value.z
  };
  return out;
}

static Mat3 rot_x(float angle)
{
  const float c = cosf(angle), s = sinf(angle);
  Mat3 out = identity3();
  out.m[1][1] = c; out.m[1][2] = -s; out.m[2][1] = s; out.m[2][2] = c;
  return out;
}

static Mat3 rot_y(float angle)
{
  const float c = cosf(angle), s = sinf(angle);
  Mat3 out = identity3();
  out.m[0][0] = c; out.m[0][2] = s; out.m[2][0] = -s; out.m[2][2] = c;
  return out;
}

static Mat3 rot_z(float angle)
{
  const float c = cosf(angle), s = sinf(angle);
  Mat3 out = identity3();
  out.m[0][0] = c; out.m[0][1] = -s; out.m[1][0] = s; out.m[1][1] = c;
  return out;
}

static Mat3 rpy(Vec3 value)
{
  return multiply3(multiply3(rot_z(value.z), rot_y(value.y)), rot_x(value.x));
}

static Mat3 axis_angle(Vec3 axis, float angle)
{
  axis = normalize3(axis);
  const float c = cosf(angle), s = sinf(angle), t = 1.0f - c;
  Mat3 out = {{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}};
  out.m[0][0] = t * axis.x * axis.x + c;
  out.m[0][1] = t * axis.x * axis.y - s * axis.z;
  out.m[0][2] = t * axis.x * axis.z + s * axis.y;
  out.m[1][0] = t * axis.x * axis.y + s * axis.z;
  out.m[1][1] = t * axis.y * axis.y + c;
  out.m[1][2] = t * axis.y * axis.z - s * axis.x;
  out.m[2][0] = t * axis.x * axis.z - s * axis.y;
  out.m[2][1] = t * axis.y * axis.z + s * axis.x;
  out.m[2][2] = t * axis.z * axis.z + c;
  return out;
}

static Pose compose_pose(Pose a, Pose b)
{
  Pose out;
  out.position = add3(a.position, multiply_vec3(a.rotation, b.position));
  out.rotation = multiply3(a.rotation, b.rotation);
  return out;
}

static Pose origin_pose(JointSpec spec)
{
  Pose out;
  out.position = spec.xyz;
  out.rotation = rpy(spec.rpy);
  return out;
}

static void forward_internal(const float q[ARM_IK_AXES], Pose *out)
{
  Pose current = {{0.0f, 0.0f, 0.0f}, identity3()};
  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) {
    const Pose frame = compose_pose(current, origin_pose(k_joint_specs[i]));
    const Pose rotation = {{0.0f, 0.0f, 0.0f}, axis_angle(k_joint_specs[i].axis, q[i])};
    current = compose_pose(frame, rotation);
  }
  *out = compose_pose(current, k_flange);
}

static void jacobian_internal(const float q[ARM_IK_AXES], Jacobian *out,Pose *pose)
{
  Pose current = {{0.0f, 0.0f, 0.0f}, identity3()};
  Vec3 joint_position[ARM_IK_AXES];
  Vec3 world_axis[ARM_IK_AXES];
  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) {
    const Pose frame = compose_pose(current, origin_pose(k_joint_specs[i]));
    joint_position[i] = frame.position;
    world_axis[i] = normalize3(multiply_vec3(frame.rotation, k_joint_specs[i].axis));
    const Pose rotation = {{0.0f, 0.0f, 0.0f}, axis_angle(k_joint_specs[i].axis, q[i])};
    current = compose_pose(frame, rotation);
  }
  if(pose)*pose=compose_pose(current,k_flange);
  const Vec3 tcp = add3(current.position, multiply_vec3(current.rotation, k_flange.position));
  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) {
    const Vec3 linear = cross3(world_axis[i], sub3(tcp, joint_position[i]));
    out->linear[0][i] = linear.x;
    out->linear[1][i] = linear.y;
    out->linear[2][i] = linear.z;
    out->angular[0][i] = world_axis[i].x;
    out->angular[1][i] = world_axis[i].y;
    out->angular[2][i] = world_axis[i].z;
  }
}

static Vec3 rotation_log(Mat3 rotation)
{
  const float cosine = clampf_local((rotation.m[0][0] + rotation.m[1][1] + rotation.m[2][2] - 1.0f) * 0.5f, -1.0f, 1.0f);
  const float theta = acosf(cosine);
  const Vec3 skew = {rotation.m[2][1] - rotation.m[1][2],
                     rotation.m[0][2] - rotation.m[2][0],
                     rotation.m[1][0] - rotation.m[0][1]};
  if (theta < 1.0e-5f) return scale3(skew, 0.5f);
  const float sine = sinf(theta);
  if (fabsf(sine) > 1.0e-4f) return scale3(skew, theta / (2.0f * sine));

  Vec3 axis = {0.0f, 0.0f, 0.0f};
  const float xx = fmaxf(0.0f, (rotation.m[0][0] + 1.0f) * 0.5f);
  const float yy = fmaxf(0.0f, (rotation.m[1][1] + 1.0f) * 0.5f);
  const float zz = fmaxf(0.0f, (rotation.m[2][2] + 1.0f) * 0.5f);
  if (xx >= yy && xx >= zz && xx > 1.0e-6f) {
    axis.x = sqrtf(xx);
    axis.y = (rotation.m[0][1] + rotation.m[1][0]) / (4.0f * axis.x);
    axis.z = (rotation.m[0][2] + rotation.m[2][0]) / (4.0f * axis.x);
  } else if (yy >= zz && yy > 1.0e-6f) {
    axis.y = sqrtf(yy);
    axis.x = (rotation.m[0][1] + rotation.m[1][0]) / (4.0f * axis.y);
    axis.z = (rotation.m[1][2] + rotation.m[2][1]) / (4.0f * axis.y);
  } else if (zz > 1.0e-6f) {
    axis.z = sqrtf(zz);
    axis.x = (rotation.m[0][2] + rotation.m[2][0]) / (4.0f * axis.z);
    axis.y = (rotation.m[1][2] + rotation.m[2][1]) / (4.0f * axis.z);
  } else {
    axis.x = 1.0f;
  }
  return scale3(normalize3(axis), theta);
}

static uint64_t now_us(const ArmIkOptions *options)
{
  return options->time_source != NULL ? options->time_source(options->time_source_context) : 0U;
}

static uint8_t deadline_expired(const ArmIkOptions *options, uint64_t start)
{
  if (options->time_budget_us == 0U || options->time_source == NULL) return 0U;
  return (uint64_t)(now_us(options) - start) >= (uint64_t)options->time_budget_us;
}

static void set_result_error(ArmIkResult *result, float position_error,
                             float orientation_error, float weighted_error)
{
  result->position_error = position_error;
  result->orientation_error = orientation_error;
  result->error = weighted_error;
}

static uint8_t valid_pose(const ArmIkPose *pose)
{
  if (pose == NULL) return 0U;
  for (uint8_t i = 0U; i < 3U; ++i) {
    if (!isfinite(pose->position[i])) return 0U;
    for (uint8_t j = 0U; j < 3U; ++j)
      if (!isfinite(pose->rotation[i][j])) return 0U;
  }
  for (uint8_t i = 0U; i < 3U; ++i) {
    for (uint8_t j = 0U; j < 3U; ++j) {
      float value = 0.0f;
      for (uint8_t k = 0U; k < 3U; ++k)
        value += pose->rotation[k][i] * pose->rotation[k][j];
      if (!isfinite(value) || fabsf(value - ((i == j) ? 1.0f : 0.0f)) > 2.0e-4f) return 0U;
    }
  }
  const Vec3 x = {pose->rotation[0][0], pose->rotation[1][0], pose->rotation[2][0]};
  const Vec3 y = {pose->rotation[0][1], pose->rotation[1][1], pose->rotation[2][1]};
  const Vec3 z = {pose->rotation[0][2], pose->rotation[1][2], pose->rotation[2][2]};
  return fabsf(dot3(x, cross3(y, z)) - 1.0f) <= 2.0e-4f;
}

static uint8_t valid_options(const ArmIkOptions *options, const float q[ARM_IK_AXES],
                             const ArmIkMotion *motion)
{
  if (options == NULL || q == NULL || options->max_iterations == 0U ||
      options->tolerance <= 0.0f || options->position_weight < 0.0f ||
      options->orientation_weight < 0.0f ||
      (options->position_weight == 0.0f && options->orientation_weight == 0.0f) ||
      options->damping < 0.0f || options->step_size <= 0.0f || options->step_size > 1.0f ||
      options->max_step_norm <= 0.0f || options->max_joint_step <= 0.0f ||
      options->posture_weight < 0.0f ||
      !isfinite(options->posture_step_max) || options->posture_step_max < 0.0f) return 0U;
  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) {
    if (!isfinite(q[i]) || q[i] < k_joint_specs[i].lower || q[i] > k_joint_specs[i].upper) return 0U;
  }
  if (motion == NULL || motion->enabled == 0U) return 1U;
  if (!isfinite(motion->period_s) || motion->period_s <= 0.0f) return 0U;
  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) {
    if (!isfinite(motion->velocity_max_rad_s[i]) || !isfinite(motion->acceleration_max_rad_s2[i]) ||
        !isfinite(motion->previous_velocity_rad_s[i]) || motion->velocity_max_rad_s[i] < 0.0f ||
        motion->acceleration_max_rad_s2[i] < 0.0f) return 0U;
  }
  return 1U;
}

static uint8_t nullspace_posture(const Jacobian *jacobian, const ArmIkOptions *options,
                                 const float posture_error[ARM_IK_AXES],
                                 float null_vector[ARM_IK_AXES],
                                 float projected_error[ARM_IK_AXES])
{
  float matrix[6][ARM_IK_AXES] = {{0.0f}};
  const float position_scale = sqrtf(options->position_weight);
  const float orientation_scale = sqrtf(options->orientation_weight);
  for (uint8_t row = 0U; row < 3U; ++row) {
    for (uint8_t col = 0U; col < ARM_IK_AXES; ++col) {
      matrix[row][col] = position_scale * jacobian->linear[row][col];
      matrix[row + 3U][col] = orientation_scale * jacobian->angular[row][col];
    }
  }
  uint8_t pivot_columns[6] = {0U};
  uint8_t rank = 0U;
  for (uint8_t col = 0U; col < ARM_IK_AXES && rank < 6U; ++col) {
    uint8_t pivot = rank;
    float largest = fabsf(matrix[pivot][col]);
    for (uint8_t row = rank + 1U; row < 6U; ++row) {
      const float value = fabsf(matrix[row][col]);
      if (value > largest) { largest = value; pivot = row; }
    }
    if (!isfinite(largest) || largest <= ARM_IK_EPSILON) continue;
    if (pivot != rank) {
      for (uint8_t c = 0U; c < ARM_IK_AXES; ++c) {
        const float temporary = matrix[pivot][c]; matrix[pivot][c] = matrix[rank][c]; matrix[rank][c] = temporary;
      }
    }
    const float divisor = matrix[rank][col];
    for (uint8_t c = col; c < ARM_IK_AXES; ++c) matrix[rank][c] /= divisor;
    for (uint8_t row = 0U; row < 6U; ++row) {
      if (row == rank) continue;
      const float factor = matrix[row][col];
      for (uint8_t c = col; c < ARM_IK_AXES; ++c) matrix[row][c] -= factor * matrix[rank][c];
    }
    pivot_columns[rank++] = col;
  }
  if (rank != 6U) return 0U;
  int8_t free_column = -1;
  for (uint8_t col = 0U; col < ARM_IK_AXES; ++col) {
    uint8_t pivot = 0U;
    for (uint8_t row = 0U; row < rank; ++row) if (pivot_columns[row] == col) pivot = 1U;
    if (pivot == 0U) { free_column = (int8_t)col; break; }
  }
  if (free_column < 0) return 0U;
  memset(null_vector, 0, sizeof(float) * ARM_IK_AXES);
  null_vector[(uint8_t)free_column] = 1.0f;
  for (int8_t row = 5; row >= 0; --row) {
    const uint8_t pivot = pivot_columns[(uint8_t)row];
    float value = 0.0f;
    for (uint8_t col = pivot + 1U; col < ARM_IK_AXES; ++col) value += matrix[(uint8_t)row][col] * null_vector[col];
    null_vector[pivot] = -value;
  }
  float squared_norm = 0.0f;
  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) squared_norm += null_vector[i] * null_vector[i];
  if (!isfinite(squared_norm) || squared_norm <= ARM_IK_EPSILON) return 0U;
  const float scale = 1.0f / sqrtf(squared_norm);
  float scalar = 0.0f;
  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) {
    null_vector[i] *= scale;
    scalar += null_vector[i] * posture_error[i];
  }
  if (!isfinite(scalar)) return 0U;
  if (options->posture_step_max > 0.0f) {
    /* The null vector is an eigenvector of the QP Hessian with eigenvalue
     * posture_weight + damping, so the unconstrained null-space step is
     * step_size * posture_weight / (posture_weight + damping) * scalar. */
    const float gain = options->step_size * options->posture_weight /
                       (options->posture_weight + fmaxf(1.0e-8f, options->damping));
    const float limit = options->posture_step_max / gain;
    scalar = clampf_local(scalar, -limit, limit);
  }
  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) projected_error[i] = null_vector[i] * scalar;
  return 1U;
}

static uint8_t cholesky(float h[ARM_IK_AXES][ARM_IK_AXES],
                        const uint8_t indices[ARM_IK_AXES], uint8_t count,
                        float lower[ARM_IK_AXES][ARM_IK_AXES])
{
  memset(lower, 0, sizeof(float) * ARM_IK_AXES * ARM_IK_AXES);
  for (uint8_t i = 0U; i < count; ++i) {
    for (uint8_t j = 0U; j <= i; ++j) {
      float value = h[indices[i]][indices[j]];
      for (uint8_t k = 0U; k < j; ++k) value -= lower[i][k] * lower[j][k];
      if (!isfinite(value)) return 0U;
      if (i == j) {
        if (value <= 1.0e-12f) return 0U;
        lower[i][j] = sqrtf(value);
      } else {
        lower[i][j] = value / lower[j][j];
      }
    }
  }
  return 1U;
}

enum { QP_OPTIMAL = 0, QP_INVALID = 1, QP_INFEASIBLE = 2, QP_ITERATION_LIMIT = 3, QP_NUMERICAL = 4, QP_TIMEOUT = 5 };

static uint8_t box_qp(float h[ARM_IK_AXES][ARM_IK_AXES], const float gradient[ARM_IK_AXES],
                      const float lower[ARM_IK_AXES], const float upper[ARM_IK_AXES],
                      float solution[ARM_IK_AXES], const ArmIkOptions *options, uint64_t budget_start)
{
  uint8_t indices[ARM_IK_AXES];
  uint8_t active[ARM_IK_AXES] = {0U};
  float factor[ARM_IK_AXES][ARM_IK_AXES];
  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) {
    indices[i] = i;
    if (lower[i] > upper[i] || !isfinite(gradient[i]) || !isfinite(lower[i]) || !isfinite(upper[i])) return QP_INVALID;
    for (uint8_t j = 0U; j < ARM_IK_AXES; ++j)
      if (!isfinite(h[i][j]) || fabsf(h[i][j] - h[j][i]) > 2.0e-5f * (1.0f + fabsf(h[i][j]))) return QP_INVALID;
    solution[i] = clampf_local(0.0f, lower[i], upper[i]);
    if (fabsf(lower[i] - upper[i]) <= 1.0e-7f) active[i] = 2U;
    else if (solution[i] <= lower[i] + 1.0e-7f) active[i] = 255U;
    else if (solution[i] >= upper[i] - 1.0e-7f) active[i] = 1U;
  }
  if (!cholesky(h, indices, ARM_IK_AXES, factor)) return QP_NUMERICAL;
  for (uint8_t iteration = 0U; iteration < ARM_IK_QP_MAX_ITERATIONS; ++iteration) {
    if (deadline_expired(options, budget_start)) return QP_TIMEOUT;
    float residual[ARM_IK_AXES];
    float tolerance = 1.0e-7f;
    uint8_t free_count = 0U;
    for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) {
      residual[i] = gradient[i];
      float scale = 1.0f + fabsf(gradient[i]);
      for (uint8_t j = 0U; j < ARM_IK_AXES; ++j) { residual[i] += h[i][j] * solution[j]; scale += fabsf(h[i][j] * solution[j]); }
      if (!isfinite(residual[i]) || !isfinite(scale)) return QP_NUMERICAL;
      if (1.0e-7f * scale > tolerance) tolerance = 1.0e-7f * scale;
      if (active[i] == 0U) indices[free_count++] = i;
    }
    if (deadline_expired(options, budget_start)) return QP_TIMEOUT;
    float free_residual = 0.0f;
    for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) if (active[i] == 0U && fabsf(residual[i]) > free_residual) free_residual = fabsf(residual[i]);
    if (free_residual <= tolerance) {
      int8_t release = -1;
      float worst = tolerance;
      for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) {
        const float violation = active[i] == 255U ? -residual[i] : (active[i] == 1U ? residual[i] : 0.0f);
        if (violation > worst) { worst = violation; release = (int8_t)i; }
      }
      if (release < 0) return QP_OPTIMAL;
      active[(uint8_t)release] = 0U;
      continue;
    }
    if (!cholesky(h, indices, free_count, factor)) return QP_NUMERICAL;
    float y[ARM_IK_AXES] = {0.0f}, p[ARM_IK_AXES] = {0.0f};
    for (uint8_t i = 0U; i < free_count; ++i) {
      float value = -residual[indices[i]];
      for (uint8_t j = 0U; j < i; ++j) value -= factor[i][j] * y[j];
      y[i] = value / factor[i][i];
    }
    for (int8_t ii = (int8_t)free_count - 1; ii >= 0; --ii) {
      const uint8_t i = (uint8_t)ii;
      float value = y[i];
      for (uint8_t j = i + 1U; j < free_count; ++j) value -= factor[j][i] * p[j];
      p[indices[i]] = value / factor[i][i];
    }
    float alpha = 1.0f;
    int8_t blocking = -1;
    uint8_t blocking_bound = 0U;
    for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) {
      if (!isfinite(p[i]) || active[i] != 0U || p[i] == 0.0f) continue;
      const float limit = p[i] > 0.0f ? (upper[i] - solution[i]) / p[i] : (lower[i] - solution[i]) / p[i];
      if (limit <= alpha) { alpha = fmaxf(0.0f, limit); blocking = (int8_t)i; blocking_bound = p[i] > 0.0f ? 1U : 255U; }
    }
    for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) solution[i] = clampf_local(solution[i] + alpha * p[i], lower[i], upper[i]);
    if (blocking >= 0) { const uint8_t i = (uint8_t)blocking; solution[i] = blocking_bound == 1U ? upper[i] : lower[i]; active[i] = blocking_bound; }
  }
  return QP_ITERATION_LIMIT;
}

static ArmIkResult iterate(const ArmIkPose *target, const float start[ARM_IK_AXES],
                           const ArmIkOptions *options, const ArmIkMotion *motion,
                           uint16_t iterations)
{
  ArmIkResult result;
  memset(&result, 0, sizeof(result));
  memcpy(result.q, start, sizeof(result.q));
  const uint64_t start_us = now_us(options);
  if (!valid_pose(target) || !valid_options(options, start, motion)) {
    result.status = ARM_IK_INVALID_INPUT;
    result.position_error = result.orientation_error = result.error = FLT_MAX;
    return result;
  }
  float cycle_lower[ARM_IK_AXES], cycle_upper[ARM_IK_AXES];
  const float trust = fminf(options->max_joint_step, options->max_step_norm / ARM_IK_SQRT7);
  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) {
    cycle_lower[i] = k_joint_specs[i].lower;
    cycle_upper[i] = k_joint_specs[i].upper;
    if (motion != NULL && motion->enabled != 0U) {
      /* Match the host solver: zero means the generated URDF velocity limit. */
      const float velocity = motion->velocity_max_rad_s[i] > 0.0f ?
                             motion->velocity_max_rad_s[i] : k_joint_specs[i].velocity;
      const float velocity_delta = velocity * motion->period_s;
      float lower_delta = -velocity_delta, upper_delta = velocity_delta;
      if (motion->previous_velocity_valid != 0U && motion->acceleration_max_rad_s2[i] > 0.0f) {
        const float acceleration_delta = motion->acceleration_max_rad_s2[i] * motion->period_s;
        lower_delta = fmaxf(lower_delta, (motion->previous_velocity_rad_s[i] - acceleration_delta) * motion->period_s);
        upper_delta = fminf(upper_delta, (motion->previous_velocity_rad_s[i] + acceleration_delta) * motion->period_s);
      }
      cycle_lower[i] = fmaxf(cycle_lower[i], start[i] + lower_delta);
      cycle_upper[i] = fminf(cycle_upper[i], start[i] + upper_delta);
    }
  }
  for (uint16_t iteration = 0U; iteration < iterations; ++iteration) {
    if (deadline_expired(options, start_us)) { result.status = ARM_IK_TIME_BUDGET_EXCEEDED; break; }
    Pose current;
    forward_internal(result.q, &current);
    const Vec3 position_error = {target->position[0] - current.position.x,
                                 target->position[1] - current.position.y,
                                 target->position[2] - current.position.z};
    const Mat3 target_rotation = {{{target->rotation[0][0], target->rotation[0][1], target->rotation[0][2]},
                                   {target->rotation[1][0], target->rotation[1][1], target->rotation[1][2]},
                                   {target->rotation[2][0], target->rotation[2][1], target->rotation[2][2]}}};
    const Vec3 orientation_error = rotation_log(multiply3(target_rotation, transpose3(current.rotation)));
    const float position_norm = norm3(position_error), orientation_norm = norm3(orientation_error);
    const float weighted_error = sqrtf(options->position_weight * position_norm * position_norm +
                                       options->orientation_weight * orientation_norm * orientation_norm);
    set_result_error(&result, position_norm, orientation_norm, weighted_error);
    result.iterations = (uint16_t)(iteration + 1U);
    if (!isfinite(weighted_error)) { result.status = ARM_IK_QP_NUMERICAL_FAILURE; break; }
    if (weighted_error <= options->tolerance) { result.success = 1U; result.status = ARM_IK_CONVERGED; break; }

    Jacobian jacobian;
    jacobian_internal(result.q, &jacobian, NULL);
    float h[ARM_IK_AXES][ARM_IK_AXES] = {{0.0f}}, gradient[ARM_IK_AXES] = {0.0f};
    float lower[ARM_IK_AXES], upper[ARM_IK_AXES];
    float posture_error[ARM_IK_AXES], null_vector[ARM_IK_AXES], projected_posture[ARM_IK_AXES];
    for (uint8_t i = 0U; i < ARM_IK_AXES; ++i)
      posture_error[i] = 0.5f * (k_joint_specs[i].lower + k_joint_specs[i].upper) - result.q[i];
    const uint8_t use_posture = options->posture_weight > 0.0f &&
        nullspace_posture(&jacobian, options, posture_error, null_vector, projected_posture);
    for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) {
      float rhs = options->position_weight * (jacobian.linear[0][i] * position_error.x + jacobian.linear[1][i] * position_error.y + jacobian.linear[2][i] * position_error.z) +
                  options->orientation_weight * (jacobian.angular[0][i] * orientation_error.x + jacobian.angular[1][i] * orientation_error.y + jacobian.angular[2][i] * orientation_error.z);
      for (uint8_t k = 0U; k < ARM_IK_AXES; ++k) {
        for (uint8_t row = 0U; row < 3U; ++row)
          h[i][k] += options->position_weight * jacobian.linear[row][i] * jacobian.linear[row][k] + options->orientation_weight * jacobian.angular[row][i] * jacobian.angular[row][k];
        if (use_posture != 0U) h[i][k] += options->posture_weight * null_vector[i] * null_vector[k];
      }
      h[i][i] += fmaxf(1.0e-8f, options->damping);
      if (use_posture != 0U) rhs += options->posture_weight * projected_posture[i];
      gradient[i] = -options->step_size * rhs;
      lower[i] = fmaxf(fmaxf(k_joint_specs[i].lower - result.q[i], cycle_lower[i] - result.q[i]), -trust);
      upper[i] = fminf(fminf(k_joint_specs[i].upper - result.q[i], cycle_upper[i] - result.q[i]), trust);
    }
    float step[ARM_IK_AXES];
    const uint8_t qp_status = box_qp(h, gradient, lower, upper, step, options, start_us);
    if (qp_status != QP_OPTIMAL) {
      result.status = qp_status == QP_TIMEOUT ? ARM_IK_TIME_BUDGET_EXCEEDED :
                      qp_status == QP_INFEASIBLE ? ARM_IK_QP_INFEASIBLE :
                      qp_status == QP_ITERATION_LIMIT ? ARM_IK_QP_ITERATION_LIMIT : ARM_IK_QP_NUMERICAL_FAILURE;
      break;
    }
    float squared_step = 0.0f;
    for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) {
      if (!isfinite(step[i]) || step[i] < lower[i] - 1.0e-5f || step[i] > upper[i] + 1.0e-5f) { result.status = ARM_IK_QP_NUMERICAL_FAILURE; break; }
      result.q[i] = clampf_local(result.q[i] + step[i], k_joint_specs[i].lower, k_joint_specs[i].upper);
      squared_step += step[i] * step[i];
    }
    if (result.status == ARM_IK_QP_NUMERICAL_FAILURE || sqrtf(squared_step) > options->max_step_norm + 1.0e-4f) { result.status = ARM_IK_QP_NUMERICAL_FAILURE; break; }
    if (deadline_expired(options, start_us)) { result.status = ARM_IK_TIME_BUDGET_EXCEEDED; break; }
  }
  if (options->time_source != NULL) {
    const uint64_t elapsed = now_us(options) - start_us;
    result.elapsed_us = elapsed > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed;
  }
  /* As the host solver: report the residual of the returned q.  Without this
   * an iteration-capped solve reported the error from before its last step. */
  const uint8_t capped = result.status == 0U && result.success == 0U;
  if (capped || result.status == ARM_IK_TIME_BUDGET_EXCEEDED) {
    Pose current;
    forward_internal(result.q, &current);
    const Vec3 pe = {target->position[0] - current.position.x, target->position[1] - current.position.y, target->position[2] - current.position.z};
    const Mat3 tr = {{{target->rotation[0][0], target->rotation[0][1], target->rotation[0][2]}, {target->rotation[1][0], target->rotation[1][1], target->rotation[1][2]}, {target->rotation[2][0], target->rotation[2][1], target->rotation[2][2]}}};
    const Vec3 oe = rotation_log(multiply3(tr, transpose3(current.rotation)));
    set_result_error(&result, norm3(pe), norm3(oe), sqrtf(options->position_weight * dot3(pe, pe) + options->orientation_weight * dot3(oe, oe)));
    if (capped) {
      result.success = isfinite(result.error) && result.error <= options->tolerance;
      result.status = result.success ? ARM_IK_CONVERGED : ARM_IK_MAX_ITERATIONS;
    }
  }
  return result;
}

void ArmIk_DefaultOptions(ArmIkOptions *options)
{
  if (options == NULL) return;
  memset(options, 0, sizeof(*options));
  options->max_iterations = 64U;
  options->tolerance = 1.0e-4f;
  options->position_weight = 1.0f;
  options->orientation_weight = 0.35f;
  options->damping = 1.0e-3f;
  options->step_size = 0.8f;
  options->max_step_norm = 0.12f;
  options->max_joint_step = 0.08f;
  options->posture_weight = 0.02f;
  options->time_budget_us = 500U;
}

void ArmIk_DefaultMotion(ArmIkMotion *motion)
{
  if (motion == NULL) return;
  memset(motion, 0, sizeof(*motion));
  motion->enabled = 1U;
  motion->period_s = 0.001f;
}

void ArmIk_Forward(const float q[ARM_IK_AXES], ArmIkPose *pose)
{
  if (q == NULL || pose == NULL) return;
  Pose result;
  forward_internal(q, &result);
  const float position[3] = {result.position.x, result.position.y, result.position.z};
  for (uint8_t i = 0U; i < 3U; ++i) {
    pose->position[i] = position[i];
    for (uint8_t j = 0U; j < 3U; ++j) pose->rotation[i][j] = result.rotation.m[i][j];
  }
}

void ArmIk_JointPoints(const float q[ARM_IK_AXES], float points[ARM_IK_AXES + 1][3],
                       float tool_rotation[3][3])
{
  if (q == NULL || points == NULL || tool_rotation == NULL) return;
  Pose current = {{0.0f, 0.0f, 0.0f}, identity3()};
  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) {
    const Pose frame = compose_pose(current, origin_pose(k_joint_specs[i]));
    points[i][0] = frame.position.x;
    points[i][1] = frame.position.y;
    points[i][2] = frame.position.z;
    const Pose rotation = {{0.0f, 0.0f, 0.0f}, axis_angle(k_joint_specs[i].axis, q[i])};
    current = compose_pose(frame, rotation);
  }
  current = compose_pose(current, k_flange);
  points[ARM_IK_AXES][0] = current.position.x;
  points[ARM_IK_AXES][1] = current.position.y;
  points[ARM_IK_AXES][2] = current.position.z;
  for (uint8_t i = 0U; i < 3U; ++i)
    for (uint8_t j = 0U; j < 3U; ++j) tool_rotation[i][j] = current.rotation.m[i][j];
}

static void store_frame(const Pose *pose, float rotation[3][3], float position[3])
{
  position[0] = pose->position.x;
  position[1] = pose->position.y;
  position[2] = pose->position.z;
  for (uint8_t i = 0U; i < 3U; ++i)
    for (uint8_t j = 0U; j < 3U; ++j) rotation[i][j] = pose->rotation.m[i][j];
}

void ArmIk_LinkFrames(const float q[ARM_IK_AXES], float rotation[ARM_IK_FRAMES][3][3],
                      float position[ARM_IK_FRAMES][3])
{
  if (q == NULL || rotation == NULL || position == NULL) return;
  Pose current = {{0.0f, 0.0f, 0.0f}, identity3()};
  store_frame(&current, rotation[0], position[0]);
  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) {
    const Pose frame = compose_pose(current, origin_pose(k_joint_specs[i]));
    const Pose turn = {{0.0f, 0.0f, 0.0f}, axis_angle(k_joint_specs[i].axis, q[i])};
    current = compose_pose(frame, turn);
    store_frame(&current, rotation[i + 1U], position[i + 1U]);
  }
  current = compose_pose(current, k_flange);
  store_frame(&current, rotation[ARM_IK_FRAMES-1U], position[ARM_IK_FRAMES-1U]);
}

void ArmIk_ForwardJacobian(const float q[ARM_IK_AXES],ArmIkPose *pose,float linear[3][ARM_IK_AXES],float angular[3][ARM_IK_AXES]){
  Pose p;Jacobian j;jacobian_internal(q,&j,&p);
  pose->position[0]=p.position.x;pose->position[1]=p.position.y;pose->position[2]=p.position.z;
  memcpy(pose->rotation,p.rotation.m,sizeof(pose->rotation));memcpy(linear,j.linear,sizeof(j.linear));memcpy(angular,j.angular,sizeof(j.angular));
}
void ArmIk_Jacobian(const float q[ARM_IK_AXES], float linear[3][ARM_IK_AXES], float angular[3][ARM_IK_AXES])
{
  if (q == NULL || linear == NULL || angular == NULL) return;
  Jacobian result;
  jacobian_internal(q, &result, NULL);
  memcpy(linear, result.linear, sizeof(result.linear));
  memcpy(angular, result.angular, sizeof(result.angular));
}

ArmIkResult ArmIk_Solve(const ArmIkPose *target, const float seed[ARM_IK_AXES], const ArmIkOptions *options, const ArmIkMotion *motion)
{
  ArmIkOptions defaults;
  if (options == NULL) { ArmIk_DefaultOptions(&defaults); options = &defaults; }
  return iterate(target, seed, options, motion, options->max_iterations);
}

ArmIkResult ArmIk_Step(const ArmIkPose *target, const float current[ARM_IK_AXES], const ArmIkOptions *options, const ArmIkMotion *motion)
{
  ArmIkOptions defaults;
  if (options == NULL) { ArmIk_DefaultOptions(&defaults); options = &defaults; }
  return iterate(target, current, options, motion, 1U);
}

void ArmIk_RotationLog(const ArmIkPose *pose, float vector[3])
{
  Mat3 r;
  memcpy(r.m, pose->rotation, sizeof(r.m));
  const Vec3 w = rotation_log(r);
  vector[0] = w.x; vector[1] = w.y; vector[2] = w.z;
}

void ArmIk_RotationExp(const float vector[3], float rotation[3][3])
{
  const Vec3 w = {vector[0], vector[1], vector[2]};
  const float angle = norm3(w);
  const Mat3 r = angle > 1.0e-9f ? axis_angle(w, angle) : identity3();
  memcpy(rotation, r.m, sizeof(r.m));
}

const float *ArmIk_JointLower(void) { return k_joint_lower; }
const float *ArmIk_JointUpper(void) { return k_joint_upper; }
const int8_t *ArmIk_MotorDirection(void) { return k_motor_direction; }
