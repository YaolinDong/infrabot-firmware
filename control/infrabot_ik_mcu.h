/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef INFRABOT_IK_MCU_H
#define INFRABOT_IK_MCU_H

/*
 * Allocation-free C11 port of infrabot_ik's host solver.
 *
 * The module deliberately contains only the fixed-size model, FK/Jacobian,
 * bounded active-set QP and the nullspace neutral-posture objective.  It has
 * no HAL, FreeRTOS, Ethernet or collision-detection dependency and can be
 * called from the MCU's 1 kHz control task.
 */
#include <stdint.h>

#define ARM_IK_AXES 7U

typedef uint64_t (*ArmIkTimeSource)(void *context);

typedef struct {
  float position[3];
  float rotation[3][3]; /* row-major 3x3 rotation matrix */
} ArmIkPose;

typedef struct {
  uint16_t max_iterations;
  float tolerance;
  float position_weight;
  float orientation_weight;
  float damping;
  float step_size;
  float max_step_norm;
  float max_joint_step;
  float posture_weight;
  /* Largest null-space (elbow) step one iteration may take toward the neutral
   * posture, rad; zero = unlimited (the host solver's behaviour). */
  float posture_step_max;
  uint32_t time_budget_us; /* zero disables the deadline */
  ArmIkTimeSource time_source;
  void *time_source_context;
} ArmIkOptions;

typedef struct {
  uint8_t enabled;
  float period_s;
  float velocity_max_rad_s[ARM_IK_AXES];
  float acceleration_max_rad_s2[ARM_IK_AXES];
  float previous_velocity_rad_s[ARM_IK_AXES];
  uint8_t previous_velocity_valid;
} ArmIkMotion;

enum ArmIkStatus {
  ARM_IK_CONVERGED = 0,
  ARM_IK_MAX_ITERATIONS = 1,
  ARM_IK_INVALID_INPUT = 2,
  ARM_IK_QP_ITERATION_LIMIT = 3,
  ARM_IK_QP_INFEASIBLE = 4,
  ARM_IK_QP_NUMERICAL_FAILURE = 5,
  ARM_IK_TIME_BUDGET_EXCEEDED = 6
};

typedef struct {
  float q[ARM_IK_AXES];
  uint8_t success;
  uint16_t iterations;
  float position_error;
  float orientation_error;
  float error;
  uint32_t elapsed_us;
  uint8_t status;
} ArmIkResult;

void ArmIk_DefaultOptions(ArmIkOptions *options);
void ArmIk_DefaultMotion(ArmIkMotion *motion);
void ArmIk_Forward(const float q[ARM_IK_AXES], ArmIkPose *pose);
/* Origins of the seven joint frames (points[0..6]) and of the tool flange
 * (points[7]) in the base frame, metres, plus the tool rotation (row-major);
 * the same chain as ArmIk_Forward.  For drawing the arm. */
void ArmIk_JointPoints(const float q[ARM_IK_AXES], float points[ARM_IK_AXES + 1][3],
                       float tool_rotation[3][3]);
/* Pose of every link frame in the base frame: index 0 is the base itself,
 * index i (1..7) the frame of Link_i after joint i turns, i.e. where the
 * URDF visual mesh of Link_i is attached.  Rotations row-major, metres. */
#define ARM_IK_FRAMES (ARM_IK_AXES + 2U) /* base, Link_1..7, Link_flange */
void ArmIk_LinkFrames(const float q[ARM_IK_AXES], float rotation[ARM_IK_FRAMES][3][3],
                      float position[ARM_IK_FRAMES][3]);
void ArmIk_ForwardJacobian(const float q[ARM_IK_AXES],ArmIkPose *pose,float linear[3][ARM_IK_AXES],float angular[3][ARM_IK_AXES]);
void ArmIk_Jacobian(const float q[ARM_IK_AXES], float linear[3][ARM_IK_AXES],
                    float angular[3][ARM_IK_AXES]);
ArmIkResult ArmIk_Solve(const ArmIkPose *target, const float seed[ARM_IK_AXES],
                        const ArmIkOptions *options, const ArmIkMotion *motion);
ArmIkResult ArmIk_Step(const ArmIkPose *target, const float current[ARM_IK_AXES],
                       const ArmIkOptions *options, const ArmIkMotion *motion);

/* Rotation vector (axis * angle) of pose->rotation, and the inverse. */
void ArmIk_RotationLog(const ArmIkPose *pose, float vector[3]);
void ArmIk_RotationExp(const float vector[3], float rotation[3][3]);

const float *ArmIk_JointLower(void);
const float *ArmIk_JointUpper(void);
const int8_t *ArmIk_MotorDirection(void);

#endif /* INFRABOT_IK_MCU_H */
