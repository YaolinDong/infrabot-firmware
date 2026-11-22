/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef ARM_ADMITTANCE_H
#define ARM_ADMITTANCE_H
#include "infrabot_ik_mcu.h"
#define ARM_ADM_AXES 7
#define ARM_ADM_LEVER .2f
/* H7C elbow/forearm contact sweep (v10 §0). Reproduce the focused cases
 * with tests/run_host_tests.sh test_admittance_shoulder_push. */
#define ARM_ADM_REFERENCE_ACCEL_RAD_S2 40.f
#define ARM_ADM_JOINT_REFERENCE_ACCEL_RAD_S2 8.f
/* Output friction feedforward tanh width. The observer keeps 0.02 rad/s on
 * measured velocity; the drive feedforward uses the reference velocity, which
 * can reverse around zero during contact. A wider transition reduces the
 * near-zero slope and resulting torque steps before output slew limiting. */
#define ARM_ADM_FEEDFORWARD_WIDTH_RAD_S .20f
typedef struct {
  float joint_k,joint_mass_scale,linear_k,rotation_k,mass,zeta,static_ratio,friction,drive_gain,null_k,null_d,return_speed;
} ArmAdmittanceParameters;
typedef struct {
  ArmIkPose pose;
  float linear[3][7],angular[3][7],j[6][7],lower[6][6];
} ArmAdmittanceFrame;
typedef struct {
  uint32_t sequence;
  uint8_t result;
  uint64_t start_us,end_us;
  float equilibrium[7];
  float elapsed,deviation2,peak_speed,final_position,final_rotation,ns_error;
} ArmAdmittanceReturn;
enum { ARM_ADM_OK,ARM_ADM_NONFINITE,ARM_ADM_FACTOR,ARM_ADM_OVERLOAD,ARM_ADM_TRACKING };
typedef struct {
  uint32_t accel_ticks, velocity_ticks;
  float peak_accel;    /* rad/s² after velocity scaling, before acceleration scaling */
  uint8_t peak_joint;  /* J1..J7; zero for an empty or stationary window */
} ArmAdmittanceLimiterStats;
typedef struct {
  ArmAdmittanceLimiterStats limiter;
  float q[7],velocity[7],equilibrium[7];
  ArmIkPose goal,reference;
  float error[6],rate[6],raw[7],external[7],ns[7],force[3],moment[3],effective_force[3],effective_moment[3];
  float torque_limit[7],overload_time[7],tracking_time[7],projection_time,quiet,release;
  float bias[7], unbiased_filtered[7], still_time;
  uint8_t bias_learning;
  float filtered_raw[7];
  float estimated_velocity[7]; /* 10 Hz observer input, joint rad/s */
  float rest_sum[7];uint32_t rest_count;
  ArmIkPose measured_history[5];uint8_t history_head,history_ticks;
  float measured_motion[6];
  uint8_t cartesian,frozen,fault_joint,active_return,return_armed;
  uint8_t drift_frozen, loaded;
  float drift_time, correction_peak_m, correction_peak_rad;
  ArmAdmittanceReturn ret;
  float return_origin[3],return_delta[3],return_time,return_dev2,return_peak;
} ArmAdmittance;
int ArmAdmittance_Factor(const float q[7],ArmAdmittanceFrame *frame);
int ArmAdmittance_Solve(const ArmAdmittanceFrame *frame,const float rhs[6],float result[6]);
int ArmAdmittance_Map(const ArmAdmittanceFrame *frame,const float torque[7],float force[3],float moment[3],float ns[7]);
int ArmAdmittance_Project(const ArmAdmittanceFrame *frame,const float joint[7],float out[7]);
void ArmAdmittance_SoftDeadzone(float *vector,unsigned count,float threshold);
void ArmAdmittance_Friction(const float velocity[7],const float fc[7],const float fv[7],float beta,float out[7]);
/* Drive feedforward: same model, ARM_ADM_FEEDFORWARD_WIDTH_RAD_S width. */
void ArmAdmittance_FrictionFeedforward(const float velocity[7],const float fc[7],const float fv[7],float beta,float out[7]);
int ArmAdmittance_Estimate(ArmAdmittance *s,const float measured_velocity[7],const float gravity[7],const float drive[7],
  const float fc[7],const float fv[7],const ArmAdmittanceParameters *p,float dt);
void ArmAdmittance_Init(ArmAdmittance *s,int cartesian,const float q[7]);
int ArmAdmittance_Tick(ArmAdmittance *s,const float measured[7],const float measured_velocity[7],const float gravity[7],
  const float drive[7],const ArmIkPose *actual,const float fc[7],const float fv[7],
  const ArmAdmittanceParameters *p,float dt,int run);
/* Host-testable substeps; production Tick calls these using the same factor. */
void ArmAdmittance_Dynamics(ArmAdmittance *s,const ArmAdmittanceParameters *p,float dt);
void ArmAdmittance_DesiredPose(const ArmAdmittance *s,ArmIkPose *pose);
int ArmAdmittance_NullVelocity(const ArmAdmittance *s,const ArmAdmittanceFrame *frame,
  const ArmAdmittanceParameters *p,float out[7]);
void ArmAdmittance_UpdateBias(ArmAdmittance *s,float dt);
int ArmAdmittance_CheckSafety(ArmAdmittance *s,const float measured[7],float dt);
/* Error here points from feasible FK towards the desired pose. */
void ArmAdmittance_Pullback(ArmAdmittance *s,const float position_lag[3],
                            const float rotation_lag[3],float dt);
int ArmAdmittance_LimitReference(ArmAdmittance *s,float desired[7],float dt);
int ArmAdmittance_JointStep(ArmAdmittance *s,const ArmAdmittanceParameters *p,float dt);
int ArmAdmittance_Reference(ArmAdmittance *s,const ArmAdmittanceFrame *frame,
  const ArmAdmittanceParameters *p,float dt);
#endif
