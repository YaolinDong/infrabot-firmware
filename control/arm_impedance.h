/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef ARM_IMPEDANCE_H
#define ARM_IMPEDANCE_H
#include "infrabot_ik_mcu.h"
/* Base-frame task stiffness/damping, SI units. No virtual inertia or force
 * feedback loop. Gravity is supplied separately by the caller. */
typedef struct {
  float joint_k, joint_d;
  float translation_k, translation_d, rotation_k, rotation_d, joint_damping;
} ArmImpedanceGains;
typedef struct {
  uint8_t valid, error_clamped; /* bits XYZ then rotation: spring error capped */
  float target[3], measured[3], position_error[3], rotation_error[3];
  float force[3], moment[3]; /* virtual spring/damper wrench, not contact force */
} ArmImpedanceDiagnostics;
int ArmImpedance_Evaluate(int cartesian,const float q[7],const float dq[7],
  const float equilibrium[7],const ArmIkPose *pose,const ArmImpedanceGains *g,float tau[7],
  ArmImpedanceDiagnostics *diagnostics);
/* Outputs corrective joint torque (Nm). Equilibrium is fixed by caller.
 * Cartesian orientation error is Log(R_target R_measured^T), in base frame.
 * Returns zero on invalid input; caller must enter controlled stop. */
int ArmImpedance_Compute(int cartesian,const float q[7],const float dq[7],
  const float equilibrium[7],const ArmIkPose *pose,const ArmImpedanceGains *g,float tau[7]);
#endif
