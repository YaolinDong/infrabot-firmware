/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_GRAVITY_H
#define APP_GRAVITY_H
#include <stdint.h>
#include "arm_impedance.h"
/* No friction compensation: it pushes along the motion, and a model that
 * overestimates friction turns into negative damping (runaway). */
enum { APP_GRAVITY_DAMPING, APP_GRAVITY_STIFFNESS, APP_GRAVITY_PARAMETERS };
typedef struct {
  float value[APP_GRAVITY_PARAMETERS];
  float applied[APP_GRAVITY_PARAMETERS];
} AppGravityParameters;
/* Bounded RAM settings; the control task slews changes. No flash writes. */
uint8_t AppGravity_SetParameter(uint8_t index, float value);
void AppGravity_GetParameters(AppGravityParameters *out);
float AppGravity_ParameterMin(uint8_t index);
float AppGravity_ParameterMax(uint8_t index);
/* Local diagnostics; do not add/change wire status or stop-reason numbers.
 * INVALID_TARGET uses checks 1..6 as stop_detail while stop_mode is gravity. */
enum { APP_GRAVITY_CHECK_NONE, APP_GRAVITY_CHECK_NONFINITE, APP_GRAVITY_CHECK_SPEED,
       APP_GRAVITY_CHECK_LOWER, APP_GRAVITY_CHECK_UPPER, APP_GRAVITY_CHECK_MODEL,
       APP_GRAVITY_CHECK_BUDGET, APP_GRAVITY_CHECK_CAN, APP_GRAVITY_CHECK_DRIVE,
       APP_GRAVITY_CHECK_FEEDBACK, APP_GRAVITY_CHECK_TEMPERATURE };
typedef struct {
  uint32_t sequence;
  uint8_t check, joint;
  float q, dq, value, limit, kd;
} AppGravityFault;
const char *AppGravity_CheckName(uint8_t check);
/* Status-task logger; never formats/prints in the 1 kHz control task. */
void AppGravity_PrintFault(void);
typedef struct {
  uint8_t mode, recovering, request_status;
  uint16_t entry_permille; /* 0..1000: actual control-task handover progress. */
  uint8_t speed_limited_mask; /* speed or joint-limit brake active; not a STOP */
  uint32_t last_us, max_us, ticks;
  ArmImpedanceDiagnostics impedance;
  ArmImpedanceGains impedance_gains;
  uint8_t torque_clipped, torque_slew_limited;
  float gravity_torque[7], impedance_torque[7];
  float torque[7];
  float feedback_torque[7]; /* Drive torque, joint signs; NOT external torque. */
  float speed_brake[7]; /* actual extra FF braking after output saturation */
  AppGravityFault fault; /* last automatic protection event, retained after IDLE/restart */
} AppGravityStatus;
void AppGravity_PrintImpedance(const AppGravityStatus *snapshot); /* status task only */
/* Start/Stop/Tick: control task only. LCD publishes a request, never sends CAN. */
uint8_t AppGravity_CanStart(void);
uint8_t AppGravity_CanStartMode(uint8_t mode);
/* Owns the IDLE -> gravity handover: snapshots the existing MIT output and
 * blends the complete hold controller into gravity. Do not zero FF first. */
void AppGravity_Start(uint8_t mode);
void AppGravity_OnStop(uint8_t reason);
void AppGravity_CancelRequest(void);
void AppGravity_Tick(void);
uint8_t AppGravity_Owned(void);
uint8_t AppGravity_Recovering(void);
void AppGravity_Request(uint8_t mode); /* IDLE means STOP */
void AppGravity_ProcessRequest(void);
void AppGravity_Status(AppGravityStatus *out);
#endif
