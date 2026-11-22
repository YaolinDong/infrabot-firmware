/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef ARM_MOTION_H
#define ARM_MOTION_H

/* HAL/RTOS independent, single-owner motion planner.  Joint moves use the
 * stop-to-stop profile; Cartesian moves use the fixed-size MCU IK step. */
#include <stdint.h>
/* RT streaming only: rate <= 0 applies the target directly. */
float ArmMotion_StreamPositionTarget(float current,float target,float rate,float dt);
#include "infrabot/protocol.h"
#include "infrabot_ik_mcu.h"

#define ARM_AXES 7U
#define ARM_MASK 0x7fU
#define ARM_MOVE_HEARTBEAT_MS 100U
/* A single motor feedback frame can be delayed by CAN arbitration/IRQ
 * service while the 1 kHz control loop remains healthy.  Do not abort a
 * trajectory on one short gap; the motion task fails it only after the
 * longer consecutive-stale window below. */
#define ARM_MOVE_FEEDBACK_MS 50U
#define ARM_MOVE_FEEDBACK_FAIL_MS 100U
/* A stationary arm sits below its command by the PD gravity sag.  Holds and
 * motions continue from the command when it is this close to the measured
 * angle, so they do not drop the arm by that sag; further apart (a blocked
 * joint) the measured angle is used instead. */
#define ARM_HOLD_KEEP_COMMAND_RAD 0.1f
/* J2/J4 have a URDF lower limit of exactly 0; encoder noise at home reads a
 * hair below it.  IK seeds this close to a limit are clamped onto it. */
#define ARM_IK_SEED_MARGIN_RAD 0.03f
/* Cartesian motion integrates on its own command; a measured MIT joint
 * further than this from it means the drive is not following (dropped off
 * the bus, blocked) and the motion fails.  Position-velocity joints are not
 * checked: their drives run their own position loop (see ArmDriveLead), and
 * the check must never stop normal motion. */
#define ARM_CARTESIAN_TRACKING_LIMIT_RAD 0.25f
/* ArmMotion_Tick runs once per 1 kHz control tick. */
#define ARM_MOVE_PERIOD_S 0.001f
/* Joint caps derived from the Cartesian limits: about 4 rad of joint motion
 * per metre of tool travel away from singularities, then capped at the
 * application's 3.14 rad/s ceiling. Speed is enforced by common step scaling (the line
 * slows down), acceleration by the follower. */
#define ARM_CARTESIAN_LEVER_M 0.2f       /* m of tool travel per joint rad */
#define ARM_CARTESIAN_ANGULAR_RATIO 0.6f /* tool rad per joint rad */
/* Joint acceleration caps are a backstop, not the shaper: a joint held back
 * by its cap lags on its own and bends the line, and at some poses a joint
 * needs more than the plain ratio.  Margin over the ratio-derived value. */
#define ARM_CARTESIAN_JOINT_ACCELERATION_MARGIN 3.0f
/* Cartesian speed limits, settable per RT packet and per movep: wire
 * uint16 {linear mm/s, angular mrad/s, linear mm/s^2, angular mrad/s^2},
 * 0 = default. Joint caps are the minimum of the ratios above and 3.14 rad/s;
 * the path slows whenever any joint would exceed its cap. */
/* Defaults: tuned on the arm (5 m/s^2 shakes visibly with the MIT PD gains). */
#define ARM_CARTESIAN_DEFAULT_LINEAR_VELOCITY 0.5f        /* m/s */
#define ARM_CARTESIAN_DEFAULT_ANGULAR_VELOCITY 3.142f     /* rad/s, 180 deg/s */
#define ARM_CARTESIAN_DEFAULT_LINEAR_ACCELERATION 2.0f    /* m/s^2 */
#define ARM_CARTESIAN_DEFAULT_ANGULAR_ACCELERATION 6.283f /* rad/s^2, 360 deg/s^2 */
/* Maxima in wire units: 0.5 m/s, 180 deg/s (pi rad/s), 2 m/s^2, 360 deg/s^2,
 * set on the arm.  Joint torque is not what limits acceleration (URDF
 * dynamics with twice the link masses: < 60 % of rated torque at 5 m/s^2);
 * the lightly damped MIT PD is: without torque feed-forward its tracking lag
 * and start/stop ringing grow linearly with acceleration (5 m/s^2 shook
 * visibly).
 * Joint speeds are further capped at the application ceiling and motors' rated speeds
 * (ArmMoveFeedback.rated_speed / the RT caller), so near the arm's limits
 * the line slows down rather than demand more than a motor delivers. */
#define ARM_CARTESIAN_MAX_LINEAR_VELOCITY_WIRE INFRABOT_CARTESIAN_MAX_LINEAR_VELOCITY
#define ARM_CARTESIAN_MAX_ANGULAR_VELOCITY_WIRE INFRABOT_CARTESIAN_MAX_ANGULAR_VELOCITY
#define ARM_CARTESIAN_MAX_LINEAR_ACCELERATION_WIRE INFRABOT_CARTESIAN_MAX_LINEAR_ACCELERATION
#define ARM_CARTESIAN_MAX_ANGULAR_ACCELERATION_WIRE INFRABOT_CARTESIAN_MAX_ANGULAR_ACCELERATION
/* A position-velocity drive (DM POS_VEL) runs its own position loop limited
 * to the v_des it is sent.  Cartesian motion commands such a joint at most
 * this fraction of it, so the drive keeps up; at or above it the joint falls
 * behind and the tool leaves the line. */
#define ARM_POSVEL_SPEED_FRACTION 0.9f
/* Velocity window of ArmDriveLead_Velocity: 10 ms, one period of the PC's
 * 100 Hz targets, which the RT joint slew turns into short bursts of motion. */
#define ARM_DRIVE_LEAD_TICKS 10U
/* IK residual beyond which the reference stops advancing (unreachable
 * target, singularity, joint limit) instead of running away from the arm. */
#define ARM_CARTESIAN_FOLLOW_POSITION_M 0.0005f
#define ARM_CARTESIAN_FOLLOW_ORIENTATION_RAD 0.01f
/* The reference slows down once the residual passes this fraction of those
 * limits, reaching a crawl at them, so it approaches the edge of reach or a
 * singularity slowly instead of being stopped there at speed.  The crawl is
 * absolute: near a singularity the IK closes only a few % of each step, so
 * leaving it again needs the same slow start whatever the speed setting. */
#define ARM_CARTESIAN_SLOWDOWN_FRACTION 0.4f
#define ARM_CARTESIAN_CRAWL_LINEAR_VELOCITY 0.002f /* m/s */
/* After a blocked step the arm holds until the target moves by more than
 * this, so it does not creep along the edge of its reach. */
#define ARM_CARTESIAN_TARGET_CHANGE_M 0.00005f
#define ARM_CARTESIAN_TARGET_CHANGE_RAD 0.0005f

enum ArmMoveState { ARM_IDLE, ARM_RUNNING, ARM_SETTLING, ARM_DONE, ARM_FAILED, ARM_CANCELLED };
enum ArmMoveResult {
  ARM_OK, ARM_BUSY, ARM_NOT_READY, ARM_INVALID, ARM_UNSUPPORTED, ARM_BAD_SESSION,
  ARM_STALE_ID, ARM_COMM_TIMEOUT, ARM_MOTION_TIMEOUT, ARM_FEEDBACK_STALE,
  ARM_MOTOR_FAULT, ARM_CANCEL, ARM_TX_ERROR
};
enum ArmMoveOp { ARM_MOVEJ = 1, ARM_MOVEP, ARM_KEEPALIVE, ARM_CANCEL_MOVE, ARM_QUERY };

/* The DMOV payloads (protocol.h): a joint move to target, and a Cartesian
 * move to pose (xyz metres + row-major rotation) whose tool speed comes from
 * cartesian_limits (see ARM_CARTESIAN_*, 0 = default); its per-joint
 * velocity/acceleration caps are optional.  Torque feed-forward remains zero
 * for Cartesian moves. */
typedef InfrabotMoveJoint ArmMoveCommand;
typedef InfrabotMoveCartesian ArmMoveCartesianCommand;
_Static_assert(ARM_AXES == INFRABOT_ARM_AXES, "arm axes of the protocol");
_Static_assert(ARM_IDLE == INFRABOT_MOVE_STATE_IDLE && ARM_RUNNING == INFRABOT_MOVE_STATE_RUNNING &&
               ARM_SETTLING == INFRABOT_MOVE_STATE_SETTLING && ARM_DONE == INFRABOT_MOVE_STATE_DONE &&
               ARM_FAILED == INFRABOT_MOVE_STATE_FAILED && ARM_CANCELLED == INFRABOT_MOVE_STATE_CANCELLED,
               "DMOS states of the protocol");
_Static_assert(ARM_OK == INFRABOT_MOVE_OK && ARM_BUSY == INFRABOT_MOVE_BUSY &&
               ARM_NOT_READY == INFRABOT_MOVE_NOT_READY && ARM_INVALID == INFRABOT_MOVE_INVALID &&
               ARM_UNSUPPORTED == INFRABOT_MOVE_UNSUPPORTED && ARM_BAD_SESSION == INFRABOT_MOVE_BAD_SESSION &&
               ARM_STALE_ID == INFRABOT_MOVE_STALE_ID && ARM_COMM_TIMEOUT == INFRABOT_MOVE_COMM_TIMEOUT &&
               ARM_MOTION_TIMEOUT == INFRABOT_MOVE_MOTION_TIMEOUT &&
               ARM_FEEDBACK_STALE == INFRABOT_MOVE_FEEDBACK_STALE &&
               ARM_MOTOR_FAULT == INFRABOT_MOVE_MOTOR_FAULT && ARM_CANCEL == INFRABOT_MOVE_CANCEL &&
               ARM_TX_ERROR == INFRABOT_MOVE_TX_ERROR, "DMOS results of the protocol");
_Static_assert(ARM_MOVEJ == INFRABOT_MOVE_OP_MOVEJ && ARM_MOVEP == INFRABOT_MOVE_OP_MOVEP &&
               ARM_KEEPALIVE == INFRABOT_MOVE_OP_KEEPALIVE && ARM_CANCEL_MOVE == INFRABOT_MOVE_OP_CANCEL &&
               ARM_QUERY == INFRABOT_MOVE_OP_QUERY, "DMOV ops of the protocol");

typedef struct {
  float position[ARM_AXES], velocity[ARM_AXES];
  float lower[ARM_AXES], upper[ARM_AXES], velocity_max[ARM_AXES], torque_max[ARM_AXES];
  uint32_t enabled_mask, fresh_mask, healthy_mask;
  uint8_t ready; /* No enable/mode/maintenance operation is in progress. */
  /* Active joint command in the same frame as position[]; command_valid = 0
   * makes motions start from position[]. */
  float command[ARM_AXES];
  uint8_t command_valid;
  /* Motor rated speeds: Cartesian moves never command a joint faster
   * (0 = no cap). */
  float rated_speed[ARM_AXES];
  /* Joints driven in position-velocity mode (bit i = joint i). */
  uint32_t posvel_mask;
  ArmIkTimeSource time_source;
  void *time_source_context;
} ArmMoveFeedback;

/* Straight-line Cartesian tracking shared by RT Cartesian and movep.  A
 * reference pose moves toward the target along a straight line and a fixed
 * rotation axis under Cartesian speed/acceleration limits with braking; a
 * warm-started IK follows the reference, and each tick's joint step is scaled
 * by one common factor so the joint limits hold without bending the path. */
typedef struct {
  float linear_velocity_max, linear_acceleration_max;   /* m/s, m/s^2 */
  float angular_velocity_max, angular_acceleration_max; /* rad/s, rad/s^2 */
  float joint_velocity_max[ARM_AXES], joint_acceleration_max[ARM_AXES];
} ArmCartesianLimits;

typedef struct {
  ArmIkPose reference;
  float reference_velocity[6];    /* m/s xyz, then rad/s base-frame rotation vector */
  float goal[ARM_AXES];           /* this tick's scaled IK step target */
  float q[ARM_AXES], v[ARM_AXES]; /* joint command and its velocity */
  ArmIkOptions ik_options;
  float scale;                    /* last common step scaling; < 1 = joint-limited */
  uint8_t status;                 /* last IK status */
  float orientation_error;        /* last IK residual, rad */
  uint8_t blocked;                /* holding: target not followable */
  ArmIkPose blocked_target;       /* the target that blocked it */
  uint32_t elapsed_us;            /* last IK solve time */
  float position_error;           /* last IK residual, m */
} ArmCartesianTracker;

typedef struct {
  uint64_t session, closed_session;
  uint32_t id, started_ms, last_heartbeat_ms, duration_ms, dwell_started_ms;
  uint32_t arrived_mask;
  float max_error;
  uint32_t feedback_stale_since_ms;
  uint8_t state, reason, owned, dwelling, feedback_stale;
  uint8_t cartesian;
  ArmMoveCommand command;
  ArmMoveCartesianCommand cartesian_command;
  ArmCartesianLimits cartesian_limits;
  ArmCartesianTracker tracker;
  float start[ARM_AXES], position[ARM_AXES], velocity[ARM_AXES];
} ArmMotion;

/* Target shaping for a position-velocity drive.  A DM drive in this mode
 * follows its target with a first-order lag, time constant gear ratio /
 * KP_APR (its position loop turns the joint-angle error into a rotor
 * speed): with KP_APR 54, 0.17 s on J1/J2, 0.74 s on J3/J4, 0.19 s on
 * J5-J7.  Its frame carries no velocity feed-forward, so a joint moving at v
 * trails its command by tau * v (J4 at 0.1 rad/s: 74 mrad, measured 73).
 * Sending command + tau * v instead lets the joint track the command.  The
 * velocity is the command's own where the planner knows it (RT Cartesian,
 * movej, movep), else estimated over the window (RT joint). */
typedef struct {
  float past[ARM_DRIVE_LEAD_TICKS]; /* recent commands, ring */
  uint8_t next, count;
} ArmDriveLead;
uint8_t ArmMotion_Request(ArmMotion *m, uint8_t op, uint64_t session, uint32_t id,
                         const void *command, const ArmMoveFeedback *fb, uint32_t now);
void ArmMotion_Tick(ArmMotion *m, const ArmMoveFeedback *fb, uint32_t now);
void ArmMotion_Fail(ArmMotion *m, uint8_t reason);
void ArmMotion_Cancel(ArmMotion *m);
/* IK settings of the host infrabot_ik UI, for a solve warm-started from the
 * previous solution every tick (iterations capped per tick). */
void ArmMotion_TrackingIkOptions(ArmIkOptions *options);
/* Moves q toward goal with |v| <= velocity_max and |dv/dt| <= acceleration_max
 * (<= 0: unlimited).  A goal moving at goal_velocity (NULL = still) is
 * tracked exactly; a jump is approached with braking, without overshoot. */
void ArmMotion_FollowJoints(float q[ARM_AXES], float v[ARM_AXES], const float goal[ARM_AXES],
                            const float goal_velocity[ARM_AXES],
                            const float velocity_max[ARM_AXES],
                            const float acceleration_max[ARM_AXES], float dt);
/* Decodes wire limits (0 = default) and derives the joint caps.  Returns 0,
 * with defaults filled in, when a value exceeds its ARM_CARTESIAN_MAX_*. */
uint8_t ArmMotion_CartesianLimitsFromWire(const uint16_t wire[4], ArmCartesianLimits *limits);
/* The same from values in m/s, rad/s, m/s^2, rad/s^2, without range check. */
void ArmMotion_CartesianLimitsFromValues(float linear_velocity, float angular_velocity,
                                         float linear_acceleration, float angular_acceleration,
                                         ArmCartesianLimits *limits);
/* Caps position-velocity joints (posvel_mask) at ARM_POSVEL_SPEED_FRACTION
 * of the v_des their drives are sent. */
void ArmMotion_CapDriveSpeeds(ArmCartesianLimits *limits, const float drive_speed[ARM_AXES],
                              uint32_t posvel_mask);
/* Lowers joint caps where tighter (entries <= 0 are ignored); acceleration
 * caps also lower the tool's acceleration limits by the same ratios. */
void ArmMotion_CapJointLimits(ArmCartesianLimits *limits, const float velocity[ARM_AXES],
                              const float acceleration[ARM_AXES]);
/* A higher speed applies at once; a lower one ramps down at the acceleration
 * limits, so the joint caps derived from it never drop faster than the
 * joints can slow down (which would bend the line). */
void ArmMotion_RampCartesianLimits(ArmCartesianLimits *applied,
                                   const ArmCartesianLimits *requested, float dt);
void ArmCartesianTracker_Start(ArmCartesianTracker *t, const float q[ARM_AXES],
                               ArmIkTimeSource time_source, void *time_source_context);
/* One control tick toward target.  Returns the IK status: INVALID_INPUT and
 * QP_NUMERICAL_FAILURE must stop the arm; other failures hold the reference. */
uint8_t ArmCartesianTracker_Step(ArmCartesianTracker *t, const ArmIkPose *target,
                                 const ArmCartesianLimits *limits, float dt);
void ArmDriveLead_Reset(ArmDriveLead *lead, float q);
/* One control tick: the velocity of command q over the last
 * ARM_DRIVE_LEAD_TICKS ticks, |v| <= speed_max.  A step larger than twice
 * speed_max * dt is a re-latched command, not motion: the window restarts
 * at 0. */
float ArmDriveLead_Velocity(ArmDriveLead *lead, float q, float speed_max, float dt);
/* The target for command q moving at velocity: q + tau * velocity (tau <= 0:
 * q).  The target may pass a joint limit while the joint, which follows q,
 * approaches it (clipping it there left J2/J4, limit 0, up to 0.27 rad behind
 * on the way home); but a joint measured at or beyond a limit is not pushed
 * further out, so a drive faster than tau stops there. */
float ArmDriveLead_Target(float q, float velocity, float tau, float measured,
                          float lower, float upper);

#endif
