# Joint and Cartesian impedance (fixed equilibrium)

This MCU implementation uses the drive's existing MIT torque interface and
reported torque. It does not read raw Iq or implement an additional current PI
loop. Reported motor torque is not an external/contact-torque measurement: it
contains the effort supporting gravity, accelerating the arm, and overcoming
friction. No external wrench observer or force feedback is claimed here.

## Reference implementations

- Franka Robotics: https://github.com/frankarobotics/libfranka/blob/main/examples/joint_impedance_control.cpp
- Franka Robotics: https://github.com/frankarobotics/libfranka/blob/main/examples/cartesian_impedance_control.cpp
- Matthias Mayr: https://github.com/matthias-mayr/Cartesian-Impedance-Controller

These provide reference architectures for joint spring/damper and task-space
J-transpose torque controllers. This repository uses a
fixed-size C implementation without ROS/Eigen or allocation. Do not transfer
Franka's gains, collision settings, or gravity-compensation assumptions to this
arm. Here gravity/offset/spring feedforward comes from the existing calibrated
ArmGravity model. Coriolis/inertial shaping and nullspace posture optimization
are not implemented; Cartesian mode includes passive all-joint damping to damp
redundant motion, but no preferred nullspace posture.

## Control law and coordinates

All computation is in the robot joint frame after zero/direction conversion.
The existing CAN sender converts joint torque to motor signs exactly once.
J8 remains independently controlled in position/current mode.

JOINT IMP: `tau = gravity(q) + K(q_eq - q) - D dq`.

CART IMP: `tau = gravity(q) + Jv^T(Kx(x_eq-x)-Dx Jv dq)`
`+ Jw^T(Kr Log(R_eq R^T)-Dr Jw dq) - D_joint dq`.

Position/rotation errors and both Jacobians use the base frame. Position is in
metres; rotation error is a rotation vector in radians, not Euler subtraction.
Joint position error is clamped to +/-0.35 rad, each Cartesian translation
component to +/-0.10 m, and rotation-vector component to +/-0.5 rad before its
spring calculation. These bound spring effort, not physical travel.

The equilibrium is latched from measured state once upon entering each mode.
It does not chase the arm while it is being pushed. STOP/RESUME re-enters at the
new measured equilibrium. This differs from the existing yielding gravity LOCK.
There is currently no moving-equilibrium/trajectory command for these modes.

## Operation and gains

Use HOME to enable the arm with a verified MIT profile and established zeros.
From stationary IDLE, open IMPED and select JOINT IMP or CART IMP. This reuses
the existing one-second complete-controller handover, preserving load support.
After handover, MIT Kp/Kd are zero: the MCU computes the full gravity + impedance
feedforward torque each tick. No double-counted drive PD is applied.

The IMPED tab cycles through four parameter groups and a fifth diagnostic page: joint K/D, Cartesian
translation K/D, Cartesian rotation K/D, and additional joint damping. Mode
entry buttons and STOP / HOLD remain available on every group. Browsing groups
only selects parameters to edit, never changes the active control mode. The
screen shows requested and applied values plus units and permitted ranges.
Mode feedback is based on the control-task snapshot: WAIT means queued, RAMP
means the entry handover is still in progress, and a persistent green ACTIVE
button confirms completion. The status line states the active impedance mode;
rejected requests show their reason and never select the rejected mode. STOP or
exit clears the highlight. Browsing away and back restores the actual state.

These seven parameters are independent of the gravity FREE/LOCK sliders.
They are held in RAM and reset to app_config.h defaults after a reboot.
The control task slews them each tick; editing does not re-latch equilibrium.
They are shared by J1-J7 / the three Cartesian axes (isotropic gains), not
per-joint or per-axis tuning. Allowed UI ranges are engineering bounds,
not a hardware-validated stable gain envelope for every pose or payload.

- Joint K: default 8, range 0.5..30 Nm/rad; rate 20 Nm/rad per second.
- Joint D: default 0.8, range 0.1..3 Nm s/rad; rate 2 units per second.
- Cartesian translation K: default 80, range 5..300 N/m; rate 100 units/s.
- Cartesian translation D: default 12, range 1..50 N s/m; rate 20 units/s.
- Cartesian rotation K: default 3, range 0.1..15 Nm/rad; rate 5 units/s.
- Cartesian rotation D: default 0.6, range 0.05..5 Nm s/rad; rate 2 units/s.
- Additional joint damping in CART IMP: default 0.3, range 0.05..3 Nm s/rad;
  rate 2 units/s. This damps redundant motion without a nullspace posture spring.

- Both modes preserve the existing torque caps, speed/joint-limit braking,
  finite-state, drive-state, temperature and 50 ms fresh-feedback checks.
  The model+impedance calculation retains the 250 us runtime budget.
- Normal impedance feedforward changes are limited to 0.85 times each joint's
  torque limit per 0.2 s. Protective braking is added separately and final
  torque is capped. No passivity guarantee is claimed for delay/saturation.
- FF/Drive Nm shows MCU feedforward and driver-reported torque, in joint signs.
  During handover/recovery or gravity LOCK, drive PD also contributes to output.
- STOP / HOLD and KEY1 use controlled stop and restore holding gains. KEY2 or
  RESUME may re-enter a resumable local mode at its new equilibrium. Mode
  changes require STOP/settling. The controller does not automatically disable
  drives as a reaction to a protective stop.

## Protocol

Existing field sizes and IDs remain unchanged. Added supervisory modes:
JOINT_IMPEDANCE=7, CARTESIAN_IMPEDANCE=8; operational status values 17 and 18.
The existing SET_CONTROL_MODE request accepts these values; no RT joint or
Cartesian target stream is consumed by these fixed-equilibrium modes. They use
the same offline local-hold policy as gravity FREE/LOCK. Legacy PC GUIs may show
unknown mode/status until their protocol enums/UI are updated. The current InfraBot SDK and the LCD both expose the joint and Cartesian impedance modes.

## Validation and remaining hardware checks

Host tests check equilibrium, Cartesian virtual-work mapping/orientation sign,
dissipative damping, finite-input validation, full MIT output through entry
ramps, fixed equilibrium under displacement, feedforward slew/torque caps,
feedback direction, STOP/RESUME, disable priority and profile/zero/stale-feedback
gates. Both native LCD layouts are rendered and tested. These tests do not
simulate the current loop, gear friction/backlash, full robot dynamics, or
contact stability. Hardware validation remains required.

Start with unloaded, supported hardware and a small displacement. Validate
JOINT IMP before CART IMP, check torque directions and restoration to the latched
equilibrium, and confirm STOP holds. Record ctrl=1000Hz, miss, period_max and
GRAV mode=7/8 us=current/max; a host test cannot certify the MCU timing budget.

## Return-to-equilibrium diagnostics

IMPED 5/5 reports the target and actual FK positions from the same control sample,
raw position and rotation-vector errors, and the virtual spring/damper force and
moment (not measured contact wrench). The largest joint impedance correction is
also displayed. Invalid/stopped/non-Cartesian states do not present stale data as
live diagnostics. Viewing this page sends no control or parameter requests.

The status task prints one IMP group per status interval (normally one second).
Positions/errors are mm, rotation errors mrad, gains multiplied by 1000, and
model/imp/ff/drive joint torques mNm. `model` includes gravity/offset/spring;
`imp` is the corrective impedance contribution before saturation; `ff` is the
final commanded feedforward; `drive` is the signed drive-reported torque.
The printed gains are those actually applied to the sampled calculation.

`cap` marks torque saturation, `slew` a feedforward rate limit, and `brake` the
existing speed/joint-limit brake (bit 0 = J1). `errcap` marks spring error caps:
bits 0..2 = XYZ position, 3..5 = rotation-vector components. Flags are instantaneous,
not interval maxima. Interpretation requires the captured pose and gains; a
nonzero position error alone does not prove insufficient stiffness. The MCU
copies existing intermediates instead of computing an extra FK/Jacobian, and
only the low-priority status task formats output.
