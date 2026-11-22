# Stopping and drive power

The firmware starts with drives disabled. Verify encoder zero, joint directions, motor mode, available clearance and mechanical support before enabling. The arm enable group covers J1–J7. The optional J8 gripper has its own enable group.

A STOP request brakes the arm and normally leaves it enabled in IDLE hold. Stream timeouts, stale feedback, drive faults, excessive temperature or speed and persistent tracking errors are supervised by the relevant control mode. Examine the stop reason and drive status before resuming. See [mode transitions and recovery](control-modes.md) and [admittance thresholds](admittance-control.md).

Software joint, speed and feedforward torque limits operate inside the controller. The drive also generates torque from its internal PD terms. A host STOP, on-screen STOP or software timeout cannot isolate a failed drive or remove energy after a controller failure. Provide an independently wired hardware emergency stop and support the arm before cutting power. For maintenance disable the drives, isolate motor power and retain controller power for communication.
