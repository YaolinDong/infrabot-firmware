# Changelog

## 0.2.0 — unreleased

- Parameter service (protocol 1.1, capability `INFRABOT_CAP_PARAMETERS`): DMPR and DMPA on the command port list, read and write the gravity, impedance and admittance parameters by id, with name, unit, range, default and slew rate. Writes use the touchscreen limits and slew and live in RAM.
- Diagnostic snapshot (capability `INFRABOT_CAP_DIAGNOSTICS`): DMDR and DMDA on the command port return the status task's last one-second window, totals since boot, CAN error state, heap free and lowest, least free stack of the control, status and Ethernet tasks, and drive states. Read only and open to any PC.
- The DMIO golden sample carries the new capability bits.
- Telemetry right after boot reports each drive's real state: bring-up asks every drive for feedback once more after disabling it. Before, J1–J7 showed ENABLED (the answer to the preflight probe) until the first enable.
- While the gripper group is enabled with the arm already holding, the arm's MIT hold frames carry the torque feed-forward in the drive frame, as the 1 kHz tick does. Before, J2 and J4 got it with the sign reversed for each attempt, so an IDLE hold after a local mode shook while an absent gripper was being retried.

## 0.1.2 — 2026-11-26

- **Safety:** a controlled stop within 10 ms of enabling the arm no longer drives it. The stop's speed estimate kept the commands from before the disable, read the enable's rebase to the current angle as motion (a step of about 55 mrad as about 5.5 rad/s) and braked the command along it for up to 0.25 s of that speed. On the arm, a STOP right after a refused mode switch moved J5 by 1.0 rad and J6 by 0.9 rad from rest, hit the J6 torque limit and overheated the J5 coil. The estimate now starts over at every enable. Update from 0.1.0 or 0.1.1.

## 0.1.1 — 2026-11-23

- The update prompt shows the running version, an arrow and the version reported for the new image. The image ID remains the identity to check; the resident loader is unchanged.

## 0.1.0 — 2026-11-23

- STM32H743 control for seven H7C joints and an optional gripper, with Ethernet commands, telemetry and local panel operation.
- Signed A/B firmware updates with touchscreen approval, confirmed trial boot and SWD recovery tools.
- Board, driver, protocol, application and control modules with native regression tests and reproducible firmware comparisons.
