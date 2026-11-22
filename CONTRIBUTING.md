# Contributing

Describe the hardware, active control mode and commands needed to reproduce the change. For protocol changes document the wire layout and SDK compatibility. For UI changes include native screenshots for both panel sizes.

Run the firmware host tests and build Release, Debug and StandaloneDebug. For changes to motion control include the fixture, simulation or replay result and the effect on the control task's execution budget. [Test commands](docs/testing.md) · [Build instructions](docs/build.md)

Use English commit subjects in the form `type(scope): description`. Keep changes to vendor code separate and record them in the patch inventory.

## Checking a source release

Set `INFRABOT_PRIVATE_DENYLIST` to a regular file outside the checkout, readable only by its owner (mode 0600), with one literal to reject per line. Run `tools/prepublish_check.sh --history` on the release repository to check the index, all reachable snapshots and Git metadata. In CI without the local file, the checker prints a skip notice for private literals and still checks key markers, paths, licenses and size exceptions.
