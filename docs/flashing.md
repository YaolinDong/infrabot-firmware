# Provision and update

An IFW package contains two application images linked at their respective slot addresses, signed manifests and image digests. The resident loader verifies the image and starts the candidate on trial. The SDK queries the running application to confirm the update. The factory BIN also contains the resident loader and is used for SWD provisioning.

## Publisher keys

The repository ships the public verification key in `bootloader/release.pub`. A board accepts releases signed by the publisher trusted by its resident loader. Anyone with the SDK and the matching public key can install an official signed release after local approval. To distribute your own builds, generate your own Ed25519 key once, keep the private PEM outside the checkout and back it up privately. These commands print only the public key and use OpenSSL 3.

```sh
umask 077
set -C  # protect an existing public-key file from shell redirection
mkdir -p "$HOME/infrabot-keys"
python3 -B tools/firmware_signing.py generate --key "$HOME/infrabot-keys/publisher.pem" > "$HOME/infrabot-keys/publisher.pub"
cmake --preset Release \
  -DFW_PUBLIC_KEY_FILE="$HOME/infrabot-keys/publisher.pub" \
  -DFW_SIGNING_KEY_FILE="$HOME/infrabot-keys/publisher.pem"
cmake --build --preset Release --target netboot_images --parallel 4
```

Run key generation only for a new publisher, with unused output paths. Existing publishers configure the corresponding existing keys. The signing key is needed at packaging time. Ordinary ELF/BIN builds use only the public key. Your signatures require your own loader and matching SDK public key. Changing a provisioned board's trust root requires physical SWD service.

## First installation

Support and disable the arm, disconnect motor power and keep the controller powered. With a CMSIS-DAP probe and OpenOCD, provision a new board from the firmware checkout. Use the public key selected at build time.

```sh
python3 -B tools/netboot_factory.py --image build/Release/factory-bank1.bin \
  --public-key "$HOME/infrabot-keys/publisher.pub" \
  --backup-dir "$HOME/infrabot-flash-backups" --motor-power-off
```

For official provisioning select `bootloader/release.pub`. Existing installations and write-protected loaders have specific migration and restoration procedures. [Flash layout, backup and recovery](ethernet-update.md).

## Routine upgrades

Copy `build/Release/infrabot_firmware.ifw` to the SDK distribution. The SDK default delivery path is `firmware/infrabot_firmware.ifw`. The previous basename `fdcantest.ifw` is accepted for one SDK release. To try a new release before changing the default, pass its path explicitly. For a private publisher, select its public key with `--public-key /path/to/publisher.pub` or install that key in the SDK kit. From the SDK directory, with the robot supported, drives disabled and motor power disconnected, choose one method.

```sh
./flash-net --package /path/to/infrabot_firmware.ifw
# OR, via a connected SWD probe
./flash-swd --package /path/to/infrabot_firmware.ifw
```

For Ethernet verify the image ID on the touchscreen, then choose Allow. Rejecting the request or letting it expire cancels the update. Run `./flash-net --probe` after restart and compare the confirmed slot and image ID. For source debugging, select the Debug ELF that matches the confirmed slot.
