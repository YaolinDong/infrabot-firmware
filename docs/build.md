# Build the firmware

On Ubuntu, install the GNU Arm bare-metal toolchain (C and C++), Newlib, CMake 3.22 or later, Ninja, Python 3 and host build tools.

```sh
sudo apt install cmake ninja-build gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib python3 build-essential
cmake --preset Release
cmake --build --preset Release --parallel 4
```

The toolchain file is selected by the preset. Release and Debug emit independently linked A/B application ELF and BIN files, plus the resident loader in `build/<preset>/`. The A application is `infrabot_firmware.elf`, B is `infrabot_firmware_b.elf`. Debug includes symbols and optimizes the real-time loop. StandaloneDebug links a single image at the flash base for boards provisioned without an A/B loader. Use a separate build directory per preset.

Ninja is the preset default. For GNU Make, use an empty directory and configure with `cmake --preset Release -G "Unix Makefiles" -B build/release-make`, then run `cmake --build build/release-make`. The middleware is included under `third_party/`. For deterministic offline comparisons set `SOURCE_DATE_EPOCH=1763700000` and build an exported tree without Git metadata. Development builds include the checkout revision.

For a signed update configure the publisher's key explicitly. [Build a release package](flashing.md).
