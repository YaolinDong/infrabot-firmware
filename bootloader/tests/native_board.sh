#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Build a loopback board with real protocol/install code and a temporary key.
set -eu
root=$(cd "$(dirname "$0")/../.." && pwd)
out=$1
mbed=${STM32CUBE_ROOT:-$root/third_party/STM32CubeH7}/Middlewares/Third_Party/mbedTLS
cd "$root"
python3 - "$out" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1]);key=bytes.fromhex((p/'public').read_text().strip())
(p/'fw_public.h').write_text('#define FW_PUBLIC_BYTES {'+','.join(map(str,key))+'}\n')
PY
compile() { cc -include "$root/bootloader/tests/inc/main.h" -std=gnu11 -Wall -Wextra -Werror -O2 -Ibootloader/tests/inc -Ibootloader -I"$out" -I"$mbed/include" -Ibootloader/vendor/monocypher "$@"; }
compile -DFW_LOADER_BUILD -Dfw_service_start=loader_service_start -c bootloader/service.c -o "$out/loader.o"
compile -Dfw_service_start=application_service_start -c bootloader/service.c -o "$out/app.o"
compile -Itests/mocks -Iapp -Icontrol -Idrivers/damiao -Idrivers/can -Idrivers/net -Idrivers/display -Iui -Iprotocol -Iboards/fire-h743-challenger-v2 -c bootloader/consent.c -o "$out/consent.o"
compile bootloader/tests/consent_test.c "$out/consent.o" -o "$out/consent_test"
"$out/consent_test"
crypto="bootloader/vendor/monocypher/monocypher.c bootloader/vendor/monocypher/monocypher-ed25519.c"
compile $crypto "$out/consent.o" bootloader/tests/board.c bootloader/tests/flash.c bootloader/slots.c bootloader/install.c bootloader/image.c bootloader/staged_sha.c "$mbed/library/sha256.c" "$out/app.o" "$out/loader.o" -o "$out/board"

compile $crypto bootloader/tests/slots_test.c bootloader/tests/flash.c bootloader/slots.c bootloader/install.c bootloader/image.c bootloader/staged_sha.c "$mbed/library/sha256.c" -o "$out/slots_test"
compile -DAPP_GRAVITY_REAL_TEST -Itests/mocks -Iapp -Icontrol -Idrivers/damiao -Idrivers/can -Idrivers/net -Idrivers/display -Iui -Iprotocol -Iboards/fire-h743-challenger-v2 -ffunction-sections -fdata-sections \
   bootloader/app_gate.c bootloader/tests/gate_test.c drivers/can/bus_state.c control/control_stats.c drivers/damiao/dm_runtime.c drivers/net/eth_state.c app/uart_tx_state.c -no-pie -Wl,--defsym,g_pfnVectors=0x08020400,--gc-sections -o "$out/gate_test"
"$out/gate_test"
cc -std=c11 -O2 -Wall -Wextra -Werror -Ibootloader/tests/rng_mock bootloader/challenge.c bootloader/tests/rng_test.c -o "$out/rng_test"
"$out/rng_test"

cc -std=c11 -O2 -Wall -Wextra -Werror -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast \
  -DSTM32H743xx -DUSE_HAL_DRIVER -Iapp -Icontrol -Idrivers/damiao -Idrivers/can -Idrivers/net -Idrivers/display -Iui -Iprotocol -Iboards/fire-h743-challenger-v2 -Ithird_party/CMSIS/Include \
  -Ithird_party/CMSIS/Device/ST/STM32H7xx/Include -Ithird_party/STM32H7xx_HAL_Driver/Inc \
  bootloader/tests/scan_clock_test.c -o "$out/scan_clock_test"
"$out/scan_clock_test"
