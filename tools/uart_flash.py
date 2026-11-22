#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""STM32H743 USART1 ROM downloader (Linux). 'prepare' never opens a port."""
import argparse
import hashlib
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
BASE = 0x08000000
BANK_SIZE = 0x100000
SECTOR_SIZE = 0x20000


def prepare():
    subprocess.run(["cmake", "--preset", "StandaloneDebug"], cwd=ROOT, check=True)
    subprocess.run(["cmake", "--build", "--preset", "StandaloneDebug"], cwd=ROOT, check=True)
    elf = ROOT / "build/StandaloneDebug/infrabot_firmware-standalone.elf"
    image = elf.with_name("infrabot_firmware-uart.bin")
    subprocess.run(["arm-none-eabi-objcopy", "-O", "binary", elf, image], check=True)
    data = image.read_bytes()
    if not 8 <= len(data) <= BANK_SIZE:
        raise ValueError("Image does not fit Bank 1; refusing to program.")
    sp, entry = struct.unpack_from("<II", data)
    if sp != 0x20020000 or not entry & 1 or not BASE <= (entry & ~1) < BASE + len(data):
        raise ValueError("Unexpected initial SP/reset vector for this project's linker layout.")
    sectors = list(range((len(data) + SECTOR_SIZE - 1) // SECTOR_SIZE))
    print(f"{image}: {len(data)} bytes, SHA256 {hashlib.sha256(data).hexdigest()}\n"
          f"0x{BASE:08X}..0x{BASE+len(data)-1:08X}, Bank 1 sectors {sectors}", flush=True)
    return image, sectors


def programmer():
    exe = os.environ.get("STM32_PROGRAMMER_CLI") or shutil.which("STM32_Programmer_CLI")
    if not exe:
        matches = sorted((Path.home()/".local/share/stm32cube/bundles/programmer").glob("*/bin/STM32_Programmer_CLI"))
        if not matches:
            raise ValueError("Set STM32_PROGRAMMER_CLI to the installed programmer executable.")
        exe = str(matches[-1])
    return exe


def require_non_netboot(conn):
    # ROM UART mode: binary reads are unambiguous even if CLI output changes.
    # A surviving slot also protects a partially overwritten loader.
    with tempfile.TemporaryDirectory() as tmp:
        for address, markers in ((BASE + 0x400, (b"FAG1", b"FAG2")),
                                 (0x08020000, (b"IFH743v1", b"IFH743v2")),
                                 (0x08100000, (b"IFH743v1", b"IFH743v2"))):
            data = Path(tmp) / f"{address:x}.bin"
            subprocess.run([*conn, "--upload", hex(address), "8", str(data)], check=True)
            got = data.read_bytes()
            if len(got) != 8 or got.startswith(markers):
                raise ValueError("A/B loader/slot detected or incomplete read; refusing standalone erase. Use flash-swd or flash-net.")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("action", choices=["prepare", "probe", "flash"])
    p.add_argument("port", nargs="?", help="Confirmed USART1 tty or /dev/serial/by-id/... path")
    args = p.parse_args()
    if args.action == "prepare":
        prepare()
        return
    if not args.port:
        p.error("an explicit port is required")
    port = Path(args.port).resolve(strict=True)
    if not re.fullmatch(r"/dev/tty(?:USB|ACM|S)[0-9]+", str(port)) or not port.is_char_device():
        p.error("expected a serial tty")
    if not os.access(port, os.R_OK | os.W_OK):
        p.error(f"no read/write permission on {port}; check serial group/ACL")
    conn = [programmer(), "-c", f"port={port}", "br=115200", "P=EVEN", "db=8", "sb=1", "fc=OFF"]
    if args.action == "probe":
        subprocess.run([*conn, "-r32", hex(BASE), "8"], check=True)
        return
    image, sectors = prepare()
    require_non_netboot(conn)
    subprocess.run([*conn, "-e", *map(str, sectors), "--skipErase", "-w", str(image), hex(BASE), "-v"], check=True)
    print("Restore BOOT0 to normal Flash boot, then press RESET.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        sys.exit(str(exc))
