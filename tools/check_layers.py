#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Check layer includes and the transitive dependencies of portable control code."""
from pathlib import Path
import re
import sys
import subprocess
import shlex
import os

ROOT = Path(__file__).resolve().parents[1]
INCLUDE = re.compile(r'^\s*#\s*include\s*[<"]([^">]+)[">]', re.M)
# These headers contain wire codec declarations and plain data types only.
CONTROL_ALLOWED = {'dm_protocol.h', 'dm_types.h', 'can_types.h',
                   'bus_state.h', 'dm_runtime.h', 'eth_state.h'}  # plain shared-state types/instances
HARDWARE_HEADERS = {'main.h', 'FreeRTOS.h', 'task.h'}


def check():
    ui = {p.name for p in (ROOT/'ui').iterdir() if p.is_file()}
    drivers = {p.name for p in (ROOT/'drivers').rglob('*') if p.is_file()}
    errors = []
    for layer in ('control', 'drivers'):
        for file in sorted((ROOT/layer).rglob('*')):
            if file.suffix not in {'.c', '.cpp', '.h', '.inc'}:
                continue
            source = file.read_text()
            for match in INCLUDE.finditer(source):
                path = Path(match[1])
                if (path.name in ui or 'ui' in path.parts or
                    (layer == 'control' and
                     (path.name in HARDWARE_HEADERS or path.name.startswith('stm32') or
                      'lwip' in path.parts or
                      ((path.name in drivers or 'drivers' in path.parts) and path.name not in CONTROL_ALLOWED)))):
                    line = source[:match.start()].count('\n') + 1
                    errors.append(f'{file.relative_to(ROOT)}:{line}: forbidden include {path}')
    # Use the real public interfaces without selecting a hardware backend.
    # No mock headers shadow production includes. Supplying the board include
    # path also lets -MM diagnose newly introduced indirect main.h includes.
    dirs = ["app", "control", "protocol", "bootloader", "ui",
            "boards/fire-h743-challenger-v2", *sorted((ROOT / "drivers").iterdir())]
    for file in sorted((ROOT / 'control').rglob('*')):
        if file.suffix not in {'.c', '.cpp'}:
            continue
        compiler = os.environ.get('CXX', 'c++') if file.suffix == '.cpp' else os.environ.get('CC', 'cc')
        command = [*shlex.split(compiler), '-MM', '-MT', 'unit', '-DFW_NETBOOT_APP',
                   *['-I' + str(d) for d in dirs], str(file.relative_to(ROOT))]
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        if result.returncode:
            errors.append(f'{file.name}: dependency preprocessing failed:\n{result.stderr}')
            continue
        deps = shlex.split(result.stdout.partition(':')[2].replace(chr(92) + '\n', ' '))
        for name in deps:
            path = Path(name)
            if (path.name in HARDWARE_HEADERS or path.name.startswith('stm32')
                    or 'lwip' in (part.lower() for part in path.parts)):
                errors.append(f'{file.name}: indirect hardware dependency {path}')
    return errors


if __name__ == '__main__':
    errors = check()
    if errors:
        sys.exit('\n'.join(errors))
    print('Portable control include closures (-MM) and direct control/driver boundaries passed')
