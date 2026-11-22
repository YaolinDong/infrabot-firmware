#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Compare preprocessor macro names, parameters and replacement tokens across a config split."""
import argparse
import difflib
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
INCLUDES = ("app", "protocol", "control", "drivers/damiao", "boards/fire-h743-challenger-v2")


def dump(source, overrides):
    args = ["cc", "-std=c11", "-Werror", "-dM", "-E", "-x", "c"]
    args += ["-I" + str(ROOT / directory) for directory in INCLUDES]
    args += ["-D" + definition for definition in overrides]
    text = subprocess.check_output([*args, "-"], input=source, text=True, cwd=ROOT)
    return sorted(text.splitlines())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--baseline", help="git revision before the split")
    group.add_argument("--before", type=Path, help="standalone copy of the original header")
    args = parser.parse_args()
    if not args.baseline:
        before = (args.before or ROOT/"tests/data/app_config_before.h").read_text()
    else:
        paths = subprocess.check_output(
            ["git", "ls-tree", "-r", "--name-only", args.baseline, "--",
             "app/app_config.h", "Core/Inc/app_config.h"], cwd=ROOT, text=True).splitlines()
        if len(paths) != 1:
            parser.error("baseline must contain one application config header")
        before = subprocess.check_output(["git", "show", f"{args.baseline}:{paths[0]}"],
                                         cwd=ROOT, text=True)
    after = '#include "app_config.h"\n'
    for overrides in [[], ["DM_LCD_PANEL=43"], ["DM_LCD_FALLBACK_PANEL=5"],
                      ["DM_LCD_PANEL=5", "DM_LCD_FALLBACK_PANEL=43", "DM_LCD_43_LEGACY_TIMING=1"]]:
        expected, actual = dump(before, overrides), dump(after, overrides)
        if actual != expected:
            raise SystemExit("\n".join(difflib.unified_diff(expected, actual, "before", "after", lineterm="")))
    count = len(re.findall(r"^#define\s+\w+", before, flags=re.MULTILINE))
    print(f"{count} original definitions (including include guard); identical complete -dM output, four configurations")


if __name__ == "__main__":
    main()
