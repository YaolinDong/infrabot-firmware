#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Provision using a trusted local v2 build; preserve config and verify application startup."""
import argparse
import subprocess
import sys
from pathlib import Path
from netboot_swd import build_package_path

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--image",required=True,type=Path)
    p.add_argument("--backup-dir",required=True,type=Path)
    p.add_argument("--motor-power-off",action="store_true",required=True)
    p.add_argument("--migrate-v1",action="store_true")
    p.add_argument("--public-key",type=Path,default=Path(__file__).resolve().parents[1]/"bootloader/release.pub")
    a=p.parse_args()
    sys.exit(subprocess.call([sys.executable,"-B",Path(__file__).with_name("netboot_swd.py"),
        "--package",build_package_path(a.image.parent),"--factory",a.image,"--public-key",a.public_key,
        "--backup-dir",a.backup_dir,"--motor-power-off"]+(["--migrate-v1"] if a.migrate_v1 else [])))
