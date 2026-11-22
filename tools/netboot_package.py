#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Create factory SWD images and an authenticated network application package."""
import argparse
import hashlib
from firmware_signing import public_key, generate, sign
from pathlib import Path
import struct
import subprocess


def pack(loader, app_a, app_b, key):
    pub=public_key(key)
    if loader[0x400:0x424]!=b"FAG2"+pub:raise ValueError("Signing key does not match the loader public key")
    def check(image, address, limit):
        if not 8 <= len(image) <= limit:
            raise ValueError("image exceeds slot capacity; old firmware must remain recoverable")
        sp, pc = struct.unpack_from("<II", image)
        if sp != 0x20020000 or not pc & 1 or not address <= (pc & ~1) < address+len(image):
            raise ValueError(f"not an image linked at {address:#x}")
    check(loader,0x08000000,0x20000)
    release=hashlib.sha256(app_a+app_b).digest()[:16]
    images=[]
    for base,body in [(0x08020400,app_a),(0x08100400,app_b)]:
        check(body,base,0xdfc00-64)
        meta=struct.pack("<8sII16s",b"IFH743v2",base,len(body)+64,release)+hashlib.sha256(body).digest()
        images.append(meta+body+sign(key,meta))
    a,b=images
    # Factory creates exactly one confirmed slot, generation 1.
    def mark(kind):
        return struct.pack("<IIII16s",kind,1,0xfffffffe,0x08020400,a[32:48])
    header=a[:64]+mark(0x59444152)+bytes([255])*32+mark(0x444f4f47)
    factory=loader.ljust(0x20000,b'\xff')+header.ljust(1024,b'\xff')+a[64:]
    return struct.pack("<8sII",b"IFABv002",len(a),len(b))+a+b,factory


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("action",choices=["keygen","package"])
    p.add_argument("--key",type=Path,required=True)
    p.add_argument("--build",type=Path,default=Path("build/Release"))
    a = p.parse_args()
    if a.action=="keygen":
        print(generate(a.key).hex())  # public verification key only
        return
    key=a.key
    outputs=[]
    for name in ["netboot_loader","infrabot_firmware","infrabot_firmware_b"]:
        binary=a.build/(name+".bin")
        subprocess.run(["arm-none-eabi-objcopy","-O","binary",a.build/(name+".elf"),binary],check=True)
        outputs.append(binary.read_bytes())
    package,factory=pack(*outputs,key)
    (a.build/"infrabot_firmware.ifw").write_bytes(package)
    for name,buf in [("factory-bank1.bin",factory[:0x100000]),("factory-bank2.bin",factory[0x100000:])]:
        path=a.build/name
        if buf:path.write_bytes(buf)
        elif path.exists():path.unlink()
    print("Slot A/B:",len(outputs[1]),len(outputs[2]),"bytes. IFW SHA256:",hashlib.sha256(package).hexdigest())
    print("Factory ends at",hex(0x08000000+len(factory)),"(exclusive); reserved config starts at 0x081e0000")


if __name__=="__main__":
    main()
