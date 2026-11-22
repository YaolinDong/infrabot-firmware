# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Run the SWD planner and its emitted Tcl against local data, never a board."""
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
import netboot_swd as swd
from netboot_package import pack

from firmware_signing import generate
keys=tempfile.TemporaryDirectory()
secret=Path(keys.name)/"seed"
key=generate(secret)
def image(addr,n):return struct.pack('<II',0x20020000,addr+9)+bytes([n])*150000
loader=bytearray(image(swd.FLASH,1)[:2048]);loader[1024:1060]=b'FAG2'+key
def build(n):
    return pack(bytes(loader),image(swd.A+1024,n),image(swd.B+1024,n+1),secret)
old,factory_old=build(10);new,factory=build(20)
images=swd.unpack(new,key)
def rejected(fn):
    try:fn()
    except ValueError:return
    raise AssertionError('unsafe input accepted')

with tempfile.TemporaryDirectory() as temp:
    d=Path(temp)
    blank=b'\xff'*0x200000
    v1=bytearray(blank);v1[0x400:0x404]=b'FAG1'
    rejected(lambda:swd.plan(v1,images,factory,key,d))
    assert 'First' in swd.plan(v1,images,factory,key,d,migrate_v1=True)[0]
    m,t=swd.plan(blank,images,factory,key,d);assert 'First' in m
    assert 'flash write_image erase factory.bin' in t
    a=factory_old.ljust(0x200000,b'\xff')
    assert swd.current(a,key)[:2]==(1,swd.A)
    rejected(lambda:swd.unpack(new[:-1],key))
    rejected(lambda:swd.unpack(new,b'x'*32))
    rejected(lambda:swd.plan(a,images,factory,b'x'*32,d))
    rejected(lambda:swd.plan(a[:10],images,factory,key,d))
    rejected(lambda:swd.plan(a[:1024]+b'xxxx'+a[1028:],images,factory,key,d))
    for active,base,gen in ((swd.A,swd.B,2),(swd.B,swd.A,3)):
        method,t=swd.plan(a,images,factory,key,d);assert 'SWD update' in method
        assert (d/'active-slot.bin').read_bytes()==a[active-swd.FLASH:active-swd.FLASH+swd.SLOT]
        assert (d/'config.bin').read_bytes()==a[swd.CFG-swd.FLASH:]
        assert [l for l in t.splitlines() if l.startswith('flash erase')]==[f'flash erase_address {base:#x} 0x40000']
        assert t.index('verify_image app.bin')<t.index('flash write_image ready.bin')
        assert t.index('wait_app 0x31545753')<t.index('mww 0x20000014 0x31435753')<t.index('wait_app 0x314e5753')
        good=swd.mark(swd.GOOD,gen,base,images[base][32:64])
        # Exercise the same polling/marker checks fed to OpenOCD, with stubbed I/O.
        stubs='''proc init {} {}; proc halt {} {}; proc reset args {}; proc verify_image args {}; proc flash args {}; proc sleep args {}
set state 0; proc mww {addr value args} {if {$addr == 0x20000014} {set ::state 1}}
proc mrw {addr} {
 if {$addr == 0x20000010} {return [expr {$::state ? 0x314e5753 : 0x31545753}]}
'''
        stubs+=f'if {{$addr == 0x20000018}} {{return {base+1024}}}\nif {{$addr == 0x2000001c}} {{return {gen}}}\n'
        for i,w in enumerate(struct.unpack('<8I',good)):
            stubs+=f'if {{$addr == {base+128+4*i}}} {{return {w}}}\n'
        script=d/'test.tcl';script.write_text(stubs+'error "unexpected read"\n}\n'+t)
        subprocess.run(['tclsh',script],check=True)
        # A different older release confirmed on B for the reverse-direction case.
        blob=swd.unpack(old,key)[base];tag=blob[32:64];r=swd.mark(swd.READY,gen,base,tag)
        slot=(blob[:64]+r+b'\xff'*32+swd.mark(swd.GOOD,gen,base,tag)).ljust(1024,b'\xff')+blob[64:]
        off=base-swd.FLASH;a=a[:off]+slot.ljust(swd.SLOT,b'\xff')+a[off+swd.SLOT:]
        assert swd.current(a,key)[:2]==(gen,base)
    m,t=swd.plan(factory.ljust(0x200000,b'\xff'),images,factory,key,d)
    assert 'Already' in m and 'flash erase' not in t
print('SWD: legacy provisioning, authenticated A->B and B->A, readback/ready/confirm order, unchanged image and rejection paths passed')
