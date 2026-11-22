#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""SDK + firmware integration on 127.0.0.1 only. No board or permanent key used."""
import argparse
import hashlib
import hmac
import os
from pathlib import Path
import secrets
import socket
import struct
import subprocess
import tempfile
import time
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from netboot_package import pack
from firmware_signing import generate
from netboot_swd import A,B,unpack


def image(base,fill):
    return struct.pack("<II",0x20020000,base+9)+bytes([fill])*32799


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--updater',required=True,type=Path)
    exe=parser.parse_args().updater.resolve()
    with tempfile.TemporaryDirectory() as folder:
        tmp=Path(folder);secret=tmp/'seed';pub=generate(secret);keyfile=tmp/'public';keyfile.write_text(pub.hex())
        loader=bytearray(image(0x08000000,19));loader[0x400:0x424]=b'FAG2'+pub
        old,factory=pack(bytes(loader),image(0x08020400,11),image(0x08100400,12),secret)
        new,_=pack(bytes(loader),image(0x08020400,21),image(0x08100400,22),secret)
        small,_=pack(bytes(loader),image(0x08020400,41)[:448],image(0x08100400,42)[:448],secret)
        for slot,blob in unpack(small,pub).items():(tmp/str(slot)).write_bytes(blob)
        (tmp/'factory').write_bytes(factory)
        pkg=tmp/'new.ifw';pkg.write_bytes(new)
        subprocess.run([ROOT/'bootloader/tests/native_board.sh',tmp],check=True)
        subprocess.run([tmp/'slots_test',tmp/'factory',tmp/str(A),tmp/str(B)],check=True)
        for scenario in ('success','busy','denied','bad-trial','interrupted','rng-app','timeout'):
            board=subprocess.Popen([tmp/'board',tmp/'factory',scenario],stdout=subprocess.PIPE,text=True)
            try:
                port=board.stdout.readline().strip();assert port.isdigit()
                def run(*args,ok=True):
                    r=subprocess.run([exe,*args,port],capture_output=True,text=True,timeout=100)
                    assert (r.returncode==0)==ok,(args,r.stdout,r.stderr)
                    return r.stdout+r.stderr
                def probe():return run('--probe','127.0.0.1',keyfile).strip()
                if scenario=='rng-app':
                    assert 'RNG' in run('127.0.0.1',pkg,keyfile,ok=False)
                    assert 'confirmed A' in probe()
                    print('Loopback:',scenario,'passed',flush=True);continue
                old_id=probe();assert 'confirmed A' in old_id
                if scenario=='interrupted':
                    # Actual v2 request, with the test operator approving only this image, through ENTER/BEGIN, only one
                    # chunk, then abandon the client. Emulated 4.5 s watchdog.
                    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.connect(('127.0.0.1',int(port)));sock.settimeout(.7)
                    seq=0;sid=0
                    def request(op,offset=0,data=b''):
                        nonlocal seq,sid
                        seq+=1;head=struct.pack('<4sBBBBQII',b'IFU2',2,op,0,0,sid,seq,offset)
                        sock.send(head+data)
                        ack=sock.recv(128)
                        _,_,_,status,role,reply_sid,_,off=struct.unpack('<4sBBBBQII',ack[:24])
                        if op==1:sid=reply_sid
                        return role,status
                    n=struct.unpack_from('<I',new,8)[0];b=new[16+n:]
                    request(1);assert request(2,data=b[32:64])==(1,8)
                    request(2,data=b[32:64]);assert request(1)==(2,0)
                    wrong=bytearray(b[:64]);wrong[32]^=1
                    assert request(3,data=wrong)==(2,2)  # approved digest cannot be substituted
                    assert request(3,data=b[:64])==(2,0)
                    try:request(4,data=b[64:1088])
                    except socket.timeout:pass
                    sock.close();time.sleep(4.8);assert probe()==old_id
                elif scenario=='success':
                    corrupt=tmp/'bad.ifw';corrupt.write_bytes(new[:-1]+bytes([new[-1]^1]))
                    assert 'signature' in run('127.0.0.1',corrupt,keyfile,ok=False)
                    assert probe()==old_id
                    run('127.0.0.1',pkg,keyfile)
                    assert 'confirmed B' in probe()
                    run('127.0.0.1',pkg,keyfile) # same release is idempotent
                    run('--rollback','127.0.0.1',keyfile);assert probe()==old_id
                else:
                    msg=run('127.0.0.1',pkg,keyfile,ok=False)
                    assert ({'busy':'busy','denied':'denied','timeout':'expired'}.get(scenario,'rolled back')) in msg,msg
                    assert probe()==old_id
                print('Loopback:',scenario,'passed',flush=True)
            finally:
                board.terminate();board.wait(timeout=5)
if __name__=='__main__':main()
