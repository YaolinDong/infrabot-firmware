# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Ed25519 manifests (RFC 8032), using the system OpenSSL 3 CLI."""
from pathlib import Path
import os
import subprocess
import tempfile

SPKI = bytes.fromhex('302a300506032b6570032100')

def public_key(private):
    der = subprocess.check_output(['openssl','pkey','-in',str(private),'-pubout','-outform','DER'])
    if len(der)!=len(SPKI)+32 or not der.startswith(SPKI):
        raise ValueError('Expected an Ed25519 private key')
    return der[len(SPKI):]

def generate(private):
    private.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
    with os.fdopen(os.open(private,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600),'wb') as f:
        subprocess.run(['openssl','genpkey','-algorithm','ED25519'],stdout=f,check=True)
    return public_key(private)

def sign(private, manifest):
    with tempfile.TemporaryDirectory() as d:
        msg=Path(d)/'manifest';msg.write_bytes(manifest)
        return subprocess.check_output(['openssl','pkeyutl','-sign','-rawin','-inkey',str(private),'-in',str(msg)])

def verify(public, manifest, signature):
    if len(public)!=32 or len(signature)!=64:return False
    with tempfile.TemporaryDirectory() as d:
        d=Path(d);(d/'public').write_bytes(SPKI+public);(d/'manifest').write_bytes(manifest);(d/'sig').write_bytes(signature)
        return subprocess.run(['openssl','pkeyutl','-verify','-rawin','-pubin','-keyform','DER','-inkey',str(d/'public'),
            '-in',str(d/'manifest'),'-sigfile',str(d/'sig')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    keygen = commands.add_parser("generate", help="create a private PEM and print the raw public key in hex")
    keygen.add_argument("--key", type=Path, required=True)
    args = parser.parse_args()
    print(generate(args.key).hex())
