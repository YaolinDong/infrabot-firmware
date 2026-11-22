# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Compare compiled MCU (and optional SDK) poses/Jacobians against the H7C XML.

Run: python tests/test_h7c_kinematics.py [--sdk /path/to/infrabot-sdk]
Requires numpy, scipy, a C compiler and (for --sdk) a C++ compiler.
"""
import argparse
import ctypes as C
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

root = Path(__file__).resolve().parents[1]
urdf = root / "models/h7c/infrabot-H7C_v1.urdf"
joints = ET.parse(urdf).findall("joint")
assert [j.find("child").get("link") for j in joints] == [*(f"Link_{i}" for i in range(1,8)), "Link_flange"]
def transform(r, t):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r, t
    return m
def vector(node, key):
    return np.fromstring(node.get(key), sep=" ")
def fk(q):
    frames = [np.eye(4)]
    for i, joint in enumerate(joints):
        o = joint.find("origin")
        m = frames[-1] @ transform(Rotation.from_euler("xyz", vector(o, "rpy")).as_matrix(), vector(o, "xyz"))
        if joint.get("type") == "revolute":
            m = m @ transform(Rotation.from_rotvec(vector(joint.find("axis"), "xyz") * q[i]).as_matrix(), np.zeros(3))
        frames.append(m)
    return np.asarray(frames)

def reference_jac(q):
    h = 1e-6
    result = np.empty((6,7))
    for i, step in enumerate(h*np.eye(7)):
        a, b = fk(q+step)[-1], fk(q-step)[-1]
        result[:,i] = np.r_[a[:3,3]-b[:3,3], Rotation.from_matrix(a[:3,:3] @ b[:3,:3].T).as_rotvec()] / (2*h)
    return result

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--sdk", type=Path)
sdk = parser.parse_args().sdk
with tempfile.TemporaryDirectory() as directory:
    tmp = Path(directory)
    subprocess.run(["cc","-O2","-std=c11","-fPIC","-shared",f"-I{root}/control",
                    root/"control/infrabot_ik_mcu.c","-lm","-o",tmp/"arm.so"],check=True)
    mcu = C.CDLL(str(tmp/"arm.so"))
    f = np.ctypeslib.ndpointer(dtype=np.float32,flags="C_CONTIGUOUS")
    mcu.ArmIk_Forward.argtypes = [f,f]
    mcu.ArmIk_Jacobian.argtypes = [f,f,f]
    mcu.ArmIk_LinkFrames.argtypes = [f,f,f]
    if sdk:
        vendor = sdk.resolve()/"third_party/infrabot_ik"
        if not vendor.is_dir():  # SDK checkouts before the directory migration
            vendor = sdk.resolve()/"vendor/infrabot_ik"
            assert (vendor/"urdf/identified/infrabot-H7C_v1.urdf").read_bytes() == urdf.read_bytes()
        # New SDKs embed geometry in generated_model.hpp; compare the compiled
        # solver's FK and Jacobian to this URDF over the full pose set below.
        (tmp/"wrapper.cpp").write_text("""
#include "infrabot_ik/kinematics.hpp"
extern "C" void sample(const double* in, double* pose, double* jac) {
  infrabot_ik::JointVector q; std::copy_n(in,7,q.begin());
  infrabot_ik::Kinematics k; auto p=k.forward(q); auto j=k.jacobian(q);
  pose[0]=p.position.x;pose[1]=p.position.y;pose[2]=p.position.z;
  for(int r=0;r<3;r++){
    for(int c=0;c<3;c++)pose[3+r*3+c]=p.rotation.m[r][c];
    for(int c=0;c<7;c++){jac[r*7+c]=j.linear[r][c];jac[(r+3)*7+c]=j.angular[r][c];}
  }
}
""")
        subprocess.run(["c++","-std=c++17","-fPIC","-shared",f"-I{vendor}/include",vendor/"src/kinematics.cpp",
                        tmp/"wrapper.cpp","-o",tmp/"sdk.so"],check=True)
        host = C.CDLL(str(tmp/"sdk.so"))
        host.sample.argtypes = [np.ctypeslib.ndpointer(dtype=np.float64,flags="C_CONTIGUOUS")]*3
    lo, hi = np.array([[float(j.find("limit").get(key)) for j in joints[:7]] for key in ("lower","upper")])
    positions = [np.zeros(7),lo,hi,*np.random.default_rng(1104).uniform(lo,hi,(60,7))]
    for q in positions:
        q32 = q.astype(np.float32)
        frames = fk(q)
        pose = np.r_[frames[-1,:3,3],frames[-1,:3,:3].ravel()]
        expected = reference_jac(q)
        meas, jac = np.empty(12,np.float32), np.empty((6,7),np.float32)
        r, pos = np.empty((9,3,3),np.float32),np.empty((9,3),np.float32)
        mcu.ArmIk_LinkFrames(q32,r,pos)
        mcu.ArmIk_Forward(q32,meas)
        mcu.ArmIk_Jacobian(q32,jac[:3],jac[3:])
        for actual, ref in [(meas,pose),(jac,expected),(r,frames[:,:3,:3]),(pos,frames[:,:3,3])]:
            np.testing.assert_allclose(actual, ref, atol=8e-7,rtol=0)
        if sdk:
            sp, sj = np.empty(12),np.empty((6,7))
            host.sample(q,sp,sj)
            np.testing.assert_allclose(sp,pose,atol=2e-12,rtol=0)
            np.testing.assert_allclose(sj,expected,atol=8e-10,rtol=0)
    print("H7C zero flange XYZ (mm):", np.round(fk(positions[0])[-1,:3,3]*1000,3))
    print(len(positions),"poses passed: URDF, all MCU frames, flange FK and finite-difference Jacobian",
          "(including SDK)" if sdk else "")
