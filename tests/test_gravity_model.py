# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Compare the actual float32 MCU C++ core against dynparam (no robot IO)."""
import ctypes
from pathlib import Path
import subprocess
import tempfile
import numpy as np
from dynparam.runtime.common import Dynamics
from dynparam.export import friction_torque, joint_bias, load_friction
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
urdf = root / "models/h7c/infrabot-H7C_v1.urdf"
friction = urdf.with_name(urdf.stem + "_friction.yaml")
dyn = Dynamics(urdf, "Link_flange")
kind, joints = load_friction(friction, dyn.robot.joint_names)
tcp_local = np.fromstring(ET.parse(urdf).find("joint[@name='Joint_flange']/origin").get("xyz"),sep=" ")
lo = np.array([j.lower for j in dyn.robot.config.links]); hi = np.array([j.upper for j in dyn.robot.config.links])
rng = np.random.default_rng(724)
q = np.vstack([np.zeros(7), lo, hi, rng.uniform(lo-.04, hi+.04, (500,7))]).astype(np.float32)
dq = rng.uniform(-1,1,q.shape).astype(np.float32)
# As in the PC gravity controller: static bias is always part of the holding
# torque, whereas only the velocity-dependent part is scaled and gated.
base = dyn.gravity(q) + joint_bias(kind, joints, q)
base += 20*(np.maximum(lo-q,0)-np.maximum(q-hi,0))
moving = friction_torque(kind, joints, dq, velocity_eps=.05, include_offset=False)
def friction_at(deadband):
    return moving * np.clip((np.abs(dq)-deadband)/deadband, 0, 1)

with tempfile.TemporaryDirectory() as tmp:
    lib = Path(tmp)/"gravity.so"
    subprocess.run(["c++","-std=c++17","-O2","-shared","-fPIC","-fno-exceptions","-fno-rtti",
                    "-I"+str(root/"control"),str(root/"control/arm_gravity.cpp"),"-o",str(lib)],check=True)
    core=ctypes.CDLL(str(lib))
    vector=np.ctypeslib.ndpointer(dtype=np.float32,ndim=1,flags="C_CONTIGUOUS")
    core.ArmGravity_Evaluate.argtypes=[vector,vector,ctypes.c_float,vector,vector]
    core.ArmGravity_Evaluate.restype=ctypes.c_int
    core.ArmGravity_ModelHash.restype=ctypes.c_char_p
    import hashlib
    assert hashlib.sha256(urdf.read_bytes() + friction.read_bytes()).hexdigest() == "69a9f237dae3e1e34c889511c8d5a70d0e975c59f909357871576fbe5bbb1f66"
    assert core.ArmGravity_ModelHash().decode() == "157078479caa4648dda4e07e3892a70aa4acccda68edc93e2da87e35cf16b29a"
    maximum=0.
    for scale in (0.,.7,1.):
        expected=base + scale*friction_at(.15)
        actual=[]
        for qi,vi in zip(q,dq):
            tau=np.zeros(7,np.float32); twist=np.zeros(6,np.float32)
            assert core.ArmGravity_Evaluate(qi,vi,scale,tau,twist)==1
            actual.append(tau)
            rot,origin,axis=dyn.robot._joint_geometry(qi)
            tcp=origin[-1]+rot[-1]@tcp_local
            expected_twist=np.r_[sum(np.cross(a,tcp-p)*v for a,p,v in zip(axis,origin,vi)),
                                  sum(a*v for a,v in zip(axis,vi))]
            np.testing.assert_allclose(twist,expected_twist,atol=3e-6,rtol=1e-5)
        error=float(np.max(np.abs(np.array(actual)-expected))); maximum=max(maximum,error)
        assert error < 3e-5, error
    core.ArmGravity_EvaluateTuned.argtypes=[vector,vector,ctypes.c_float,ctypes.c_float,vector,vector]
    core.ArmGravity_EvaluateTuned.restype=ctypes.c_int
    core.ArmGravity_EvaluateParts.argtypes=[vector,vector,ctypes.c_float,ctypes.c_float,vector,vector,vector]
    core.ArmGravity_EvaluateParts.restype=ctypes.c_int
    for deadband in (.02,.15,.5):
        expected_friction=friction_at(deadband)
        expected=base+expected_friction
        for index,(qi,vi) in enumerate(zip(q,dq)):
            tau=np.zeros(7,np.float32); twist=np.zeros(6,np.float32)
            assert core.ArmGravity_EvaluateTuned(qi,vi,1.,deadband,tau,twist)==1
            np.testing.assert_allclose(tau,expected[index],atol=3e-5,rtol=1e-5)
            parts=np.zeros(7,np.float32); total=np.zeros(7,np.float32)
            assert core.ArmGravity_EvaluateParts(qi,vi,1.,deadband,total,twist,parts)==1
            np.testing.assert_array_equal(total,tau)
            np.testing.assert_allclose(parts,expected_friction[index],atol=3e-6,rtol=1e-5)
    for invalid_deadband in (0.,-.1,.51,float("nan")):
        assert core.ArmGravity_EvaluateTuned(q[0],dq[0],1.,invalid_deadband,
                                           np.zeros(7,np.float32),np.zeros(6,np.float32))==0
    print(f"Tunable friction deadband: {len(q)*3} cases and invalid input rejection OK")
    invalid=q[0].copy();invalid[3]=np.nan
    assert core.ArmGravity_Evaluate(invalid,dq[0],0,np.zeros(7,np.float32),np.zeros(6,np.float32))==0
    print(f"MCU float32 vs Python: {len(q)*3} torque cases, max error {maximum:.8f} Nm; twists/hash/NaN OK")
