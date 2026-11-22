#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Offline CAD-link inertia probe, not a runtime inertia feedforward or observer."""
from pathlib import Path
import argparse,json,xml.etree.ElementTree as E
import numpy as np
def probe(path,q):
    u=E.parse(path).getroot()
    def rot(v):
        a=np.linalg.norm(v)
        if a<1e-12:return np.eye(3)
        x,y,z=v/a;k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        return np.eye(3)+np.sin(a)*k+(1-np.cos(a))*(k@k)
    def origin(j):
        o=j.find("origin")
        if o is None:return np.zeros(3),np.eye(3)
        xyz=np.fromstring(o.get("xyz","0 0 0"),sep=" ");r,p,y=np.fromstring(o.get("rpy","0 0 0"),sep=" ")
        return xyz,rot(np.array([0.,0.,y]))@rot(np.array([0.,p,0.]))@rot(np.array([r,0.,0.]))
    R=np.eye(3);p=np.zeros(3);points=[];axes=[];mass=np.zeros((7,7))
    for n in range(7):
        j=u.find(f"joint[@name='Joint_{n+1}']");xyz,r=origin(j);p=p+R@xyz;R=R@r
        axis=np.fromstring(j.find("axis").get("xyz"),sep=" ");points.append(p.copy());axes.append(R@axis);R=R@rot(axis*q[n])
        a=u.find(f"link[@name='{j.find('child').get('link')}']/inertial")
        if a is None:continue
        co,ri=origin(a);com=p+R@co;v=np.zeros((3,7));w=np.zeros((3,7))
        for k in range(n+1):v[:,k]=np.cross(axes[k],com-points[k]);w[:,k]=axes[k]
        m=float(a.find("mass").get("value"));d=a.find("inertia").attrib
        inertia=np.array([[float(d["ixx"]),float(d["ixy"]),float(d["ixz"])],[float(d["ixy"]),float(d["iyy"]),float(d["iyz"])],[float(d["ixz"]),float(d["iyz"]),float(d["izz"])]])
        mass+=m*v.T@v+w.T@R@ri@inertia@ri.T@R.T@w
    xyz,_=origin(u.find("joint[@name='Joint_flange']"));tcp=p+R@xyz
    jv=np.array([np.cross(a,tcp-b) for a,b in zip(axes,points)]).T;jw=np.array(axes).T
    j=np.vstack((jv/.2,jw));matrix=j@j.T+.0004*np.eye(6);out={}
    for i,name in enumerate("XYZ"):
        t=np.zeros(6);t[i]=5;velocity=j.T@np.linalg.solve(matrix,t)
        out[name]=float(velocity@mass@velocity)
    return dict(q_rad=q,path_mass_kg=out,joint_inertia_diagonal_kg_m2=list(np.diag(mass)),
                limitation="One pose, CAD-link inertia only; excludes motor/gear reflected inertia and is not a workspace upper bound.")
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--urdf",type=Path,default=Path(__file__).resolve().parents[1]/"models/h7c/infrabot-H7C_v1.urdf")
    p.add_argument("--out",type=Path)
    a=p.parse_args();r=probe(a.urdf,[.2,.9,-.5,1.3,.3,.6,-.2]);s=json.dumps(r,indent=2)+"\n"
    if a.out:a.out.write_text(s)
    print(s)
if __name__=="__main__":main()

