#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Generate MCU or InfraBot SDK joint tables. No runtime XML/ROS dependencies."""
import argparse
from math import sin, cos, isfinite
from pathlib import Path
import xml.etree.ElementTree as ET


def emit(urdf, sdk):
    robot = ET.parse(urdf).getroot()
    joints = {j.attrib["name"]: j for j in robot.findall("joint")}
    names = [f"Joint_{i}" for i in range(1, 8)] + ["Joint_flange"]
    links = ["base_link"] + [f"Link_{i}" for i in range(1, 8)] + ["Link_flange"]
    if set(joints) != set(names):
        raise ValueError("Expected seven arm joints and a fixed flange")

    def scalar(v):
        if not isfinite(float(v)):
            raise ValueError("non-finite joint parameter")
        return format(float(v), ".17g") if sdk else format(float(v), ".9e") + "f"

    def vec(v):
        return "{" + ", ".join(map(scalar, v)) + "}"

    rows, limits = [], [[], []]
    for i, name in enumerate(names):
        j = joints[name]
        if (j.find("parent").get("link"), j.find("child").get("link"), j.get("type")) != (
                links[i], links[i+1], "fixed" if i == 7 else "revolute"):
            raise ValueError(f"Unexpected chain at {name}")
        origin = j.find("origin")
        xyz = list(map(float, origin.attrib["xyz"].split()))
        rpy = list(map(float, origin.attrib["rpy"].split()))
        if i == 7:
            break
        axis = j.find("axis").attrib["xyz"].split()
        lm = j.find("limit").attrib
        for arr, key in zip(limits, ["lower", "upper"]):
            arr.append(lm[key])
        fields = ([f'"{name}"'] if sdk else []) + [vec(xyz), vec(rpy), vec(axis)] + [
            scalar(lm[k]) for k in ("lower","upper","velocity")]
        rows.append("  {" + ", ".join(fields) + "},")
    cr,cp,cy = map(cos,rpy)
    sr,sp,sy = map(sin,rpy)
    R = [(cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
         (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),(-sp,cp*sr,cp*cr)]
    out = ["/* SPDX-License-Identifier: Apache-2.0 */", "/* Copyright (c) 2026 Yaolin Dong */",
           "// H7C: generated from URDF by export_kinematics.py."]
    if sdk:
        out += ['#pragma once', '#include "infrabot_ik/math.hpp"', 'namespace infrabot_ik {',
                'struct JointSpec { const char* name; Vec3 origin_xyz, origin_rpy, axis; double lower, upper, velocity; };',
                'inline constexpr JointSpec kJointSpecs[7] = {', *rows, '};',
                'inline constexpr Vec3 kFlangePosition = '+vec(xyz)+';',
                'inline constexpr Vec3 kFlangeRpy = '+vec(rpy)+';',
                'inline constexpr int kMotorDirection[7]={1,-1,1,-1,1,1,1};', '}  // namespace infrabot_ik']
    else:
        out += ['static const JointSpec k_joint_specs[ARM_IK_AXES] = {', *rows, '};',
                'static const Pose k_flange = {'+vec(xyz)+', {{'+', '.join(vec(row) for row in R)+'}}};',
                'static const float k_joint_lower[ARM_IK_AXES] = '+vec(limits[0])+';',
                'static const float k_joint_upper[ARM_IK_AXES] = '+vec(limits[1])+';']
    return '\n'.join(out)+'\n'


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--sdk", action="store_true")
    args = parser.parse_args()
    args.output.write_text(emit(args.urdf, args.sdk))
