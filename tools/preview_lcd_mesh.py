#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Host preview of the LCD 3D page's solid model (no hardware needed).

Parses the arrays in ui/app_lcd_mesh.c, poses them with the URDF joint
chain at the given joint angles and renders them the way app_lcd_3d.c does
(orthographic orbit camera, back-face culling, painter's sort, flat shading),
writing a PNG.  Standard library only.

  python3 tools/preview_lcd_mesh.py [--no-cull] URDF out.png [yaw_deg pitch_deg [q1..q7 rad]]
"""

import math
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET
import zlib

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
W, H = 584, 431
SCALE = 420.0
TARGET = (0.0, 0.0, 0.30)


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def rot_z(a):
    return [[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]]


def link_frames(urdf, q):
    joints = ET.parse(urdf).getroot().findall("joint")
    frames = [([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0])]
    rot, pos = frames[0]
    for i, joint in enumerate(joints):
        origin = joint.find("origin")
        xyz = [float(x) for x in origin.get("xyz").split()]
        r = rpy(*[float(x) for x in origin.get("rpy").split()])
        pos = [pos[k] + matvec(rot, xyz)[k] for k in range(3)]
        rot = matmul(rot, r)
        if joint.get("type") != "fixed":
            rot = matmul(rot, rot_z(q[i]))
        frames.append((rot, pos))
    return frames


def parse_meshes():
    text = open(os.path.join(REPO, "ui", "app_lcd_mesh.c")).read()
    meshes = []
    for name, color in re.findall(r"\{(\w+)_vertices, \w+_triangles, \w+_normals, \d+U, \d+U, 0x([0-9A-F]+)U\}", text):
        v = re.search(name + r"_vertices\[\d+\]\[3\] = \{(.*?)\};", text, re.S).group(1)
        t = re.search(name + r"_triangles\[\d+\]\[3\] = \{(.*?)\};", text, re.S).group(1)
        verts = [tuple(int(x) * 1e-4 for x in m) for m in re.findall(r"\{(-?\d+), (-?\d+), (-?\d+)\}", v)]
        tris = [tuple(int(x) for x in m) for m in re.findall(r"\{(\d+), (\d+), (\d+)\}", t)]
        meshes.append((verts, tris, int(color, 16)))
    return meshes


def write_png(path, pixels):
    raw = b"".join(b"\x00" + bytes(row) for row in pixels)
    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", W, H, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    open(path, "wb").write(png)


def main():
    cull = "--no-cull" not in sys.argv
    if not cull:
        sys.argv.remove("--no-cull")
    urdf, out = sys.argv[1], sys.argv[2]
    yaw = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else math.radians(35)
    pitch = math.radians(float(sys.argv[4])) if len(sys.argv) > 4 else math.radians(25)
    q = [float(x) for x in sys.argv[5:12]] if len(sys.argv) > 11 else [0.0] * 7
    d = (math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch))
    n = math.hypot(d[0], d[1])
    r = (-d[1] / n, d[0] / n, 0.0)
    u = (d[1] * r[2] - d[2] * r[1], d[2] * r[0] - d[0] * r[2], d[0] * r[1] - d[1] * r[0])
    light = [d[k] + 0.6 * u[k] - 0.3 * r[k] for k in range(3)]
    ln = math.sqrt(sum(x * x for x in light)); light = [x / ln for x in light]

    def project(p):
        v = [p[k] - TARGET[k] for k in range(3)]
        return (W * 0.5 + SCALE * sum(v[k] * r[k] for k in range(3)),
                H * 0.55 - SCALE * sum(v[k] * u[k] for k in range(3)),
                sum(v[k] * d[k] for k in range(3)))

    frames = link_frames(urdf, q)
    faces = []
    for (verts, tris, color), (rot, pos) in zip(parse_meshes(), frames):
        world = [[pos[k] + matvec(rot, v)[k] for k in range(3)] for v in verts]
        screen = [project(p) for p in world]
        for a, b, c in tris:
            e1 = [world[b][k] - world[a][k] for k in range(3)]
            e2 = [world[c][k] - world[a][k] for k in range(3)]
            nrm = (e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0])
            ln = math.sqrt(sum(x * x for x in nrm))
            facing = sum(nrm[k] * d[k] for k in range(3))
            if ln == 0.0 or (cull and facing <= 0.0):
                continue
            # Unculled back faces are lit as if they faced the camera.
            sign = 1.0 if facing > 0.0 else -1.0
            lit = 0.30 + 0.70 * max(0.0, sign * sum(nrm[k] * light[k] for k in range(3)) / ln)
            rgb = (((color >> 11) & 31) * 255 // 31, ((color >> 5) & 63) * 255 // 63, (color & 31) * 255 // 31)
            depth = (screen[a][2] + screen[b][2] + screen[c][2]) / 3.0
            faces.append((depth, (screen[a], screen[b], screen[c]), tuple(int(x * lit) for x in rgb)))
    faces.sort(key=lambda f: f[0])

    pixels = [[10, 12, 18] * W for _ in range(H)]
    for _, tri, rgb in faces:
        (x0, y0, _), (x1, y1, _), (x2, y2, _) = sorted(tri, key=lambda p: p[1])
        for y in range(max(0, math.ceil(y0)), min(H - 1, math.floor(y2)) + 1):
            xa = x0 + (x2 - x0) * (y - y0) / (y2 - y0) if y2 != y0 else x0
            if y < y1:
                xb = x0 + (x1 - x0) * (y - y0) / (y1 - y0) if y1 != y0 else x0
            else:
                xb = x1 + (x2 - x1) * (y - y1) / (y2 - y1) if y2 != y1 else x1
            lo, hi = sorted((xa, xb))
            row = pixels[y]
            for x in range(max(0, math.ceil(lo)), min(W - 1, math.floor(hi)) + 1):
                row[3 * x: 3 * x + 3] = rgb
    write_png(out, pixels)
    print(f"{len(faces)} front faces -> {out}")


if __name__ == "__main__":
    main()
