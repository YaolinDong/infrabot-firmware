#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Decimate the arm's URDF meshes into C arrays for the LCD 3D page.

Reads the binary STL meshes the URDF references (base_link, Link_1..Link_7; optional Link_flange visual),
simplifies each by vertex clustering (vertices in one grid cell merge into
one point; degenerate and duplicate triangles are dropped) with the cell size
binary-searched to hit a per-link triangle budget, and writes
ui/app_lcd_mesh.c. At the LCD scale (495 px/m) one pixel is about 2 mm.
Clustering removes small details and can change silhouettes; inspect several
poses/views and measure complete UI frames on the board after changing budgets.

Standard library only.  Usage:
  python3 tools/gen_lcd_mesh.py /path/to/urdf/infrabot-H7C.urdf
  # Reproduce the previous, more detailed mesh for comparison:
  python3 tools/gen_lcd_mesh.py /path/to/model.urdf --profile detail --output-root /tmp/lcd-detail
"""

import argparse
import math
import os
import struct
import xml.etree.ElementTree as ET

# Per-link upper targets. The 30hz profile leaves more CPU time for camera
# dragging while the 1 kHz Cartesian controller is active. It is a display
# mesh only; neither joint geometry nor the control/IK model is simplified.
# Actual triangle counts may be below the targets as grid topology changes.
PROFILES = {
    "30hz": {"base_link": 360, "Link_1": 260, "Link_2": 720, "Link_3": 720,
             "Link_4": 720, "Link_5": 720, "Link_6": 720, "Link_7": 260, "Link_flange": 100},
    "detail": {"base_link": 580, "Link_1": 380, "Link_2": 1150, "Link_3": 1150,
               "Link_4": 1150, "Link_5": 1150, "Link_6": 1150, "Link_7": 380, "Link_flange": 160},
}
UNIT_M = 0.0001  # vertex coordinates are int16 in 0.1 mm

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_stl(path):
    data = open(path, "rb").read()
    count = struct.unpack_from("<I", data, 80)[0]
    if 84 + 50 * count != len(data):
        raise SystemExit(f"{path}: not a binary STL")
    vertices, index, triangles = [], {}, []
    for k in range(count):
        values = struct.unpack_from("<12f", data, 84 + 50 * k)
        tri = []
        for v in range(3):
            p = values[3 + 3 * v: 6 + 3 * v]
            key = (round(p[0], 7), round(p[1], 7), round(p[2], 7))
            if key not in index:
                index[key] = len(vertices)
                vertices.append(p)
            tri.append(index[key])
        triangles.append(tri)
    return vertices, triangles


def solve3(a, b):
    """Cramer's rule; None when singular."""
    det = (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
           a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
           a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    scale = max(abs(x) for row in a for x in row) or 1.0
    if abs(det) < 1e-9 * scale ** 3:
        return None
    out = []
    for col in range(3):
        m = [list(row) for row in a]
        for r in range(3):
            m[r][col] = b[r]
        out.append((m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) / det)
    return out


def face_quadrics(vertices, triangles):
    """Per vertex, the sum of the area-weighted planes of its faces (Garland-
    Heckbert quadric as A (3x3), b (3)); keeps edges and corners sharp when a
    cell's vertices merge."""
    quads = [[[0.0] * 3 for _ in range(3)] + [[0.0] * 3] for _ in vertices]
    for tri in triangles:
        p0, p1, p2 = (vertices[i] for i in tri)
        e1 = [p1[k] - p0[k] for k in range(3)]
        e2 = [p2[k] - p0[k] for k in range(3)]
        n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]]
        length = math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2])
        if length == 0.0:
            continue
        weight = length  # twice the area
        n = [x / length for x in n]
        d = -(n[0] * p0[0] + n[1] * p0[1] + n[2] * p0[2])
        for i in tri:
            q = quads[i]
            for r in range(3):
                for c in range(3):
                    q[r][c] += weight * n[r] * n[c]
                q[3][r] += -weight * n[r] * d
    return quads


def cluster(vertices, triangles, cell, quads):
    key_of, sums = [], {}
    for idx, p in enumerate(vertices):
        key = (math.floor(p[0] / cell), math.floor(p[1] / cell), math.floor(p[2] / cell))
        key_of.append(key)
        s = sums.setdefault(key, [0.0, 0.0, 0.0, 0, [[0.0] * 3 for _ in range(3)], [0.0] * 3])
        s[0] += p[0]; s[1] += p[1]; s[2] += p[2]; s[3] += 1
        q = quads[idx]
        for r in range(3):
            for c in range(3):
                s[4][r][c] += q[r][c]
            s[5][r] += q[3][r]
    new_index, out_vertices = {}, []
    out_triangles, seen = [], set()
    for tri in triangles:
        keys = [key_of[i] for i in tri]
        if keys[0] == keys[1] or keys[1] == keys[2] or keys[0] == keys[2]:
            continue
        ids = []
        for key in keys:
            if key not in new_index:
                s = sums[key]
                new_index[key] = len(out_vertices)
                mean = (s[0] / s[3], s[1] / s[3], s[2] / s[3])
                # Point closest to the merged faces' planes, within one cell
                # width of their mean (singular flat regions use the mean).
                best = solve3(s[4], s[5])
                if best is None or any(abs(best[k] - mean[k]) > cell for k in range(3)):
                    best = mean
                out_vertices.append(tuple(best))
            ids.append(new_index[key])
        signature = tuple(sorted(ids))
        if signature in seen:
            continue
        seen.add(signature)
        out_triangles.append(ids)
    return out_vertices, out_triangles


def decimate(vertices, triangles, budget):
    if len(triangles) <= budget:
        return vertices, triangles, 0.0
    quads = face_quadrics(vertices, triangles)
    low, high = 0.0002, 0.05
    best = None
    for _ in range(14):
        cell = math.sqrt(low * high)
        v, t = cluster(vertices, triangles, cell, quads)
        if len(t) > budget:
            low = cell
        else:
            high = cell
            best = (v, t, cell)
    if best is None:
        best = cluster(vertices, triangles, high, quads) + (high,)
    return best


def rgb565(rgba):
    r, g, b = (max(0, min(255, round(float(c) * 255))) for c in rgba.split()[:3])
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("urdf", help="source URDF; its package must contain meshes/")
    parser.add_argument("--profile", choices=PROFILES, default="30hz")
    parser.add_argument("--output-root", default=REPO,
                        help="write ui/ beneath this directory")
    args = parser.parse_args()
    budget = PROFILES[args.profile]
    urdf = os.path.abspath(args.urdf)
    out_c = os.path.join(args.output_root, "ui", "app_lcd_mesh.c")
    out_h = os.path.join(args.output_root, "ui", "app_lcd_mesh.h")
    root = ET.parse(urdf).getroot()
    # ROS package: urdf/ and meshes/ are siblings. The standalone SDK keeps
    # its meshes/ next to the URDF for its Qt viewer.
    urdf_dir = os.path.dirname(urdf)
    mesh_dir = os.path.join(urdf_dir, "meshes")
    if not os.path.isdir(mesh_dir):
        mesh_dir = os.path.join(os.path.dirname(urdf_dir), "meshes")
    meshes = []
    links = {link.get("name"): link for link in root.findall("link")}
    if set(links) != set(budget):
        raise ValueError("Expected base, seven links and a fixed flange")
    for name in budget:  # frame order used by ArmIk_LinkFrames, independent of XML order
        link = links[name]
        visual = link.find("visual")
        if visual is None:
            if name == "Link_flange":
                continue  # terminal TCP can be a frame without any physical mesh
            raise ValueError(f"{name}: missing arm visual")
        origin = visual.find("origin")
        if (origin is not None and any(float(x) != 0.0 for x in
                                       (origin.get("xyz", "0 0 0") + " " + origin.get("rpy", "0 0 0")).split())):
            raise SystemExit(f"{name}: non-identity visual origin is not supported")
        filename = visual.find("geometry/mesh").get("filename")
        path = os.path.join(mesh_dir, os.path.basename(filename))
        color = visual.find("material/color")
        vertices, triangles = read_stl(path)
        v, t, cell = decimate(vertices, triangles, budget[name])
        meshes.append((name, v, t, rgb565(color.get("rgba")) if color is not None else 0xC618))
        print(f"{name}: {len(triangles)} -> {len(t)} triangles, {len(v)} vertices, cell {cell * 1000:.1f} mm")

    max_vertices = max(len(v) for _, v, _, _ in meshes)
    total_triangles = sum(len(t) for _, _, t, _ in meshes)
    total_vertices = sum(len(v) for _, v, _, _ in meshes)
    os.makedirs(os.path.dirname(out_h), exist_ok=True)
    os.makedirs(os.path.dirname(out_c), exist_ok=True)
    with open(out_h, "w") as h:
        h.write("""/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_LCD_MESH_H
#define APP_LCD_MESH_H

#include <stdint.h>

/* Decimated URDF visual meshes for the LCD 3D page, generated by
 * tools/gen_lcd_mesh.py -- do not edit.  Mesh 0 is base_link (base frame),
 * meshes 1..7 are Link_i after joint i turns, a flange visual, if present, is last. Vertices are
 * int16 in APP_LCD_MESH_UNIT_M; the decimation does not keep a reliable
 * winding, so render both sides (no back-face culling). */
#define APP_LCD_MESH_COUNT           %dU
#define APP_LCD_MESH_UNIT_M          %gf
#define APP_LCD_MESH_MAX_VERTICES    %dU
#define APP_LCD_MESH_TOTAL_TRIANGLES %dU
#define APP_LCD_MESH_TOTAL_VERTICES  %dU

typedef struct
{
  const int16_t (*vertices)[3];
  const uint16_t (*triangles)[3];
  const int8_t (*normals)[3];   /* unit face normal * 127, link frame */
  uint16_t vertex_count;
  uint16_t triangle_count;
  uint16_t color;  /* RGB565 from the URDF material */
} AppLcdMesh;

extern const AppLcdMesh app_lcd_meshes[APP_LCD_MESH_COUNT];

#endif /* APP_LCD_MESH_H */
""" % (len(meshes), UNIT_M, max_vertices, total_triangles, total_vertices))

    with open(out_c, "w") as c:
        c.write('/* SPDX-License-Identifier: Apache-2.0 */\n/* Copyright (c) 2026 Yaolin Dong */\n')
        c.write('#include "app_lcd_mesh.h"\n\n')
        c.write("/* Generated by tools/gen_lcd_mesh.py from %s\n"
                " * Profile: %s -- do not edit. */\n\n"
                % (os.path.basename(urdf), args.profile))
        for name, v, t, _ in meshes:
            c.write(f"static const int16_t {name}_vertices[{len(v)}][3] = {{\n")
            for p in v:
                c.write("  {%d, %d, %d},\n" % tuple(round(x / UNIT_M) for x in p))
            c.write("};\n\n")
            c.write(f"static const uint16_t {name}_triangles[{len(t)}][3] = {{\n")
            for tri in t:
                c.write("  {%d, %d, %d},\n" % tuple(tri))
            c.write("};\n\n")
            c.write(f"static const int8_t {name}_normals[{len(t)}][3] = {{\n")
            for tri in t:
                q = [tuple(round(x / UNIT_M) for x in v[i]) for i in tri]
                e1 = [q[1][k] - q[0][k] for k in range(3)]
                e2 = [q[2][k] - q[0][k] for k in range(3)]
                n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]]
                ln = math.sqrt(sum(x * x for x in n)) or 1.0
                c.write("  {%d, %d, %d},\n" % tuple(round(127 * x / ln) for x in n))
            c.write("};\n\n")
        c.write("const AppLcdMesh app_lcd_meshes[APP_LCD_MESH_COUNT] = {\n")
        for name, v, t, color in meshes:
            c.write(f"  {{{name}_vertices, {name}_triangles, {name}_normals, {len(v)}U, {len(t)}U, 0x{color:04X}U}},\n")
        c.write("};\n")
    print(f"profile {args.profile}: {total_triangles} triangles, {total_vertices} vertices, "
          f"max {max_vertices} vertices per mesh -> {out_c}")


if __name__ == "__main__":
    main()
