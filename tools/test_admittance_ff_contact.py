#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Run v11's contact and return regressions with the companion H7C plant (no board IO)."""
import argparse
import csv
import itertools
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import re
import statistics
import subprocess

ROOT = Path(__file__).resolve().parents[1]
POINTS = {
    "elbow_y": (3, (0, 1, 0)), "elbow_in_plane": (3, (.46, 0, .89)),
    "upper_z": (2, (0, 0, 1)), "forearm_y": (5, (0, 1, 0)),
    "forearm_z": (5, (0, 0, 1)), "tcp_y": (8, (0, 1, 0)), "tcp_z": (8, (0, 0, 1)),
}
HOLD_POINTS = ["elbow_y", "elbow_in_plane", "forearm_y", "forearm_z", "tcp_y"]


def cases():
    for width, mode in itertools.product((.02, .10), ("CART", "JOINT")):
        for point, k, delay, tremor in itertools.product(HOLD_POINTS, (1000, 4000, 10000), (2, 4, 6), (0, .4)):
            yield "hold", width, mode, point, k, delay, tremor, .05, 0
        for point, k, travel, delay, mass in itertools.product(POINTS, (1000, 4000, 10000, 30000), (.03, .06), (2, 4), (0, 1.5)):
            yield "contact", width, mode, point, k, delay, 0, travel, mass


def invoke(exe, mode, *args):
    cmd = [exe, *map(str, args)]
    if mode == "JOINT":
        cmd.append("joint")
    return subprocess.check_output(cmd, text=True).strip()


def contact(exe, case):
    grid, width, mode, point, k, delay, tremor, travel, mass = case
    link, direction = POINTS[point]
    fields = invoke(exe, mode, "tsv", "link", link, "force", *direction,
                    "hand_k", k, "hand_d", 40, "travel", travel, "delay", delay,
                    "hand_m", mass, "tremor", tremor, "ff_width", width,
                    "hold", 4, "settle", 2.5, "seconds", 6.5).split()
    fault, amp, _, ripple, rev, _, _, _, ff, _ = map(float, fields)
    return dict(grid=grid, width=width, mode=mode, point=point, k=k, delay_ms=delay,
                tremor_mm=tremor, travel=travel, hand_mass=mass, fault=int(fault),
                amp_mrad_s=amp, ripple_mm=ripple, rev_s=rev, ff_step_mNm=ff)


def save(path, rows):
    with path.open("w", newline="") as stream:
        w = csv.DictWriter(stream, fieldnames=rows[0], lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def releases(exe):
    for width, mode, (name, force) in itertools.product(
            (.02, .10), ("CART", "JOINT"), (("side", (0, 15, 0)), ("down", (0, 0, -12)))):
        output = invoke(exe, mode, "quiet", "release", "link", 8, "force", *force,
                        "down", .05, "seconds", 9, "ff_width", width)
        values = re.search(r"line_dev=([\d.]+)mm.*settle\(2mm\)=([\d.]+)s final=([\d.]+)mm/([\d.]+)mrad", output)
        if not values or "fault=0 " not in output:
            raise RuntimeError(output)
        dev, settle, final, rotation = map(float, values.groups())
        yield dict(width=width, mode=mode, push=name, dev_mm=dev, settle_s=settle,
                   final_mm=final, final_mrad=rotation)


def drag_cases(exe, out):
    """Compare contact-point phase lag and per-joint velocity-band RMS at matched settings."""
    import ctypes
    import numpy as np
    import tempfile

    lib = out / "ff_contact_fk.so"
    subprocess.run(["cc", "-std=c11", "-O2", "-shared", "-fPIC", "-Icontrol",
                    "control/infrabot_ik_mcu.c", "-lm", "-o", lib], cwd=ROOT, check=True)
    fk = ctypes.CDLL(str(lib)).ArmIk_LinkFrames
    fk.argtypes = [np.ctypeslib.ndpointer(np.float32, flags="C_CONTIGUOUS")] * 3
    fk.restype = None
    rotations, points = np.empty((9, 3, 3), "f"), np.empty((9, 3), "f")
    with tempfile.TemporaryDirectory() as tmp:
        trace = Path(tmp) / "drag.csv"
        for width, mode, link, hz in itertools.product((.02, .10), ("CART", "JOINT"), (3, 5, 8), (.5, 1)):
            output = invoke(exe, mode, "quiet", "link", link, "force", 0, 1, 0, "ff_width", width,
                            "sine", hz, "hand_k", 4000, "hand_d", 40, "travel", .04,
                            "hold", 8, "seconds", 10, "csv", trace)
            if "fault=0 " not in output:
                raise RuntimeError(output)
            data = np.genfromtxt(trace, delimiter=",", names=True)
            data = data[(data["t"] >= 3) & (data["t"] < 9)]
            q = np.stack([data[f"q{i}"] for i in range(1, 8)], axis=1).astype("f")
            y = np.empty(len(q))
            for i, sample in enumerate(q):
                fk(sample, rotations, points)
                y[i] = points[link, 1]
            # Plant q is recorded at the end of its 1 ms step. Grip starts at t=1.
            phase = 2*np.pi*hz*(data["t"]+.001-1.)
            basis = np.column_stack((np.sin(phase), np.cos(phase), np.ones(len(phase))))
            sine, cosine, _ = np.linalg.lstsq(basis, y, rcond=None)[0]
            lag = -1000*np.arctan2(cosine, sine)/(2*np.pi*hz)
            v = np.stack([data[f"v{i}"] for i in range(1, 8)], axis=1)
            freq = np.fft.rfftfreq(len(v), .001)
            fft = np.fft.rfft(v, axis=0)/len(v)
            rms = 1000*np.sqrt(2*np.sum(abs(fft[(freq >= 2) & (freq <= 20)])**2, axis=0))
            yield dict(width=width, mode=mode, link=link, hz=hz, worst_joint=int(np.argmax(rms)+1),
                       rms_mrad_s=round(rms.max(), 3), lag_ms=round(lag, 3))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim-dir", type=Path, help="optional separately supplied simulator directory containing sim.c")
    parser.add_argument("--out", type=Path, default=ROOT / "build/contact")
    parser.add_argument("--jobs", type=int, default=4)
    args = parser.parse_args()
    if args.sim_dir is None or not (args.sim_dir / "sim.c").is_file():
        print("SKIP: optional external simulator unavailable; supply --sim-dir to run this sweep")
        return
    args.out.mkdir(parents=True, exist_ok=True)
    exe = (args.out / "ff_contact_plant").resolve()
    subprocess.run(["cc", "-std=c11", "-O2", "-Icontrol", args.sim_dir.resolve() / "sim.c",
                    "control/arm_admittance.c", "control/infrabot_ik_mcu.c", "-lm",
                    "-o", exe], cwd=ROOT, check=True)
    with ThreadPoolExecutor(args.jobs) as pool:
        rows = list(pool.map(lambda c: contact(exe, c), cases()))
    save(args.out / "contact.csv", rows)
    for mode, width in itertools.product(("CART", "JOINT"), (.02, .10)):
        group = [r for r in rows if r["mode"] == mode and r["width"] == width]
        hold = [r for r in group if r["grid"] == "hold"]
        bad_hold = [r for r in hold if r["rev_s"] > 3 or r["fault"]]
        bad = [r for r in group if r["grid"] == "contact" and
               (r["fault"] or r["amp_mrad_s"] > 60 or r["ripple_mm"] > 1)]
        print(mode, width, "hold failures", len(bad_hold), "/90; worst rev/s",
              max(r["rev_s"] for r in hold), "FF step median/max mNm",
              statistics.median(r["ff_step_mNm"] for r in hold), max(r["ff_step_mNm"] for r in hold),
              "contact failures", len(bad), "/224", flush=True)
        if mode == "CART" and width == .10:
            assert len(bad) == 4 and all(r["fault"] == 4 and r["point"] == "forearm_z"
                                         and r["k"] == 30000 and r["travel"] == .06 for r in bad)
            assert len(bad_hold) <= 6 and all(not r["fault"] and r["point"] == "tcp_y"
                                               and r["tremor_mm"] == .4 for r in bad_hold)
    drag = list(drag_cases(exe, args.out.resolve()))
    save(args.out / "drag.csv", drag)
    # Published fixtures lock RMS and lag separately, including the known
    # slow-TCP RMS increase; neither metric is a substitute for the other.
    with (ROOT / "tests/admittance_plant/ff_drag_reference.csv").open() as stream:
        expected = {(float(r["width"]), r["mode"], int(r["link"]), float(r["hz"])): r
                    for r in csv.DictReader(stream)}
    assert len(expected) == len(drag)
    for r in drag:
        ref = expected[r["width"], r["mode"], r["link"], r["hz"]]
        assert abs(r["rms_mrad_s"]-float(ref["rms_mrad_s"])) <= 1., r
        assert abs(r["lag_ms"]-float(ref["lag_ms"])) <= .5, r
    result = list(releases(exe))
    save(args.out / "release.csv", result)
    for r in result:
        if r["mode"] != "CART":
            continue
        # Rounded model outputs; allow small floating-point/platform variations.
        expected = {
            (.02, "side"): (3.58, 1.16, 1.37), (.10, "side"): (3.60, 1.20, 1.70),
            (.02, "down"): (.70, 1.07, 1.00),  (.10, "down"): (.73, 1.11, .90),
        }[r["width"], r["push"]]
        for key, target, tol in zip(("dev_mm", "settle_s", "final_mm"), expected, (.1, .05, .15)):
            assert abs(r[key] - target) <= tol, r


if __name__ == "__main__":
    main()
