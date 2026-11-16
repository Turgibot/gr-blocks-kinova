"""Release repeatability: move one cube back and forth between two towers and log where each placement ends up.

    ros2 run gr_blocks_ros release_test [CYCLES] [LETTER] [OUT.csv]       default: 12 L release_test.csv
Variants are interleaved: A = 2 mm drop, fast open;  B = 2 mm drop, gradual open;  C = 0.5 mm drop, fast open.
Direction alternates (middle tower -> left tower, then back), so every variant meets both directions equally often.
"""
import csv
import json
import sys

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory  # noqa: F401  (kept for parity with other tools)
from scipy.spatial.transform import Rotation

from gr_blocks_ros.skills import CUBE, Arm

VARIANTS = {"A": dict(drop=0.002, slow_open=False), "B": dict(drop=0.002, slow_open=True), "C": dict(drop=0.0005, slow_open=False)}


def main():
    cycles = int(sys.argv[1]) if len(sys.argv) > 1 else 12
    letter = sys.argv[2] if len(sys.argv) > 2 else "L"
    out = sys.argv[3] if len(sys.argv) > 3 else "release_test.csv"
    towers = {1: ["R", "E", "T", "H"], 2: ["O", "M", "S", "L"]}  # bottom -> top, initial layout (tower 1 and 2 of blockworld.json)
    rclpy.init()
    arm = Arm()
    arm.wait_ready()
    rows = []
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["cycle", "variant", "direction", "dx_mm", "dy_mm", "dz_mm", "tilt_deg", "placed", "other_moved_mm"])
        for i in range(cycles):
            variant = "ABC"[i % 3]
            src, dst = (2, 1) if i % 2 == 0 else (1, 2)
            assert towers[src][-1] == letter
            on = towers[dst][-1]
            before = {l: arm.lookup(f"block_{l}")[0] for l in "ABEFHLMORSTW"}
            ok = arm.pick(letter) and arm.place_on(on, **VARIANTS[variant])
            if not ok:
                print(f"cycle {i}: FAILED (variant {variant}, {src}->{dst})")
                w.writerow([i, variant, f"{src}->{dst}", "", "", "", "", False, ""])
                fh.flush()
                break
            towers[src].pop()
            towers[dst].append(letter)
            e = arm.last_error
            tilt = np.degrees(np.arccos(np.clip(Rotation.from_quat(arm.lookup(f"block_{letter}")[1]).apply([0, 0, 1]) @ [0, 0, 1], -1, 1)))
            moved = max(np.linalg.norm(arm.lookup(f"block_{l}")[0] - before[l]) for l in before if l != letter)
            row = [i, variant, f"{src}->{dst}", *np.round(1000 * e, 1), round(tilt, 1), True, round(1000 * moved, 1)]
            w.writerow(row)
            fh.flush()
            rows.append(row)
            print(f"cycle {i:2d} variant {variant} {src}->{dst}: error mm {np.round(1000 * e, 1)} tilt {tilt:.1f} deg, max other-cube move {1000 * moved:.1f} mm", flush=True)
    print("summary (horizontal error = |(dx,dy)| in mm):")
    for v in "ABC":
        h = [np.hypot(r[3], r[4]) for r in rows if r[1] == v]
        if h:
            print(f"  variant {v}: n={len(h)}  mean {np.mean(h):.1f}  max {np.max(h):.1f}")
    rclpy.shutdown()
