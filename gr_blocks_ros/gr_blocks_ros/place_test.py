"""Pick-and-place test: pick HELD, put it on top of ON, check where it ended up.

    ros2 run gr_blocks_ros place_test [HELD] [ON]        default: L H
PASS: cube centre within 6 mm in x/y and 4 mm in z of 'on top of ON', tilt < 10 deg, no other cube moved > 5 mm.
"""
import sys

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

from gr_blocks_ros.skills import CUBE, Arm

LETTERS = "ABEFHLMORSTW"


def main():
    held = sys.argv[1] if len(sys.argv) > 1 else "L"
    on = sys.argv[2] if len(sys.argv) > 2 else "H"
    rclpy.init()
    arm = Arm()
    arm.wait_ready()
    snap = {l: arm.lookup(f"block_{l}") for l in LETTERS}
    target = snap[on][0]
    expected = target + np.array([0, 0, CUBE])
    print(f"placing block_{held} on block_{on} (top at z={target[2] + CUBE / 2:.4f}); expected centre {np.round(expected, 4)}")
    ok = arm.pick(held)
    print("[pick]", "ok" if ok else "FAILED")
    ok = ok and arm.place_on(on)
    print("[place]", "ok" if ok else "FAILED")
    arm.settle(2.0)
    p, q = arm.lookup(f"block_{held}")
    err = p - expected
    tilt = np.degrees(np.arccos(np.clip(Rotation.from_quat(q).apply([0, 0, 1]) @ [0, 0, 1], -1, 1)))
    print(f"block_{held} error vs expected (mm): {np.round(1000 * err, 1)}, tilt {tilt:.1f} deg")
    others = {l: round(1000 * np.linalg.norm(arm.lookup(f'block_{l}')[0] - snap[l][0]), 1) for l in LETTERS if l != held}
    bad = {l: m for l, m in others.items() if m > 5}
    print("other cubes displaced > 5 mm:", bad or "none")
    good = ok and np.hypot(err[0], err[1]) < 0.006 and abs(err[2]) < 0.004 and tilt < 10 and not bad
    print("PLACE_TEST", "PASS" if good else "FAIL")
    rclpy.shutdown()
    sys.exit(0 if good else 3)
