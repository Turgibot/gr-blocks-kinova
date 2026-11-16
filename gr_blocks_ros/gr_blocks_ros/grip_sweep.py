"""Sweep the gripper close command: for each value pick a cube, hold it still, log the slip, then place it on the other tower.

    ros2 run gr_blocks_ros grip_sweep [LETTER] [HOLD_S] [REPEATS] [CLOSE ...]     default: L 10 2 0.46 0.50 0.54 0.58
Values are interleaved (v1 v2 v3 v4 v1 v2 ...); the cube alternates between the middle and the left tower.
"""
import sys

import numpy as np
import rclpy

from gr_blocks_ros.skills import Arm


def main():
    letter = sys.argv[1] if len(sys.argv) > 1 else "L"
    hold = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    repeats = int(sys.argv[3]) if len(sys.argv) > 3 else 2
    values = [float(v) for v in sys.argv[4:]] or [0.46, 0.50, 0.54, 0.58]
    towers = {1: ["R", "E", "T", "H"], 2: ["O", "M", "S", "L"]}
    rclpy.init()
    arm = Arm()
    arm.wait_ready()
    rows = []
    for i in range(repeats * len(values)):
        close = values[i % len(values)]
        src, dst = (2, 1) if i % 2 == 0 else (1, 2)
        on = towers[dst][-1]
        if not arm.pick(letter, close=close):
            print(f"cycle {i}: pick FAILED (close {close})")
            break
        finger = arm.finger()
        tcp0, _ = arm.tcp_now()
        rel0 = tcp0 - arm.block(letter)
        arm.settle(hold)
        tcp1, _ = arm.tcp_now()
        slip = (tcp1 - arm.block(letter)) - rel0
        print("gripper joints after hold:", {k: round(v, 4) for k, v in sorted(arm.js.items()) if "finger" in k or "knuckle" in k}, flush=True)
        ok = arm.place_on(on)
        if not ok:
            print(f"cycle {i}: place FAILED (close {close}, slip {np.round(1000 * slip, 1)} mm)")
            break
        towers[src].pop()
        towers[dst].append(letter)
        e = arm.last_error
        rows.append((close, finger, slip, e))
        print(f"cycle {i:2d} close {close:.2f} finger {finger:.3f}: slip in {hold}s hold (mm) {np.round(1000 * slip, 1)}  "
              f"placement error (mm) {np.round(1000 * e, 1)}", flush=True)
    print("summary per close command: |slip| after hold, horizontal placement error (mm)")
    for v in values:
        r = [x for x in rows if x[0] == v]
        if r:
            print(f"  close {v:.2f}: n={len(r)}  |slip| mean {np.mean([np.linalg.norm(x[2]) for x in r]) * 1000:.1f}  "
                  f"placement err mean {np.mean([np.hypot(x[3][0], x[3][1]) for x in r]) * 1000:.1f} max {np.max([np.hypot(x[3][0], x[3][1]) for x in r]) * 1000:.1f}")
    rclpy.shutdown()
