"""First pick: approach a cube from above, close the gripper, lift, check that the cube came along.

    ros2 run gr_blocks_ros pick_test [LETTER] [CLOSE_ANGLE] [LIFT_M] [GRASP_DZ_M]    default: L 0.46 0.10 0.0125
(sim + MoveIt launch must be running). GRASP_DZ raises the grasp point above the cube centre.
PASS needs: the cube rises with the gripper (within 15 mm), stays level (< 10 deg) and no other cube moved (> 5 mm).
"""
import sys

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

from gr_blocks_ros.skills import LEFT, Arm

HOVER = 0.12  # m above the grasp point
LETTERS = "ABEFHLMORSTW"


def snapshot(arm):
    return {l: arm.lookup(f"block_{l}") for l in LETTERS}


def main():
    arg = lambda i, d: float(sys.argv[i]) if len(sys.argv) > i else d  # noqa: E731
    letter = sys.argv[1] if len(sys.argv) > 1 else "L"
    close_angle, lift, dz = arg(2, 0.46), arg(3, 0.10), arg(4, 0.0125)
    rclpy.init()
    arm = Arm()
    arm.wait_ready()
    q = arm.grasp_quat()
    pr = Rotation.from_quat(arm.lookup(LEFT, "end_effector_link")[1]).as_matrix()  # pad axes in the EE frame
    gr = Rotation.from_quat(q).as_matrix()  # EE axes in base_link
    pad_in_base = gr @ pr
    print("pad x/y axes in base_link (z = vertical component):",
          [np.round(pad_in_base[:, i], 2).tolist() for i in range(3)], "  pad box is 30 x 70 x 7.5 mm along its x, y, z")
    before = snapshot(arm)
    c0 = before[letter][0]
    print(f"block_{letter} at {np.round(c0, 4)}  grasp point {1000 * dz:.1f} mm above its centre, close command {close_angle}")
    z_up = np.array([0, 0, 1.0])
    target = c0 + z_up * dz
    hover_ee = arm.ee_for_tcp(target + z_up * HOVER, q)
    grasp_ee = arm.ee_for_tcp(target, q)

    def step(name, ok):
        print(f"[{name}] {'ok' if ok else 'FAILED'}")
        if not ok:
            print("PICK_TEST FAIL at", name)
            rclpy.shutdown()
            sys.exit(2)

    step("open", arm.gripper(0.0) is not None)
    step("hover", arm.move_to(hover_ee, q))
    step("descend", arm.cartesian([grasp_ee], q))
    f = arm.gripper(close_angle, settle=1.5)
    print(f"finger_joint after close command {close_angle}: {f:.3f}")
    mid = snapshot(arm)
    step("lift", arm.cartesian([grasp_ee + z_up * lift], q, avoid_collisions=False))
    arm.settle(1.5)
    after = snapshot(arm)

    d = after[letter][0] - c0
    tilt = np.degrees(Rotation.from_quat(after[letter][1]).apply([0, 0, 1]) @ [0, 0, 1])
    tilt = np.degrees(np.arccos(np.clip(Rotation.from_quat(after[letter][1]).apply([0, 0, 1]) @ [0, 0, 1], -1, 1)))
    print(f"block_{letter} moved by {np.round(1000 * d, 1)} mm (lift {1000 * lift:.0f} mm), tilt {tilt:.1f} deg")
    moved = {l: round(1000 * np.linalg.norm(after[l][0] - before[l][0]), 1) for l in LETTERS if l != letter}
    bad = {l: m for l, m in moved.items() if m > 5}
    moved_at_close = {l: round(1000 * np.linalg.norm(mid[l][0] - before[l][0]), 1) for l in LETTERS if l != letter}
    print("other cubes displaced > 5 mm after lift:", bad or "none", "| already at close time:",
          {l: m for l, m in moved_at_close.items() if m > 5} or "none")
    ok = abs(d[2] - lift) < 0.015 and np.hypot(d[0], d[1]) < 0.015 and tilt < 10 and not bad
    print("PICK_TEST", "PASS" if ok else "FAIL")
    rclpy.shutdown()
    sys.exit(0 if ok else 3)
