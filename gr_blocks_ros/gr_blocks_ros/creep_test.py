"""Hold a picked cube still and log how it slides in the grip, together with the gripper's tilt from vertical.

    ros2 run gr_blocks_ros creep_test [LETTER] [SECONDS]        default: L 15
"""
import sys

import numpy as np
import rclpy

from gr_blocks_ros.skills import Arm


def main():
    letter = sys.argv[1] if len(sys.argv) > 1 else "L"
    seconds = int(sys.argv[2]) if len(sys.argv) > 2 else 15
    rclpy.init()
    arm = Arm()
    arm.wait_ready()
    a0 = arm.approach_axis()
    print(f"tilt of the approach axis before the pick (deg from straight down): {np.degrees(np.arccos(-a0[2])):.2f}, xy components {np.round(a0[:2], 4)}")
    if not arm.pick(letter):
        print("pick failed")
        sys.exit(2)
    ref = None
    for t in range(seconds + 1):
        tcp, _ = arm.tcp_now()
        rel = tcp - arm.block(letter)
        ref = rel if ref is None else ref
        a = arm.approach_axis()
        print(f"t={t:2d}s  cube slip in grip (mm) {np.round(1000 * (rel - ref), 1)}   approach axis xy {np.round(a[:2], 4)}  tilt {np.degrees(np.arccos(-a[2])):.2f} deg", flush=True)
        arm.settle(1.0)
    rclpy.shutdown()
