"""R_ing alone: put cubes down on table slots and pick them up again (low grasps, table contact), no human.

    ros2 run gr_blocks_ros table_test        (sim in the default start world)

Plan (all by R_ing): unstack L S, put-down L, unstack S M, put-down S, pick-up L, stack L S, unstack L S, stack L M.
Every action is checked against the symbolic world (stacks rebuilt from the cube poses). The final cube positions are
also compared with their slot positions.
"""
import json
import sys

import numpy as np
import rclpy

from gr_blocks_ros.blocksworld import DATA, START_SLOTS, World
from gr_blocks_ros.executor import Executor, HumanSim, TABLE_Z
from gr_blocks_ros.skills import CUBE, Arm

PLAN = ["unstack L S", "put-down L", "unstack S M", "put-down S", "pick-up L", "stack L S", "unstack L S", "stack L M"]


def main():
    towers = json.loads((DATA / "blockworld.json").read_text())["towers"]
    world = World.from_towers(towers, START_SLOTS)
    rclpy.init()
    arm = Arm()
    arm.wait_ready()
    ex = Executor(arm, world, HumanSim())
    print(f"layout reset (cubes re-placed after the sim start): {ex.reset_layout()}")
    same, phys, want = ex.consistent()
    print(f"start world physically equals the symbolic start: {same}")
    if not same:
        print("  physical:", {s: ''.join(v) for s, v in phys.items()}, "\n  symbolic:", {s: ''.join(v) for s, v in want.items()})
        sys.exit(2)
    ok = True
    for text in PLAN:
        print(f"R_ing: {text}", flush=True)
        good = ex.do(text, "R_ing")
        ok &= good
        if not good:
            break
    # final positions against their slots
    poses = ex.poses()
    worst = 0.0
    for slot, stack in world.stacks.items():
        for level, letter in enumerate(stack):
            want = np.array(world.expected_xyz(slot, level, CUBE, TABLE_Z))
            err = 1000 * np.linalg.norm(np.array(poses[letter])[:2] - want[:2])
            worst = max(worst, err)
    print(f"largest horizontal offset of any cube from its slot centre: {worst:.1f} mm")
    print("TABLE_TEST", "PASS" if ok else "FAIL")
    rclpy.shutdown()
    sys.exit(0 if ok else 3)
