"""Replay R_ing's part of the cached helper plan (data/cached_plans/helper_MASTER.json) in the simulation, no recognition.

Start the sim in the world at t_join (worlds/blocks_world_tjoin.sdf, see make_world --trace-step 6), then:

    ros2 run gr_blocks_ros replay_helper

The cached joint plan has two arms: arm 1 is R_ed (the simulated human, cubes teleported), arm 2 is R_ing (the Gen3).
In every joint step arm 1's action runs first, then arm 2's (the order the cached worlds assume).
Success: every action physically done, the physical stacks equal the symbolic ones after each action, and the final
towers equal the cached world of the last joint step.
"""
import json
import sys
import time
from pathlib import Path

import rclpy

from gr_blocks_ros.blocksworld import DATA, START_SLOTS, World
from gr_blocks_ros.executor import Executor, HumanSim
from gr_blocks_ros.skills import Arm


def main():
    plans = DATA / "cached_plans"
    helper = json.loads((plans / "helper_MASTER.json").read_text())
    trace = json.loads((plans / "trace_MASTER.json").read_text())
    key = sys.argv[1] if len(sys.argv) > 1 else "joint_Ghat"
    world = World.from_towers(trace["meta"]["start_towers"], START_SLOTS)
    for text in trace["meta"]["observed_plan"][: helper["t_join"]]:
        world.apply_text(text, "R_ed")  # symbolic state at t_join (same slots as the generated world)
    rclpy.init()
    arm = Arm()
    arm.wait_ready()
    ex = Executor(arm, world, HumanSim())
    print(f"layout reset (cubes re-placed after the sim start): {ex.reset_layout()}")
    same, phys, want = ex.consistent()
    print(f"initial physical state matches the symbolic t_join world: {same}")
    if not same:
        print("  physical:", {s: ''.join(v) for s, v in phys.items()}, "\n  symbolic:", {s: ''.join(v) for s, v in want.items()})
        sys.exit(2)
    t0, ok, ring_done = time.time(), True, 0
    for step in helper[key]["steps"]:
        print(f"-- {step['name']} {step['args']}")
        for arm_id, agent in (("1", "R_ed"), ("2", "R_ing")):
            text = step["comps"][arm_id]
            if not text:
                continue
            print(f"   {agent}: {text}", flush=True)
            good = ex.do(text, agent)
            ring_done += good and agent == "R_ing"
            if not good:
                ok = False
                break
        if not ok:
            break
    final = helper[key]["steps"][-1]["world"]["towers"]
    ends = sorted(tuple(t) for t in final) == world.towers()
    print(f"R_ing actions done: {ring_done} of {len(helper['pi_ring'])}   elapsed {time.time() - t0:.0f} s")
    print(f"final towers equal the cached world: {ends}   (symbolic {world.towers()})")
    print("REPLAY_HELPER", "PASS" if ok and ends and ring_done == len(helper["pi_ring"]) else "FAIL")
    rclpy.shutdown()
    sys.exit(0 if ok and ends else 3)
