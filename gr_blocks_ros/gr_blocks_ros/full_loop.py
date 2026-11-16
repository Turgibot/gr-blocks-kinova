"""Whole MASTER scenario in the simulation: scripted R_ed + recognition (cached) + R_ing's helper plan.

Start the sim in the default start world (three towers), then:

    ros2 run gr_blocks_ros full_loop [TEMPO_S]            TEMPO_S = pause between R_ed's early actions (default 2)

1. R_ed (teleported cubes) plays the first t_join actions of the cached observed plan; after each, the belief over the
   12 goal words is printed (cached trace, eps3).
2. At t_join (helper_MASTER.json) R_ing joins: the cached joint plan runs, R_ed's parts by teleport, R_ing's with the Gen3.
3. R_ed completes the word (the remaining goal condition on(M,A)).
Success: every action physically done and consistent with the symbolic world, R_ing performed exactly pi_ring, and the
tower R-E-T-S-A-M (MASTER read top to bottom) stands.
"""
import json
import sys
import time

import rclpy

from gr_blocks_ros.blocksworld import DATA, START_SLOTS, World
from gr_blocks_ros.executor import Executor, HumanSim
from gr_blocks_ros.recognizer import CachedRecognizer
from gr_blocks_ros.skills import Arm


def main():
    tempo = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    plans = DATA / "cached_plans"
    helper = json.loads((plans / "helper_MASTER.json").read_text())
    rec = CachedRecognizer("MASTER", "eps3")
    plan = rec.plan
    world = World.from_towers(rec.trace["meta"]["start_towers"], START_SLOTS)
    rclpy.init()
    arm = Arm()
    arm.wait_ready()
    ex = Executor(arm, world, HumanSim())
    print(f"layout reset (cubes re-placed after the sim start): {ex.reset_layout()}")
    same, *_ = ex.consistent()
    print(f"start world physically equals the symbolic start: {same}")
    if not same:
        sys.exit(2)
    t0 = time.time()
    observed, ring, ok = [], [], True

    def run(text, agent):
        nonlocal ok
        good = ex.do(text, agent)
        ok &= good
        if agent == "R_ing":
            ring.append(text)
        return good

    print(f"== phase 1: R_ed alone, first {helper['t_join']} actions")
    for text in plan[: helper["t_join"]]:
        if not run(text, "R_ed"):
            break
        observed.append(text)
        print(f"   R_ed: {text:12s} belief top3 {[(w, round(p, 3)) for w, p in rec.top(observed)]}", flush=True)
        arm.settle(tempo)  # keep spinning (a blocking sleep lets the TF / clock backlog pile up)
    if ok:
        print(f"== phase 2: t_join = {helper['t_join']} reached, R_ing joins (cached joint plan {helper['joint_Ghat']['cost']} steps)")
        for step in helper["joint_Ghat"]["steps"]:
            for arm_id, agent in (("1", "R_ed"), ("2", "R_ing")):
                text = step["comps"][arm_id]
                if text and ok:
                    print(f"   {agent}: {text}", flush=True)
                    run(text, agent)
    if ok:
        print("== phase 3: R_ed completes the word")
        for text in ("unstack M O", "stack M A"):
            if ok:
                print(f"   R_ed: {text}", flush=True)
                run(text, "R_ed")
    word_ok = any(st == list("RETSAM") for st in world.stacks.values())
    phys = ex.physical_stacks()
    phys_ok = any(st == list("RETSAM") for st in phys.values())
    ring_ok = ring == helper["pi_ring"]
    print(f"R_ing actions {ring} == pi_ring: {ring_ok}")
    print(f"tower R-E-T-S-A-M: symbolic {word_ok}, physical {phys_ok}; physical stacks {{{', '.join(s + ':' + ''.join(v) for s, v in sorted(phys.items()))}}}")
    print(f"elapsed {time.time() - t0:.0f} s")
    good = ok and word_ok and phys_ok and ring_ok
    print("FULL_LOOP", "PASS" if good else "FAIL")
    rclpy.shutdown()
    sys.exit(0 if good else 3)
