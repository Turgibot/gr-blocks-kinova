"""Step-by-step replica of Arm.pick (skills.py:326) with logging, to see where the gripper opens after closing.

    python3 pick_debug.py [LETTER] [--ease 0.08] [--no-weld] [--touch 0.44] [--dz 0.015] [--no-pause]

Needs the sim (rigid_grasp:=true) and the MoveIt launch running in the same ROS_DOMAIN_ID / GZ_PARTITION.
Between steps it waits for Enter so you can look at the Gazebo window.
Logged continuously: finger_joint whenever it moves > 0.005 rad, with the cube's offset from the pad midpoint (tcp).
Logged per gripper command: the controller's result (status, reached_goal, stalled, final position).
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from control_msgs.action import ParallelGripperCommand

from gr_blocks_ros.blocksworld import START_SLOTS, World
from gr_blocks_ros.executor import PICK_DZ, Executor, HumanSim
from gr_blocks_ros.recognizer import CachedRecognizer
from gr_blocks_ros.skills import Arm

STATUS = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("letter", nargs="?", default="L")
    ap.add_argument("--close", type=float, default=0.46)
    ap.add_argument("--ease", type=float, default=0.08, help="pick() eases the fingers off by this much after the weld")
    ap.add_argument("--no-weld", action="store_true", help="skip the Gazebo weld (physical grasp only)")
    ap.add_argument("--no-pause", action="store_true")
    ap.add_argument("--dz", type=float, default=PICK_DZ, help=f"grasp point above the cube centre in m (executor default {PICK_DZ})")
    ap.add_argument("--touch", type=float, default=None,
                    help="weld-first mode: close only to this angle (first contact, e.g. 0.44), weld, no ease-off")
    a = ap.parse_args()
    L = a.letter
    t0 = time.time()

    def say(msg):
        print(f"{time.time() - t0:7.2f}s  {msg}", flush=True)

    def pause(next_step):
        if not a.no_pause and sys.stdin.isatty():
            input(f"--- press Enter for: {next_step} ")
        say(f"=== {next_step}")

    rclpy.init()
    arm = Arm()
    arm.wait_ready()
    say(f"rigid grasp (weld) available: {arm.rigid}")
    rec = CachedRecognizer("MASTER", "eps3")
    ex = Executor(arm, World.from_towers(rec.trace["meta"]["start_towers"], START_SLOTS), HumanSim())

    # continuous finger log (runs whenever Arm spins, i.e. during every wait)
    last = {"f": None}

    def watch():
        f = arm.finger()
        if f is None or (last["f"] is not None and abs(f - last["f"]) < 0.005):
            return
        last["f"] = f
        try:
            tcp, _ = arm.tcp_now()
            off = np.round(1000 * (arm.block(L) - tcp), 1)
        except Exception:
            off = "?"
        say(f"    finger_joint={f:.3f}   cube minus tcp (mm)={off}")
    arm.create_timer(0.05, watch)

    def grip(angle, settle):
        """Same as Arm.gripper, but print what the controller answers."""
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ["finger_joint"]
        goal.command.position = [float(angle)]
        say(f"  gripper goal {angle:.3f} (finger now {arm.finger():.3f})")
        h = arm.spin_until(arm.grip.send_goal_async(goal), 10.0)
        if h is None or not h.accepted:
            say("  !! goal rejected")
            return
        t = time.time()
        res = arm.spin_until(h.get_result_async(), 30.0)
        if res is None:
            say(f"  !! no result after {time.time() - t:.1f} s (goal still active)")
        else:
            r = res.result
            say(f"  result after {time.time() - t:.1f} s: {STATUS.get(res.status, res.status)} reached_goal={r.reached_goal} "
                f"stalled={r.stalled} position={list(r.state.position)}")
        arm.settle(settle)
        say(f"  finger after settle: {arm.finger():.3f}")

    pause("reset the layout (teleport the towers back)")
    say(f"layout ok: {ex.reset_layout()}")

    up = np.array([0, 0, 1.0])
    q = arm.grasp_quat()
    clear = all(st[-1] == L for st in ex.world.stacks.values() if L in st)
    if not clear:
        say(f"!! block {L} is not clear (something is stacked on it): pick a top cube, e.g. L, H or F"); return
    target = arm.block(L) + up * a.dz
    say(f"grasp point {1000 * a.dz:.1f} mm above the cube centre: pads span {1000 * (a.dz - 0.035):+.1f} .. {1000 * (a.dz + 0.035):+.1f} mm, cube -22.5 .. +22.5 mm")
    hover_ee, grasp_ee = arm.ee_for_tcp(target + up * 0.12, q), arm.ee_for_tcp(target, q)

    pause("open + move above the cube + descend  (skills.py:336-337)")
    grip(0.0, 1.0)
    if not (arm.move_to(hover_ee, q) and arm.cartesian([grasp_ee], q)):
        say("approach failed"); return

    if a.touch is not None:
        pause(f"CLOSE ONLY TO FIRST CONTACT {a.touch}")
        grip(a.touch, 0.3)
        pause("WELD right away (no squeeze, no ease-off)")
        arm._gz_weld("a", L)
        arm.settle(2.0)
        say(f"  finger 2 s after weld: {arm.finger():.3f}")
    else:
        pause(f"CLOSE to {a.close}  (skills.py:344)")
        grip(a.close, 1.5)

    if a.touch is None and arm.rigid and not a.no_weld:
        pause("WELD the cube to the arm  (skills.py:346)")
        arm._gz_weld("a", L)
        arm.settle(1.0)
        say(f"  finger 1 s after weld: {arm.finger():.3f}")
        if a.ease > 0:
            pause(f"EASE OFF to {a.close - a.ease:.3f}  (skills.py:347)")
            grip(max(0.0, a.close - a.ease), 0.5)

    pause("MoveIt attach (skills.py:348)")
    arm.attach(L)

    pause("LIFT 10 cm  (skills.py:350)")
    z0 = arm.block(L)[2]
    arm.cartesian([grasp_ee + up * 0.10], q, avoid_collisions=False)
    arm.settle(2.0)
    say(f"  cube rose {1000 * (arm.block(L)[2] - z0):.0f} mm, finger {arm.finger():.3f}")

    pause("put it back down, unweld, open")
    arm.cartesian([grasp_ee + up * 0.002], q, avoid_collisions=False)
    if arm.rigid and (not a.no_weld or a.touch is not None):
        arm._gz_weld("d", L)
    grip(0.0, 1.0)
    arm.detach()
    arm.cartesian([grasp_ee + up * 0.12], q)
    say("done")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
