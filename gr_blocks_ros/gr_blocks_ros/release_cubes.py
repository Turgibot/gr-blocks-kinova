"""Sim helper for rigid_grasp:=true: detach every cube from the gripper as soon as the robot has been spawned.

Gazebo's DetachableJoint starts attached (the cubes are welded to the gripper when the robot appears); the arm would drag
them around until the arm node releases them a minute later. The launch starts this right after the spawn; it keeps
publishing the detach message for a few seconds (the plugin may not be listening yet at first; repeats are harmless).
"""
import sys
import time

from gz.msgs10.empty_pb2 import Empty
from gz.transport13 import Node

from gr_blocks_ros.sim_patches import LETTERS, detach_topic


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    node = Node()
    pubs = [node.advertise(detach_topic(l), Empty) for l in LETTERS]
    time.sleep(0.5)
    end = time.time() + seconds
    while time.time() < end:
        for p in pubs:
            p.publish(Empty())
        time.sleep(0.1)


if __name__ == "__main__":
    main()
