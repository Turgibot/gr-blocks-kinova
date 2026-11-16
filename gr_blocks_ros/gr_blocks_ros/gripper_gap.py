"""Measure the distance between the two 2F-140 finger pads for a series of finger_joint angles.

    ros2 run gr_blocks_ros gripper_gap [angle ...]      (sim must be running; default angles 0 .. 0.7)
"""
import sys

import rclpy
from control_msgs.action import ParallelGripperCommand
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

LEFT, RIGHT = "left_inner_finger_pad", "right_inner_finger_pad"


def main():
    angles = [float(a) for a in sys.argv[1:]] or [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    rclpy.init()
    node = Node("gripper_gap", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    buf = Buffer()
    TransformListener(buf, node)
    client = ActionClient(node, ParallelGripperCommand, "/robotiq_gripper_controller/gripper_cmd")
    client.wait_for_server(timeout_sec=15.0)
    print("finger_joint [rad] -> pad gap [mm]")
    for a in angles:
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ["finger_joint"]
        goal.command.position = [a]
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(node, res, timeout_sec=20.0)
        end = node.get_clock().now() + Duration(seconds=1.0)
        while node.get_clock().now() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        try:
            t = buf.lookup_transform(LEFT, RIGHT, rclpy.time.Time(), Duration(seconds=2.0)).transform.translation
            print(f"  {a:5.2f} -> {1000 * (t.x**2 + t.y**2 + t.z**2) ** 0.5:7.1f}   (dx,dy,dz mm = {1000*t.x:.1f}, {1000*t.y:.1f}, {1000*t.z:.1f})")
        except Exception as e:  # noqa: BLE001
            print(f"  {a:5.2f} -> tf lookup failed: {e}")
    rclpy.shutdown()
