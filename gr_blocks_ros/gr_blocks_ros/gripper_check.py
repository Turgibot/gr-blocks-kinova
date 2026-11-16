"""Open/close the 2F-140 through /robotiq_gripper_controller/gripper_cmd and print finger_joint before/after."""

import sys

import rclpy
from control_msgs.action import ParallelGripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main():
    rclpy.init()
    node = Node("gripper_check")
    pos = {}
    node.create_subscription(JointState, "/joint_states", lambda m: pos.update(zip(m.name, m.position)), 10)
    client = ActionClient(node, ParallelGripperCommand, "/robotiq_gripper_controller/gripper_cmd")
    if not client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error("gripper_cmd action server not available")
        sys.exit(1)
    ok = True
    for target in (0.7, 0.0):  # closed, open (rad on finger_joint)
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ["finger_joint"]
        goal.command.position = [target]
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            node.get_logger().error(f"goal {target} rejected")
            sys.exit(1)
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(node, res, timeout_sec=20.0)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
        got = pos.get("finger_joint")
        print(f"target={target:.2f} finger_joint={got}")
        ok &= got is not None and abs(got - target) < 0.1
    print("GRIPPER_CHECK", "PASS" if ok else "FAIL")
    rclpy.shutdown()
    sys.exit(0 if ok else 2)
