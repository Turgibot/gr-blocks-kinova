"""Move the arm to a named pose of the SRDF (Home, Retract, Vertical) through MoveGroup, then print the joints.

    ros2 run gr_blocks_ros move_named Home
"""
import sys
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

SRDF = "config/2f140/gen3_2f140.srdf"


def named_pose(name, group="manipulator"):
    root = ET.parse(f"{get_package_share_directory('kinova_example')}/{SRDF}").getroot()
    for gs in root.findall("group_state"):
        if gs.get("name") == name and gs.get("group") == group:
            return {j.get("name"): float(j.get("value")) for j in gs.findall("joint")}
    raise SystemExit(f"no group_state {name!r} for {group!r}")


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "Home"
    target = named_pose(name)
    rclpy.init()
    node = Node("move_named", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.update(zip(m.name, m.position)), 10)
    client = ActionClient(node, MoveGroup, "/move_action")
    if not client.wait_for_server(timeout_sec=20.0):
        print("move_action not available")
        sys.exit(1)

    goal = MoveGroup.Goal()
    r = goal.request
    r.group_name = "manipulator"
    r.pipeline_id = "ompl"
    r.num_planning_attempts = 5
    r.allowed_planning_time = 5.0
    r.max_velocity_scaling_factor = 0.3
    r.max_acceleration_scaling_factor = 0.3
    c = Constraints()
    for j, v in target.items():
        c.joint_constraints.append(JointConstraint(joint_name=j, position=v, tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
    r.goal_constraints.append(c)
    goal.planning_options.plan_only = False

    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    handle = fut.result()
    if handle is None or not handle.accepted:
        print("goal rejected")
        sys.exit(1)
    res = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=90.0)
    code = res.result().result.error_code.val
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    err = max(abs(js.get(j, 99) - v) for j, v in target.items())
    print(f"error_code={code} (SUCCESS={MoveItErrorCodes.SUCCESS}) max_joint_error={err:.4f} rad")
    print("MOVE_NAMED", "PASS" if code == MoveItErrorCodes.SUCCESS and err < 0.03 else "FAIL", name)
    rclpy.shutdown()
    sys.exit(0 if code == MoveItErrorCodes.SUCCESS and err < 0.03 else 2)
