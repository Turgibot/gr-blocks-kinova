"""Move the arm to a named configuration from config/poses.yaml (no MoveIt: straight joint interpolation, NO collision check).

    python3 -m gr_blocks_ros.go_pose [NAME] [SECONDS]        default: observe 3

Sends one JointTrajectory point to /joint_trajectory_controller/joint_trajectory (the same as the `ros2 topic pub`
command), then waits until /joint_states is within 0.01 rad of the target on every joint (or times out).
Needs sim_home.launch.py (or any launch with joint_trajectory_controller) running in the same ROS_DOMAIN_ID.
"""
import math
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [f"joint_{i}" for i in range(1, 8)]
TOPIC = "/joint_trajectory_controller/joint_trajectory"


def angle_err(a, b):
    """|a - b| on the circle (joints 1, 3, 5, 7 are continuous: 3.14 and -3.14 are the same angle)."""
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def load_poses():
    with open(os.path.join(get_package_share_directory("gr_blocks_ros"), "config", "poses.yaml")) as f:
        return yaml.safe_load(f)


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "observe"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    poses = load_poses()
    if name not in poses:
        raise SystemExit(f"unknown pose {name!r}; known: {', '.join(poses)}")
    target = [float(v) for v in poses[name]]

    rclpy.init()
    node = rclpy.create_node("go_pose")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.update(zip(m.name, m.position)), 10)
    pub = node.create_publisher(JointTrajectory, TOPIC, 10)

    deadline = time.time() + 10.0
    while pub.get_subscription_count() == 0:  # the controller must be listening, or the single message is lost
        if time.time() > deadline:
            raise SystemExit(f"nobody subscribes to {TOPIC}: is the sim running in this ROS_DOMAIN_ID?")
        rclpy.spin_once(node, timeout_sec=0.1)

    msg = JointTrajectory(joint_names=JOINTS)
    msg.points = [JointTrajectoryPoint(positions=target, time_from_start=Duration(sec=int(seconds), nanosec=int(seconds % 1 * 1e9)))]
    pub.publish(msg)
    print(f"sent {name}: {target} in {seconds:g} s")

    deadline = time.time() + seconds + 10.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(j in js for j in JOINTS):
            err = max(angle_err(js[j], t) for j, t in zip(JOINTS, target))
            if err < 0.01:
                print(f"reached {name} (max joint error {err:.4f} rad)")
                break
    else:
        err = max((angle_err(js[j], t) for j, t in zip(JOINTS, target) if j in js), default=float("nan"))
        print(f"did NOT reach {name} within {seconds + 10:g} s (max joint error {err:.3f} rad)")
        sys.exit(1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
