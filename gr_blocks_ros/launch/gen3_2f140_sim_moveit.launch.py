"""move_group (+ optional RViz) for the Gen3 + Robotiq 2F-140 running in Gazebo (gen3_2f140_sim.launch.py).

Start the simulation first, then this launch in a second terminal (same ROS_DOMAIN_ID / GZ_PARTITION):

    ros2 launch gr_blocks_ros gen3_2f140_sim.launch.py
    ros2 launch gr_blocks_ros gen3_2f140_sim_moveit.launch.py [launch_rviz:=true] [world_state:=false]

The SRDF, joint limits and controller mapping are the 2F-140 files from kinova_example (branch gripper-2f140);
kinematics, OMPL and the RViz layout are reused from Kinova's 2F-85 MoveIt package. Nothing in either is modified.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

MOVEIT_PKG = "kinova_gen3_7dof_robotiq_2f_85_moveit_config"  # kinematics / OMPL / RViz layout only


def launch_setup(context):
    cfg = os.path.join(get_package_share_directory("kinova_example"), "config", "2f140")
    kortex_description = get_package_share_directory("kortex_description")
    use_sim_time = {"use_sim_time": True}

    mappings = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "false",
        "gripper": "robotiq_2f_140",
        "gripper_joint_name": "finger_joint",
        "dof": "7",
        "sim_gazebo": "true",
        "vision": "false",
    }
    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name=MOVEIT_PKG)
        .robot_description(file_path=os.path.join(kortex_description, "robots", "gen3.xacro"), mappings=mappings)
        .robot_description_semantic(file_path=os.path.join(cfg, "gen3_2f140.srdf"))
        .joint_limits(file_path=os.path.join(cfg, "joint_limits_2f140.yaml"))
        .trajectory_execution(file_path=os.path.join(cfg, "moveit_controllers_2f140.yaml"))
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory(MOVEIT_PKG), "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            use_sim_time,
        ],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )
    world_state = Node(
        package="gr_blocks_ros",
        executable="world_state",
        output="screen",
        condition=IfCondition(LaunchConfiguration("world_state")),
    )
    return [move_group, rviz, world_state]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?"),
            DeclareLaunchArgument("world_state", default_value="true", description="Run world_state (cube TF + planning scene from Gazebo)."),
            OpaqueFunction(function=launch_setup),
        ]
    )
