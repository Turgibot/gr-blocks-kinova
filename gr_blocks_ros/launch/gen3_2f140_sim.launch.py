"""Gazebo simulation of the Gen3 7-DoF + Robotiq 2F-140 (no MoveIt yet).

Based on Kinova's kortex_sim_control.launch.py (not modified), with two differences:
  * config/ros2_controllers_2f140_sim.yaml names `finger_joint` as the gripper joint (Kinova's file names the 2F-85
    joint, so the gripper controller cannot start for the 2F-140);
  * the controller spawners run one after another instead of all at once. Started together, one spawner
    sometimes never discovers /controller_manager/list_controllers and holds the spawner lock, which stalls the rest.

    ros2 launch gr_blocks_ros gen3_2f140_sim.launch.py [gui:=true] [launch_rviz:=true]
"""

import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

from gr_blocks_ros.sim_patches import add_attach_plugins, controllers_with_gain, patch_pads

# Spawned in this order; `--inactive` ones are configured but not started.
CONTROLLERS = [
    ("joint_state_broadcaster", []),
    ("joint_trajectory_controller", []),
    ("robotiq_gripper_controller", []),
    ("twist_controller", ["--inactive"]),
    ("joint_group_velocity_controller", ["--inactive"]),
]


def launch_setup(context):
    kortex_description = get_package_share_directory("kortex_description")
    controllers_file = os.path.join(
        get_package_share_directory("gr_blocks_ros"), "config", "ros2_controllers_2f140_sim.yaml"
    )
    controllers_file = controllers_with_gain(controllers_file, LaunchConfiguration("position_gain").perform(context))
    use_sim_time = {"use_sim_time": True}
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    engine = LaunchConfiguration("physics_engine").perform(context)
    worlds = os.path.join(get_package_share_directory("gr_blocks_ros"), "worlds")
    world = LaunchConfiguration("world").perform(context) or "empty.sdf"
    if world != "empty.sdf" and not os.path.isabs(world):
        world = os.path.join(worlds, world)

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            os.path.join(kortex_description, "robots", "kinova.urdf.xacro"),
            " robot_ip:=xxx.yyy.zzz.www name:=gen3 arm:=gen3 dof:=7 vision:=false prefix:=''",
            " sim_gazebo:=true gripper:=robotiq_2f_140 gazebo_renderer:=ogre2",
            " simulation_controllers:=",
            controllers_file,
        ]
    ).perform(context)
    robot_description_content = patch_pads(
        robot_description_content,
        mu=LaunchConfiguration("pad_mu").perform(context),
        kp=LaunchConfiguration("pad_kp").perform(context),
        kd=LaunchConfiguration("pad_kd").perform(context),
    )
    if LaunchConfiguration("rigid_grasp").perform(context).lower() == "true":
        robot_description_content = add_attach_plugins(robot_description_content)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}, use_sim_time],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f" -r {'' if gui else '-s --headless-rendering '}-v 3 {world}"
            f" --physics-engine {engine}"
        }.items(),
    )

    gz_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-string", robot_description_content, "-name", "gen3", "-allow_renaming", "true",
                   "-x", "0.0", "-y", "0.0", "-z", "0.3"],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[use_sim_time],
        arguments=[
            "/wrist_mounted_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/wrist_mounted_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/wrist_mounted_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/wrist_mounted_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output="screen",
    )

    resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(get_package_prefix("robotiq_description"), "share")
    )
    world_path = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", worlds)
    view_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[use_sim_time],
        arguments=[f"/{c}/image@sensor_msgs/msg/Image[gz.msgs.Image" for c in ("view_cam", "top_cam")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("view_cams")),
    )

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, *extra, "-c", "/controller_manager"],
        )
        for name, extra in CONTROLLERS
    ]
    # Chain: start spawner i+1 when spawner i has exited; the first starts once the robot has been spawned.
    chain = [
        RegisterEventHandler(OnProcessExit(target_action=a, on_exit=[b]))
        for a, b in zip(spawners, spawners[1:])
    ]
    on_spawned = [spawners[0]]
    if LaunchConfiguration("rigid_grasp").perform(context).lower() == "true":
        # the cubes are welded to the gripper when the robot appears: let them go at once (see release_cubes.py)
        on_spawned.append(ExecuteProcess(cmd=[sys.executable, "-m", "gr_blocks_ros.release_cubes"], output="screen"))
    start_first = RegisterEventHandler(OnProcessExit(target_action=gz_spawn, on_exit=on_spawned))

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(kortex_description, "rviz", "view_robot.rviz")],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return [
        resource_path,
        world_path,
        view_bridge,
        clock_bridge,
        robot_state_publisher,
        gz_sim,
        gz_spawn,
        camera_bridge,
        start_first,
        *chain,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?"),
            DeclareLaunchArgument("world", default_value="blocks_world.sdf", description="World file (name in worlds/, absolute path, or empty.sdf)."),
            DeclareLaunchArgument("view_cams", default_value="true", description="Bridge the fixed view_cam/top_cam images to ROS."),
            DeclareLaunchArgument("pad_mu", default_value="", description="Finger-pad friction (empty = model default)."),
            DeclareLaunchArgument("pad_kp", default_value="", description="Finger-pad contact stiffness (empty = default)."),
            DeclareLaunchArgument("pad_kd", default_value="", description="Finger-pad contact damping (empty = default)."),
            DeclareLaunchArgument("rigid_grasp", default_value="false", description="Sim-only: weld a grasped cube to the gripper (DetachableJoint per cube)."),
            DeclareLaunchArgument("physics_engine", default_value="gz-physics-bullet-featherstone-plugin", description="Gazebo physics engine plugin (e.g. gz-physics-dartsim-plugin)."),
            DeclareLaunchArgument("position_gain", default_value="", description="gz_ros2_control position_proportional_gain (empty = its default 0.1)."),
            DeclareLaunchArgument("gui", default_value="false", description="Open the Gazebo GUI (server only if false)."),
            OpaqueFunction(function=launch_setup),
        ]
    )
