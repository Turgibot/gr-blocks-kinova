"""Fresh start, step 1: Gazebo with the Gen3 + Robotiq 2F-140 whose arm is already at the SRDF "Home" pose at startup.

    ros2 launch gr_blocks_ros sim_home.launch.py [gui:=false] [world:=blocks_world_long.sdf]

Only the simulation and the controllers: no MoveIt, no weld plugins, no scripts. Nothing moves the arm; it is spawned at Home.

How the arm starts at Home:
  1. the Home joint values are read from the same SRDF MoveIt uses (kinova_example/config/2f140/gen3_2f140.srdf);
  2. they are written to a small YAML file and passed to Kinova's xacro as `initial_positions_file` (an existing xacro
     argument; its default file has every joint at 0, which is the straight-up pose the old launch started in);
  3. the xacro puts them into the ros2_control tags as `initial_value`, and gz_ros2_control sets each joint to its
     initial_value when the robot is spawned; joint_trajectory_controller then holds that pose once it is active.
"""

import os
import re
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

from gr_blocks_ros.move_named import named_pose

# Started one after another (started together, a spawner sometimes stalls the others; see gen3_2f140_sim.launch.py).
CONTROLLERS = ["joint_state_broadcaster", "joint_trajectory_controller", "robotiq_gripper_controller"]


def home_positions_file():
    """Write the SRDF Home pose as the YAML that kinova.urdf.xacro's initial_positions_file expects; return its path."""
    home = named_pose("Home")  # {"joint_1": 0.0, "joint_2": 0.26, ...}
    path = os.path.join(tempfile.gettempdir(), "gr_blocks_ros_initial_positions_home.yaml")
    with open(path, "w") as f:
        f.writelines(f"{joint}: {value}\n" for joint, value in sorted(home.items()))
    return path


def gui_config_file(camera_pose):
    """Copy of Gazebo's default GUI config with the start camera of the 3D view replaced; return its path.

    camera_pose = "x y z roll pitch yaw" of the GUI camera in the world frame (m, rad). Gazebo's default is "-6 0 6 0 0.5 0"."""
    default = os.path.join(get_package_prefix("gz_sim_vendor"), "opt", "gz_sim_vendor", "share", "gz", "gz-sim8", "gui", "gui.config")
    with open(default) as f:
        text = f.read()
    text, n = re.subn(r"<camera_pose>[^<]*</camera_pose>", f"<camera_pose>{camera_pose}</camera_pose>", text, count=1)
    if n != 1:
        raise RuntimeError(f"no <camera_pose> in {default}")
    path = os.path.join(tempfile.gettempdir(), "gr_blocks_ros_gui.config")
    with open(path, "w") as f:
        f.write(text)
    return path


def launch_setup(context):
    share = get_package_share_directory("gr_blocks_ros")
    kortex_description = get_package_share_directory("kortex_description")
    controllers_file = os.path.join(share, "config", "ros2_controllers_2f140_sim.yaml")  # names finger_joint for the 2F-140
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    world = LaunchConfiguration("world").perform(context)
    if not os.path.isabs(world):
        world = os.path.join(share, "worlds", world)

    robot_description = Command(
        [
            FindExecutable(name="xacro"), " ",
            os.path.join(kortex_description, "robots", "kinova.urdf.xacro"),
            " robot_ip:=xxx.yyy.zzz.www name:=gen3 arm:=gen3 dof:=7 vision:=true prefix:=''",  # vision: adds camera_color_frame, without it Gazebo creates no wrist camera
            " sim_gazebo:=true gripper:=robotiq_2f_140 gazebo_renderer:=ogre2",
            " simulation_controllers:=", controllers_file,
            " initial_positions_file:=", home_positions_file(),
        ]
    ).perform(context)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        # bullet-featherstone: Gazebo's default engine (DART) cannot do the 2F-140's mimic joints
        # ("does not support mimic constraints"), so the finger linkage would not be coupled.
        launch_arguments={"gz_args": f" -r {'' if gui else '-s --headless-rendering '}-v 3 {world}"
                                     " --physics-engine gz-physics-bullet-featherstone-plugin"
                                     f" --gui-config {gui_config_file(LaunchConfiguration('view').perform(context))}"}.items(),
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        # base on the table top (z = 0.30), as in the existing launch
        arguments=["-string", robot_description, "-name", "gen3", "-x", "0.0", "-y", "0.0", "-z", "0.3"],
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    spawners = [
        Node(package="controller_manager", executable="spawner", arguments=[name, "-c", "/controller_manager"])
        for name in CONTROLLERS
    ]
    start_spawners = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[spawners[0]]))
    chain = [RegisterEventHandler(OnProcessExit(target_action=a, on_exit=[b])) for a, b in zip(spawners, spawners[1:])]

    # wrist camera (rgbd_camera "wrist_mounted_camera" in the Kortex xacro): colour image + intrinsics to ROS
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/wrist_mounted_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/wrist_mounted_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output="screen",
    )

    return [
        camera_bridge,
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(get_package_prefix("robotiq_description"), "share")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(share, "worlds")),
        clock_bridge,
        robot_state_publisher,
        gz_sim,
        spawn,
        start_spawners,
        *chain,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true", description="Open the Gazebo window (false = server only)."),
            DeclareLaunchArgument("world", default_value="blocks_world_long.sdf",
                                  description="World file in worlds/ or an absolute path. blocks_world_long.sdf: table extended in +x "
                                              "(the direction the gripper points at Home) to x = 1.25 m."),
            # the user's view, read from /gui/camera/pose on 2026-11-18: front-right of the towers, looking down ~43 degrees
            DeclareLaunchArgument("view", default_value="1.213 -0.435 0.962 0 0.744 2.531",
                                  description="Start camera of the Gazebo window: 'x y z roll pitch yaw' (m, rad)."),
            OpaqueFunction(function=launch_setup),
        ]
    )
