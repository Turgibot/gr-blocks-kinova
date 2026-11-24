"""The one launch file: Gazebo with the Gen3 + Robotiq 2F-140 (arm at the SRDF "Home" pose), optional wrist camera,
optional move_group (+ RViz). Everything is configured in config/gr_blocks.json (each "_field" key lists the options of
"field"). Edit the json in the source folder, rebuild, launch:

    ros2 launch gr_blocks_ros gr_blocks.launch.py

No MoveIt unless "moveit" is true; no weld plugins, no scripts. Nothing moves the arm; it is spawned at Home.

How the arm starts at Home:
  1. the Home joint values are read from the same SRDF MoveIt uses (kinova_example/config/2f140/gen3_2f140.srdf);
  2. they are written to a small YAML file and passed to Kinova's xacro as `initial_positions_file` (an existing xacro
     argument; its default file has every joint at 0, which is the straight-up pose the old launch started in);
  3. the xacro puts them into the ros2_control tags as `initial_value`, and gz_ros2_control sets each joint to its
     initial_value when the robot is spawned; joint_trajectory_controller then holds that pose once it is active.
"""

import json
import os
import re
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

MOVEIT_PKG = "kinova_gen3_7dof_robotiq_2f_85_moveit_config"  # kinematics / OMPL / RViz layout only
SRDF = "config/2f140/gen3_2f140.srdf"


def named_pose(name, group="manipulator"):
    """Joint values of a group_state of the kortex SRDF, e.g. Home."""
    root = ET.parse(f"{get_package_share_directory('kinova_example')}/{SRDF}").getroot()
    for gs in root.findall("group_state"):
        if gs.get("name") == name and gs.get("group") == group:
            return {j.get("name"): float(j.get("value")) for j in gs.findall("joint")}
    raise SystemExit(f"no group_state {name!r} for {group!r}")

# Started one after another (started together, a spawner sometimes stalls the others; see gen3_2f140_sim.launch.py).
CONTROLLERS = ["joint_state_broadcaster", "joint_trajectory_controller", "robotiq_gripper_controller"]


def home_positions_file():
    """Write the SRDF Home pose as the YAML that kinova.urdf.xacro's initial_positions_file expects; return its path."""
    home = named_pose("Home")  # {"joint_1": 0.0, "joint_2": 0.26, ...}
    path = os.path.join(tempfile.gettempdir(), "gr_blocks_ros_initial_positions_home.yaml")
    with open(path, "w") as f:
        f.writelines(f"{joint}: {value}\n" for joint, value in sorted(home.items()))
    return path


PHYSICS_ENGINE = {"dart": "", "bullet": " --physics-engine gz-physics-bullet-featherstone-plugin"}  # dart: no argument (Gazebo's default)
LETTERS = "ABEFHLMORSTW"  # the cubes: models block_<letter> in the world
ATTACH_PARENT = "bracelet_link"  # real link that carries the gripper body (the gripper base is lumped into it)

# Run after the robot is spawned: the DetachableJoint plugins start attached, so detach every cube (the arm has not moved yet,
# so the cubes stay where they are). The detach is repeated every 0.5 s until the plugin reports "detached" on its state topic
# (the plugins may not be listening yet when the first message goes out), for at most 40 s.
DETACH_ALL = """
import time
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node
node = Node()
letters = "{letters}"
state = {{}}
for letter in letters:
    node.subscribe(StringMsg, "/model/block_" + letter + "/detachable_joint/state", lambda m, l=letter: state.__setitem__(l, m.data))
pubs = {{l: node.advertise("/gr_blocks/detach/" + l, Empty) for l in letters}}
time.sleep(1.0)
deadline = time.time() + 40.0
while time.time() < deadline:
    pending = [l for l in letters if state.get(l) != "detached"]
    if not pending:
        break
    for l in pending:
        pubs[l].publish(Empty())
    time.sleep(0.5)
print("detach_all: still attached:", [l for l in letters if state.get(l) != "detached"] or "none", flush=True)
"""


GRIPPER_START = 0.271  # rad, finger_joint at the start: 85 mm open (the 2F-140 xacro starts it at 0.695, almost closed)


def start_gripper_open(urdf):
    """Replace the initial value of finger_joint in the ros2_control block, so the simulation starts with the gripper open."""
    urdf, n = re.subn(r'(<joint name="finger_joint">.*?<param name="initial_value">)[^<]*(</param>)', rf"\g<1>{GRIPPER_START}\g<2>", urdf, count=1, flags=re.S)
    if n != 1:
        raise RuntimeError("no initial_value for finger_joint in the robot description")
    return urdf


def add_weld_plugins(urdf):
    """One Gazebo DetachableJoint per cube: an empty message on /gr_blocks/attach/<letter> welds the cube to the gripper at
    its current relative pose, one on /gr_blocks/detach/<letter> releases it. The cube models must exist in the world
    already when the robot is spawned."""
    plugins = "".join(
        "<gazebo><plugin filename=\"gz-sim-detachable-joint-system\" name=\"gz::sim::systems::DetachableJoint\">"
        f"<parent_link>{ATTACH_PARENT}</parent_link><child_model>block_{letter}</child_model><child_link>link</child_link>"
        f"<attach_topic>/gr_blocks/attach/{letter}</attach_topic><detach_topic>/gr_blocks/detach/{letter}</detach_topic>"
        "<suppress_child_warning>true</suppress_child_warning></plugin></gazebo>\n"
        for letter in LETTERS
    )
    i = urdf.rindex("</robot>")
    return urdf[:i] + plugins + urdf[i:]


def controllers_with_gain(yaml_path, gain):
    """Copy of the controllers file with gz_ros2_control's position_proportional_gain set (its default 0.1 1/s holds the
    2F-140's mimic joints so softly that the finger linkage goes loose); gain 0 returns the original file."""
    if not gain:
        return yaml_path
    with open(yaml_path) as f:
        text = f.read()
    text += f"\ngz_ros_control:\n  ros__parameters:\n    position_proportional_gain: {float(gain)}\n"
    path = os.path.join(tempfile.gettempdir(), f"gr_blocks_ros_controllers_gain_{float(gain)}.yaml")
    with open(path, "w") as f:
        f.write(text)
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


def moveit_actions(rviz_on, camera):
    """move_group (collision-aware planning, IK) and optionally RViz; same MoveIt config as the old pipeline."""
    srdf_dir = os.path.join(get_package_share_directory("kinova_example"), "config", "2f140")
    kortex_description = get_package_share_directory("kortex_description")
    use_sim_time = {"use_sim_time": True}

    mappings = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "false",
        "gripper": "robotiq_2f_140",
        "gripper_joint_name": "finger_joint",
        "dof": "7",
        "sim_gazebo": "true",
        "vision": "true" if camera else "false",  # same model as the simulated robot
    }
    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name=MOVEIT_PKG)
        .robot_description(file_path=os.path.join(kortex_description, "robots", "gen3.xacro"), mappings=mappings)
        .robot_description_semantic(file_path=os.path.join(srdf_dir, "gen3_2f140.srdf"))
        .joint_limits(file_path=os.path.join(srdf_dir, "joint_limits_2f140.yaml"))
        .trajectory_execution(file_path=os.path.join(srdf_dir, "moveit_controllers_2f140.yaml"))
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
    )
    return [move_group] + ([rviz] if rviz_on else [])


def launch_setup(context):
    share = get_package_share_directory("gr_blocks_ros")
    kortex_description = get_package_share_directory("kortex_description")
    controllers_file = os.path.join(share, "config", "ros2_controllers_2f140_sim.yaml")  # names finger_joint for the 2F-140
    with open(os.path.join(share, "config", "gr_blocks.json")) as f:
        cfg = json.load(f)
    controllers_file = controllers_with_gain(controllers_file, cfg["position_gain"])
    gui, camera = cfg["gui"], cfg["camera"]
    world = cfg["world"]
    if not os.path.isabs(world):
        world = os.path.join(share, "worlds", world)

    robot_description = Command(
        [
            FindExecutable(name="xacro"), " ",
            os.path.join(kortex_description, "robots", "kinova.urdf.xacro"),
            f" robot_ip:=xxx.yyy.zzz.www name:=gen3 arm:=gen3 dof:=7 vision:={str(camera).lower()} prefix:=''",  # vision: adds camera_color_frame, without it Gazebo creates no wrist camera
            " sim_gazebo:=true gripper:=robotiq_2f_140 gazebo_renderer:=ogre2",
            " simulation_controllers:=", controllers_file,
            " initial_positions_file:=", home_positions_file(),
        ]
    ).perform(context)
    robot_description = start_gripper_open(robot_description)
    if cfg["weld"]:
        robot_description = add_weld_plugins(robot_description)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        # "physics" in gr_blocks.json: dart = Gazebo's default engine (it cannot couple the 2F-140's left_inner_finger_joint
        # mimic: "does not support mimic constraints"); bullet = bullet-featherstone (couples all mimic joints).
        launch_arguments={"gz_args": f" -r {'' if gui else '-s --headless-rendering '}-v 3 {world}"
                                     f"{PHYSICS_ENGINE[cfg['physics']]}"
                                     f" --gui-config {gui_config_file(cfg['view'])}"}.items(),
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        # base on the 5.8 cm riser (model `ramp` in blocks_world_long.sdf) on the table top (z = 0.30): 0.30 + 0.058
        arguments=["-string", robot_description, "-name", "gen3", "-x", "0.0", "-y", "0.0", "-z", "0.358"],
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

    actions = [
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(get_package_prefix("robotiq_description"), "share")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(share, "worlds")),
        clock_bridge,
        robot_state_publisher,
        gz_sim,
        spawn,
        start_spawners,
        *chain,
    ]
    if camera:
        actions.append(camera_bridge)
    if cfg["weld"]:
        detach_all = ExecuteProcess(cmd=["python3", "-c", DETACH_ALL.replace("{letters}", LETTERS).replace("{{", "{").replace("}}", "}")], output="screen")
        actions.append(RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[detach_all])))
    if cfg["moveit"]:
        actions += moveit_actions(cfg["rviz"], camera)
    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
