"""Arm primitives for the block world (Gazebo ground truth): gripper, pose goals, Cartesian moves, block poses.

Frames: everything is in `base_link` (see world_state.py). Gripper geometry is measured from TF at start-up:
  * tcp  = midpoint of the two finger pads, expressed in end_effector_link;
  * approach axis a = direction from the flange to the tcp, closing axis c = direction between the pads.
A grasp orientation is chosen so that a points straight down and c points along base x (towers are spaced along y).
"""
import os
import time

import numpy as np
import rclpy
from control_msgs.action import ParallelGripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, BoundingVolume, CollisionObject, Constraints, JointConstraint,
                             MoveItErrorCodes, OrientationConstraint, PlanningScene, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from gr_blocks_ros.move_named import named_pose
from gr_blocks_ros.sim_patches import LETTERS, attach_topic, detach_topic

FRAME, EE = "base_link", "end_effector_link"
LEFT, RIGHT = "left_inner_finger_pad", "right_inner_finger_pad"
GROUP = "manipulator"
CUBE = 0.045
DROP = 0.002  # m: a cube is released this far above its support
TOUCH = ["robotiq_140_base_link", "left_outer_knuckle", "left_outer_finger", "left_inner_finger", "left_inner_finger_pad",
         "left_inner_knuckle", "right_outer_knuckle", "right_outer_finger", "right_inner_finger", "right_inner_finger_pad",
         "right_inner_knuckle", "end_effector_link", "bracelet_link"]


def make_pose(p, quat):
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = (float(v) for v in p)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = (float(v) for v in quat)
    return msg


class Arm(Node):
    def __init__(self):
        super().__init__("arm_skills", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.js = {}
        self.create_subscription(JointState, "/joint_states", lambda m: self.js.update(zip(m.name, m.position)), 10)
        self.grip = ActionClient(self, ParallelGripperCommand, "/robotiq_gripper_controller/gripper_cmd")
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.exec = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.cart = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.held_pub = self.create_publisher(String, "/held_block", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.held = None
        self.last_error = None
        self.rigid = False  # True: weld the cube to the gripper in Gazebo while it is held (sim launch rigid_grasp:=true)
        self._gz = None
        self._gz_pub = {}

    # ---- plumbing ---------------------------------------------------------------------------------------------
    def spin_until(self, fut, timeout):
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None

    def settle(self, seconds):
        """Spin for `seconds` of simulated time AND of wall time (whichever is longer). The wall-time floor matters after
        a blocking pause: the backlog of /clock messages would otherwise use the whole wait up at once, before the
        newest TF / poses have been received."""
        end = self.get_clock().now() + Duration(seconds=seconds)
        wall_end = time.monotonic() + seconds
        while self.get_clock().now() < end or time.monotonic() < wall_end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self):
        for c in (self.grip, self.move, self.exec):
            if not c.wait_for_server(timeout_sec=30.0):
                raise RuntimeError("action server missing")
        if not self.cart.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("compute_cartesian_path missing")
        self.settle(1.0)
        deadline = time.time() + 30.0  # TF may lag behind the controllers at start-up
        while not (self.buf.can_transform(EE, LEFT, rclpy.time.Time()) and self.buf.can_transform(EE, RIGHT, rclpy.time.Time())):
            if time.time() > deadline:
                raise RuntimeError(f"no TF between {EE} and the finger pads after 30 s")
            rclpy.spin_once(self, timeout_sec=0.2)
        self._calibrate()
        self.clear_attached()
        self._maybe_enable_rigid()

    def lookup(self, target, source=FRAME, timeout=3.0):
        t = self.buf.lookup_transform(source, target, rclpy.time.Time(), Duration(seconds=timeout)).transform
        return (np.array([t.translation.x, t.translation.y, t.translation.z]),
                np.array([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]))

    def _calibrate(self):
        pl, _ = self.lookup(LEFT, EE)
        pr, _ = self.lookup(RIGHT, EE)
        self.tcp = (pl + pr) / 2  # in EE frame
        a = self.tcp / np.linalg.norm(self.tcp)
        c = pr - pl
        c = c - a * (c @ a)
        c /= np.linalg.norm(c)
        b = np.cross(c, a)
        self._E = np.column_stack([c, a, b])  # gripper axes (closing, approach, third) in the EE frame
        self.get_logger().info(f"tcp in EE frame = {np.round(self.tcp, 4)}  closing={np.round(c, 3)} approach={np.round(a, 3)}")

    # ---- gripper ----------------------------------------------------------------------------------------------
    def finger(self):
        return self.js.get("finger_joint")

    def gripper(self, angle, settle=1.0):
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ["finger_joint"]
        goal.command.position = [float(angle)]
        h = self.spin_until(self.grip.send_goal_async(goal), 10.0)
        if h is None or not h.accepted:
            raise RuntimeError("gripper goal rejected")
        self.spin_until(h.get_result_async(), 30.0)
        self.settle(settle)
        return self.finger()

    # ---- orientation / tcp helpers ------------------------------------------------------------------------------
    def grasp_quat(self, yaw=0.0):
        """EE orientation (in base_link) with approach pointing down and the closing axis along base x (rotated by yaw)."""
        B = np.column_stack([[1, 0, 0], [0, 0, -1], [0, 1, 0]])  # targets for (closing, approach, third), right-handed
        R = Rotation.from_euler("z", yaw).as_matrix() @ B @ self._E.T
        return Rotation.from_matrix(R).as_quat()

    def ee_for_tcp(self, tcp_pos, quat):
        """Position of end_effector_link that puts the tcp at tcp_pos for the given orientation."""
        return np.asarray(tcp_pos) - Rotation.from_quat(quat).apply(self.tcp)

    def approach_axis(self):
        """Unit vector, in base_link, from the flange towards the fingertips (straight down = (0, 0, -1))."""
        _, q = self.lookup(EE)
        return Rotation.from_quat(q).apply(self.tcp / np.linalg.norm(self.tcp))

    def tcp_now(self):
        p, q = self.lookup(EE)
        return p + Rotation.from_quat(q).apply(self.tcp), q

    # ---- motion -----------------------------------------------------------------------------------------------
    def move_to(self, ee_pos, quat, tries=4, vel=0.3):
        """Plan + execute a free-space move of end_effector_link to the pose (retries: ValidateSolution can reject)."""
        for k in range(tries):
            goal = MoveGroup.Goal()
            r = goal.request
            r.group_name, r.pipeline_id = GROUP, "ompl"
            r.num_planning_attempts, r.allowed_planning_time = 10, 5.0
            r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = vel
            c = Constraints()
            pc = PositionConstraint()
            pc.header.frame_id, pc.link_name, pc.weight = FRAME, EE, 1.0
            pc.constraint_region = BoundingVolume(
                primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002])],
                primitive_poses=[make_pose(ee_pos, [0, 0, 0, 1])])
            oc = OrientationConstraint()
            oc.header.frame_id, oc.link_name, oc.weight = FRAME, EE, 1.0
            oc.orientation = make_pose([0, 0, 0], quat).orientation
            oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 0.02
            c.position_constraints.append(pc)
            c.orientation_constraints.append(oc)
            r.goal_constraints.append(c)
            h = self.spin_until(self.move.send_goal_async(goal), 15.0)
            if h is None or not h.accepted:
                raise RuntimeError("move goal rejected")
            res = self.spin_until(h.get_result_async(), 120.0)
            code = res.result.error_code.val if res else None
            if code == MoveItErrorCodes.SUCCESS:
                self.settle(0.5)
                return True
            self.get_logger().warn(f"move_to attempt {k + 1}/{tries} failed, error {code}")
        return False

    def move_joints(self, target, tries=3, vel=0.3):
        """Plan + execute a joint-space move; target: {joint name: angle}."""
        for k in range(tries):
            goal = MoveGroup.Goal()
            r = goal.request
            r.group_name, r.pipeline_id = GROUP, "ompl"
            r.num_planning_attempts, r.allowed_planning_time = 10, 5.0
            r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = vel
            c = Constraints()
            for j, v in target.items():
                c.joint_constraints.append(JointConstraint(joint_name=j, position=float(v), tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
            r.goal_constraints.append(c)
            h = self.spin_until(self.move.send_goal_async(goal), 15.0)
            if h is None or not h.accepted:
                raise RuntimeError("move goal rejected")
            res = self.spin_until(h.get_result_async(), 120.0)
            if res and res.result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.settle(0.5)
                return True
            self.get_logger().warn(f"move_joints attempt {k + 1}/{tries} failed")
        return False

    def go_named(self, name="Home"):
        """Move to an SRDF pose of the manipulator group (a known, reachable configuration to restart approaches from)."""
        return self.move_joints(named_pose(name))

    def cartesian(self, ee_waypoints, quat, avoid_collisions=True, vel=0.1, min_fraction=0.99):
        """Straight-line moves of end_effector_link through the waypoints (fixed orientation)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.group_name, req.link_name = GROUP, EE
        if hasattr(req, "start_state"):
            req.start_state.is_diff = True
        req.waypoints = [make_pose(p, quat) for p in ee_waypoints]
        req.max_step, req.jump_threshold = 0.005, 0.0
        req.avoid_collisions = avoid_collisions
        req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = vel
        resp = self.spin_until(self.cart.call_async(req), 15.0)
        if resp is None or resp.fraction < min_fraction:
            self.get_logger().warn(f"cartesian path fraction {None if resp is None else round(resp.fraction, 3)}")
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = resp.solution
        h = self.spin_until(self.exec.send_goal_async(goal), 15.0)
        if h is None or not h.accepted:
            return False
        res = self.spin_until(h.get_result_async(), 120.0)
        self.settle(0.5)
        return res is not None and res.result.error_code.val == MoveItErrorCodes.SUCCESS

    # ---- world ------------------------------------------------------------------------------------------------
    def block(self, letter):
        return self.lookup(f"block_{letter}")[0]

    def _apply(self, scene, must=True):
        if not self.scene.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("apply_planning_scene missing")
        resp = self.spin_until(self.scene.call_async(ApplyPlanningScene.Request(scene=scene)), 10.0)
        if must and (resp is None or not resp.success):
            raise RuntimeError("apply_planning_scene failed")

    def attach(self, letter):
        """The cube in the gripper becomes an attached object of the end effector (no collision with the gripper)."""
        self.held_pub.publish(String(data=letter))  # world_state stops treating it as a world object
        self.settle(0.5)
        cp, cq = self.lookup(f"block_{letter}")
        ep, eq = self.lookup(EE)
        rel_p = Rotation.from_quat(eq).inv().apply(cp - ep)
        rel_q = (Rotation.from_quat(eq).inv() * Rotation.from_quat(cq)).as_quat()
        obj = CollisionObject(id=f"block_{letter}", operation=CollisionObject.ADD)
        obj.header.frame_id = EE
        obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[CUBE] * 3))
        obj.primitive_poses.append(make_pose(rel_p, rel_q))
        drop = PlanningScene(is_diff=True)  # best effort: the cube may or may not be a world object right now
        drop.world.collision_objects.append(CollisionObject(id=f"block_{letter}", operation=CollisionObject.REMOVE))
        self._apply(drop, must=False)
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(
            AttachedCollisionObject(link_name=EE, object=obj, touch_links=TOUCH))
        self._apply(scene)
        self.held = letter

    def clear_attached(self, letters="ABEFHLMORSTW"):
        """Remove any cube still attached to the gripper in MoveIt's scene (left over from an earlier run)."""
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        for l in letters:
            scene.robot_state.attached_collision_objects.append(AttachedCollisionObject(
                link_name=EE, object=CollisionObject(id=f"block_{l}", operation=CollisionObject.REMOVE)))
        self._apply(scene, must=False)
        self.held_pub.publish(String(data=""))
        self.settle(0.5)
        self.held = None

    def detach(self):
        letter, self.held = self.held, None
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(AttachedCollisionObject(
            link_name=EE, object=CollisionObject(id=f"block_{letter}", operation=CollisionObject.REMOVE)))
        self._apply(scene)
        self.held_pub.publish(String(data=""))  # world_state puts it back into the scene at its measured pose
        self.settle(0.5)

    # ---- rigid grasp (sim only) ---------------------------------------------------------------------------------
    def enable_rigid_grasp(self):
        """Use the DetachableJoint plugins of the sim launch (rigid_grasp:=true): advertise every attach/detach topic."""
        from gz.msgs10.empty_pb2 import Empty
        from gz.transport13 import Node as GzNode
        self._gz = GzNode()
        self._empty = Empty
        for l in LETTERS:
            self._gz_pub[("a", l)] = self._gz.advertise(attach_topic(l), Empty)
            self._gz_pub[("d", l)] = self._gz.advertise(detach_topic(l), Empty)
        self.rigid = True
        self.settle(1.0)  # let the subscribers in Gazebo discover the publishers
        # The plugin welds every cube to the gripper when the robot is spawned (it starts 'attached'); the arm has not
        # moved yet, so releasing them all now leaves the layout untouched.
        for l in LETTERS:
            self._gz_pub[("d", l)].publish(self._empty())
        self.settle(1.0)

    def _maybe_enable_rigid(self):
        """Rigid grasp is on when the sim was launched with rigid_grasp:=true (its attach topics exist in Gazebo);
        GR_RIGID_GRASP=0 / 1 forces it off / on."""
        force = os.environ.get("GR_RIGID_GRASP")
        if force == "0":
            return
        if force == "1":
            return self.enable_rigid_grasp()
        from gz.transport13 import Node as GzNode
        probe = GzNode()
        for _ in range(20):  # discovery needs a moment
            if attach_topic("L") in probe.topic_list():
                return self.enable_rigid_grasp()
            self.settle(0.25)

    def _gz_weld(self, kind, letter):
        self._gz_pub[(kind, letter)].publish(self._empty())
        self.settle(0.3)

    # ---- pick -------------------------------------------------------------------------------------------------
    def pick(self, letter, dz=0.0125, close=0.46, lift=0.10, hover=0.12):
        """Grasp the cube from above (grasp point dz above its centre so the 70 mm pads clear the cube below), lift it.

        The gripper keeps holding only while this node keeps running its goal: call place_* in the same process."""
        q = self.grasp_quat()
        up = np.array([0, 0, 1.0])
        target = self.block(letter) + up * dz
        hover_ee = self.ee_for_tcp(target + up * hover, q)
        grasp_ee = self.ee_for_tcp(target, q)
        for attempt in range(3):  # some arm configurations cannot finish the vertical descent: restart from Home
            self.gripper(0.0)
            if self.move_to(hover_ee, q) and self.cartesian([grasp_ee], q):
                break
            if attempt == 2:
                return False
            self.get_logger().warn(f"approach to block_{letter} failed (attempt {attempt + 1}/3), restarting from Home")
            if not self.go_named("Home"):
                return False
        self.gripper(close, settle=1.5)
        if self.rigid:
            self._gz_weld("a", letter)
            print(letter, "before close")
            self.gripper(max(0.0, close - 0.08), settle=0.5)  # ease the pads off: the weld carries the cube, no contact fight
            print(letter, "after close")
        print("before attach ", letter)    
        self.attach(letter)
        print("after attach ", letter)    
        z0 = self.block(letter)[2]
        if not self.cartesian([grasp_ee + up * lift], q, avoid_collisions=False):
            return False
        self.settle(0.5)
        rose = self.block(letter)[2] - z0
        if rose < lift - 0.015:  # the cube did not come along: the grasp failed
            self.get_logger().warn(f"pick of block_{letter}: cube rose {1000 * rose:.0f} mm of {1000 * lift:.0f} mm, grasp failed")
            self.detach()
            return False
        return True

    # ---- place ------------------------------------------------------------------------------------------------
    def open_slowly(self, step=0.03, dt=0.1):
        """Open the gripper in small steps instead of one jump (gentler release)."""
        f = self.finger() or 0.0
        while f > 0.0:
            f = max(0.0, f - step)
            self.gripper(f, settle=dt)
        self.gripper(0.0, settle=0.5)

    def place_at(self, center_xy, center_z, hover=0.10, vel=0.3, slow_open=False):
        """Put the held cube down with its centre at (x, y, z); then open, detach and retreat. Orientation: grasp_quat."""
        letter = self.held
        q = self.grasp_quat()
        cube_now = self.block(letter)
        tcp_now, _ = self.tcp_now()
        offset = tcp_now - cube_now  # tcp relative to the held cube (the cube rides with the gripper)
        goal_center = np.array([center_xy[0], center_xy[1], center_z])
        up = np.array([0, 0, 1.0])
        grasp_ee = self.ee_for_tcp(goal_center + offset, q)
        self._log_offset("start of place", letter, offset)
        for attempt in range(3):  # a configuration that cannot finish the vertical descent: restart from Home
            if (self.move_to(grasp_ee + up * hover, q, vel=vel)
                    and self.cartesian([grasp_ee + up * 0.012], q)  # collision-checked down to 12 mm above the support ...
                    and self.cartesian([grasp_ee], q, avoid_collisions=False)):  # ... then the last 12 mm on purpose
                break
            if attempt == 2:
                return False
            self.get_logger().warn(f"approach to the place position failed (attempt {attempt + 1}/3), restarting from Home")
            if not self.go_named("Home"):
                return False
        self._log_offset("at target (before release)", letter, offset)
        if self.rigid:
            self._gz_weld("d", letter)
        if slow_open:
            self.open_slowly()
        else:
            self.gripper(0.0, settle=1.0)
        self.detach()
        self.settle(1.0)
        err = self.block(letter) - goal_center
        self.get_logger().info(f"cube centre minus goal after release (mm): {np.round(1000 * err, 1)}")
        ok = self.cartesian([grasp_ee + up * hover], q) or self.cartesian([grasp_ee + up * hover], q, avoid_collisions=False)
        self.settle(1.0)
        self.last_error = self.block(letter) - goal_center  # final, after the gripper has retreated
        return ok

    def _log_offset(self, where, letter, ref):
        tcp, _ = self.tcp_now()
        d = (tcp - self.block(letter)) - ref
        self.get_logger().info(f"tcp-minus-cube offset change {where} (mm): {np.round(1000 * d, 1)}")

    def place_on(self, letter, drop=DROP, **kw):
        t = self.block(letter)
        return self.place_at(t[:2], t[2] + CUBE + drop, **kw)


def timestamp():
    return time.strftime("%H:%M:%S")
