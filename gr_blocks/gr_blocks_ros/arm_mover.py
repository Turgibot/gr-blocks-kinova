"""ArmMover: a ROS 2 node whose only purpose is to move the arm.

Basic operations are small helper methods. Complex movements (e.g. grab a cube from the side) are built later by
calling these helpers in sequence.
"""
import atexit
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [f"joint_{i}" for i in range(1, 8)]
EE_LINK = "end_effector_link"  # the tool frame that move_ik places
# Axes of end_effector_link (from the URDF, finger_joint = 0): the fingertips are at +z (approach axis, 0.177 m ahead);
# the pads sit at x = +-0.068 (closing axis); y is the third axis.
ALIGN_STRAIGHT = 0.4  # align: this fraction of the start-to-goal distance (and of the duration) is the straight final approach

# finger_joint [rad] -> gap between the finger pad frames [mm], measured in the sim (docs/PHASE2_NOTES.md, gripper_gap)
_ANGLE = [0.0, 0.2, 0.4, 0.5, 0.6, 0.7]
_FRAME_GAP_MM = [134.2, 105.4, 69.1, 50.0, 30.4, 10.6]
_PAD_THICKNESS_MM = 7.5  # pad collision box along its normal (URDF); contact-surface gap = frame gap - this


class MoveHandle:
    """Returned by every move_* call, which starts the move and returns at once.

    done():        True when the move has finished (successfully, with an error, or cancelled).
    wait(timeout): block until done; returns True when finished, False on timeout; raises RuntimeError if the move failed
                   or was cancelled.
    cancel():      stop the move (the handle then counts as failed with "cancelled").
    """

    def __init__(self):
        self._event = threading.Event()
        self._error = None
        self._cancel_fn = None
        self._callbacks = []

    def done(self):
        return self._event.is_set()

    def wait(self, timeout=None):
        if not self._event.wait(timeout):
            return False
        if self._error:
            raise RuntimeError(self._error)
        return True

    def cancel(self):
        if self._cancel_fn is not None and not self.done():
            self._cancel_fn()
        self._finish("cancelled")

    def _finish(self, error=None):
        if not self._event.is_set():  # the first outcome wins
            self._error = error
            self._event.set()
            for fn in self._callbacks:
                fn()

    def _add_done_callback(self, fn):
        """Internal: call fn() when the handle finishes (at once if it already has)."""
        self._callbacks.append(fn)
        if self._event.is_set():
            fn()


CONTINUOUS = (0, 2, 4, 6)  # joint_1, 3, 5, 7: MoveIt refuses to plan when one of them reads outside +-pi (CheckStartStateBounds)
BOUND_MARGIN = 0.003  # rad, kept inside +-pi so a small tracking overshoot of the controller still reads inside


def _keep_inside_pi(positions):
    """The 7 joint `positions` with the continuous joints limited to +-(pi - BOUND_MARGIN)."""
    limit = np.pi - BOUND_MARGIN
    return [float(np.clip(p, -limit, limit)) if i in CONTINUOUS else float(p) for i, p in enumerate(positions)]


def _seconds(d):
    return d.sec + d.nanosec * 1e-9


def _duration(t):
    return Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))


def _retime(traj, duration):
    """Stretch the timing of a JointTrajectory to `duration` seconds in place. False if it is already at its goal."""
    pts = traj.points
    if not pts:
        return False
    end = _seconds(pts[-1].time_from_start)
    if end <= 0.0:
        if len(pts) < 2:
            return False
        for i, p in enumerate(pts):  # no timing given (e.g. a Cartesian path): spread the points evenly
            p.time_from_start = _duration(duration * i / (len(pts) - 1))
        return True
    f = duration / end
    for p in pts:
        p.time_from_start = _duration(_seconds(p.time_from_start) * f)
        p.velocities = [v / f for v in p.velocities]
        p.accelerations = [a / f**2 for a in p.accelerations]
    return True


def _quaternion(R):
    """Rotation matrix (3x3) -> quaternion (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        w = np.sqrt(1 + t) / 2
        return ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    r = np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) / 2
    q = [0.0] * 4
    q[i] = r
    q[j] = (R[j, i] + R[i, j]) / (4 * r)
    q[k] = (R[k, i] + R[i, k]) / (4 * r)
    q[3] = (R[k, j] - R[j, k]) / (4 * r)
    return tuple(q)


def _aim_quaternion(position, target, roll=0.0):
    """Orientation of end_effector_link (quaternion x, y, z, w) whose approach axis (+z) aims from `position` at `target`.

    The closing axis (x) is horizontal when roll = 0 (x = world z cross approach); `roll` turns the gripper about the
    approach axis. Not defined when aiming straight up or down at roll = 0 only in the sign of x (a fallback is used).
    """
    a = np.asarray(target, float) - np.asarray(position, float)
    if np.linalg.norm(a) < 1e-6:
        raise ValueError("target and position are the same point")
    a /= np.linalg.norm(a)
    c = np.cross([0.0, 0.0, 1.0], a)
    if np.linalg.norm(c) < 1e-6:  # aiming straight up or down: any horizontal closing axis will do
        c = np.array([1.0, 0.0, 0.0])
    c /= np.linalg.norm(c)
    R = np.column_stack([c, np.cross(a, c), a])  # columns = ee x, y, z in base_link
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    Rroll = np.eye(3) + np.sin(roll) * K + (1 - np.cos(roll)) * (K @ K)  # Rodrigues: turn about the approach axis
    return _quaternion(Rroll @ R)


class ArmMover(Node):
    def __init__(self, collision_check=True, pointing=False, align=False):
        """Default behaviour of the moves; each move may later override it.

        collision_check: True = plan with MoveIt (avoids collisions); False = straight to the trajectory controller.
        pointing:        True = the gripper's approach axis aims at a target point.
        align:           (only with pointing) True = first line up with the target, then go straight to it.
        """
        super().__init__("arm_mover")
        if align and not pointing:
            raise ValueError("align needs pointing=True")
        self._collision_check = bool(collision_check)
        self._pointing = bool(pointing)
        self._align = bool(align)
        self._arm_traj = ActionClient(self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")
        self._move_group = ActionClient(self, MoveGroup, "/move_action")
        self._ik = self.create_client(GetPositionIK, "/compute_ik")
        self._cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._gripper = ActionClient(self, ParallelGripperCommand, "/robotiq_gripper_controller/gripper_cmd")
        self._joints = {}  # latest /joint_states: name -> position
        self.create_subscription(JointState, "/joint_states", lambda m: self._joints.update(zip(m.name, m.position)), 10)
        # TODO: use self._joints for the arm joints (the finger is already used by move_gripper_angle)
        # TODO: named poses loaded from config/poses.json
        # The node spins itself in a background thread: no move_* call blocks on the executor, so moves can run in parallel.
        # (All callbacks share the node's default callback group, so they run one at a time: no locks needed.)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        atexit.register(self._stop_spinning)  # stop the thread before the interpreter exits (else rclpy aborts at exit)

    def _stop_spinning(self):
        self._executor.shutdown()
        self._spin_thread.join(timeout=2.0)

    # ---- gripper ----
    def move_gripper_angle(self, angle, duration=1.0):
        """Start moving finger_joint to `angle` (rad) in `duration` seconds; returns a MoveHandle.

        The gripper action has no duration, so a timer sends a waypoint every 0.1 s (a straight line in angle from the
        current finger_joint position); the handle finishes when the last waypoint is reached (about `duration` s).
        """
        if duration <= 0:
            raise ValueError(f"duration must be > 0, got {duration}")
        if not self._gripper.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("no /robotiq_gripper_controller/gripper_cmd: is the sim running in this ROS_DOMAIN_ID?")
        deadline = time.monotonic() + 5.0
        while "finger_joint" not in self._joints:  # filled by the executor thread
            if time.monotonic() > deadline:
                raise RuntimeError("no finger_joint in /joint_states")
            time.sleep(0.05)
        start = self._joints["finger_joint"]
        steps = max(1, round(duration / 0.1))
        handle = MoveHandle()
        state = {"k": 0, "goal": None}

        def on_result(fut):
            handle._finish()

        def on_response(fut):
            state["goal"] = fut.result()
            if not state["goal"].accepted:
                handle._finish("gripper goal rejected")
            elif state["k"] == steps:  # the last waypoint: finished when it is reached
                state["goal"].get_result_async().add_done_callback(on_result)

        def tick():
            if handle.done():
                timer.cancel()
                return
            state["k"] += 1
            goal = ParallelGripperCommand.Goal()
            goal.command.name = ["finger_joint"]
            goal.command.position = [float(start + (angle - start) * state["k"] / steps)]
            self._gripper.send_goal_async(goal).add_done_callback(on_response)
            if state["k"] == steps:
                timer.cancel()

        def cancel():
            timer.cancel()
            if state["goal"] is not None:
                state["goal"].cancel_goal_async()

        timer = self.create_timer(duration / steps, tick)
        handle._cancel_fn = cancel
        return handle

    def move_gripper_dist(self, mm, duration=1.0):
        """Open the gripper so the contact surfaces of the fingers are `mm` millimetres apart (0.0 to 100.0).

        mm is converted to a finger_joint angle by interpolating the measured table above (frame gap - pad thickness),
        then move_gripper_angle starts the move, taking `duration` seconds. Returns its MoveHandle.
        """
        if not 0.0 <= mm <= 100.0:
            raise ValueError(f"mm must be in 0.0..100.0, got {mm}")
        surface_gap_mm = [g - _PAD_THICKNESS_MM for g in _FRAME_GAP_MM]
        return self.move_gripper_angle(np.interp(mm, surface_gap_mm[::-1], _ANGLE[::-1]), duration)  # np.interp clamps outside the table

    def open(self, mm):
        """TODO: open the gripper so the contact surfaces of the fingers are `mm` millimetres apart."""

    def close(self):
        """TODO: close the gripper."""

    # ---- arm: joint space (FK) ----
    def move_fk(self, angles, duration=3.0, collision=None):
        """Start moving the arm to the 7 joint `angles` (rad) in `duration` seconds; returns a MoveHandle.

        collision: None = use the default from __init__.
          False: straight to the trajectory controller (no collision check).
          True:  MoveIt plans a collision-free path first (needs move_group); its timing is then stretched to `duration`
                 (a duration much shorter than MoveIt's own may exceed the joint velocity limits).
        """
        if len(angles) != 7:
            raise ValueError(f"angles needs 7 values, got {len(angles)}")
        if duration <= 0:
            raise ValueError(f"duration must be > 0, got {duration}")
        collision = self._collision_check if collision is None else collision
        client = self._move_group if collision else self._arm_traj
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"no action server for {'MoveIt (/move_action)' if collision else 'the arm trajectory controller'}: "
                               "is the sim (with moveit true for collision checking) running in this ROS_DOMAIN_ID?")
        handle = MoveHandle()
        self._start_fk(handle, [float(a) for a in angles], duration, collision)
        return handle

    def _start_fk(self, handle, angles, duration, collision):
        """The non-waiting part of move_fk (also called from callbacks, where blocking is not allowed)."""
        if collision:
            self._plan_then_run(handle, angles, duration)
        else:
            point = JointTrajectoryPoint(positions=angles, time_from_start=_duration(duration))
            self._run_trajectory(handle, JointTrajectory(joint_names=ARM_JOINTS, points=[point]))

    def _run_trajectory(self, handle, trajectory):
        """Send `trajectory` to the arm controller; finish `handle` when it is done."""
        for point in trajectory.points:  # a joint just past +-pi would make MoveIt reject every later plan
            point.positions = _keep_inside_pi(point.positions)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        def on_result(fut):
            r = fut.result().result
            handle._finish(None if r.error_code == FollowJointTrajectory.Result.SUCCESSFUL else f"trajectory error {r.error_code}: {r.error_string}")

        def on_response(fut):
            gh = fut.result()
            if not gh.accepted:
                handle._finish("arm trajectory goal rejected")
                return
            handle._cancel_fn = gh.cancel_goal_async
            gh.get_result_async().add_done_callback(on_result)

        self._arm_traj.send_goal_async(goal).add_done_callback(on_response)

    def _plan_then_run(self, handle, angles, duration):
        """MoveIt plan-only to the joint goal, stretch the timing to `duration`, then run it on the arm controller."""
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = "manipulator"
        r.pipeline_id = "ompl"
        r.num_planning_attempts = 5
        r.allowed_planning_time = 5.0
        r.max_velocity_scaling_factor = 0.3
        r.max_acceleration_scaling_factor = 0.3
        c = Constraints()
        for name, v in zip(ARM_JOINTS, _keep_inside_pi(angles)):
            c.joint_constraints.append(JointConstraint(joint_name=name, position=v, tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        r.goal_constraints.append(c)
        goal.planning_options.plan_only = True

        def on_result(fut):
            res = fut.result().result
            if res.error_code.val != MoveItErrorCodes.SUCCESS:
                handle._finish(f"MoveIt planning failed (error code {res.error_code.val})")
                return
            traj = res.planned_trajectory.joint_trajectory
            if not _retime(traj, duration):  # already at the goal
                handle._finish()
                return
            self._run_trajectory(handle, traj)

        def on_response(fut):
            gh = fut.result()
            if not gh.accepted:
                handle._finish("MoveIt goal rejected")
                return
            handle._cancel_fn = gh.cancel_goal_async
            gh.get_result_async().add_done_callback(on_result)

        self._move_group.send_goal_async(goal).add_done_callback(on_response)

    # ---- arm: Cartesian space (IK) ----
    def move_ik(self, position, orientation=None, duration=3.0, collision=None, target=None, pointing=None, align=None, roll=0.0, straight=False):
        """Start moving the end effector (end_effector_link) to `position` (x, y, z in base_link, m); returns a MoveHandle.

        orientation: quaternion (x, y, z, w) in base_link; not used (and may be None) when pointing.
        collision:   None = default from __init__. True = collision-free IK solution and a MoveIt-planned path; False = IK
                     ignores collisions and the move goes straight to the controller.
        target:      point (x, y, z in base_link) to aim at; required when pointing.
        pointing:    None = default from __init__. True = the orientation is computed so the approach axis (+z of
                     end_effector_link) aims from `position` at `target`; the closing axis is horizontal.
        roll:        (pointing) rotation about the approach axis in rad; 0 = fingers close horizontally.
        align:       None = default from __init__; needs pointing. True = first move to the line-up pose on the approach
                     line, 50 % of the way (joint-space move, 50 % of `duration`), then a straight Cartesian line to
                     `position` for the last 50 % (50 % of `duration`).
        straight:    True = go from the current pose to `position` along a straight Cartesian line (no IK, no line-up);
                     the orientation is the given one (or the pointing one), so give the current orientation to keep it.
                     Not together with align.

        IK is MoveIt's /compute_ik, the straight line is /compute_cartesian_path: both need move_group (moveit true in
        gr_blocks.json), even with collision=False.
        """
        if len(position) != 3:
            raise ValueError("position needs 3 values (x, y, z)")
        if duration <= 0:
            raise ValueError(f"duration must be > 0, got {duration}")
        pointing = self._pointing if pointing is None else pointing
        align = self._align if align is None else align
        if align and not pointing:
            raise ValueError("align needs pointing")
        if straight and align:
            raise ValueError("straight and align cannot be used together")
        if pointing:
            if target is None or len(target) != 3:
                raise ValueError("pointing needs target (x, y, z)")
            orientation = _aim_quaternion(position, target, roll)
        elif orientation is None or len(orientation) != 4:
            raise ValueError("orientation needs 4 values (x, y, z, w)")
        collision = self._collision_check if collision is None else collision
        if (not straight and not self._ik.wait_for_service(timeout_sec=10.0)) or (
                (align or straight) and not self._cartesian.wait_for_service(timeout_sec=10.0)):
            raise RuntimeError("no /compute_ik or /compute_cartesian_path: is move_group running (moveit true in gr_blocks.json)?")
        client = self._move_group if collision and not straight else self._arm_traj
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("no action server for the move: is the sim running in this ROS_DOMAIN_ID?")

        handle = MoveHandle()
        if straight:
            self._start_cartesian(handle, position, orientation, duration, collision)
            return handle
        if not align:
            self._start_ik(handle, position, orientation, duration, collision)
            return handle

        deadline = time.monotonic() + 5.0
        while not self._tf_buffer.can_transform("base_link", EE_LINK, rclpy.time.Time()):  # the listener needs a moment after start-up
            if time.monotonic() > deadline:
                raise RuntimeError("no TF base_link -> end_effector_link: is the sim running in this ROS_DOMAIN_ID?")
            time.sleep(0.05)
        now = self._tf_buffer.lookup_transform("base_link", EE_LINK, rclpy.time.Time()).transform.translation
        start = np.array([now.x, now.y, now.z])
        goal = np.asarray(position, float)
        direction = np.asarray(target, float) - goal
        direction /= np.linalg.norm(direction)
        line_up = goal - ALIGN_STRAIGHT * np.linalg.norm(goal - start) * direction  # on the approach line, behind `position`
        first = MoveHandle()
        handle._cancel_fn = first.cancel

        def after_first():
            if handle.done():
                return
            if first._error:
                handle._finish(first._error)
                return
            handle._cancel_fn = None
            self._start_cartesian(handle, goal, orientation, ALIGN_STRAIGHT * duration, collision)

        first._add_done_callback(after_first)
        self._start_ik(first, line_up, orientation, (1 - ALIGN_STRAIGHT) * duration, collision)
        return handle

    def _start_ik(self, handle, position, orientation, duration, collision):
        """Solve IK for the pose, then start the joint-space move (the non-waiting part of move_ik)."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = "manipulator"
        req.ik_request.ik_link_name = EE_LINK
        req.ik_request.avoid_collisions = bool(collision)
        req.ik_request.timeout = _duration(1.0)
        pose = req.ik_request.pose_stamped
        pose.header.frame_id = "base_link"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (float(v) for v in position)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = (float(v) for v in orientation)

        def on_ik(fut):
            if handle.done():  # cancelled while solving
                return
            res = fut.result()
            if res.error_code.val != MoveItErrorCodes.SUCCESS:
                handle._finish(f"no IK solution (MoveIt error code {res.error_code.val})")
                return
            q = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            self._start_fk(handle, [q[j] for j in ARM_JOINTS], duration, collision)

        self._ik.call_async(req).add_done_callback(on_ik)

    def _start_cartesian(self, handle, position, orientation, duration, collision):
        """Straight Cartesian line from the current pose to the pose, timed to `duration`."""
        req = GetCartesianPath.Request()
        req.group_name = "manipulator"
        req.link_name = EE_LINK
        req.header.frame_id = "base_link"
        req.max_step = 0.01
        req.avoid_collisions = bool(collision)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (float(v) for v in orientation)
        req.waypoints = [pose]

        def on_path(fut):
            if handle.done():
                return
            res = fut.result()
            if res.fraction < 0.99:
                handle._finish(f"straight line only {res.fraction:.0%} feasible (collision or limits)")
                return
            traj = res.solution.joint_trajectory
            if not _retime(traj, duration):
                handle._finish()
                return
            self._run_trajectory(handle, traj)

        self._cartesian.call_async(req).add_done_callback(on_path)

    # ---- named poses (angles in config/poses.json, executed with move_fk) ----
    def go_home(self):
        """TODO: move_fk(poses['home'])."""

    def go_zero(self):
        """TODO: move_fk(poses['zero']) (all joints 0; not yet in poses.json)."""

    def go_gaze(self):
        """TODO: move_fk(poses['gaze']) (camera sees all cubes; today called 'observe' in poses.json)."""
