"""Ground truth of the Gazebo world for ROS / MoveIt.

Subscribes (gz transport, Python bindings) to the world's dynamic pose stream, then
  * broadcasts TF  base_link -> block_<L>  for every cube,
  * keeps the planning scene up to date: the table plus one box per cube (CollisionObjects in frame base_link).

Frames: Gazebo places the robot base at BASE_IN_GZ (z = table top), MoveIt's base_link is its own origin, so a Gazebo
position p becomes  p - BASE_IN_GZ  (no rotation: the robot is spawned with zero yaw).

    ros2 run gr_blocks_ros world_state
"""
import threading

import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from shape_msgs.msg import SolidPrimitive
from tf2_ros import TransformBroadcaster

from gr_blocks_ros.make_world import CUBE, TABLE_X0, TABLE_X1, TABLE_Y, TABLE_Z

FRAME = "base_link"
BASE_IN_GZ = (0.0, 0.0, TABLE_Z)
MOVE_EPS = 0.001  # m: a cube must move this much before the planning scene is updated


def _pose(p, q, ):
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = (p[i] - BASE_IN_GZ[i] for i in range(3))
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
    return msg


def _box(name, size, pose):
    o = CollisionObject()
    o.header.frame_id = FRAME
    o.id = name
    o.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size)))
    o.primitive_poses.append(pose)
    o.operation = CollisionObject.ADD
    return o


class WorldState(Node):
    def __init__(self):
        super().__init__("world_state", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.declare_parameter("world", "blocks_world")
        world = self.get_parameter("world").value
        self.lock = threading.Lock()
        self.poses = {}  # name -> (position tuple, quaternion tuple), Gazebo world frame
        self.sent = {}  # name -> position last pushed to the planning scene
        self.held = ""  # letter of the cube in the gripper (it is an attached object then, not a world object)
        self.create_subscription(String, "/held_block", self._on_held,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf = TransformBroadcaster(self)
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.get_scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.gz = GzNode()
        topic = f"/world/{world}/dynamic_pose/info"
        if not self.gz.subscribe(Pose_V, topic, self._on_poses):
            raise RuntimeError(f"cannot subscribe to {topic}")
        self.create_timer(0.05, self._tf)  # 20 Hz
        self.create_timer(0.25, self._scene)  # 4 Hz
        self.create_timer(3.0, self._resend)  # move_group may start after us: re-send everything now and then
        self._table_sent = False
        self.get_logger().info(f"listening on {topic}")

    def _on_poses(self, msg):
        with self.lock:
            for p in msg.pose:
                if p.name.startswith("block_"):
                    self.poses[p.name] = (
                        (p.position.x, p.position.y, p.position.z),
                        (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
                    )

    def _on_held(self, msg):
        prev, self.held = self.held, msg.data
        for letter in (prev, self.held):
            self.sent.pop(f"block_{letter}", None)  # force a fresh ADD once it is a world object again
        self.get_logger().info(f"held block: {self.held or 'none'}")

    def _snapshot(self):
        with self.lock:
            return dict(self.poses)

    def _world_blocks(self):
        return {n: v for n, v in self._snapshot().items() if n != f"block_{self.held}"}

    def _tf(self):
        now = self.get_clock().now().to_msg()
        out = []
        for name, (p, q) in self._snapshot().items():
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = now, FRAME, name
            pm = _pose(p, q)
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (
                pm.position.x, pm.position.y, pm.position.z)
            t.transform.rotation = pm.orientation
            out.append(t)
        if out:
            self.tf.sendTransform(out)

    def _table(self):
        center = ((TABLE_X0 + TABLE_X1) / 2, 0.0, TABLE_Z / 2)
        return _box("table", (TABLE_X1 - TABLE_X0, TABLE_Y, TABLE_Z), _pose(center, (0.0, 0.0, 0.0, 1.0)))

    def _publish(self, objects):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects
        self.scene_pub.publish(scene)

    def _ensure_acm(self):
        """base_link stands flush on the table top: do not treat that contact as a collision.

        A diff replaces the whole allowed-collision matrix, so fetch the current one (it holds the SRDF's pairs)
        and add the table to it."""
        if not self.get_scene.service_is_ready():
            return
        req = GetPlanningScene.Request(
            components=PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX))
        self.get_scene.call_async(req).add_done_callback(self._extend_acm)

    def _extend_acm(self, fut):
        acm = fut.result().scene.allowed_collision_matrix
        if "table" in acm.entry_names or "base_link" not in acm.entry_names:
            return
        base = acm.entry_names.index("base_link")
        for i, row in enumerate(acm.entry_values):
            row.enabled = list(row.enabled) + [i == base]
        n = len(acm.entry_names)
        acm.entry_names = list(acm.entry_names) + ["table"]
        acm.entry_values.append(AllowedCollisionEntry(enabled=[i == base for i in range(n)] + [False]))
        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = acm
        self.scene_pub.publish(scene)
        self.get_logger().info("allowed table <-> base_link contact in the planning scene")

    def _scene(self):
        snap = self._world_blocks()
        changed = [
            _box(n, (CUBE,) * 3, _pose(p, q))
            for n, (p, q) in snap.items()
            if n not in self.sent or max(abs(p[i] - self.sent[n][i]) for i in range(3)) > MOVE_EPS
        ]
        if changed:
            self._publish(changed)
            for o in changed:
                self.sent[o.id] = snap[o.id][0]

    def _resend(self):
        snap = self._world_blocks()
        self._publish([self._table()] + [_box(n, (CUBE,) * 3, _pose(p, q)) for n, (p, q) in snap.items()])
        self._ensure_acm()
        self.sent = {n: p for n, (p, _) in snap.items()}


def main():
    rclpy.init()
    node = WorldState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
