import rclpy, time, sys
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n = rclpy.create_node("finger_probe")
st = {"last": None, "t": 0, "held": ""}
t0 = time.time()
def cb(m):
    if "finger_joint" in m.name:
        v = m.position[m.name.index("finger_joint")]
        if st["last"] is None or abs(v - st["last"]) > 0.02 or time.time() - st["t"] > 5:
            print(f"{time.time()-t0:6.1f}s finger_joint={v:.3f} held={st['held']!r}", flush=True)
            st["last"], st["t"] = v, time.time()
def hb(m):
    st["held"] = m.data; print(f"{time.time()-t0:6.1f}s held -> {m.data!r}", flush=True)
n.create_subscription(JointState, "/joint_states", cb, 10)
n.create_subscription(String, "/held_block", hb, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
end = time.time() + float(sys.argv[1])
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
