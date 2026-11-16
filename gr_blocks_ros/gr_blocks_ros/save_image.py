"""Save one frame of a sensor_msgs/Image topic as PNG:  ros2 run gr_blocks_ros save_image TOPIC OUT.png"""
import sys

import numpy as np
import rclpy
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg


def main():
    topic, out = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = Node("save_image")
    got = []
    node.create_subscription(ImageMsg, topic, got.append, 1)
    end = node.get_clock().now().nanoseconds + 30e9
    while not got and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not got:
        print("no image on", topic)
        sys.exit(1)
    m = got[0]
    assert m.encoding in ("rgb8", "rgba8"), m.encoding
    ch = 3 if m.encoding == "rgb8" else 4
    arr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, : m.width * ch].reshape(m.height, m.width, ch)
    Image.fromarray(arr[:, :, :3]).save(out)
    print("saved", out, m.width, "x", m.height)
    rclpy.shutdown()
