#!/usr/bin/env python3
"""Read the Odometry messages of a rosbag2 and publish them once as a latched nav_msgs/Path.

The bags only contain the odometry (no Path topic); reading the file directly shows the whole
trajectory immediately instead of while the bag plays.
  odom_to_path.py <bag_dir> [odom_topic] [path_topic]
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.serialization import deserialize_message
import rosbag2_py
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

MIN_STEP = 0.02  # metres between stored poses


def read_path(bag_dir, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir), rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    path, last = Path(), None
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, Odometry)
        p = msg.pose.pose.position
        if last is not None and (p.x - last[0]) ** 2 + (p.y - last[1]) ** 2 + (p.z - last[2]) ** 2 < MIN_STEP ** 2:
            continue
        last = (p.x, p.y, p.z)
        path.header = msg.header
        path.poses.append(PoseStamped(header=msg.header, pose=msg.pose.pose))
    return path


def main():
    bag_dir = sys.argv[1]
    topic = sys.argv[2] if len(sys.argv) > 2 else "/visual_slam/tracking/odometry"
    path_topic = sys.argv[3] if len(sys.argv) > 3 else "/replay/path"
    path = read_path(bag_dir, topic)
    if not path.poses:
        sys.exit(f"No Odometry messages on {topic} in {bag_dir}")
    # Keep the frame but stamp 0 so RViz draws the path without waiting for a matching TF time.
    path.header.stamp.sec = path.header.stamp.nanosec = 0

    rclpy.init()
    node = Node("odom_to_path")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_publisher(Path, path_topic, qos).publish(path)
    node.get_logger().info(f"Published {len(path.poses)} poses on {path_topic} (frame {path.header.frame_id})")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
