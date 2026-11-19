#!/usr/bin/env python3
"""Simulated visual odometry: Gazebo ground truth under the cuVSLAM interface.

Publishes /visual_slam/tracking/odometry (nav_msgs/Odometry) and the odom -> base_link TF,
starting at the spawn pose like cuVSLAM starts at its first frame. Ground truth means no drift.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class SimOdom(Node):
    def __init__(self):
        super().__init__('sim_odom')
        self._pub = self.create_publisher(Odometry, '/visual_slam/tracking/odometry', 10)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/gz_odom', self._on_odom, 10)

    def _on_odom(self, msg):
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        self._pub.publish(msg)

        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = 'base_link'
        p = msg.pose.pose
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = p.position.z
        t.transform.rotation = p.orientation
        self._tf.sendTransform(t)


def main():
    rclpy.init()
    node = SimOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
