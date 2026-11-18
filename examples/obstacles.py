#!/usr/bin/env python3
"""Find obstacles around the car in Nav2's local costmap.

    python3 examples/obstacles.py                  # nearest obstacle ahead / left / right, twice a second
    python3 examples/obstacles.py --corridor 0.6   # width of the strip straight ahead that counts as "ahead" (m)
    python3 examples/obstacles.py --topic /nvblox_node/static_occupancy_grid

The costmap is an OccupancyGrid in the `odom` frame (0 = free, 100 = lethal obstacle, -1 = unknown). The
script transforms the occupied cells into the car's frame (`base_link`: x forward, y left, origin in the middle
of the rear axle) so you can reason with "metres ahead" instead of grid indices.
Needs the `nav` pipeline on the car (the costmaps are produced by Nav2).
"""
import argparse
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ObstacleFinder(Node):
    def __init__(self, topic, threshold, corridor, max_range):
        super().__init__('obstacles_example')
        self.threshold, self.half_width, self.max_range = threshold, corridor / 2, max_range
        self.last_print = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Reliable + volatile matches both volatile and transient-local publishers.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, topic, self.on_grid, qos)
        self.get_logger().info(f'waiting for {topic} and the base_link transform ...')

    def on_grid(self, grid):
        if time.monotonic() - self.last_print < 0.5:
            return
        try:
            # Pose of the car in the grid's frame.
            t = self.tf_buffer.lookup_transform(
                grid.header.frame_id, 'base_link', Time(), timeout=Duration(seconds=0.2))
        except TransformException as e:
            self.get_logger().warning(f'no transform yet: {e}')
            return
        self.last_print = time.monotonic()

        info = grid.info
        cells = np.array(grid.data, dtype=np.int8).reshape(info.height, info.width)
        rows, cols = np.nonzero(cells >= self.threshold)
        if rows.size == 0:
            self.get_logger().info('no obstacles in the costmap')
            return

        # Cell centres in the grid frame, then relative to the car: x = forward, y = left.
        wx = info.origin.position.x + (cols + 0.5) * info.resolution - t.transform.translation.x
        wy = info.origin.position.y + (rows + 0.5) * info.resolution - t.transform.translation.y
        yaw = yaw_of(t.transform.rotation)
        x = math.cos(yaw) * wx + math.sin(yaw) * wy
        y = -math.sin(yaw) * wx + math.cos(yaw) * wy

        near = np.hypot(x, y) <= self.max_range
        x, y = x[near], y[near]
        ahead = (x > 0) & (np.abs(y) <= self.half_width)
        left = (x > 0) & (y > self.half_width)
        right = (x > 0) & (y < -self.half_width)

        def nearest(mask):
            return f'{np.hypot(x[mask], y[mask]).min():.2f} m' if mask.any() else '-'

        self.get_logger().info(
            f'nearest obstacle: ahead {nearest(ahead)} | left {nearest(left)} | right {nearest(right)} '
            f'| anywhere {nearest(np.ones_like(ahead))}')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--topic', default='/local_costmap/costmap')
    p.add_argument('--threshold', type=int, default=99, help='cells with at least this cost count as obstacle')
    p.add_argument('--corridor', type=float, default=0.6, help='width of the "ahead" strip in m')
    p.add_argument('--max-range', type=float, default=5.0, help='ignore obstacles further away (m)')
    args = p.parse_args()

    rclpy.init()
    node = ObstacleFinder(args.topic, args.threshold, args.corridor, args.max_range)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
