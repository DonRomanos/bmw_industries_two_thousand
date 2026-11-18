#!/usr/bin/env python3
"""Send the car a navigation goal (Nav2 `navigate_to_pose` action) and follow its progress.

    python3 examples/goal.py 2.0 0.0              # 2 m straight ahead
    python3 examples/goal.py 3.0 1.0 --yaw 90     # 3 m ahead, 1 m left, finish turned 90 deg to the left
    python3 examples/goal.py 5.0 2.0 --absolute   # x/y in the `odom` frame instead of relative to the car

By default x (forward) and y (left) are in metres relative to where the car is now, and --yaw (degrees,
counter-clockwise) is relative to its current heading. The car has Ackermann steering: it cannot turn on the
spot, so very close or sideways goals may fail. Ctrl-C cancels the goal.

The car only moves in drive mode 2 (see examples/drive_mode.py), and the remote controller always takes over.
Needs the `nav` pipeline on the car.
"""
import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from action_msgs.msg import GoalStatus

GLOBAL_FRAME = 'odom'


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('x', type=float, help='metres forward (or odom x with --absolute)')
    p.add_argument('y', type=float, help='metres left (or odom y with --absolute)')
    p.add_argument('--yaw', type=float, default=0.0, help='final heading in degrees')
    p.add_argument('--absolute', action='store_true', help=f'interpret x, y, yaw in the {GLOBAL_FRAME} frame')
    args = p.parse_args()

    rclpy.init()
    node = Node('goal_example')
    log = node.get_logger()
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    try:
        x, y, yaw = args.x, args.y, math.radians(args.yaw)
        if not args.absolute:
            # Express the relative goal in the odom frame using the car's current pose.
            log.info('looking up the car pose ...')
            deadline = node.get_clock().now() + Duration(seconds=10)
            while True:
                try:
                    t = tf_buffer.lookup_transform(GLOBAL_FRAME, 'base_link', Time())
                    break
                except TransformException:
                    if node.get_clock().now() > deadline:
                        log.error(f'no {GLOBAL_FRAME} -> base_link transform; is the nav pipeline running?')
                        return
                    rclpy.spin_once(node, timeout_sec=0.2)
            cx, cy = t.transform.translation.x, t.transform.translation.y
            cyaw = yaw_of(t.transform.rotation)
            x, y = (cx + math.cos(cyaw) * args.x - math.sin(cyaw) * args.y,
                    cy + math.sin(cyaw) * args.x + math.cos(cyaw) * args.y)
            yaw = cyaw + yaw

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = GLOBAL_FRAME
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        log.info('waiting for the navigate_to_pose action server ...')
        client.wait_for_server()
        log.info(f'sending goal x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f} deg ({GLOBAL_FRAME})')

        def on_feedback(fb):
            log.info(f'distance remaining: {fb.feedback.distance_remaining:.2f} m', throttle_duration_sec=1.0)

        goal_future = client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(node, goal_future)
        handle = goal_future.result()
        if not handle.accepted:
            log.error('goal rejected')
            return

        result_future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(node, result_future)
        except KeyboardInterrupt:
            log.info('cancelling goal ...')
            rclpy.spin_until_future_complete(node, handle.cancel_goal_async(), timeout_sec=2.0)
            return
        status = result_future.result().status
        names = {GoalStatus.STATUS_SUCCEEDED: 'reached', GoalStatus.STATUS_ABORTED: 'failed (aborted)',
                 GoalStatus.STATUS_CANCELED: 'cancelled'}
        log.info(f'goal {names.get(status, status)}')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
