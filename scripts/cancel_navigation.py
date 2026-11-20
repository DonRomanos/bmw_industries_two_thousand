#!/usr/bin/env python3
"""Abort the active Nav2 goal so the car stops. Run it inside the dev container.

    scripts/cancel_navigation.py

Cancels every goal of the navigate_to_pose action, whoever sent it (RViz, Foxglove, examples/goal.py, your own code).
It does the same as the "Cancel navigation" button of scripts/controls.py and:
    ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
Exit code 0: a goal was cancelled; 1: no goal was active or Nav2 did not answer.
"""
import sys

try:
    import rclpy
    from action_msgs.srv import CancelGoal
except ImportError as e:
    sys.exit(f'{e}\nROS is not sourced: open a new terminal in the dev container (see TROUBLESHOOTING.md, "Dev container").')

CANCEL_SERVICE = '/navigate_to_pose/_action/cancel_goal'
SERVICE_TIMEOUT_S = 5.0
ANSWER_TIMEOUT_S = 5.0


def main():
    rclpy.init()
    node = rclpy.create_node('cancel_navigation')
    try:
        client = node.create_client(CancelGoal, CANCEL_SERVICE)
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
            sys.exit(f'Nav2 not reachable ({CANCEL_SERVICE}). Run scripts/doctor.py and see TROUBLESHOOTING.md.')
        # An empty request (zero goal id and stamp) cancels all goals.
        future = client.call_async(CancelGoal.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=ANSWER_TIMEOUT_S)
        if not future.done():
            sys.exit('Nav2 did not answer the cancel request.')
        cancelling = len(future.result().goals_canceling)
        if cancelling == 0:
            sys.exit('No active navigation goal.')
        print(f'Navigation cancelled ({cancelling} goal(s)).')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
