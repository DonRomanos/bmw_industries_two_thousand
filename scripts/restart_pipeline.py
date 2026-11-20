#!/usr/bin/env python3
"""Restart the car's whole pipeline (cameras, SLAM, nvblox, Nav2) via the control room. Run it inside the dev container.

    scripts/restart_pipeline.py [--no-wait]

Calls /control_room/restart_pipeline, prints what the car answers and then waits until /jetson_heartbeat is back
(about 1-2 minutes). The car refuses a second request within a minute. It does the same as:
    ros2 service call /control_room/restart_pipeline std_srvs/srv/Trigger
"""
import argparse
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from std_msgs.msg import Header, String
    from std_srvs.srv import Trigger
except ImportError as e:
    sys.exit(f'{e}\nROS is not sourced: open a new terminal in the dev container (see TROUBLESHOOTING.md, "Dev container").')

SERVICE_TIMEOUT_S = 10.0
ANSWER_TIMEOUT_S = 30.0
GAP_TIMEOUT_S = 20.0   # the heartbeat must stop within this long after the request, otherwise nothing restarted
GAP_S = 3.0            # silence that counts as "pipeline went down"
BACK_TIMEOUT_S = 240.0


class RestartPipeline(Node):
    def __init__(self):
        super().__init__('restart_pipeline')
        self.last_beat = None
        self.messages = []
        self.create_subscription(Header, '/jetson_heartbeat', self.on_heartbeat, qos_profile_sensor_data)
        latched = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/control_room/status', lambda m: self.messages.append(m.data), latched)
        self.client = self.create_client(Trigger, '/control_room/restart_pipeline')

    def on_heartbeat(self, _):
        self.last_beat = time.monotonic()

    def spin_until(self, done, timeout):
        end = time.monotonic() + timeout
        while not done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return done()

    def silent_for(self, seconds):
        return self.last_beat is None or time.monotonic() - self.last_beat > seconds


def main():
    parser = argparse.ArgumentParser(description='Restart the car pipeline via the control room.')
    parser.add_argument('--no-wait', action='store_true', help='return after the car accepted the request')
    args = parser.parse_args()

    rclpy.init()
    node = RestartPipeline()
    try:
        if not node.client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
            sys.exit('Cannot reach the car\'s control room (/control_room/restart_pipeline). '
                     'Run scripts/doctor.py and see TROUBLESHOOTING.md.')

        node.spin_until(lambda: False, 0.5)  # let the latched status of earlier requests arrive
        node.messages.clear()
        future = node.client.call_async(Trigger.Request())
        if not node.spin_until(future.done, ANSWER_TIMEOUT_S):
            sys.exit('No answer from the car.')
        response = future.result()
        print(f'Car: {response.message}' if response.message else f'Car: success={response.success}')
        node.spin_until(lambda: False, 0.5)
        for message in node.messages:
            print(f'Car: {message}')
        if not response.success:
            sys.exit('The car did not restart the pipeline.')
        if args.no_wait:
            return

        print('Waiting for the pipeline to go down and come back (1-2 minutes) ...')
        went_down = node.spin_until(lambda: node.silent_for(GAP_S), GAP_TIMEOUT_S)
        if not went_down:
            sys.exit('The heartbeat never stopped: the pipeline does not seem to have restarted. '
                     'Check TROUBLESHOOTING.md or ask your instructors.')
        if not node.spin_until(lambda: not node.silent_for(GAP_S), BACK_TIMEOUT_S):
            sys.exit(f'No heartbeat after {BACK_TIMEOUT_S:.0f} s. Run scripts/doctor.py, then ask your instructors.')
        print('The pipeline is back. Run scripts/doctor.py to check it.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
