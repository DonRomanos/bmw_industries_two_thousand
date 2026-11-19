#!/usr/bin/env python3
"""Set the car's clock from this computer. Run it inside the dev container, connected to the car.

    scripts/set_car_time.py [--force]

The Jetson has no battery-backed clock and boots in 1970 until someone sets the time. This script compares
the car's clock (from /jetson_heartbeat) with yours and, if they differ by more than a few seconds, sends your
time to /control_room/set_time and prints what the car answers. The car then restarts its pipeline, which takes
about 1-2 minutes. Check your own computer's clock first (automatic time on).
It does the same as:  ros2 topic pub --once /control_room/set_time builtin_interfaces/msg/Time "{sec: $(date +%s)}"
"""
import argparse
import sys
import time

try:
    import rclpy
    from builtin_interfaces.msg import Time
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from std_msgs.msg import Header, String
except ImportError as e:
    sys.exit(f'{e}\nROS is not sourced: open a new terminal in the dev container (see TROUBLESHOOTING.md, "Dev container").')

SYNCED_WITHIN_S = 5.0
RESULT_TIMEOUT_S = 30.0


class SetCarTime(Node):
    def __init__(self):
        super().__init__('set_car_time')
        self.skew = None
        self.messages = []
        self.create_subscription(Header, '/jetson_heartbeat', self.on_heartbeat, qos_profile_sensor_data)
        latched = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/control_room/status', lambda m: self.messages.append(m.data), latched)
        self.pub = self.create_publisher(Time, '/control_room/set_time', 10)

    def on_heartbeat(self, msg):
        self.skew = time.time() - (msg.stamp.sec + msg.stamp.nanosec / 1e9)

    def spin_until(self, done, timeout):
        end = time.monotonic() + timeout
        while not done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return done()


def main():
    parser = argparse.ArgumentParser(description='Set the car clock from this computer.')
    parser.add_argument('--force', action='store_true', help='send the time even if the clocks already agree')
    args = parser.parse_args()

    rclpy.init()
    node = SetCarTime()
    try:
        if not node.spin_until(lambda: node.pub.get_subscription_count() > 0, 10.0):
            sys.exit('Cannot reach the car. Run scripts/doctor.py and see TROUBLESHOOTING.md.')

        if node.spin_until(lambda: node.skew is not None, 3.0):
            print(f'Car clock is {-node.skew:+.1f} s from yours.')
            if abs(node.skew) < SYNCED_WITHIN_S and not args.force:
                print('Already in sync, nothing to do (--force to send anyway).')
                return
        else:
            print('No heartbeat from the car (is the pipeline running?), sending the time anyway.')

        node.spin_until(lambda: False, 0.5)  # let the latched status of earlier requests arrive
        node.messages.clear()
        node.pub.publish(Time(sec=int(time.time())))
        shown = 0
        deadline = time.monotonic() + RESULT_TIMEOUT_S
        while time.monotonic() < deadline:
            node.spin_until(lambda: len(node.messages) > shown, 0.5)
            while shown < len(node.messages):
                print(f'Car: {node.messages[shown]}')
                shown += 1
            if shown and not node.messages[-1].startswith('Setting'):
                return
        sys.exit('No answer from the car.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
