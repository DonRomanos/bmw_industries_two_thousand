#!/usr/bin/env python3
"""Control the car's lights and horn.

    python3 examples/lights.py headlights
    python3 examples/lights.py left | right | hazard
    python3 examples/lights.py brake | reverse
    python3 examples/lights.py color 255 0 255        # every LED in one colour
    python3 examples/lights.py off
    python3 examples/lights.py horn                   # short beep

One LightsCmd message sets all lights at once; the last message wins, so send "off" to clear.
The ESP32 reports what it actually shows on /lights_state (`ros2 topic echo /lights_state`).
Layout: 4 front lamps (1 RGB LED each) and 4 rear bars (8 RGB pixels each), numbered left to right.
"""
import argparse
import time

import rclpy
from car_interfaces.msg import LightsCmd
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

WHITE, RED, AMBER, OFF = (255, 255, 255), (255, 0, 0), (255, 120, 0), (0, 0, 0)


def make_lights(front=OFF, rear=OFF, mode=LightsCmd.MODE_SOLID, flags=0, blink_period_ms=500):
    msg = LightsCmd()
    msg.front_rgb = [c for _ in range(4) for c in front]            # 4 lamps x RGB
    msg.rear_rgb = [c for _ in range(4 * 8) for c in rear]          # 4 bars x 8 pixels x RGB
    msg.mode = mode
    msg.flags = flags
    msg.blink_period_ms = blink_period_ms
    return msg


PRESETS = {
    'off': lambda: make_lights(),
    'headlights': lambda: make_lights(front=WHITE, rear=(60, 0, 0)),
    'left': lambda: make_lights(front=AMBER, rear=AMBER, mode=LightsCmd.MODE_INDICATOR_LEFT),
    'right': lambda: make_lights(front=AMBER, rear=AMBER, mode=LightsCmd.MODE_INDICATOR_RIGHT),
    'hazard': lambda: make_lights(front=AMBER, rear=AMBER, mode=LightsCmd.MODE_HAZARD),
    'brake': lambda: make_lights(rear=RED, flags=LightsCmd.FLAG_BRAKE),
    'reverse': lambda: make_lights(rear=WHITE, flags=LightsCmd.FLAG_REVERSE),
}


def wait_for_subscriber(node, pub, timeout=5.0):
    # A message published before discovery finished is lost, so wait for the car to show up.
    end = time.monotonic() + timeout
    while pub.get_subscription_count() == 0 and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    return pub.get_subscription_count() > 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('what', choices=[*PRESETS, 'color', 'horn'])
    p.add_argument('rgb', nargs='*', type=int, help='R G B (0-255) for "color"')
    args = p.parse_args()
    if args.what == 'color' and len(args.rgb) != 3:
        p.error('"color" needs three values: R G B')

    rclpy.init()
    node = rclpy.create_node('lights_example')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)   # as in the ESP contract
    try:
        if args.what == 'horn':
            pub = node.create_publisher(Bool, '/horn_cmd', qos)
            if not wait_for_subscriber(node, pub):
                node.get_logger().warning('no subscriber on /horn_cmd (is the ESP32 connected?), sending anyway')
            pub.publish(Bool(data=True))
            time.sleep(0.3)
            pub.publish(Bool(data=False))
        else:
            pub = node.create_publisher(LightsCmd, '/lights_cmd', qos)
            if not wait_for_subscriber(node, pub):
                node.get_logger().warning('no subscriber on /lights_cmd (is the ESP32 connected?), sending anyway')
            msg = make_lights(front=tuple(args.rgb), rear=tuple(args.rgb)) if args.what == 'color' \
                else PRESETS[args.what]()
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.3)   # let the message leave before shutting down
        node.get_logger().info(f'sent: {args.what}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
