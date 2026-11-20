#!/usr/bin/env python3
"""Show the car's battery, wheel speed and e-stop as a text above the car in RViz.

RViz cannot display car_interfaces messages, so this converts /batt_status and
/wheel_odom into a visualization_msgs/Marker on /car_status/marker (frame base_link).
scripts/rviz.sh starts it; run it by hand if you start RViz yourself:

    python3 scripts/car_status_viz.py
"""
import time

import rclpy
from car_interfaces.msg import BattStatus, WheelOdom
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker

STALE_S = 3.0
# 12 V AGM: about 12.7 V full, below 11.8 V half empty, below 11.0 V nearly flat.
BATT_OK_V, BATT_LOW_V = 11.8, 11.0
GREEN, AMBER, RED, GREY = (0.2, 0.9, 0.4), (1.0, 0.7, 0.1), (1.0, 0.25, 0.25), (0.7, 0.7, 0.7)


class CarStatusViz(Node):
    def __init__(self):
        super().__init__('car_status_viz')
        # Best effort matches both the car and a bag replay.
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.batt = None
        self.odom = None
        self.batt_t = self.odom_t = 0.0
        self.create_subscription(BattStatus, '/batt_status', self.on_batt, qos)
        self.create_subscription(WheelOdom, '/wheel_odom', self.on_odom, qos)
        self.pub = self.create_publisher(Marker, '/car_status/marker', 1)
        self.create_timer(0.2, self.publish)

    def on_batt(self, msg):
        self.batt, self.batt_t = msg, time.monotonic()

    def on_odom(self, msg):
        self.odom, self.odom_t = msg, time.monotonic()

    def publish(self):
        now = time.monotonic()
        lines, colour = [], GREEN
        if self.batt is not None and now - self.batt_t < STALE_S:
            b = self.batt
            lines.append(f'Battery {b.voltage_total:.1f} V')
            if not b.estop_clear:
                lines.append('E-STOP PRESSED')
                colour = RED
            elif b.voltage_total < BATT_LOW_V:
                colour = RED
            elif b.voltage_total < BATT_OK_V:
                colour = AMBER
        else:
            lines.append('Battery: no data')
            colour = GREY
        if self.odom is not None and now - self.odom_t < STALE_S:
            o = self.odom
            lines.append(f'Wheels L {o.left_speed_mps:+.2f}  R {o.right_speed_mps:+.2f} m/s')
        else:
            lines.append('Wheels: no data')

        m = Marker()
        m.header.frame_id = 'base_link'
        m.ns, m.id = 'car_status', 0
        m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.x, m.pose.position.z = 0.6, 1.0
        m.pose.orientation.w = 1.0
        m.scale.z = 0.12
        m.color.r, m.color.g, m.color.b = colour
        m.color.a = 1.0
        m.text = '\n'.join(lines)
        self.pub.publish(m)


def main():
    rclpy.init()
    node = CarStatusViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
