#!/usr/bin/env python3
"""Status of the car in one view: battery, wheel speed, steering, controller, SLAM pose, lights.

    scripts/car_status.py            print the status once and exit (exit code 1 if nothing arrived)
    scripts/car_status.py --watch    refresh every second until Ctrl-C
    scripts/car_status.py --rviz     publish battery and wheel speed as text above the car for RViz
                                     (/car_status/marker, frame base_link; scripts/rviz.sh starts it)

Run it inside the dev container with the terminal switched to the car or to a replay.
Rates are measured while the script listens, so give slow topics a moment (--wait).
"""
import argparse
import math
import sys
import time

import rclpy
from car_interfaces.msg import BattStatus, ControllerState, LightsState, SteerState, WheelOdom
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker

STALE_S = 3.0
# 12 V AGM: about 12.7 V full, below 11.8 V half empty, below 11.0 V nearly flat.
BATT_OK_V, BATT_LOW_V = 11.8, 11.0
GREEN, AMBER, RED, GREY = (0.2, 0.9, 0.4), (1.0, 0.7, 0.1), (1.0, 0.25, 0.25), (0.7, 0.7, 0.7)
TOPICS = {
    'battery': ('/batt_status', BattStatus),
    'wheels': ('/wheel_odom', WheelOdom),
    'steer': ('/steer_state', SteerState),
    'controller': ('/controller_state', ControllerState),
    'slam': ('/visual_slam/tracking/odometry', Odometry),
    'lights': ('/lights_state', LightsState),
}


class Latest:
    """Last message of a topic plus its receive rate."""

    def __init__(self):
        self.msg = None
        self.first = self.last = 0.0
        self.count = 0

    def update(self, msg):
        now = time.monotonic()
        if self.count == 0:
            self.first = now
        self.msg, self.last = msg, now
        self.count += 1

    def fresh(self):
        return self.msg is not None and time.monotonic() - self.last < STALE_S

    def rate(self):
        span = self.last - self.first
        return (self.count - 1) / span if self.count > 2 and span > 0 else 0.0


class CarStatus(Node):
    def __init__(self, rviz):
        super().__init__('car_status')
        # Best effort matches both the car and a bag replay.
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.data = {name: Latest() for name in TOPICS}
        for name, (topic, msg_type) in TOPICS.items():
            self.create_subscription(msg_type, topic, self.data[name].update, qos)
        if rviz:
            self.pub = self.create_publisher(Marker, '/car_status/marker', 1)
            self.create_timer(0.2, self.publish_marker)

    def battery_line(self):
        d = self.data['battery']
        if not d.fresh():
            return 'Battery: no data', GREY
        b = d.msg
        text = f'Battery {b.voltage_total:.1f} V'
        if not b.estop_clear:
            return text + '  E-STOP PRESSED', RED
        if b.voltage_total < BATT_LOW_V:
            return text, RED
        return text, AMBER if b.voltage_total < BATT_OK_V else GREEN

    def wheels_line(self):
        d = self.data['wheels']
        if not d.fresh():
            return 'Wheels: no data'
        return f'Wheels L {d.msg.left_speed_mps:+.2f}  R {d.msg.right_speed_mps:+.2f} m/s'

    def publish_marker(self):
        text, colour = self.battery_line()
        m = Marker()
        m.header.frame_id = 'base_link'
        m.ns, m.id = 'car_status', 0
        m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.x, m.pose.position.z = 0.6, 1.0
        m.pose.orientation.w = 1.0
        m.scale.z = 0.12
        m.color.r, m.color.g, m.color.b = colour
        m.color.a = 1.0
        m.text = f'{text}\n{self.wheels_line()}'
        self.pub.publish(m)

    def report(self):
        """The status as a list of lines for the terminal."""
        def row(name, describe):
            d = self.data[name]
            if not d.fresh():
                return f'{name.capitalize():<11}no data'
            rate = f'   ({d.rate():.0f} Hz)' if d.rate() >= 0.5 else ''
            return f'{name.capitalize():<11}{describe(d.msg)}{rate}'

        def battery(b):
            current = f'  {b.current_a:.1f} A' if b.current_a else ''
            return (f'{b.voltage_total:.2f} V{current}   e-stop '
                    + ('clear' if b.estop_clear else 'PRESSED'))

        def wheels(w):
            return (f'L {w.left_speed_mps:+.2f}  R {w.right_speed_mps:+.2f} m/s   '
                    f'ticks L {w.left_count}  R {w.right_count}')

        def steer(s):
            return (f'{s.angle_rad:+.3f} rad ({math.degrees(s.angle_rad):+.1f} deg, + left)   '
                    + ('calibrated' if s.calibrated else 'NOT CALIBRATED'))

        def controller(c):
            battery_pct = f', battery {c.battery_pct:.0f} %' if c.battery_pct >= 0 else ''
            return ('connected' if c.connected else 'NOT CONNECTED') + battery_pct \
                + (', in use' if c.last_input_active else '')

        def slam(o):
            p, q = o.pose.pose.position, o.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (f'x {p.x:+.2f}  y {p.y:+.2f} m  yaw {math.degrees(yaw):+.0f} deg   '
                    f'v {o.twist.twist.linear.x:+.2f} m/s')

        return [row('battery', battery), row('wheels', wheels), row('steer', steer),
                row('controller', controller), row('slam', slam),
                row('lights', lambda l: 'FAILSAFE ACTIVE' if l.failsafe_active else 'ok')]

    def got_data(self):
        return any(d.msg is not None for d in self.data.values())


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--watch', action='store_true', help='refresh every second')
    p.add_argument('--rviz', action='store_true', help='publish /car_status/marker for RViz')
    p.add_argument('--wait', type=float, default=3.0, help='seconds to listen before printing')
    args = p.parse_args()

    rclpy.init()
    node = CarStatus(args.rviz)
    code = 0
    try:
        if args.rviz:
            rclpy.spin(node)
        else:
            end = time.monotonic() + args.wait
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            while True:
                lines = node.report()
                print(('\033[H\033[J' if args.watch else '') + '\n'.join(lines), flush=True)
                if not args.watch:
                    code = 0 if node.got_data() else 1
                    break
                end = time.monotonic() + 1.0
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
