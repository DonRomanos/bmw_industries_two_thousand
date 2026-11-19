#!/usr/bin/env python3
"""Check that this container can see and talk to the car. Run it inside the dev container.

    scripts/doctor.py [--ip 10.42.0.1] [--seconds 5]

Checks, in order: ROS environment, route and ping to the car, ROS discovery, data flow of the important topics,
clock difference to the car and the Nav2 action server. Every failing check prints a hint; TROUBLESHOOTING.md
explains the fixes. Exit code 1 if a check failed (`warn` means something optional is missing).
In a terminal switched to bag replay (`source scripts/bitt_env.sh replay`) the network and clock checks are
skipped, so it also tells you whether play_bag.sh is working.
"""
import argparse
import os
import re
import statistics
import subprocess
import sys
import time

try:
    import rclpy
    from car_interfaces.msg import SteerState
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Header
    from tf2_msgs.msg import TFMessage
except ImportError as e:
    sys.exit(f'{e}\nROS is not sourced: open a new terminal in the dev container (see TROUBLESHOOTING.md, "Dev container").')

DOC = 'TROUBLESHOOTING.md'
failed = False


def report(level, text, hint=''):
    global failed
    failed |= level == 'FAIL'
    print(f'  [{level:^4}] {text}' + (f'\n         -> {hint}' if hint else ''))


def run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=30).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ''


def check_network(ip):
    """Route and ping to the car. Returns False if the car cannot be reached."""
    route = run(['ip', 'route', 'get', ip])
    dev = (re.search(r' dev (\S+)', route) or [None, ''])[1]
    src = (re.search(r' src (\S+)', route) or [None, ''])[1]
    prefix = ip.rsplit('.', 1)[0] + '.'
    if not route:
        report('FAIL', f'no route to {ip}', f'connect to the KITT Wi-Fi (see "Cannot ping the car" in {DOC})')
        return False
    if dev == 'docker0' or src.startswith('172.'):
        report('warn', f'traffic to the car leaves through a Docker network ({dev}, {src}), not through the host\'s Wi-Fi',
               f'the container must use host networking (see "Ping works but no topics" in {DOC})')
    elif not src.startswith(prefix):
        report('warn', f'your address {src} is not in the car\'s network ({prefix}x): traffic to the car goes through {dev}',
               f'connect to the KITT Wi-Fi (see "Cannot ping the car" in {DOC})')
    else:
        report('ok', f'route to {ip} via {dev} from {src}')

    out = run(['ping', '-c', '5', '-i', '0.3', '-W', '2', ip])
    loss = re.search(r'([\d.]+)% packet loss', out)
    rtt = re.search(r'= [\d.]+/([\d.]+)/', out)
    if not loss or float(loss.group(1)) >= 100:
        report('FAIL', f'{ip} does not answer ping',
               f'check that the laptop is on the KITT Wi-Fi and the Jetson is booted (see "Cannot ping the car" in {DOC})')
        return False
    avg = float(rtt.group(1)) if rtt else 0.0
    if float(loss.group(1)) > 0 or avg >= 50:
        report('warn', f'ping {avg:.0f} ms average, {loss.group(1)}% loss', 'weak Wi-Fi: move closer to the car; expect lag and dropped frames')
    else:
        report('ok', f'ping {avg:.0f} ms average, no loss')
    return True


# topic, type, severity if it carries no data, what a missing topic means
STREAMS = [
    ('/tf', TFMessage, 'FAIL', 'the nav/slam pipeline is not running on the car, or discovery is incomplete'),
    ('/preview/left/compressed', CompressedImage, 'warn', 'cameras are not running'),
    ('/local_costmap/costmap', OccupancyGrid, 'warn', 'Nav2 is not running (the car must be started in "nav" mode)'),
    ('/jetson_heartbeat', Header, 'warn', 'the nav pipeline is not running (it publishes the 1 Hz heartbeat)'),
    ('/steer_state', SteerState, 'warn', 'the ESP32 is not connected to the micro-ROS agent'),
]


class Doctor(Node):
    def __init__(self):
        super().__init__('doctor')
        self.count = {t: 0 for t, *_ in STREAMS}
        self.bytes = {t: 0 for t, *_ in STREAMS}
        self.skews = {'/tf': [], '/jetson_heartbeat': []}
        for topic, msg_type, *_ in STREAMS:
            # Best effort matches both best-effort and reliable publishers.
            self.create_subscription(msg_type, topic, lambda m, t=topic: self.on_msg(t, m), qos_profile_sensor_data)

    def on_msg(self, topic, msg):
        self.count[topic] += 1
        if hasattr(msg, 'data') and hasattr(msg.data, '__len__'):
            self.bytes[topic] += len(msg.data)
        stamp = None
        if topic == '/jetson_heartbeat':
            stamp = msg.stamp
        elif topic == '/tf' and msg.transforms:
            stamp = msg.transforms[0].header.stamp
        if stamp is not None:
            now = self.get_clock().now().nanoseconds / 1e9
            self.skews[topic].append(now - (stamp.sec + stamp.nanosec / 1e9))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def user_topics(self):
        return {n for n, _ in self.get_topic_names_and_types()} - {'/parameter_events', '/rosout'}


def check_ros(seconds, replay):
    rclpy.init()
    node = Doctor()
    try:
        # Discovery: wait until anything other than our own topics shows up.
        node.spin_for(0.5)
        end = time.monotonic() + 10.0
        while not node.user_topics() and time.monotonic() < end:
            node.spin_for(0.5)
        topics = node.user_topics()
        if not topics:
            report('FAIL', 'no ROS topics visible', f'discovery does not work: see "Ping works but no topics" in {DOC}')
            return
        report('ok', f'discovery works, {len(topics)} topics visible')

        node.spin_for(seconds)

        for topic, _, severity, meaning in STREAMS:
            n = node.count[topic]
            if n:
                kbs = node.bytes[topic] / seconds / 1024
                report('ok', f'{topic}: {n / seconds:.1f} Hz' + (f', {kbs:.0f} kB/s' if kbs else ''))
            elif node.get_publishers_info_by_topic(topic):
                report(severity, f'{topic}: advertised but no data received',
                       f'discovery works but data does not arrive (firewall blocks incoming UDP?): see "Topics visible but no data" in {DOC}')
            else:
                report(severity, f'{topic}: not published', meaning)

        if replay:
            print('  [info] replay mode: clock check skipped')
        else:
            samples = node.skews['/jetson_heartbeat'] or node.skews['/tf']
            if samples:
                skew = statistics.median(samples)
                text = f'clock difference to the car: {skew:+.2f} s'
                hint = f'the car clock needs to be set, see "Clock skew" in {DOC}'
                report('FAIL' if abs(skew) > 5 else 'warn' if abs(skew) > 0.5 else 'ok', text, hint if abs(skew) > 0.5 else '')

        if ActionClient(node, NavigateToPose, 'navigate_to_pose').wait_for_server(timeout_sec=3.0):
            report('ok', 'Nav2 navigate_to_pose action server is up')
        else:
            report('warn', 'Nav2 navigate_to_pose action server not found', 'Nav2 is not running (the car must be started in "nav" mode)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--ip', default=os.environ.get('BITT_IP', '10.42.0.1'))
    p.add_argument('--seconds', type=float, default=5.0, help='how long to measure the data streams')
    args = p.parse_args()

    # bitt_env.sh replay selects shared-memory-only transport
    replay = os.environ.get('FASTDDS_BUILTIN_TRANSPORTS') == 'SHM'
    reachable = True

    print('Environment')
    domain, peers = os.environ.get('ROS_DOMAIN_ID', '0'), os.environ.get('ROS_STATIC_PEERS', '')
    if replay:
        report('info', f'REPLAY mode (domain {domain}): only play_bag.sh data is visible, the car is not')
    else:
        report('ok', f'live mode: domain {domain}, static peer {peers or "none"}')
        if peers != args.ip:
            report('warn', f"ROS_STATIC_PEERS is '{peers}', not the car ({args.ip})",
                   f'run: source scripts/bitt_env.sh live (see "Ping works but no topics" in {DOC})')
        print('Network')
        reachable = check_network(args.ip)

    print('ROS')
    if reachable:
        check_ros(args.seconds, replay)
    else:
        report('info', 'skipping the ROS checks until the network works')

    print('\n' + ('Problems found, see TROUBLESHOOTING.md.' if failed else 'No failed checks (look at the warnings above).'))
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
