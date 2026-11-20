#!/usr/bin/env python3
"""Simulated ESP32: the car's drive and steering firmware seen from ROS.

Same interface as the real firmware:
    in : /cmd_vel (Twist), /steer_cmd (SteerCmd), /jetson_heartbeat, /drive_mode
    out: /steer_state (SteerState)
plus the Gazebo side: wheel speeds and the (bicycle-model) steering angle.

Like the firmware it only drives in AUTONOMY mode while the heartbeat is fresh (<3 s) and
stops when no /cmd_vel arrived for 1 s. Steering and speed follow the command with a short lag
(motor and servo dynamics of the real car are unknown, see robot_config.yaml "not_modelled").
"""

import math

import rclpy
from car_interfaces.msg import DriveMode, SteerCmd, SteerState
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64, Header

CMD_TIMEOUT_S = 1.0
HEARTBEAT_TIMEOUT_S = 3.0
RATE_HZ = 50.0
STEER_TAU_S = 0.15
SPEED_TAU_S = 0.2


class SimEsp(Node):
    def __init__(self):
        super().__init__('sim_esp')
        self.declare_parameter('wheel_radius', 0.1125)
        self.declare_parameter('max_steering_angle', 0.3)
        self._radius = self.get_parameter('wheel_radius').value
        self._max_steer = self.get_parameter('max_steering_angle').value

        self._speed_cmd = 0.0
        self._steer_cmd = 0.0
        self._speed = 0.0
        self._steer = 0.0
        self._last_cmd = None
        self._last_beat = None
        self._mode = DriveMode.MODE_MANUAL_PHYSICAL

        beat_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        mode_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.create_subscription(SteerCmd, 'steer_cmd', self._on_steer, 10)
        self.create_subscription(Header, 'jetson_heartbeat', self._on_beat, beat_qos)
        self.create_subscription(DriveMode, 'drive_mode', self._on_mode, mode_qos)

        self._state_pub = self.create_publisher(SteerState, 'steer_state', 10)
        self._steer_gz = self.create_publisher(Float64, '/gz_steer_angle', 10)
        self._left_gz = self.create_publisher(Float64, '/gz_left_wheel_cmd', 10)
        self._right_gz = self.create_publisher(Float64, '/gz_right_wheel_cmd', 10)
        self._last_t = self.get_clock().now()
        self.create_timer(1.0 / RATE_HZ, self._step)

    def _on_cmd(self, msg):
        self._speed_cmd = msg.linear.x
        self._last_cmd = self.get_clock().now()

    def _on_steer(self, msg):
        self._steer_cmd = max(-self._max_steer, min(self._max_steer, msg.angle_rad))

    def _on_beat(self, _msg):
        self._last_beat = self.get_clock().now()

    def _on_mode(self, msg):
        self._mode = msg.mode

    def _age(self, stamp, now):
        return math.inf if stamp is None else (now - stamp).nanoseconds * 1e-9

    def _step(self):
        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        if dt <= 0.0:
            return

        allowed = (self._mode == DriveMode.MODE_AUTONOMY
                   and self._age(self._last_beat, now) < HEARTBEAT_TIMEOUT_S
                   and self._age(self._last_cmd, now) < CMD_TIMEOUT_S)
        target = self._speed_cmd if allowed else 0.0

        self._speed += (target - self._speed) * (1.0 - math.exp(-dt / SPEED_TAU_S))
        self._steer += (self._steer_cmd - self._steer) * (1.0 - math.exp(-dt / STEER_TAU_S))

        omega = Float64(data=self._speed / self._radius)
        self._left_gz.publish(omega)
        self._right_gz.publish(omega)
        self._steer_gz.publish(Float64(data=self._steer))

        state = SteerState()
        state.header.stamp = now.to_msg()
        state.angle_rad = float(self._steer)
        state.angle_velocity_rad_s = float((self._steer_cmd - self._steer) / STEER_TAU_S)
        state.calibrated = True
        self._state_pub.publish(state)


def main():
    rclpy.init()
    node = SimEsp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
