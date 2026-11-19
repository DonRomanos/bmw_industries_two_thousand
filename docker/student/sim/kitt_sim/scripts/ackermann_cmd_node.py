#!/usr/bin/env python3
"""Nav2 velocity command -> ESP command interface (see ESP Contract/CONTRACT.md).

Subscribes  cmd_vel_smoothed (Nav2, TwistStamped)
Publishes   cmd_vel           geometry_msgs/Twist     speed; the ESP maps m/s to drive percent
            steer_cmd         car_interfaces/SteerCmd bicycle-model angle of the virtual centre wheel
            jetson_heartbeat  std_msgs/Header         1 Hz, always (the ESP allows autonomy while <3 s old)
            drive_mode        car_interfaces/DriveMode AUTONOMY, 1 Hz while Nav2 is commanding

The ESP subscribes drive_mode volatile, so the mode is re-sent while commands flow instead of
once; it is not sent while Nav2 is idle so the remote controller can switch modes. When Nav2
stops commanding, zero speed is sent for `stop_hold` seconds and then nothing: the ESP stops by
itself when cmd_vel is older than 1 s. Reverse is limited to `max_reverse_speed` (0 blocks it);
it is blind, the cameras face forward.

Corrections for the open-loop car: the ESP stops the car below 0.065 m/s, but MPPI gets its
speed feedback from odometry and can only raise the command by ax_max * model_dt (0.05 m/s) from
the measured speed, so a standing car would never start. Any request is therefore raised to
`min_drive_speed` (just above the ESP's stop threshold), and requests below `min_drive_trigger`
stop the car. The sampling controller's speed and yaw rate are noisy (it even flips between
forward and reverse near zero speed), so both are low-passed (`speed_filter_tau`); the steering
angle is atan(L * w / v) of the filtered values, with the divisor floored at `steer_speed_floor`
so the noise is not amplified at small speeds, then low-passed (`steer_filter_tau`) and
rate-limited (`steer_rate_limit`; the steering follows with about 0.25 s lag). The yaw rate has its
own, slower low-pass (`yaw_filter_tau`) and angles below `steer_deadband` are dropped (soft
deadband), which calms the steering on a straight path. The car never drives against the active path segment: the direction (forward/reverse)
comes from the poses of /transformed_global_plan (heading vs. tangent, Smac Reeds-Shepp), and a
speed of the opposite sign is zeroed (`enforce_path_direction`). The angle sent to
the servo comes from SteerLoop: feedforward plus a PI term on the measured angle (/steer_state),
so the servo's gain error, stiction and saturation are compensated; a new command sequence starts
from the measured angle. Direction changes (forward <-> reverse) are held at speed 0 for
`direction_dwell` seconds so the mechanics can settle. Steering-only tests: publish an angle (std_msgs/Float32) on
/nav_debug/steer_ref; it goes straight into the loop and no drive command is sent.
"""

import math

import rclpy
from car_interfaces.msg import DriveMode, SteerCmd, SteerState
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32, Header

# Below this speed the steering angle cannot be derived from the yaw rate.
MIN_STEER_SPEED = 0.02


def steering_angle(v, w, wheelbase, max_steer, last):
    """Bicycle model w = v * tan(delta) / L, clamped; keeps `last` when (almost) standing.

    Valid in both directions: reversing flips the sign of v, so the same yaw rate needs the
    opposite steering angle.
    """
    if abs(v) <= MIN_STEER_SPEED:
        return last
    return max(-max_steer, min(max_steer, math.atan(wheelbase * w / v)))


def drive_speed(v, trigger, min_drive, max_speed, max_reverse):
    """Speed request -> command: stop below `trigger`, at least `min_drive`, at most the limit.

    Reverse is limited to `max_reverse` (0 blocks it).
    """
    if abs(v) < trigger:
        return 0.0
    if v > 0.0:
        return min(max(v, min_drive), max_speed)
    if max_reverse <= 0.0:
        return 0.0
    return -min(max(-v, min_drive), max_reverse)


def lowpass(previous, new, dt, tau):
    """First-order low-pass step (tau <= 0 passes the new value through)."""
    return new if tau <= 0.0 else previous + (new - previous) * dt / (tau + dt)


def soft_deadband(value, band):
    """Shrinks |value| by `band` (continuous, 0 inside the band)."""
    return math.copysign(max(abs(value) - band, 0.0), value)


def path_direction(poses, span=1.0, margin=0.3):
    """Direction of travel the path asks for: +1 forward, -1 reverse, 0 unknown.

    `poses` are (x, y, yaw) along the path. A reverse segment has its heading opposite to the
    direction of travel (Smac Reeds-Shepp), so the sign of tangent . heading tells the direction.
    Only the first `span` metres count (the path handler hands over one segment at a time).
    """
    score = length = 0.0
    for (x0, y0, yaw), (x1, y1, _) in zip(poses, poses[1:]):
        dx, dy = x1 - x0, y1 - y0
        seg = math.hypot(dx, dy)
        if seg < 1e-3:
            continue
        score += dx * math.cos(yaw) + dy * math.sin(yaw)
        length += seg
        if length >= span:
            break
    if length < 0.05 or abs(score) < margin * length:
        return 0
    return 1 if score > 0.0 else -1


class SteerLoop:
    """Feedback on the measured steering angle around the firmware's own position loop.

    The steering servo reaches only 0.8 to 1.0 of the commanded angle (0.04 rad short at small
    angles), sticks when it returns toward the centre (up to 0.04 rad of hysteresis), saturates at
    about 0.30 rad and needs ~0.4 s for a step. A fixed gain cannot fix errors that depend on the
    direction, so the command is feedforward (ref / ff_gain) plus a PI term on the measured error.
    The integral is only updated when it does not push further into the command limit.
    """

    def __init__(self, ff_gain, kp, ki, i_max, cmd_limit, rest_band=0.005):
        self.ff_gain, self.kp, self.ki = ff_gain, kp, ki
        self.i_max, self.cmd_limit = i_max, cmd_limit
        self.rest_band = rest_band
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def step(self, ref, measured, dt):
        """Command for the reference angle; without a measurement it is feedforward only."""
        if measured is None:
            return max(-self.cmd_limit, min(self.cmd_limit, ref / self.ff_gain))
        if abs(ref) < self.rest_band:
            # Driving straight: an integral winding up against the servo's stiction would make
            # it break loose and hunt around the centre.
            self.integral = 0.0
        error = ref - measured
        raw = ref / self.ff_gain + self.kp * error + self.integral
        cmd = max(-self.cmd_limit, min(self.cmd_limit, raw))
        if not (cmd != raw and error * cmd > 0.0):
            self.integral = max(-self.i_max, min(self.i_max, self.integral + self.ki * error * dt))
        return cmd


class DirectionDwell:
    """A direction change (forward <-> reverse) needs the car to stand still for `dwell` seconds.

    The speed request is passed through unless it asks for the opposite direction of the last
    movement less than `dwell` seconds after that movement; then the output is 0. A request that
    returns to the previous direction resumes at once. `dwell` <= 0 disables it.
    """

    def __init__(self, dwell):
        self.dwell = dwell
        self.direction = 0
        self.last_moving = 0.0

    def step(self, speed, now):
        sign = (speed > 0.0) - (speed < 0.0)
        if sign == 0:
            return 0.0
        if self.direction not in (0, sign) and now - self.last_moving < self.dwell:
            return 0.0
        self.direction, self.last_moving = sign, now
        return speed


class AckermannCmdNode(Node):
    def __init__(self):
        super().__init__('ackermann_cmd')
        self.declare_parameter('wheelbase', 0.72)
        self.declare_parameter('max_steering_angle', 0.30)
        self.declare_parameter('max_speed', 0.4)
        self.declare_parameter('max_reverse_speed', 0.2)
        self.declare_parameter('min_drive_speed', 0.07)
        self.declare_parameter('min_drive_trigger', 0.04)
        # Below this speed MPPI's yaw rate / speed ratio is noise: at 0.05 a yaw-rate error of
        # 0.02 rad/s gave 0.27 rad (15 deg) of steering while the car crept or stood.
        self.declare_parameter('steer_speed_floor', 0.15)
        self.declare_parameter('speed_filter_tau', 0.3)
        self.declare_parameter('yaw_filter_tau', 0.6)
        self.declare_parameter('steer_filter_tau', 0.3)
        self.declare_parameter('steer_deadband', 0.015)
        self.declare_parameter('enforce_path_direction', True)
        self.declare_parameter('path_timeout', 1.0)
        self.declare_parameter('steer_rate_limit', 1.0)
        self.declare_parameter('steer_ff_gain', 0.95)
        self.declare_parameter('steer_kp', 0.6)
        self.declare_parameter('steer_ki', 3.0)
        self.declare_parameter('steer_i_max', 0.08)
        self.declare_parameter('steer_cmd_limit', 0.3456)
        self.declare_parameter('steer_state_timeout', 0.5)
        self.declare_parameter('direction_dwell', 0.6)
        self.declare_parameter('stamped_input', True)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('stop_hold', 1.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('set_autonomy_mode', True)
        self.declare_parameter('frame_id', 'base_link')

        self._wheelbase = self.get_parameter('wheelbase').value
        self._max_steer = self.get_parameter('max_steering_angle').value
        self._max_speed = self.get_parameter('max_speed').value
        self._max_reverse = self.get_parameter('max_reverse_speed').value
        self._min_drive = self.get_parameter('min_drive_speed').value
        self._trigger = self.get_parameter('min_drive_trigger').value
        self._steer_floor = self.get_parameter('steer_speed_floor').value
        self._v_tau = self.get_parameter('speed_filter_tau').value
        self._w_tau = self.get_parameter('yaw_filter_tau').value
        self._tau = self.get_parameter('steer_filter_tau').value
        self._deadband = self.get_parameter('steer_deadband').value
        self._enforce_direction = self.get_parameter('enforce_path_direction').value
        self._path_timeout = self.get_parameter('path_timeout').value
        self._rate = self.get_parameter('steer_rate_limit').value
        self._state_timeout = self.get_parameter('steer_state_timeout').value
        self._dwell = DirectionDwell(self.get_parameter('direction_dwell').value)
        self._loop = SteerLoop(
            self.get_parameter('steer_ff_gain').value, self.get_parameter('steer_kp').value,
            self.get_parameter('steer_ki').value, self.get_parameter('steer_i_max').value,
            self.get_parameter('steer_cmd_limit').value)
        self._timeout = self.get_parameter('cmd_timeout').value
        self._stop_hold = self.get_parameter('stop_hold').value
        self._set_mode = self.get_parameter('set_autonomy_mode').value
        self._frame_id = self.get_parameter('frame_id').value
        self._dt = 1.0 / self.get_parameter('publish_rate_hz').value

        self._speed = 0.0
        self._v_f = 0.0
        self._w_f = 0.0
        self._steer_target = 0.0
        self._steer = 0.0
        self._measured = None  # (angle, time) from /steer_state
        self._testing = False
        self._override = None  # (angle, time) from /nav_debug/steer_ref, for steering-only tests
        self._path_dir = None  # (+1 forward / -1 reverse / 0 unknown, time) of the active segment
        self._last_rx = None
        self._stale_since = None

        self.create_subscription(SteerState, 'steer_state', self._on_state, 10)
        self.create_subscription(Path, 'transformed_global_plan', self._on_path, 1)
        self.create_subscription(Float32, 'nav_debug/steer_ref', self._on_override, 10)
        if self.get_parameter('stamped_input').value:
            self.create_subscription(TwistStamped, 'cmd_vel_smoothed', self._on_stamped, 10)
        else:
            self.create_subscription(Twist, 'cmd_vel_smoothed', self._on_twist, 10)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._steer_pub = self.create_publisher(SteerCmd, 'steer_cmd', 10)
        self._beat_pub = self.create_publisher(
            Header, 'jetson_heartbeat',
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self._mode_pub = self.create_publisher(
            DriveMode, 'drive_mode',
            QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self._publish)
        self.create_timer(1.0, self._slow)

    def _on_stamped(self, msg):
        self._on_twist(msg.twist)

    def _on_state(self, msg):
        self._measured = (msg.angle_rad, self.get_clock().now())

    def _on_override(self, msg):
        self._override = (msg.data, self.get_clock().now())

    def _on_path(self, msg):
        poses = []
        for stamped in msg.poses[:60]:
            p, q = stamped.pose.position, stamped.pose.orientation
            poses.append((p.x, p.y, math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))))
        self._path_dir = (path_direction(poses), self.get_clock().now())

    def _path_sign(self):
        """Direction the active path segment asks for (+1/-1), 0 when unknown or stale."""
        if not self._enforce_direction or self._path_dir is None:
            return 0
        if self._age(self._path_dir) > self._path_timeout:
            return 0
        return self._path_dir[0]

    def _age(self, stamped):
        return (self.get_clock().now() - stamped[1]).nanoseconds * 1e-9

    def _measured_angle(self):
        """Measured steering angle, or None when /steer_state is missing or stale."""
        if self._measured is None or self._age(self._measured) > self._state_timeout:
            return None
        return self._measured[0]

    def _override_angle(self):
        if self._override is None or self._age(self._override) > self._timeout:
            return None
        return self._override[0]

    def _on_twist(self, twist):
        v, w = twist.linear.x, twist.angular.z
        if self._max_reverse <= 0.0 and v < 0.0:
            v = 0.0
        # The sampling controller sometimes asks for the opposite of what the path says (it backed
        # up for 5 s on a clear straight path): it must not drive against the active segment.
        if v * self._path_sign() < 0.0:
            v = 0.0
        now = self.get_clock().now()
        if self._last_rx is None or not self._fresh():
            self._v_f, self._w_f = v, w  # a new command sequence starts without filter lag
            # ...and from the steering angle the car really has, not from a remembered command
            measured = self._measured_angle()
            if measured is not None:
                self._steer = self._steer_target = measured
            self._loop.reset()
        else:
            dt = (now - self._last_rx).nanoseconds * 1e-9
            self._v_f = lowpass(self._v_f, v, dt, self._v_tau)
            self._w_f = lowpass(self._w_f, w, dt, self._w_tau)
        wanted = drive_speed(
            self._v_f, self._trigger, self._min_drive, self._max_speed, self._max_reverse)
        # A direction change needs the car to stand still for a while (mechanics, drivetrain);
        # the steering already moves to the new direction's angle during that time.
        self._speed = self._dwell.step(wanted, now.nanoseconds * 1e-9)
        # Nav2 plans the yaw rate for its own speed, so the curvature it wants is w / v_nav and
        # the steering angle atan(L * w / v_nav) (the car follows that curve at any speed). Both
        # are low-passed and the divisor has a floor, so the noise of the sampling controller is
        # not amplified at small speeds. A stopped car keeps its last steering angle.
        if wanted != 0.0:
            divisor = math.copysign(max(abs(self._v_f), self._steer_floor), self._v_f)
            angle = steering_angle(divisor, self._w_f, self._wheelbase, self._max_steer, 0.0)
            # Tiny angles are noise on a straight path. Keep it small: a larger band removes most
            # of the weak steering demand of a wide curve (the car then drives straight).
            self._steer_target = soft_deadband(angle, self._deadband)
        self._last_rx = now

    def _fresh(self):
        if self._last_rx is None:
            return False
        return (self.get_clock().now() - self._last_rx).nanoseconds * 1e-9 <= self._timeout

    def _header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        return header

    def _publish_steer(self, reference):
        steer = SteerCmd()
        steer.header = self._header()
        steer.angle_rad = float(self._loop.step(reference, self._measured_angle(), self._dt))
        self._steer_pub.publish(steer)

    def _publish(self):
        test_ref = self._override_angle()
        if test_ref is not None:
            # steering-only test: the reference goes straight into the loop, no drive command
            self._publish_steer(test_ref)
            self._testing = True
            return
        if self._testing:
            self._testing = False
            self._loop.reset()
        if self._fresh():
            self._stale_since = None
            speed = float(self._speed)
        else:
            if self._last_rx is None:
                return
            now = self.get_clock().now()
            if self._stale_since is None:
                self._stale_since = now
            if (now - self._stale_since).nanoseconds * 1e-9 > self._stop_hold:
                return
            speed = 0.0
        twist = Twist()
        twist.linear.x = speed
        self._cmd_pub.publish(twist)
        step = lowpass(self._steer, self._steer_target, self._dt, self._tau) - self._steer
        limit = self._rate * self._dt
        self._steer += max(-limit, min(limit, step))
        self._publish_steer(self._steer)

    def _slow(self):
        self._beat_pub.publish(self._header())
        if self._set_mode and self._fresh():
            mode = DriveMode()
            mode.header = self._header()
            mode.mode = DriveMode.MODE_AUTONOMY
            self._mode_pub.publish(mode)


def main():
    rclpy.init()
    node = AckermannCmdNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
