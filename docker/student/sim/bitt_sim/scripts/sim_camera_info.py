#!/usr/bin/env python3
"""camera_info for the simulated stereo pair, from the real calibration (calib/*.yaml).

The Gazebo cameras render an ideal pinhole with the calibrated fx, fy, cx, cy (set in the
generated sensor), so the distortion is published as zero. K, R and P are the calibrated ones.
Each camera_info copies the header of the image it belongs to, as the camera node does.
"""

import os

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def load(path):
    with open(path) as f:
        c = yaml.safe_load(f)
    info = CameraInfo()
    info.width = c['image_width']
    info.height = c['image_height']
    info.distortion_model = c['distortion_model']
    info.d = [0.0] * len(c['distortion_coefficients']['data'])
    info.k = [float(v) for v in c['camera_matrix']['data']]
    info.r = [float(v) for v in c['rectification_matrix']['data']]
    info.p = [float(v) for v in c['projection_matrix']['data']]
    return info


class SimCameraInfo(Node):
    def __init__(self):
        super().__init__('sim_camera_info')
        self.declare_parameter('calib_dir', '')
        calib_dir = self.get_parameter('calib_dir').value
        self._info = {}
        self._pubs = {}
        for idx, side in ((0, 'left'), (1, 'right')):
            self._info[idx] = load(os.path.join(calib_dir, f'{side}.yaml'))
            self._pubs[idx] = self.create_publisher(
                CameraInfo, f'/visual_slam/camera_info_{idx}', 4)
            self.create_subscription(
                Image, f'/visual_slam/image_{idx}',
                lambda msg, i=idx: self._on_image(i, msg), qos_profile_sensor_data)

    def _on_image(self, idx, msg):
        info = self._info[idx]
        info.header = msg.header
        self._pubs[idx].publish(info)


def main():
    rclpy.init()
    node = SimCameraInfo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
