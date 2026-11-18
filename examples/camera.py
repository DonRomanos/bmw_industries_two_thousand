#!/usr/bin/env python3
"""Receive the car's camera preview, decode it with OpenCV and save the latest frame.

    python3 examples/camera.py                          # left preview, saves examples/out/frame.jpg
    python3 examples/camera.py --topic /back_camera/image/compressed
    python3 examples/camera.py --show                   # needs a display (not available in the dev container)

Open the saved file in VS Code to look at it; it is rewritten about once per second.
Use the /preview topics over Wi-Fi: the raw /visual_slam/image_* topics are far too big.
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class CameraViewer(Node):
    def __init__(self, topic, out, show):
        super().__init__('camera_example')
        self.out, self.show = out, show
        self.last_save = 0.0
        self.count = 0
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        # Best-effort QoS (sensor data) matches best-effort and reliable publishers alike.
        self.create_subscription(CompressedImage, topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f'waiting for {topic} ...')

    def on_image(self, msg):
        # msg.data is a JPEG byte stream; decode it into an HxWx3 BGR numpy array.
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning('could not decode image')
            return
        self.count += 1

        # <-- your image processing goes here (detect a person, a red car, ...)

        now = time.monotonic()
        if now - self.last_save >= 1.0:
            self.last_save = now
            cv2.imwrite(self.out, frame)
            h, w = frame.shape[:2]
            self.get_logger().info(
                f'{w}x{h}, mean brightness {frame.mean():.0f}, {self.count} frames so far -> {self.out}')
        if self.show:
            cv2.imshow('camera', frame)
            cv2.waitKey(1)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--topic', default='/preview/left/compressed')
    p.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out', 'frame.jpg'))
    p.add_argument('--show', action='store_true', help='open a window (needs a display)')
    args = p.parse_args()

    rclpy.init()
    node = CameraViewer(args.topic, args.out, args.show)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
