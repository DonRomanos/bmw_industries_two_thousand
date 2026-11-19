#!/usr/bin/env python3
"""Small button window for the car, meant to sit next to RViz (scripts/rviz.sh starts it).

Lights (/lights_cmd), indicators, hazard, horn (/horn_cmd, sounds while the button is held) and
"Cancel navigation" (cancels the active Nav2 goal). Send a goal with RViz's "2D Goal Pose" tool.
Uses whatever the terminal is switched to: the car (default) or a bag replay (`source scripts/bitt_env.sh replay`).
"""
import os
import sys

import rclpy
from action_msgs.srv import CancelGoal
from car_interfaces.msg import LightsCmd
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QApplication, QGridLayout, QLabel, QPushButton, QWidget
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'examples'))
from lights import PRESETS  # noqa: E402  (same presets as examples/lights.py)

CANCEL_SERVICE = '/navigate_to_pose/_action/cancel_goal'
BUTTON_STYLE = 'QPushButton {{ background: {bg}; color: white; font-size: 15px; padding: 10px; border-radius: 4px; }}'


class Controls(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)   # as in the ESP contract
        self.lights_pub = node.create_publisher(LightsCmd, '/lights_cmd', qos)
        self.horn_pub = node.create_publisher(Bool, '/horn_cmd', qos)
        self.cancel_client = node.create_client(CancelGoal, CANCEL_SERVICE)

        self.setWindowTitle('BITT controls')
        self.setWindowFlag(Qt.WindowType.WindowStaysOnTopHint)
        grid = QGridLayout(self)
        self.status = QLabel('Ready')
        self.status.setWordWrap(True)

        self.add_button(grid, 0, 0, 'Headlights', '#3b78d8', lambda: self.send_lights('headlights'))
        self.add_button(grid, 0, 1, 'All off', '#6b7280', lambda: self.send_lights('off'))
        self.add_button(grid, 0, 2, 'Hazard', '#d97706', lambda: self.send_lights('hazard'))
        self.add_button(grid, 1, 0, '◀ Left', '#d97706', lambda: self.send_lights('left'))
        self.add_button(grid, 1, 2, 'Right ▶', '#d97706', lambda: self.send_lights('right'))
        horn = self.add_button(grid, 2, 0, 'Horn (hold)', '#7c3aed', None, colspan=3)
        horn.pressed.connect(lambda: self.send_horn(True))
        horn.released.connect(lambda: self.send_horn(False))
        self.add_button(grid, 3, 0, 'Cancel navigation', '#dc2626', self.cancel_navigation, colspan=3)
        grid.addWidget(self.status, 4, 0, 1, 3)

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.spin)
        self.spin_timer.start(20)

    def spin(self):
        # rclpy shuts its context down on SIGINT/SIGTERM (rviz.sh stops this window when RViz closes).
        if not rclpy.ok():
            QApplication.quit()
            return
        rclpy.spin_once(self.node, timeout_sec=0)

    def add_button(self, grid, row, col, text, color, on_click, colspan=1):
        button = QPushButton(text)
        button.setStyleSheet(BUTTON_STYLE.format(bg=color))
        if on_click:
            button.clicked.connect(on_click)
        grid.addWidget(button, row, col, 1, colspan)
        return button

    def send_lights(self, name):
        msg = PRESETS[name]()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.lights_pub.publish(msg)
        self.status.setText(f'Lights: {name}' if self.lights_pub.get_subscription_count()
                            else f'Lights: {name} (nobody listens on /lights_cmd, is the ESP32 connected?)')

    def send_horn(self, on):
        self.horn_pub.publish(Bool(data=on))
        self.status.setText('Horn' if on else 'Ready')

    def cancel_navigation(self):
        if not self.cancel_client.service_is_ready():
            self.status.setText(f'Nav2 not reachable ({CANCEL_SERVICE})')
            return
        # An empty request cancels all goals.
        future = self.cancel_client.call_async(CancelGoal.Request())
        future.add_done_callback(lambda f: self.status.setText(
            f'Navigation cancelled ({len(f.result().goals_canceling)} goal(s))'))
        self.status.setText('Cancelling navigation...')

    def closeEvent(self, event):
        if rclpy.ok():
            self.horn_pub.publish(Bool(data=False))
        super().closeEvent(event)


def main():
    rclpy.init()
    node = rclpy.create_node('bitt_controls')
    app = QApplication(sys.argv)
    window = Controls(node)
    window.show()
    try:
        app.exec()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
