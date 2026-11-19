#!/usr/bin/env python3
"""Extra camera views for the bag replay, published as plain images for RViz Image displays.

  /replay/features   left camera with OpenCV corner tracks (Shi-Tomasi + Lucas-Kanade) drawn on it
  /replay/dashboard  rolling plots of speed, stereo skew and cuVSLAM tracking time, plus heading,
                     distance and tracking state

The features are computed here from the raw images, they are not cuVSLAM's own landmarks
(those are shown as a point cloud when the bag contains /visual_slam/vis/landmarks_cloud).
Plot times come from message stamps, so the dashboard follows the replay rate and resets on --loop.
"""
import collections
import math

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

LEFT = "/visual_slam/image_0"
ODOM = "/visual_slam/tracking/odometry"
SKEW = "/stereo/skew_ms"
DIAG = "/diagnostics"

MAX_TRACKS = 250
MIN_TRACKS = 150
TRAIL = 12  # frames of history drawn per track
WINDOW_S = 30.0  # seconds shown in the plots
SPEED_BASE_S = 0.25  # speed = displacement over this time, the odometry twist is not reliable
DASH_W, DASH_H = 640, 480
LOOP_JUMP_S = 1.0


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def to_gray(msg):
    if msg.encoding == "mono8":
        return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, : msg.width]
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = img[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if msg.encoding == "rgb8" else cv2.COLOR_BGR2GRAY)


def yaw_deg(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class Series:
    """(time, value) samples of the last WINDOW_S seconds."""

    def __init__(self):
        self.t = collections.deque()
        self.v = collections.deque()

    def add(self, t, v):
        if self.t and t < self.t[-1] - LOOP_JUMP_S:
            self.t.clear()
            self.v.clear()
        self.t.append(t)
        self.v.append(v)
        while self.t and self.t[0] < t - WINDOW_S:
            self.t.popleft()
            self.v.popleft()


class ReplayVis(Node):
    def __init__(self):
        super().__init__("replay_vis")
        self.pub_feat = self.create_publisher(Image, "/replay/features", 1)
        self.pub_dash = self.create_publisher(Image, "/replay/dashboard", 1)
        self.create_subscription(Image, LEFT, self.on_left, qos_profile_sensor_data)
        self.create_subscription(Odometry, ODOM, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Float32, SKEW, self.on_skew, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, DIAG, self.on_diag, qos_profile_sensor_data)
        self.create_timer(1 / 15, self.draw_dashboard)

        self.prev_gray = None
        self.pts = np.empty((0, 2), np.float32)
        self.ages = np.empty(0, np.int32)
        self.trails = []
        self.last_img_t = None

        self.speed, self.skew, self.track_ms = Series(), Series(), Series()
        self.poses = collections.deque()
        self.dist = 0.0
        self.last_xy = None
        self.heading = 0.0
        self.pos = (0.0, 0.0, 0.0)
        self.vo_status = "n/a"
        self.now = 0.0

    def on_left(self, msg):
        t = stamp(msg.header)
        gray = to_gray(msg)
        if self.last_img_t is not None and t < self.last_img_t - LOOP_JUMP_S:
            self.prev_gray = None
        self.last_img_t = t
        if self.prev_gray is None or self.prev_gray.shape != gray.shape:
            self.pts = np.empty((0, 2), np.float32)
            self.ages = np.empty(0, np.int32)
            self.trails = []
        elif len(self.pts):
            self.track(gray)
        if len(self.pts) < MIN_TRACKS:
            self.detect(gray)
        self.prev_gray = gray
        self.publish_features(msg, gray)

    def track(self, gray):
        p0 = self.pts.reshape(-1, 1, 2)
        p1, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, p0, None, winSize=(21, 21), maxLevel=3)
        back, st_b, _ = cv2.calcOpticalFlowPyrLK(gray, self.prev_gray, p1, None, winSize=(21, 21), maxLevel=3)
        ok = (st.ravel() == 1) & (st_b.ravel() == 1) & (np.abs(back - p0).max(axis=(1, 2)) < 1.0)
        h, w = gray.shape
        x, y = p1[:, 0, 0], p1[:, 0, 1]
        ok &= (x >= 0) & (x < w) & (y >= 0) & (y < h)
        self.pts = p1[ok, 0, :]
        self.ages = self.ages[ok] + 1
        self.trails = [tr for tr, k in zip(self.trails, ok) if k]
        for tr, p in zip(self.trails, self.pts):
            tr.append((float(p[0]), float(p[1])))

    def detect(self, gray):
        mask = np.full(gray.shape, 255, np.uint8)
        for x, y in self.pts:
            cv2.circle(mask, (int(x), int(y)), 12, 0, -1)
        new = cv2.goodFeaturesToTrack(gray, MAX_TRACKS - len(self.pts), 0.01, 12, mask=mask)
        if new is None:
            return
        new = new.reshape(-1, 2).astype(np.float32)
        self.pts = np.vstack([self.pts, new])
        self.ages = np.concatenate([self.ages, np.zeros(len(new), np.int32)])
        self.trails += [collections.deque([tuple(p)], maxlen=TRAIL) for p in new]

    def publish_features(self, msg, gray):
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for tr in self.trails:
            if len(tr) > 1:
                cv2.polylines(vis, [np.int32(tr).reshape(-1, 1, 2)], False, (0, 200, 255), 1, cv2.LINE_AA)
        for (x, y), age in zip(self.pts, self.ages):
            hue = int(min(age, 60))  # OpenCV hue 0..60: young = red, long-lived = green
            col = cv2.cvtColor(np.uint8([[[hue, 255, 255]]]), cv2.COLOR_HSV2BGR)[0, 0]
            cv2.circle(vis, (int(x), int(y)), 3, tuple(int(c) for c in col), -1, cv2.LINE_AA)
        mean_age = float(self.ages.mean()) if len(self.ages) else 0.0
        cv2.rectangle(vis, (0, 0), (vis.shape[1], 26), (0, 0, 0), -1)
        cv2.putText(vis, f"{len(self.pts)} tracks, mean age {mean_age:.0f} frames (OpenCV, not cuVSLAM)",
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        out = Image(header=msg.header, height=vis.shape[0], width=vis.shape[1], encoding="bgr8",
                    step=vis.shape[1] * 3, data=vis.tobytes())
        self.pub_feat.publish(out)

    def on_odom(self, msg):
        t = stamp(msg.header)
        self.now = t
        if self.poses and t < self.poses[-1][0] - LOOP_JUMP_S:
            self.poses.clear()
            self.dist = 0.0
            self.last_xy = None
        p = msg.pose.pose.position
        self.pos = (p.x, p.y, p.z)
        self.heading = yaw_deg(msg.pose.pose.orientation)
        if self.last_xy is not None:
            self.dist += math.dist(self.last_xy, (p.x, p.y))
        self.last_xy = (p.x, p.y)
        self.poses.append((t, p.x, p.y, p.z))
        while len(self.poses) > 1 and self.poses[1][0] <= t - SPEED_BASE_S:
            self.poses.popleft()
        t0, x0, y0, z0 = self.poses[0]
        if t - t0 > 0.05:
            self.speed.add(t, math.dist((x0, y0, z0), (p.x, p.y, p.z)) / (t - t0))

    def on_skew(self, msg):
        self.skew.add(self.now, msg.data)

    def on_diag(self, msg):
        for status in msg.status:
            values = {kv.key: kv.value for kv in status.values}
            if "vo_status" in values:
                self.vo_status = values["vo_status"]
            if "track_execution_time" in values:
                try:
                    self.track_ms.add(self.now, float(values["track_execution_time"]) * 1000)
                except ValueError:
                    pass

    def plot(self, img, rect, series, title, unit, color):
        x0, y0, w, h = rect
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (70, 70, 70), 1)
        vals = list(series.v)
        top = max(max(vals) if vals else 0.0, 1e-3) * 1.15
        cur = f"{vals[-1]:.2f}" if vals else "-"
        cv2.putText(img, f"{title}: {cur} {unit}  (max {top / 1.15:.2f})", (x0 + 6, y0 + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
        if len(vals) > 1:
            t_end = series.t[-1]
            pts = [(x0 + int((t - (t_end - WINDOW_S)) / WINDOW_S * w), y0 + h - int(v / top * (h - 24)) - 2)
                   for t, v in zip(series.t, series.v)]
            cv2.polylines(img, [np.int32(pts).reshape(-1, 1, 2)], False, color, 1, cv2.LINE_AA)

    def draw_dashboard(self):
        img = np.full((DASH_H, DASH_W, 3), 30, np.uint8)
        ok = self.vo_status == "OK"
        cv2.putText(img, f"VO: {self.vo_status}", (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (80, 220, 80) if ok else (60, 60, 255), 2, cv2.LINE_AA)
        cv2.putText(img, f"heading {self.heading:6.1f} deg   distance {self.dist:6.2f} m",
                    (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1, cv2.LINE_AA)
        x, y, z = self.pos
        cv2.putText(img, f"x {x:6.2f}  y {y:6.2f}  z {z:6.2f} m", (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1, cv2.LINE_AA)
        h = (DASH_H - 100 - 3 * 8) // 3
        for i, (series, title, unit, color) in enumerate((
                (self.speed, "speed", "m/s", (255, 200, 80)),
                (self.skew, "stereo skew", "ms", (80, 200, 255)),
                (self.track_ms, "cuVSLAM track time", "ms", (120, 255, 120)))):
            self.plot(img, (10, 96 + i * (h + 8), DASH_W - 20, h), series, title, unit, color)
        self.pub_dash.publish(Image(height=DASH_H, width=DASH_W, encoding="bgr8", step=DASH_W * 3,
                                    data=img.tobytes()))


def main():
    rclpy.init()
    rclpy.spin(ReplayVis())


if __name__ == "__main__":
    main()
