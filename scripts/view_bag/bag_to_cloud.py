#!/usr/bin/env python3
"""Rebuild a point cloud of the whole run from the recorded raw stereo pair and publish it once.

The bags contain no depth or map, only the raw mono8 stereo images, their camera_info, the static
TFs (mounting + left->right extrinsics) and the cuVSLAM odometry. Every STEP seconds a pair is
rectified, matched with OpenCV SGBM, back-projected, moved into the odom frame with the odometry
pose and merged into a voxel grid. The result is a latched PointCloud2 in the odom frame
(grey values as rgb), cached on disk because reading the images takes a while.
  bag_to_cloud.py <bag_dir> [--cache DIR] [--step 0.5] [--voxel 0.06] [--max-range 4.0]
"""
import argparse
import glob
import hashlib
import os
import sys
import time
import warnings

import cv2
import numpy as np
import rclpy
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_msgs.msg import TFMessage

warnings.filterwarnings("ignore", category=DeprecationWarning)

LEFT, RIGHT = "/visual_slam/image_0", "/visual_slam/image_1"
INFO_LEFT, INFO_RIGHT = "/visual_slam/camera_info_0", "/visual_slam/camera_info_1"
ODOM = "/visual_slam/tracking/odometry"
CACHE_VERSION = 1
MIN_RANGE = 0.3  # m, below this the 128 px disparity search range runs out anyway
MAX_STAMP_SKEW = 0.008  # s between the left and right image of a pair
POINT_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def read(bag_dir, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir), rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while reader.has_next():
        yield reader.read_next()


def matrix(translation, quat):
    x, y, z, w = quat
    m = np.eye(4)
    m[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    m[:3, 3] = translation
    return m


def load_static(bag_dir):
    """Return {child_frame: (parent_frame, T_parent_child)} from /tf_static."""
    frames = {}
    for _, data, _ in read(bag_dir, ["/tf_static"]):
        for t in deserialize_message(data, TFMessage).transforms:
            tr, q = t.transform.translation, t.transform.rotation
            frames[t.child_frame_id] = (t.header.frame_id, matrix([tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w]))
    return frames


def chain(frames, child, target):
    """T_target_child by walking up the static tree."""
    m = np.eye(4)
    while child != target:
        parent, t = frames[child]
        m = t @ m
        child = parent
    return m


def load_camera_infos(bag_dir):
    infos = {}
    for topic, data, _ in read(bag_dir, [INFO_LEFT, INFO_RIGHT]):
        infos.setdefault(topic, deserialize_message(data, CameraInfo))
        if len(infos) == 2:
            break
    return infos[INFO_LEFT], infos[INFO_RIGHT]


def load_odometry(bag_dir):
    rows = []
    for _, data, _ in read(bag_dir, [ODOM]):
        m = deserialize_message(data, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        rows.append((stamp(m.header), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    a = np.array(rows)
    if len(a) == 0:
        sys.exit(f"No Odometry messages on {ODOM}")
    return a[np.argsort(a[:, 0])]


def pose_at(odom, t):
    i = int(np.clip(np.searchsorted(odom[:, 0], t), 1, len(odom) - 1))
    i = i if abs(odom[i, 0] - t) < abs(odom[i - 1, 0] - t) else i - 1
    if abs(odom[i, 0] - t) > 0.1:
        return None
    return matrix(odom[i, 1:4], odom[i, 4:8])


def voxel_unique(points, grey, voxel):
    k = np.floor(points / voxel).astype(np.int64) + (1 << 20)
    keys = (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]
    keys, idx = np.unique(keys, return_index=True)
    return keys, points[idx], grey[idx]


def build_cloud(bag_dir, step, voxel, max_range):
    t0 = time.time()
    frames = load_static(bag_dir)
    info_l, info_r = load_camera_infos(bag_dir)
    odom = load_odometry(bag_dir)
    print(f"[cloud] {len(odom)} odometry poses, reading images every {step} s", flush=True)

    k_l, k_r = np.array(info_l.k).reshape(3, 3), np.array(info_r.k).reshape(3, 3)
    size = (info_l.width, info_l.height)
    # TF left->right is the pose of the right camera in the left frame; OpenCV wants X_r = R X_l + T.
    m_lr = chain(frames, "right_optical", "left_optical")
    r_lr, t_lr = m_lr[:3, :3], m_lr[:3, 3]
    r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(
        k_l, np.array(info_l.d), k_r, np.array(info_r.d), size, r_lr.T, -r_lr.T @ t_lr,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    baseline = -p2[0, 3] / p2[0, 0]
    print(f"[cloud] baseline {baseline * 1000:.1f} mm, rectified focal {p1[0, 0]:.1f} px", flush=True)
    maps_l = cv2.initUndistortRectifyMap(k_l, np.array(info_l.d), r1, p1[:, :3], size, cv2.CV_16SC2)
    maps_r = cv2.initUndistortRectifyMap(k_r, np.array(info_r.d), r2, p2[:, :3], size, cv2.CV_16SC2)

    m_base_left = chain(frames, "left_optical", "base_link")
    sgbm = cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=128, blockSize=5, P1=8 * 25, P2=32 * 25, disp12MaxDiff=1,
        uniquenessRatio=10, speckleWindowSize=100, speckleRange=2, preFilterCap=31,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    chunks, total_frames, next_t, left = [], 0, None, None

    def merge(parts):
        keys = np.concatenate([p[0] for p in parts])
        pts = np.concatenate([p[1] for p in parts])
        gry = np.concatenate([p[2] for p in parts])
        _, idx = np.unique(keys, return_index=True)
        return [(keys[idx], pts[idx], gry[idx])]

    for topic, data, ts in read(bag_dir, [LEFT, RIGHT]):
        t = ts * 1e-9
        if topic == LEFT:
            left = deserialize_message(data, Image) if next_t is None or t >= next_t else None
            continue
        if left is None:
            continue
        right = deserialize_message(data, Image)
        pair, left = left, None
        if abs(stamp(pair.header) - stamp(right.header)) > MAX_STAMP_SKEW:
            continue
        m_odom_base = pose_at(odom, stamp(pair.header))
        if m_odom_base is None:
            continue
        next_t = t + step

        img_l = np.frombuffer(pair.data, np.uint8).reshape(pair.height, pair.width)
        img_r = np.frombuffer(right.data, np.uint8).reshape(right.height, right.width)
        rect_l = cv2.remap(img_l, *maps_l, cv2.INTER_LINEAR)
        rect_r = cv2.remap(img_r, *maps_r, cv2.INTER_LINEAR)
        disp = sgbm.compute(rect_l, rect_r).astype(np.float32) / 16.0
        xyz = cv2.reprojectImageTo3D(disp, q)
        z = xyz[..., 2]
        ok = (disp > 0) & (z > MIN_RANGE) & (z < max_range)
        ok[1::2, :] = False  # every second row and column is plenty for a 6 cm voxel map
        ok[:, 1::2] = False
        if not ok.any():
            continue
        m_odom_left = m_odom_base @ m_base_left
        p_left = xyz[ok] @ r1  # X_left = R1^T X_rect
        p_odom = p_left @ m_odom_left[:3, :3].T + m_odom_left[:3, 3]
        chunks.append(voxel_unique(p_odom, rect_l[ok], voxel))
        total_frames += 1
        if total_frames % 25 == 0:
            chunks = merge(chunks)
            print(f"[cloud] {total_frames} frames, {len(chunks[0][0])} points, "
                  f"{time.time() - t0:.0f} s", flush=True)

    if not chunks:
        sys.exit("No usable stereo pairs found")
    _, pts, gry = merge(chunks)[0]
    out = np.empty(len(pts), POINT_DTYPE)
    out["x"], out["y"], out["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    g = gry.astype(np.uint32)
    out["rgb"] = (g << 16) | (g << 8) | g
    print(f"[cloud] done: {total_frames} frames, {len(out)} points, {time.time() - t0:.0f} s", flush=True)
    return out


def cache_path(cache_dir, bag_dir, step, voxel, max_range):
    files = sorted(glob.glob(os.path.join(bag_dir, "*.mcap")))
    desc = repr((CACHE_VERSION, step, voxel, max_range,
                 [(os.path.basename(f), os.stat(f).st_size) for f in files]))
    return os.path.join(cache_dir, "cloud_" + hashlib.sha1(desc.encode()).hexdigest()[:16] + ".npy")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_dir")
    ap.add_argument("--cache")
    ap.add_argument("--step", type=float, default=0.5)
    ap.add_argument("--voxel", type=float, default=0.06)
    ap.add_argument("--max-range", type=float, default=4.0)
    ap.add_argument("--topic", default="/replay/cloud")
    args = ap.parse_args()

    path = cache_path(args.cache, args.bag_dir, args.step, args.voxel, args.max_range) if args.cache else None
    if path and os.path.exists(path):
        cloud = np.load(path)
        print(f"[cloud] loaded {len(cloud)} points from cache", flush=True)
    else:
        cloud = build_cloud(args.bag_dir, args.step, args.voxel, args.max_range)
        if path:
            np.save(path, cloud)

    rclpy.init()
    node = Node("bag_to_cloud")
    msg = PointCloud2()
    msg.header.frame_id = "odom"  # stamp 0: RViz draws it without waiting for a matching TF time
    msg.height, msg.width = 1, len(cloud)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=d, count=1)
                  for i, (n, d) in enumerate([("x", 7), ("y", 7), ("z", 7), ("rgb", 7)])]
    msg.point_step = POINT_DTYPE.itemsize
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = cloud.view(np.uint8).reshape(-1)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_publisher(PointCloud2, args.topic, qos).publish(msg)
    node.get_logger().info(f"Published {len(cloud)} points on {args.topic}")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
