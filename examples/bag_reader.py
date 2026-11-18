#!/usr/bin/env python3
"""Read a recorded rosbag2 (MCAP) without a running ROS graph: list it, print messages, export images / odometry.

    python3 examples/bag_reader.py info   bags/Office
    python3 examples/bag_reader.py echo   bags/Office /visual_slam/tracking/odometry -n 3
    python3 examples/bag_reader.py images bags/Office /visual_slam/image_0 --every 60 --max 20 --out examples/out/bag_images
    python3 examples/bag_reader.py odom   bags/Office --out examples/out/odom.csv

Use it in your own code too (put this file next to your script, or add examples/ to PYTHONPATH):

    from bag_reader import read_messages, image_to_bgr
    for topic, t_ns, msg in read_messages('bags/Office', ['/visual_slam/image_0']):
        frame = image_to_bgr(msg)          # numpy array, ready for OpenCV
        ...

To feed a bag to the *other* examples (camera.py, obstacles.py, goal.py, ...) as if it were the car, play it
with scripts/play_bag.sh instead.
Which topics a bag contains depends on how it was recorded; start with `info`.
"""
import argparse
import os
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_yaml
from rosidl_runtime_py.utilities import get_message


def bag_info(bag):
    """Return (metadata, [(topic, type, count)]) for a bag directory."""
    meta = rosbag2_py.Info().read_metadata(bag, '')
    topics = sorted((t.topic_metadata.name, t.topic_metadata.type, t.message_count) for t in meta.topics_with_message_count)
    return meta, topics


def read_messages(bag, topics=None):
    """Yield (topic, timestamp_ns, message) in recording order. Only the given topics are deserialized."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topics:
        missing = [t for t in topics if t not in types]
        if missing:
            sys.exit(f'topic(s) not in the bag: {", ".join(missing)} (see `info`)')
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    classes = {}
    while reader.has_next():
        topic, data, t_ns, _ = reader.read_next_ext()   # (topic, bytes, receive time, send time)
        if topic not in classes:
            classes[topic] = get_message(types[topic])
        yield topic, t_ns, deserialize_message(data, classes[topic])


def image_to_bgr(msg):
    """Convert sensor_msgs/Image or CompressedImage into an OpenCV-style BGR (or gray) numpy array."""
    if hasattr(msg, 'format'):   # CompressedImage
        return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    channels = {'mono8': 1, 'mono16': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}
    if msg.encoding not in channels:
        raise ValueError(f'unsupported image encoding {msg.encoding}')
    c = channels[msg.encoding]
    dtype = np.uint16 if msg.encoding == 'mono16' else np.uint8
    row = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = row[:, :msg.width * c * np.dtype(dtype).itemsize].copy().view(dtype)
    img = img.reshape(msg.height, msg.width, c) if c > 1 else img.reshape(msg.height, msg.width)
    if msg.encoding == 'mono16':
        img = (img >> 8).astype(np.uint8)
    elif msg.encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif msg.encoding == 'rgba8':
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    elif msg.encoding == 'bgra8':
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img


def yaw_of(q):
    return float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def cmd_info(args):
    meta, topics = bag_info(args.bag)
    duration = meta.duration.total_seconds() if hasattr(meta.duration, 'total_seconds') else meta.duration.nanoseconds / 1e9
    print(f'{args.bag}: {meta.message_count} messages, {duration:.1f} s\n')
    print(f'{"topic":<50} {"type":<42} {"count":>8} {"Hz":>7}')
    for name, typ, count in topics:
        print(f'{name:<50} {typ:<42} {count:>8} {count / duration if duration else 0:>7.1f}')


def cmd_echo(args):
    shown = 0
    for topic, t_ns, msg in read_messages(args.bag, [args.topic]):
        print(f'--- t = {t_ns / 1e9:.3f} s')
        print(message_to_yaml(msg))
        shown += 1
        if args.n and shown >= args.n:
            break


def cmd_images(args):
    os.makedirs(args.out, exist_ok=True)
    seen = saved = 0
    for _, t_ns, msg in read_messages(args.bag, [args.topic]):
        seen += 1
        if (seen - 1) % args.every:
            continue
        cv2.imwrite(os.path.join(args.out, f'{saved:05d}_{t_ns}.jpg'), image_to_bgr(msg))
        saved += 1
        if args.max and saved >= args.max:
            break
    print(f'saved {saved} of {seen}+ frames to {args.out}')


def cmd_odom(args):
    out = open(args.out, 'w') if args.out else sys.stdout
    out.write('t_s,x,y,yaw,vx,wz\n')
    t0 = None
    for _, t_ns, m in read_messages(args.bag, [args.topic]):
        t0 = t_ns if t0 is None else t0
        p, v = m.pose.pose, m.twist.twist
        out.write(f'{(t_ns - t0) / 1e9:.3f},{p.position.x:.4f},{p.position.y:.4f},{yaw_of(p.orientation):.4f},'
                  f'{v.linear.x:.4f},{v.angular.z:.4f}\n')
    if args.out:
        out.close()
        print(f'wrote {args.out}')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)

    s = sub.add_parser('info', help='list topics, types, message counts and rates')
    s.add_argument('bag')
    s.set_defaults(fn=cmd_info)

    s = sub.add_parser('echo', help='print messages of one topic')
    s.add_argument('bag')
    s.add_argument('topic')
    s.add_argument('-n', type=int, default=1, help='number of messages (0 = all)')
    s.set_defaults(fn=cmd_echo)

    s = sub.add_parser('images', help='save frames of an Image / CompressedImage topic as JPEG')
    s.add_argument('bag')
    s.add_argument('topic')
    s.add_argument('--every', type=int, default=30, help='keep every Nth frame')
    s.add_argument('--max', type=int, default=50, help='stop after this many saved frames (0 = no limit)')
    s.add_argument('--out', default='examples/out/bag_images')
    s.set_defaults(fn=cmd_images)

    s = sub.add_parser('odom', help='export an Odometry topic as CSV (time, x, y, yaw, speed, yaw rate)')
    s.add_argument('bag')
    s.add_argument('--topic', default='/visual_slam/tracking/odometry')
    s.add_argument('--out', help='CSV file (default: stdout)')
    s.set_defaults(fn=cmd_odom)

    args = p.parse_args()
    if not os.path.isfile(os.path.join(args.bag, 'metadata.yaml')):
        p.error(f'{args.bag} is not a rosbag2 directory (no metadata.yaml)')
    args.fn(args)


if __name__ == '__main__':
    main()
