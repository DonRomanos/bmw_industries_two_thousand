#!/usr/bin/env bash
# Play a recorded bag as if it were the car, inside the dev container. Extra arguments go to `ros2 bag play`.
#   scripts/play_bag.sh bags/<name> [--rate 0.5] [--loop] [--topics /tf /preview/left/compressed ...]
#   scripts/play_bag.sh <name>                      # same as bags/<name>
#
# The bag is published in an isolated DDS domain (77, shared memory inside this container), so it never reaches a real car. Run the
# examples in another terminal that is switched to the same domain:
#   source scripts/bitt_env.sh replay
#   python3 examples/camera.py
# Only topics that were recorded exist; see `python3 examples/bag_reader.py info bags/<name>`.
# Big image topics (raw stereo pair) need a lot of CPU and memory: use --topics to play only what you need.
# Reads from the bag only; for offline analysis without ROS nodes use examples/bag_reader.py.

set -eo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
[ $# -ge 1 ] || { sed -n '2,5p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
BAG=$1; shift
[ -d "$BAG" ] || { [ -d "$ROOT/bags/$BAG" ] && BAG=$ROOT/bags/$BAG; }
[ -f "$BAG/metadata.yaml" ] || { echo "$BAG is not a rosbag2 directory (no metadata.yaml)"; exit 1; }

# The ROS setup scripts are not safe under `set -u`.
source /opt/ros/lyrical/setup.bash
source /opt/bitt_ws/install/setup.bash
export ROS_DOMAIN_ID=77 FASTDDS_BUILTIN_TRANSPORTS=SHM   # see bitt_env.sh
unset ROS_AUTOMATIC_DISCOVERY_RANGE
unset ROS_STATIC_PEERS

echo "Playing $BAG on domain 77. In another terminal: source scripts/bitt_env.sh replay"
# /tf_static and /robot_description are latched; bag play re-publishes them with their recorded QoS.
exec ros2 bag play "$BAG" "$@"
