#!/usr/bin/env bash
# Container entrypoint for bag replay: publishes the whole path, plays the bag and opens RViz2.
#   replay.sh /bag [RATE]
# env: IMAGES=0 skips the camera images and the camera views (default 1), LOOP=1 loops the bag,
#      PARTIAL=1 uses only the complete .mcap files (for an incompletely copied bag),
#      CLOUD=auto|1|0 rebuilds the point cloud from the stereo images (auto: only if the bag has
#      no recorded nvblox cloud)
set -e
source /opt/ros/lyrical/setup.bash
BAG=${1:?bag directory}
RATE=${2:-1.0}

if [ "${PARTIAL:-0}" = 1 ]; then
    mkdir /tmp/bag
    for f in "$BAG"/*.mcap; do
        # A finished MCAP ends with the magic bytes "\x89MCAP0\r\n"; truncated copies do not.
        if [ "$(tail -c 7 "$f" | head -c 5)" = "MCAP0" ]; then
            ln -s "$f" /tmp/bag/
            echo "using   $(basename "$f")"
        else
            echo "skipped $(basename "$f") (truncated)"
        fi
    done
    ros2 bag reindex -s mcap /tmp/bag
    BAG=/tmp/bag
fi

EXCLUDE=""
if [ "${IMAGES:-1}" = 1 ]; then
    # Left/right images, plus feature tracks and a dashboard (speed, skew, tracking time) computed here.
    python3 /opt/replay/replay_vis.py &
else
    EXCLUDE="--exclude-topics /visual_slam/image_0 /visual_slam/image_1 /diagnostics"
fi

python3 /opt/replay/odom_to_path.py "$BAG" &
# Bags recorded by record_bag.sh contain the stack's own nvblox pointcloud, which is replayed as
# recorded. Older bags only have the raw stereo images: rebuild a cloud from them in the
# background (cached in /cache); it appears in RViz when it is ready.
RECORDED_CLOUD=0
grep -q "name: /nvblox_node/static_esdf_pointcloud" "$BAG/metadata.yaml" && RECORDED_CLOUD=1
CLOUD=${CLOUD:-auto}
[ "$CLOUD" = auto ] && CLOUD=$((1 - RECORDED_CLOUD))
echo "recorded nvblox cloud in bag: $RECORDED_CLOUD, recomputing cloud from stereo images: $CLOUD"
[ "$CLOUD" = 1 ] && python3 /opt/replay/bag_to_cloud.py "$BAG" --cache /cache &
# /tf_static and /robot_description are latched; bag play re-publishes them with their recorded QoS.
ros2 bag play "$BAG" --clock -r "$RATE" ${LOOP:+--loop} $EXCLUDE > /tmp/bag_play.log 2>&1 &
sleep 2
exec rviz2 -d /config/bag.rviz --ros-args -p use_sim_time:=true
