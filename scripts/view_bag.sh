#!/usr/bin/env bash
# Replay a recorded rosbag2 (see record_bag.sh) in RViz2 inside the student image (docker/student/Dockerfile, the
# same image as the dev container) from the host (WSL2 + WSLg).
#   scripts/view_bag.sh [bags/<name>] [--rate 1.0] [--no-images] [--loop] [--partial] [--no-cloud] [--recompute-cloud]
#
# Shows the whole recorded odometry path at once, plus the robot/TF moving while the bag plays,
# the left/right camera images, OpenCV feature tracks on the left image and a dashboard of speed,
# stereo skew and cuVSLAM tracking time (--no-images turns the camera views off to save CPU).
# Bags from record_bag.sh contain the stack's own paths and nvblox pointcloud, which are replayed
# as recorded. For bags without them the point cloud is rebuilt from the recorded stereo images
# (seconds to a minute, cached in bags/.cache); --no-cloud skips that, --recompute-cloud forces it.
# --partial uses only the complete .mcap files of a bag that was not copied completely.
# The container uses its own isolated DDS domain, so it does not touch the Jetson's ROS graph.
# The scripts and rviz config in docker/rviz are mounted into the container, so editing them needs no
# rebuild; the image is only rebuilt when it is missing, the student Dockerfile changed or REBUILD=1.
# RViz's File > Save Config (Ctrl+S) writes the panel layout back to docker/rviz/bag.rviz.
# Set GPU=1 to use the WSL GPU for OpenGL instead of software rendering, and
# BASE_IMAGE=<registry mirror>/ros:lyrical-ros-base if Docker Hub is unreachable.

set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
BAG=""
RATE=1.0
IMAGES=1
LOOP=""
PARTIAL=0
CLOUD=auto
while [ $# -gt 0 ]; do
    case "$1" in
        --rate) RATE=$2; shift 2 ;;
        --images) IMAGES=1; shift ;;
        --no-images) IMAGES=0; shift ;;
        --loop) LOOP=1; shift ;;
        --partial) PARTIAL=1; shift ;;
        --no-cloud) CLOUD=0; shift ;;
        --recompute-cloud) CLOUD=1; shift ;;
        -*) sed -n '2,3p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
        *) BAG=$1; shift ;;
    esac
done
if [ -z "$BAG" ]; then
    BAG=$(dirname "$(ls -1t "$ROOT"/bags/*/metadata.yaml | head -1)")
fi
BAG=$(cd "$BAG" && pwd)
[ -f "$BAG/metadata.yaml" ] || { echo "$BAG is not a rosbag2 directory (no metadata.yaml)"; exit 1; }
if [ "$PARTIAL" = 0 ]; then
    for f in $(sed -n '/relative_file_paths:/,/^  files:/{s/^ *- //p}' "$BAG/metadata.yaml"); do
        if [ ! -f "$BAG/$f" ] || [ "$(tail -c 7 "$BAG/$f" | head -c 5)" != "MCAP0" ]; then
            echo "$f is missing or truncated, the bag was not copied completely."
            echo "Re-copy it (e.g. rsync -avP kitt:ros_ws/bags/$(basename "$BAG")/ $BAG/) or use --partial."
            exit 1
        fi
    done
fi
echo "Bag: $BAG"

IMAGE=bmw-kitt-student
DOCKERFILE=$ROOT/docker/student/Dockerfile
# Building contacts the registry even when all layers are cached (it fails behind a proxy), so only
# build if the image is missing, REBUILD=1 is set, or the Dockerfile changed since it was built.
need_build=1
if [ "${REBUILD:-0}" != 1 ] && created=$(docker image inspect -f '{{.Created}}' "$IMAGE" 2>/dev/null); then
    stamp=$(mktemp); touch -d "$created" "$stamp"
    [ -z "$(find "$DOCKERFILE" -newer "$stamp" -print -quit)" ] && need_build=0
    rm -f "$stamp"
fi
if [ "$need_build" = 1 ]; then
    docker build ${BASE_IMAGE:+--build-arg BASE_IMAGE=$BASE_IMAGE} -f "$DOCKERFILE" -t "$IMAGE" "$ROOT"
fi

GPU_ARGS=(-e LIBGL_ALWAYS_SOFTWARE=1)
if [ "${GPU:-0}" = 1 ]; then
    GPU_ARGS=(--device /dev/dxg -v /usr/lib/wsl:/usr/lib/wsl -e LD_LIBRARY_PATH=/usr/lib/wsl/lib)
fi

mkdir -p "$ROOT/bags/.cache"
exec docker run --rm -it \
    -e DISPLAY="${DISPLAY:-:0}" -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}" \
    -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir -e QT_QPA_PLATFORM=xcb \
    -v /mnt/wslg:/mnt/wslg -v /mnt/wslg/.X11-unix:/tmp/.X11-unix \
    -e ROS_DOMAIN_ID=77 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    -e IMAGES="$IMAGES" -e PARTIAL="$PARTIAL" -e CLOUD="$CLOUD" ${LOOP:+-e LOOP=1} \
    -v "$BAG":/bag:ro -v "$ROOT/bags/.cache":/cache \
    -v "$ROOT/docker/rviz":/opt/replay:ro -v "$ROOT/docker/rviz/bag.rviz":/config/bag.rviz \
    "${GPU_ARGS[@]}" "$IMAGE" bash /opt/replay/replay.sh /bag "$RATE"
