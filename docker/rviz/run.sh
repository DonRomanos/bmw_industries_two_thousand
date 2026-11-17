#!/usr/bin/env bash
# Run RViz2 in a container on the dev host (WSL2 + WSLg) against the Jetson's ROS graph.
#   docker/rviz/run.sh [JETSON_IP]      (default 10.42.0.1 = the car's hotspot; builds the image on first use)
#
# DDS multicast is unreliable over Wi-Fi, so the Jetson is added as a static discovery peer.
# Set GPU=1 to use the WSL GPU for OpenGL instead of software rendering.

set -e
cd "$(dirname "$0")"
JETSON_IP="${1:-10.42.0.1}"
IMAGE=bmw-kitt-rviz

# BASE_IMAGE=<registry mirror>/ubuntu:24.04 selects another base image when Docker Hub is
# unreachable; the apt repos (packages.ros.org, isaac.download.nvidia.com) must be reachable too
# (pass proxies with --build-arg HTTP_PROXY=... by building manually first).
docker image inspect "$IMAGE" >/dev/null 2>&1 || \
    docker build ${BASE_IMAGE:+--build-arg BASE_IMAGE=$BASE_IMAGE} -t "$IMAGE" .

GPU_ARGS=(-e LIBGL_ALWAYS_SOFTWARE=1)
if [ "${GPU:-0}" = 1 ]; then
    GPU_ARGS=(--device /dev/dxg -v /usr/lib/wsl:/usr/lib/wsl -e LD_LIBRARY_PATH=/usr/lib/wsl/lib)
fi

exec docker run --rm -it --network host \
    -e DISPLAY="${DISPLAY:-:0}" -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}" \
    -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir -e QT_QPA_PLATFORM=xcb \
    -v /mnt/wslg:/mnt/wslg -v /mnt/wslg/.X11-unix:/tmp/.X11-unix \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" -e ROS_STATIC_PEERS="$JETSON_IP" \
    "${GPU_ARGS[@]}" "$IMAGE" "${@:2}"
