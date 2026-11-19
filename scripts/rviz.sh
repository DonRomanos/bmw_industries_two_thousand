#!/usr/bin/env bash
# Start RViz2 with the student layout (costmaps, plan, footprint, SLAM path, camera preview, goal tool).
#   scripts/rviz.sh [extra rviz2 args]
#
# Uses whatever the terminal is switched to: the car (default) or a bag replay (`source scripts/bitt_env.sh replay`).
# Pick the "2D Goal Pose" tool in the toolbar to send a navigation goal to /goal_pose (the car drives when Nav2 gets
# a goal; keep the remote controller within reach). The nvblox displays are off by default; tick them in the Displays panel.
# Needs a display: see "RViz2" in the README (the dev container forwards X11 / WSLg). Save changes with
# File > Save Config; commit them to rviz/bitt.rviz if you want to keep them.

set -eo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
if [ -z "${DISPLAY:-}" ]; then
    echo "DISPLAY is not set: no X server reachable. See the README, section RViz2."
    exit 1
fi
source /opt/ros/lyrical/setup.bash
source /opt/bitt_ws/install/setup.bash
exec rviz2 -d "$ROOT/rviz/bitt.rviz" "$@"
