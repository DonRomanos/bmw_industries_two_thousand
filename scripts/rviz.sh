#!/usr/bin/env bash
# Start RViz2 with the student layout (costmaps, plan, Nav2 debug overlays, footprint, SLAM path, camera preview, goal tool).
#   scripts/rviz.sh [extra rviz2 args]
#
# Uses whatever the terminal is switched to: the car (default) or a bag replay (`source scripts/bitt_env.sh replay`).
# The same layout is used by scripts/view_bag.sh, which switches on the "nvblox" and "Bag replay" groups.
# Pick the "2D Goal Pose" tool in the toolbar to send a navigation goal to /goal_pose (the car drives when Nav2 gets
# a goal; keep the remote controller within reach). The nvblox displays are off by default; tick them in the Displays panel.
# Needs a display: see "RViz2" in the README (the dev container forwards X11 / WSLg). Save changes with
# File > Save Config As... rviz/bitt.rviz if you want to keep them (RViz works on a temporary copy of the layout, because it
# rewrites the file it was started with when it is closed).
# Also opens the button window scripts/controls.py (lights, hazard, horn, cancel navigation); NO_CONTROLS=1 skips it.
# Also starts scripts/car_status_viz.py: battery voltage, wheel speed and e-stop as text above the car.

set -eo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
if [ -z "${DISPLAY:-}" ]; then
    echo "DISPLAY is not set: no X server reachable. See the README, section RViz2."
    exit 1
fi
source /opt/ros/lyrical/setup.bash
source /opt/bitt_ws/install/setup.bash
if [ "${NO_CONTROLS:-0}" != 1 ]; then
    if python3 -c "import PyQt6.QtWidgets" 2>/dev/null; then
        python3 "$ROOT/scripts/controls.py" &
        CONTROLS_PID=$!
    else
        echo "PyQt6 is missing, so the button window is not started (rebuild the dev container, or: sudo apt-get install python3-pyqt6)."
    fi
fi
python3 "$ROOT/scripts/car_status_viz.py" &
STATUS_PID=$!
LAYOUT=$(mktemp --suffix=.rviz)
cp "$ROOT/rviz/bitt.rviz" "$LAYOUT"
trap 'rm -f "$LAYOUT"; kill $CONTROLS_PID $STATUS_PID 2>/dev/null || true' EXIT
rviz2 -d "$LAYOUT" "$@"
