# Source this to switch the current terminal between the real car and bag replay:
#   source scripts/kitt_env.sh replay    # isolated DDS domain 77, localhost only: sees only play_bag.sh
#   source scripts/kitt_env.sh live      # the car (default; what a new terminal in the dev container uses)
#
# The examples (camera.py, obstacles.py, goal.py, ...) behave the same in both; in replay mode they cannot
# reach the car, so replaying a bag can never move it.

case "${1:-}" in
    replay)
        : "${KITT_LIVE_DOMAIN_ID:=${ROS_DOMAIN_ID:-0}}"
        export KITT_LIVE_DOMAIN_ID
        export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
        unset ROS_STATIC_PEERS
        echo "ROS mode: REPLAY (domain 77, localhost only)"
        ;;
    live)
        export ROS_DOMAIN_ID="${KITT_LIVE_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"
        export ROS_STATIC_PEERS="${KITT_IP:-10.42.0.1}"
        unset ROS_AUTOMATIC_DISCOVERY_RANGE
        echo "ROS mode: LIVE (domain $ROS_DOMAIN_ID, car at $ROS_STATIC_PEERS)"
        ;;
    *)
        echo "usage: source scripts/kitt_env.sh replay|live"
        ;;
esac
