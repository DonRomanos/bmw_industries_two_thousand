# Source this to switch the current terminal between the real car and bag replay:
#   source scripts/bitt_env.sh replay    # isolated DDS domain 77, shared memory inside this container: sees only play_bag.sh
#   source scripts/bitt_env.sh live      # the car (default; what a new terminal in the dev container uses)
#
# The examples (camera.py, obstacles.py, goal.py, ...) behave the same in both; in replay mode they cannot
# reach the car, so replaying a bag can never move it.

case "${1:-}" in
    replay)
        : "${BITT_LIVE_DOMAIN_ID:=${ROS_DOMAIN_ID:-0}}"
        export BITT_LIVE_DOMAIN_ID
        # Shared memory only: all replay participants live in this container. UDP discovery between processes
        # is unreliable with host networking (e.g. WSL2 mirrored mode), and nothing can leave the container.
        export ROS_DOMAIN_ID=77 FASTDDS_BUILTIN_TRANSPORTS=SHM
        unset ROS_AUTOMATIC_DISCOVERY_RANGE
        unset ROS_STATIC_PEERS
        echo "ROS mode: REPLAY (domain 77, this container only)"
        ;;
    live)
        export ROS_DOMAIN_ID="${BITT_LIVE_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"
        export ROS_STATIC_PEERS="${BITT_IP:-10.42.0.1}"
        unset ROS_AUTOMATIC_DISCOVERY_RANGE FASTDDS_BUILTIN_TRANSPORTS
        echo "ROS mode: LIVE (domain $ROS_DOMAIN_ID, car at $ROS_STATIC_PEERS)"
        ;;
    *)
        echo "usage: source scripts/bitt_env.sh replay|live"
        ;;
esac
