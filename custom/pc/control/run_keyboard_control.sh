#!/usr/bin/env bash
# Launch keyboard_control_full_robot.py so it can actually see the robot.
#
# Since ai_worker 2.0.0 the robot talks rmw_zenoh_cpp, not FastDDS. The host's
# ROS 2 Humble has no rmw_zenoh, so the script must run inside the jazzy
# `ai_worker` container, as a zenoh CLIENT of the ROUTER RUNNING ON THE ROBOT.
# (Connecting as a zenoh *peer* is not enough: the robot's nodes only listen on
# their own localhost, so a peer has no route to them. A client routes
# everything through the robot's router, which does reach them.)
#
# Works both ways:
#   - from the HOST shell  -> re-runs itself inside the container via docker exec
#   - from INSIDE the container (e.g. after `DAW`) -> runs directly
#
# Usage:  ./run_keyboard_control.sh [script.py] [args...]
#         ./run_keyboard_control.sh --cams [--cam_h 420] [--ref_head <image>]   (the controller with the camera row)
#   ROBOT_IP=<ip>             skip name resolution
#   ROBOT_HOST=<ip|hostname>  default: ffw-SNPR48A1016.local
#   DEFAULT_ROBOT_IP=<ip>     used when ROBOT_HOST does not resolve (inside the container,
#                             which has no mDNS for *.local); default: 192.168.6.2
#   CONTAINER=<name>          default: ai_worker
#   CTR_REPO=<path>          repository mount inside the container
#   LEROBOT_DATA_ROOT=<path> dataset root inside the container (optional)

set -euo pipefail

ROBOT_HOST="${ROBOT_HOST:-ffw-SNPR48A1016.local}"
DEFAULT_ROBOT_IP="${DEFAULT_ROBOT_IP:-192.168.6.2}"   # ffw-SNPR48A1016.local, as the host resolved it on 2026-10-16
CONTAINER="${CONTAINER:-ai_worker}"
DOMAIN="${ROS_DOMAIN_ID:-30}"
# The first argument is the script, unless it is an option (e.g. --cams): then it goes to the default script
if [ $# -gt 0 ] && [[ "$1" != -* ]]; then SCRIPT="$1"; shift; else SCRIPT=keyboard_control_full_robot.py; fi

# --- resolve the robot to an IP (zenoh endpoints need an IP, not an mDNS name) ---
if [ -z "${ROBOT_IP:-}" ]; then
    ROBOT_IP="$(getent hosts "$ROBOT_HOST" | awk '{print $1; exit}' || true)"
fi
if [ -z "$ROBOT_IP" ]; then
    # no mDNS (e.g. inside the container): fall back to the robot's known address;
    # the router check below confirms the robot is really there
    ROBOT_IP="$DEFAULT_ROBOT_IP"
    echo "'$ROBOT_HOST' does not resolve here: trying the robot's default IP $ROBOT_IP" >&2
fi

if ! timeout 5 bash -c "cat < /dev/null > /dev/tcp/$ROBOT_IP/7447" 2>/dev/null; then
    echo "ERROR: no zenoh router at $ROBOT_IP:7447." >&2
    echo "       Is the robot powered on and on this network? A different IP: ROBOT_IP=<ip> $0 $SCRIPT" >&2
    echo "       On the robot (ssh ai_worker, inside its container), run:  zenohd" >&2
    echo "       then the bringup, e.g. ffw_sg2_follower_ai.launch.py" >&2
    exit 1
fi

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID="$DOMAIN"
export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/$ROBOT_IP:7447\"]"

echo "Robot zenoh router: $ROBOT_IP:7447   domain: $DOMAIN"

# --- already inside the container: just run it ---
if [ -f /.dockerenv ]; then
    # ROS setup.bash dereferences unset vars, so -u must come off around it.
    set +u
    source /opt/ros/jazzy/setup.bash
    source /root/ros2_ws/install/setup.bash 2>/dev/null || true
    set -u
    # Bare name -> relative to this script's dir; a path -> used as given.
    case "$SCRIPT" in
        */*) TARGET="$(readlink -f "$SCRIPT")" ;;
        *)   TARGET="$(dirname "$(readlink -f "$0")")/$SCRIPT" ;;
    esac
    [ -f "$TARGET" ] || { echo "ERROR: no such script: $TARGET" >&2; exit 1; }
    cd "$(dirname "$TARGET")"
    exec python3 "$(basename "$TARGET")" "$@"
fi

# --- on the host: hop into the container ---
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "ERROR: container '$CONTAINER' is not running (cd docker && ./container.sh start)." >&2
    exit 1
fi

TTY_FLAGS=()
[ -t 0 ] && TTY_FLAGS=(-it)

# Map the host path of the target script to its path inside the container.
HOST_REPO="$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)"
CTR_REPO="${CTR_REPO:-/root/ros2_ws/src/ai_worker}"
case "$SCRIPT" in
    */*) TARGET="$(readlink -f "$SCRIPT")" ;;
    *)   TARGET="$(dirname "$(readlink -f "$0")")/$SCRIPT" ;;
esac
if [ ! -f "$TARGET" ]; then
    echo "ERROR: no such script: $TARGET" >&2
    exit 1
fi
case "$TARGET" in
    "$HOST_REPO"/*) CTR_TARGET="$CTR_REPO/${TARGET#$HOST_REPO/}" ;;
    *) echo "ERROR: '$TARGET' is outside $HOST_REPO, so it is not mounted in the container." >&2
       exit 1 ;;
esac

exec docker exec "${TTY_FLAGS[@]}" \
    -e DISPLAY="${DISPLAY:-:1}" \
    -e RMW_IMPLEMENTATION -e ROS_DOMAIN_ID -e ZENOH_CONFIG_OVERRIDE \
    -e LEROBOT_DATA_ROOT \
    "$CONTAINER" bash -c '
        source /opt/ros/jazzy/setup.bash
        source /root/ros2_ws/install/setup.bash 2>/dev/null
        cd "$(dirname "$1")" || exit 1
        script_path="$1"
        shift
        exec python3 "$(basename "$script_path")" "$@"
    ' bash "$CTR_TARGET" "$@"
