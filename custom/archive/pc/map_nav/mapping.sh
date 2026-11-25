#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

USE_SIM_TIME="${USE_SIM_TIME:-false}"
RVIZ="${RVIZ:-true}"
RVIZ_CONFIG="${RVIZ_CONFIG:-mapping.rviz}"
PARAMS_FILE="${PARAMS_FILE:-$SCRIPT_DIR/navigation_params.yaml}"

echo "Launching SLAM mapping"
echo "  use_sim_time=$USE_SIM_TIME"
echo "  rviz_config=$RVIZ_CONFIG"

exec ros2 launch ffw_navigation navigation.launch.py \
  use_slam:=true \
  params_file:="$PARAMS_FILE" \
  use_sim_time:="$USE_SIM_TIME" \
  rviz:="$RVIZ" \
  rviz_config:="$RVIZ_CONFIG" \
  "$@"
