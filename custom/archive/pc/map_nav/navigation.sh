#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MAP_DIR="$SCRIPT_DIR/maps"
MAP_PATH="${1:-${MAP_PATH:-}}"

if [[ $# -gt 0 ]]; then
  shift
fi

USE_SIM_TIME="${USE_SIM_TIME:-false}"
RVIZ="${RVIZ:-true}"
RVIZ_CONFIG="${RVIZ_CONFIG:-navigation.rviz}"
PARAMS_FILE="${PARAMS_FILE:-$SCRIPT_DIR/navigation_params.yaml}"

echo "Launching navigation"
echo "  use_sim_time=$USE_SIM_TIME"

if [[ -n "$MAP_PATH" ]]; then
  if [[ "$MAP_PATH" = /* ]]; then
    if [[ ! -f "$MAP_PATH" && -f "${MAP_PATH}.yaml" ]]; then
      MAP_PATH="${MAP_PATH}.yaml"
    fi
  else
    if [[ -f "$MAP_DIR/$MAP_PATH" ]]; then
      MAP_PATH="$MAP_DIR/$MAP_PATH"
    elif [[ "$MAP_PATH" != *.yaml && -f "$MAP_DIR/${MAP_PATH}.yaml" ]]; then
      MAP_PATH="$MAP_DIR/${MAP_PATH}.yaml"
    fi
  fi
  echo "  map=$MAP_PATH"
  exec ros2 launch ffw_navigation navigation.launch.py \
    use_slam:=false \
    map:="$MAP_PATH" \
    params_file:="$PARAMS_FILE" \
    use_sim_time:="$USE_SIM_TIME" \
    rviz:="$RVIZ" \
    "$@"
else
  echo "  map=package default"
  exec ros2 launch ffw_navigation navigation.launch.py \
    use_slam:=false \
    params_file:="$PARAMS_FILE" \
    use_sim_time:="$USE_SIM_TIME" \
    rviz:="$RVIZ" \
    "$@"
fi
