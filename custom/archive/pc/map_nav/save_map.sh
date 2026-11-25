#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MAP_DIR="$SCRIPT_DIR/maps"
DEFAULT_PREFIX="map_$(date +%Y%m%d_%H%M%S)"
MAP_PREFIX_INPUT="${1:-${MAP_PREFIX:-$DEFAULT_PREFIX}}"

if [[ "$MAP_PREFIX_INPUT" = /* ]]; then
  MAP_PREFIX="$MAP_PREFIX_INPUT"
else
  MAP_PREFIX="$MAP_DIR/$MAP_PREFIX_INPUT"
fi

mkdir -p "$(dirname "$MAP_PREFIX")"

echo "Saving map to:"
echo "  ${MAP_PREFIX}.yaml"
echo "  ${MAP_PREFIX}.pgm"

exec ros2 run nav2_map_server map_saver_cli -f "$MAP_PREFIX"
