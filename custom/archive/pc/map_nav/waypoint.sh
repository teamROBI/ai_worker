#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WAYPOINTS_FILE="${WAYPOINTS_FILE:-$SCRIPT_DIR/waypoints.yaml}"

get_pose() {
  ros2 topic echo /amcl_pose --once 2>/dev/null | awk '
    /x:/ && !got_x { x=$2; got_x=1 }
    /y:/ && !got_y { y=$2; got_y=1 }
    /z:/ && !got_z && got_y { z=$2; got_z=1 }
    /w:/ && !got_w { w=$2; got_w=1 }
    got_x && got_y && got_z && got_w {
      printf "%s %s %s %s", x, y, z, w
      exit
    }
  '
}

echo "Waypoint recorder"
echo "  Saving to: $WAYPOINTS_FILE"
echo ""
echo "Commands:"
echo "  [Enter]  - refresh current pose"
echo "  s <name> - save current pose as waypoint"
echo "  l        - list saved waypoints"
echo "  q        - quit"
echo ""

if [[ ! -f "$WAYPOINTS_FILE" ]]; then
  echo "waypoints:" > "$WAYPOINTS_FILE"
fi

while true; do
  echo "Fetching pose..."
  POSE=$(get_pose)
  if [[ -z "$POSE" ]]; then
    echo "  Could not read pose from /amcl_pose"
    echo "  Is navigation running?"
  else
    X=$(echo "$POSE" | cut -d' ' -f1)
    Y=$(echo "$POSE" | cut -d' ' -f2)
    Z=$(echo "$POSE" | cut -d' ' -f3)
    W=$(echo "$POSE" | cut -d' ' -f4)
    echo "  x: $X  y: $Y  orient_z: $Z  orient_w: $W"
  fi

  echo ""
  read -rp "> " CMD ARG

  case "$CMD" in
    q)
      echo "Bye"
      break
      ;;
    s)
      if [[ -z "$POSE" ]]; then
        echo "No pose available to save."
        continue
      fi
      NAME="${ARG:-waypoint_$(date +%s)}"
      cat >> "$WAYPOINTS_FILE" <<EOF
  $NAME:
    x: $X
    y: $Y
    orient_z: $Z
    orient_w: $W
EOF
      echo "Saved '$NAME'"
      ;;
    l)
      echo "--- Saved waypoints ---"
      cat "$WAYPOINTS_FILE"
      echo "---"
      ;;
    "")
      ;;
    *)
      echo "Unknown command: $CMD"
      ;;
  esac
  echo ""
done
