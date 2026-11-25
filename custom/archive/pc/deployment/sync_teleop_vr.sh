#!/bin/bash
# Sync the custom VR package into the same layout on the robot.
# Usage: ./sync_teleop_vr.sh [target_repository] [extra rsync options...]
# Preview: ./sync_teleop_vr.sh ai_worker:/home/robotis/ai_worker --dry-run

set -euo pipefail

TARGET="${1:-ai_worker:/home/robotis/ai_worker/}"
if [ $# -gt 0 ]; then shift; fi
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CUSTOM_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
PACKAGE_PATH="custom/archive/robot/teleop/ffw_teleop_vr"
SRC_DIR="$CUSTOM_DIR/robot/teleop/ffw_teleop_vr/"
DEST_DIR="${TARGET%/}/$PACKAGE_PATH/"

rsync -avz \
  --exclude '__pycache__' \
  --exclude '*.pyc' \
  --exclude '.git' \
  --exclude 'cert/key.pem' \
  --mkpath \
  "$@" \
  "$SRC_DIR" "$DEST_DIR"

echo "rsync completed: ffw_teleop_vr → $DEST_DIR"
