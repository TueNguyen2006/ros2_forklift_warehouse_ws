#!/usr/bin/env bash
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
source_workspace_environment
set -u

if pgrep -x gzserver >/dev/null 2>&1; then
  echo "Gazebo is already running."
  echo "Close the existing Gazebo/launch first, or run:"
  echo "  pkill -f gzserver; pkill -f gzclient; pkill -f rviz2"
  exit 1
fi

ros2 launch forklift_nav_bringup warehouse_nav_baseline.launch.py \
  use_amcl:=false \
  use_initial_pose_publisher:=false \
  enable_debug_logger:=true \
  "$@"
