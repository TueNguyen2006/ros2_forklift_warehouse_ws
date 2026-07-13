#!/usr/bin/env bash
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
source_workspace_environment
rebuild_selected_packages_if_sources_newer \
  forklift_nav_bringup \
  warehouse_visual_localization \
  forklift_description_realistic
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/source_visual_localization_env.sh"
set -u

if pgrep -x gzserver >/dev/null 2>&1; then
  echo "Gazebo is already running."
  echo "Close the existing Gazebo/launch first, or run:"
  echo "  pkill -f gzserver; pkill -f gzclient; pkill -f rviz2"
  exit 1
fi

ros2 launch forklift_nav_bringup warehouse_nav_realistic.launch.py \
  use_amcl:=false \
  "$@"
