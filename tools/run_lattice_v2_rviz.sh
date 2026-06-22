#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_BASE="${INSTALL_BASE:-${WORKSPACE_DIR}/install}"
BRINGUP_DIR="${WORKSPACE_DIR}/src/forklift_nav_bringup"

source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source "${INSTALL_BASE}/setup.bash"
set -u

if pgrep -x gzserver >/dev/null 2>&1; then
  echo "Gazebo is already running."
  echo "Close the existing launch first, or run:"
  echo "  pkill -f gzserver; pkill -f gzclient; pkill -f rviz2"
  exit 1
fi

ros2 launch forklift_nav_bringup warehouse_nav_lattice_v2.launch.py \
  gui:=true \
  rviz:=true \
  use_amcl:=false \
  use_initial_pose_publisher:=false \
  world:="${BRINGUP_DIR}/worlds/wide_open_warehouse.world" \
  map:="${BRINGUP_DIR}/maps/warehouse_map_wide.yaml" \
  spawn_x:=0.0 \
  spawn_y:=-8.0 \
  spawn_yaw:=1.57 \
  "$@"
