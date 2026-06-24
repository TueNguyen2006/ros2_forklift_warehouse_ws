#!/usr/bin/env bash
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
BRINGUP_DIR="${WORKSPACE_DIR}/src/forklift_nav_bringup"

source_workspace_environment
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
export LIBGL_ALWAYS_SOFTWARE=0
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
  enable_debug_logger:=false \
  mesa_adapter_name:="${MESA_D3D12_DEFAULT_ADAPTER_NAME}" \
  world:="${BRINGUP_DIR}/worlds/small_warehouse_open_top.world" \
  map:="${BRINGUP_DIR}/maps/warehouse_map.yaml" \
  spawn_x:=-2.3 \
  spawn_y:=-2.3 \
  spawn_yaw:=1.57 \
  "$@"
