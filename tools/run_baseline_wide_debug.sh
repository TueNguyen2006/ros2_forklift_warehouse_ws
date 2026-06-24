#!/usr/bin/env bash
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
BRINGUP_DIR="${WORKSPACE_DIR}/src/forklift_nav_bringup"

source_workspace_environment
set -u

ros2 launch forklift_nav_bringup warehouse_nav_baseline.launch.py \
  use_amcl:=false \
  world:="${BRINGUP_DIR}/worlds/wide_open_warehouse.world" \
  map:="${BRINGUP_DIR}/maps/warehouse_map_wide.yaml" \
  keepout_mask:="${BRINGUP_DIR}/maps/warehouse_keepout_mask_wide.yaml" \
  speed_mask:="${BRINGUP_DIR}/maps/warehouse_speed_mask_wide.yaml" \
  params_file:="${BRINGUP_DIR}/config/nav2_params_wide_debug.yaml" \
  collision_monitor_file:="${BRINGUP_DIR}/config/collision_monitor_smoke.yaml" \
  cmd_vel_in_topic:=cmd_vel_smoothed \
  use_initial_pose_publisher:=false \
  spawn_x:=0.0 \
  spawn_y:=-8.0 \
  spawn_z:=0.05 \
  spawn_yaw:=1.57 \
  "$@"
