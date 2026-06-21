#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-${HOME}/ros2_forklift_warehouse_artifacts}"
INSTALL_BASE="${INSTALL_BASE:-${ARTIFACT_ROOT}/install}"
BRINGUP_DIR="${WORKSPACE_DIR}/src/forklift_nav_bringup"

source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source "${INSTALL_BASE}/setup.bash"
set -u

ros2 launch forklift_nav_bringup warehouse_nav_baseline.launch.py \
  use_amcl:=false \
  params_file:="${BRINGUP_DIR}/config/nav2_params_smoke.yaml" \
  collision_monitor_file:="${BRINGUP_DIR}/config/collision_monitor_smoke.yaml" \
  keepout_mask:="${BRINGUP_DIR}/maps/warehouse_keepout_mask_smoke.yaml" \
  cmd_vel_in_topic:=cmd_vel_smoothed \
  spawn_x:=-2.3 \
  spawn_y:=-2.3 \
  spawn_z:=0.05 \
  spawn_yaw:=1.57 \
  "$@"
