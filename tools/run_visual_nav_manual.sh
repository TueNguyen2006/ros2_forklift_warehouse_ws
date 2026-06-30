#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/source_visual_localization_env.sh"

pkill -x gzserver || true
pkill -x gzclient || true
pkill -x rviz2 || true
sleep 1

ros2 launch warehouse_visual_localization nav_with_estimated_pose.launch.py \
  gui:=true \
  rviz:=true \
  headless:=false \
  localization:=false \
  use_wheel_odom_fusion:=true \
  drive_model:=planar \
  use_stability_guard:=false \
  use_collision_monitor:=false \
  "$@"
