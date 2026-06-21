#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-${HOME}/ros2_forklift_warehouse_artifacts}"
INSTALL_BASE="${INSTALL_BASE:-${ARTIFACT_ROOT}/install}"
BRINGUP_DIR="${WORKSPACE_DIR}/src/forklift_nav_bringup"
LOG_FILE="${LOG_FILE:-/tmp/baseline_drivetrain_test.log}"

source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source "${INSTALL_BASE}/setup.bash"
set -u

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" || true
    wait "${LAUNCH_PID}" || true
  fi
}
trap cleanup EXIT

ros2 launch forklift_nav_bringup warehouse_nav_baseline.launch.py \
  world:="${BRINGUP_DIR}/worlds/wide_open_warehouse.world" \
  map:="${BRINGUP_DIR}/maps/warehouse_map_wide.yaml" \
  keepout_mask:="${BRINGUP_DIR}/maps/warehouse_keepout_mask_wide.yaml" \
  speed_mask:="${BRINGUP_DIR}/maps/warehouse_speed_mask_wide.yaml" \
  params_file:="${BRINGUP_DIR}/config/nav2_params_wide_debug.yaml" \
  collision_monitor_file:="${BRINGUP_DIR}/config/collision_monitor_smoke.yaml" \
  cmd_vel_in_topic:=cmd_vel_smoothed \
  cmd_vel_out_topic:=/cmd_vel_nav_out \
  rviz:=false \
  gui:=false \
  use_initial_pose_publisher:=false \
  spawn_x:=0.0 \
  spawn_y:=-8.0 \
  spawn_z:=0.05 \
  spawn_yaw:=1.57 \
  >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 90); do
  if grep -q "Spawn Entity success" "${LOG_FILE}" 2>/dev/null; then
    break
  fi
  sleep 1
done

python3 "${WORKSPACE_DIR}/tools/drivetrain_smoke_test.py"
