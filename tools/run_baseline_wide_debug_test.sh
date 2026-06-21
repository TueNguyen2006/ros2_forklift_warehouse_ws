#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-${HOME}/ros2_forklift_warehouse_artifacts}"
INSTALL_BASE="${INSTALL_BASE:-${ARTIFACT_ROOT}/install}"
BRINGUP_DIR="${WORKSPACE_DIR}/src/forklift_nav_bringup"

SPAWN_X="${SPAWN_X:-0.0}"
SPAWN_Y="${SPAWN_Y:--8.0}"
SPAWN_Z="${SPAWN_Z:-0.05}"
SPAWN_YAW="${SPAWN_YAW:-1.57}"
GOAL_X="${GOAL_X:-0.0}"
GOAL_Y="${GOAL_Y:-7.5}"
GOAL_YAW="${GOAL_YAW:-1.57}"
LOG_FILE="${LOG_FILE:-/tmp/baseline_wide_debug_test.log}"
RESULT_FILE="${RESULT_FILE:-/tmp/baseline_wide_debug_result.json}"

source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source "${INSTALL_BASE}/setup.bash"
set -u

rm -f "${LOG_FILE}" "${RESULT_FILE}"

ros2 launch forklift_nav_bringup warehouse_nav_baseline.launch.py \
  gui:=false \
  rviz:=false \
  use_amcl:=false \
  world:="${BRINGUP_DIR}/worlds/wide_open_warehouse.world" \
  map:="${BRINGUP_DIR}/maps/warehouse_map_wide.yaml" \
  keepout_mask:="${BRINGUP_DIR}/maps/warehouse_keepout_mask_wide.yaml" \
  speed_mask:="${BRINGUP_DIR}/maps/warehouse_speed_mask_wide.yaml" \
  params_file:="${BRINGUP_DIR}/config/nav2_params_wide_debug.yaml" \
  collision_monitor_file:="${BRINGUP_DIR}/config/collision_monitor_smoke.yaml" \
  cmd_vel_in_topic:=cmd_vel_smoothed \
  use_initial_pose_publisher:=false \
  spawn_x:="${SPAWN_X}" \
  spawn_y:="${SPAWN_Y}" \
  spawn_z:="${SPAWN_Z}" \
  spawn_yaw:="${SPAWN_YAW}" \
  >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill "${LAUNCH_PID}" 2>/dev/null || true
  sleep 2
  pkill -f warehouse_nav_baseline.launch.py || true
  pkill -f gzserver || true
  pkill -f spawn_entity.py || true
  pkill -f robot_state_publisher || true
  pkill -f controller_manager || true
  pkill -f gazebo_ros2_control || true
  pkill -f nav2_ || true
  pkill -f amcl || true
  pkill -f velocity_smoother || true
  pkill -f stability_guard || true
  pkill -f odom_to_imu || true
  pkill -f initial_pose_publisher || true
}
trap cleanup EXIT

sleep 8
python3 "${WORKSPACE_DIR}/tools/nav_smoke_test.py" \
  --initial-x "${SPAWN_X}" \
  --initial-y "${SPAWN_Y}" \
  --initial-yaw "${SPAWN_YAW}" \
  --goal-x "${GOAL_X}" \
  --goal-y "${GOAL_Y}" \
  --goal-yaw "${GOAL_YAW}" \
  --cmd-topic /cmd_vel \
  --timeout-sec 120 \
  > "${RESULT_FILE}"

cat "${RESULT_FILE}"
