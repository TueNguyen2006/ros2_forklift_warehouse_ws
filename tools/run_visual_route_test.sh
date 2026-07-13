#!/usr/bin/env bash
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

VISUAL_DIR="${WORKSPACE_DIR}/src/warehouse_visual_localization"
BRINGUP_DIR="${WORKSPACE_DIR}/src/forklift_nav_bringup"

WORLD_FILE="${WORLD_FILE:-${BRINGUP_DIR}/worlds/small_warehouse_open_top.world}"
SCENARIO_FILE="${SCENARIO_FILE:-${VISUAL_DIR}/config/scenarios/visual_turnaround_sequential.yaml}"
DATABASE_PATH="${DATABASE_PATH:-${HOME}/ros2_forklift_warehouse_artifacts/results/test_mapping.db}"
RESULT_FILE="${RESULT_FILE:-/tmp/visual_route_test_result.json}"
LOG_FILE="${LOG_FILE:-/tmp/visual_route_test.log}"
WAIT_BEFORE_RUN_SEC="${WAIT_BEFORE_RUN_SEC:-95}"
GOAL_TIMEOUT_SEC="${GOAL_TIMEOUT_SEC:-240.0}"
TF_WAIT_SEC="${TF_WAIT_SEC:-90.0}"
LOCALIZATION="${LOCALIZATION:-true}"
POSE_SOURCE="${POSE_SOURCE:-rgbd_localization_fused}"
NAV_PARAMS_FILE="${NAV_PARAMS_FILE:-${VISUAL_DIR}/config/nav2_params_visual.yaml}"
USE_WHEEL_ODOM_FUSION="${USE_WHEEL_ODOM_FUSION:-true}"
ENABLE_STARTUP_MOTION_PROBE="${ENABLE_STARTUP_MOTION_PROBE:-true}"
ENABLE_NAV_DEBUG_LOGGER="${ENABLE_NAV_DEBUG_LOGGER:-true}"
ENABLE_TF_DEBUG="${ENABLE_TF_DEBUG:-true}"

source_workspace_environment
source "${WORKSPACE_DIR}/tools/source_visual_localization_env.sh"
set -u

readarray -t SPAWN_VALUES < <(
  python3 - "${SCENARIO_FILE}" <<'PY'
from pathlib import Path
import sys
import yaml

scenario = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8")) or {}
initial_pose = scenario.get("initial_pose", {})
print(initial_pose.get("x", -2.3))
print(initial_pose.get("y", -2.3))
print(initial_pose.get("yaw", 1.57))
PY
)

SPAWN_X="${SPAWN_VALUES[0]}"
SPAWN_Y="${SPAWN_VALUES[1]}"
SPAWN_YAW="${SPAWN_VALUES[2]}"
SPAWN_Z="${SPAWN_Z:-0.05}"

rm -f "${RESULT_FILE}" "${LOG_FILE}"
pkill -x gzserver || true
pkill -x gzclient || true
pkill -x rviz2 || true
pkill -x ekf_node || true
pkill -x rgbd_odometry || true
pkill -x rtabmap || true

ros2 launch warehouse_visual_localization nav_with_estimated_pose.launch.py \
  gui:=false \
  rviz:=false \
  headless:=true \
  world:="${WORLD_FILE}" \
  params_file:="${NAV_PARAMS_FILE}" \
  database_path:="${DATABASE_PATH}" \
  localization:="${LOCALIZATION}" \
  pose_source:="${POSE_SOURCE}" \
  use_wheel_odom_fusion:="${USE_WHEEL_ODOM_FUSION}" \
  enable_startup_motion_probe:="${ENABLE_STARTUP_MOTION_PROBE}" \
  enable_nav_debug_logger:="${ENABLE_NAV_DEBUG_LOGGER}" \
  enable_tf_debug:="${ENABLE_TF_DEBUG}" \
  enable_evaluator:=false \
  enable_gazebo_goal_bridge:=false \
  spawn_x:="${SPAWN_X}" \
  spawn_y:="${SPAWN_Y}" \
  spawn_z:="${SPAWN_Z}" \
  spawn_yaw:="${SPAWN_YAW}" \
  >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill "${LAUNCH_PID}" 2>/dev/null || true
  sleep 2
  pkill -x gzserver || true
  pkill -x gzclient || true
  pkill -x rviz2 || true
  pkill -x rtabmap || true
  pkill -x rgbd_odometry || true
  pkill -x ekf_node || true
  pkill -x planner_server || true
  pkill -x controller_server || true
  pkill -x bt_navigator || true
  pkill -x behavior_server || true
  pkill -x map_server || true
  pkill -x collision_monitor || true
  pkill -x velocity_smoother || true
}
trap cleanup EXIT

sleep "${WAIT_BEFORE_RUN_SEC}"

if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "Launch exited before route test started; tail of ${LOG_FILE}:"
  tail -n 120 "${LOG_FILE}" || true
  exit 1
fi

if ! ros2 run warehouse_visual_localization visual_route_runner.py --ros-args \
  -p scenario_file:="${SCENARIO_FILE}" \
  -p result_file:="${RESULT_FILE}" \
  -p goal_timeout_sec:="${GOAL_TIMEOUT_SEC}" \
  -p tf_wait_sec:="${TF_WAIT_SEC}"; then
  echo "visual_route_runner failed; tail of ${LOG_FILE}:"
  tail -n 120 "${LOG_FILE}" || true
  exit 1
fi

cat "${RESULT_FILE}"
