#!/usr/bin/env bash
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
BRINGUP_DIR="${WORKSPACE_DIR}/src/forklift_nav_bringup"
LOG_FILE="${LOG_FILE:-/tmp/lattice_v2_smoke_launch.log}"
RESULT_DIR="${RESULT_DIR:-/tmp/lattice_v2_smoke_results}"

source_workspace_environment
set -u

if pgrep -x gzserver >/dev/null 2>&1; then
  echo "Gazebo is already running; stop the existing launch before this test." >&2
  exit 1
fi

rm -rf "${RESULT_DIR}"
mkdir -p "${RESULT_DIR}"
rm -f "${LOG_FILE}"

ros2 launch forklift_nav_bringup warehouse_nav_lattice_v2.launch.py \
  gui:=false \
  rviz:=false \
  enable_debug_logger:=true \
  world:="${BRINGUP_DIR}/worlds/wide_open_warehouse.world" \
  map:="${BRINGUP_DIR}/maps/warehouse_map_wide.yaml" \
  spawn_x:=0.0 \
  spawn_y:=-8.0 \
  spawn_yaw:=1.57 \
  >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  for _ in $(seq 1 10); do
    kill -0 "${LAUNCH_PID}" 2>/dev/null || break
    sleep 1
  done
  kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
  for _ in $(seq 1 5); do
    kill -0 "${LAUNCH_PID}" 2>/dev/null || break
    sleep 1
  done
  kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT

# The action harness below already waits up to 60 seconds for Nav2. Avoid
# polling lifecycle services here: under Gazebo CPU load, spawning many ros2
# CLI daemons can itself delay discovery and make readiness nondeterministic.
sleep 12

run_goal() {
  local name="$1"
  local initial_x="$2"
  local initial_y="$3"
  local initial_yaw="$4"
  local goal_x="$5"
  local goal_y="$6"
  local goal_yaw="$7"
  local result_file="${RESULT_DIR}/${name}.json"

  python3 "${WORKSPACE_DIR}/tools/nav_smoke_test.py" \
    --initial-x "${initial_x}" \
    --initial-y "${initial_y}" \
    --initial-yaw "${initial_yaw}" \
    --goal-x "${goal_x}" \
    --goal-y "${goal_y}" \
    --goal-yaw "${goal_yaw}" \
    --cmd-topic /cmd_vel \
    --timeout-sec 120 \
    | tee "${result_file}"

  grep -q '"result": "SUCCEEDED"' "${result_file}"
}

# Curved forward path, long straight reverse, then a path that must bend around
# the small obstacle at (2.8, -5.0).
run_goal curved_forward 0.0 -8.0 1.57 4.0 -7.0 0.0
run_goal straight_reverse 4.0 -7.0 0.0 0.0 -7.0 0.0
run_goal obstacle_curve 0.0 -7.0 0.0 5.0 -3.0 1.57

echo "Lattice v2 smoke suite passed. Results: ${RESULT_DIR}"
