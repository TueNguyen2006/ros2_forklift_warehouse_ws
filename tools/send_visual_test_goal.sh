#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/source_visual_localization_env.sh"

GOAL_X="${1:--1.0}"
GOAL_Y="${2:--2.3}"
GOAL_YAW="${3:-0.0}"

QZ="$(python3 - <<PY
import math
print(math.sin(float("${GOAL_YAW}") / 2.0))
PY
)"
QW="$(python3 - <<PY
import math
print(math.cos(float("${GOAL_YAW}") / 2.0))
PY
)"

ros2 topic pub --once /gazebo/nav_goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: map},
  pose: {
    position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0},
    orientation: {z: ${QZ}, w: ${QW}}
  }
}"
