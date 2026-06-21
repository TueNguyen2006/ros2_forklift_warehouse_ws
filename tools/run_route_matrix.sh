#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-${HOME}/ros2_forklift_warehouse_artifacts}"
INSTALL_BASE="${INSTALL_BASE:-${ARTIFACT_ROOT}/install}"

source /opt/ros/humble/setup.bash
source "${INSTALL_BASE}/setup.bash"
set -u

ros2 run forklift_safety nav_matrix_runner \
  --ros-args \
  -p scenario_file:="${INSTALL_BASE}/forklift_safety/share/forklift_safety/scenarios/warehouse_route_matrix.yaml"
