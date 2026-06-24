#!/usr/bin/env bash
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

source_workspace_environment
set -u

ros2 run forklift_safety nav_matrix_runner \
  --ros-args \
  -p scenario_file:="${INSTALL_BASE}/forklift_safety/share/forklift_safety/scenarios/warehouse_route_matrix.yaml"
