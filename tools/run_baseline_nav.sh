#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-${HOME}/ros2_forklift_warehouse_artifacts}"
INSTALL_BASE="${INSTALL_BASE:-${ARTIFACT_ROOT}/install}"

source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source "${INSTALL_BASE}/setup.bash"
set -u

ros2 launch forklift_nav_bringup warehouse_nav_baseline.launch.py
