#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-${HOME}/ros2_forklift_warehouse_artifacts}"
BUILD_BASE="${BUILD_BASE:-${ARTIFACT_ROOT}/build}"
INSTALL_BASE="${INSTALL_BASE:-${ARTIFACT_ROOT}/install}"
LOG_BASE="${LOG_BASE:-${ARTIFACT_ROOT}/log}"

source /opt/ros/humble/setup.bash
set -u

cd "${WORKSPACE_DIR}"
mkdir -p "${BUILD_BASE}" "${INSTALL_BASE}" "${LOG_BASE}"
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys="ament_python aws_robomaker_small_warehouse_world catkin forklift_gym_env" || true

colcon \
  --log-base "${LOG_BASE}" \
  build \
  --symlink-install \
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  --packages-skip aws_robomaker_small_warehouse_world forklift_gym_env

echo
echo "Build complete. Source the workspace with:"
echo "  source ${INSTALL_BASE}/setup.bash"
