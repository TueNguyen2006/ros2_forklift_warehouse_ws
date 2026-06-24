#!/usr/bin/env bash
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
ensure_required_sources
source_ros_environment
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
