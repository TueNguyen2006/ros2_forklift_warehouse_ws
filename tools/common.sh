#!/usr/bin/env bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-${HOME}/ros2_forklift_warehouse_artifacts}"
BUILD_BASE="${BUILD_BASE:-${ARTIFACT_ROOT}/build}"
INSTALL_BASE="${INSTALL_BASE:-${ARTIFACT_ROOT}/install}"
LOG_BASE="${LOG_BASE:-${ARTIFACT_ROOT}/log}"

ensure_required_sources() {
  local missing=()

  [[ -d "${WORKSPACE_DIR}/src/forklift_nav_bringup" ]] || missing+=("src/forklift_nav_bringup")
  [[ -d "${WORKSPACE_DIR}/src/third_party/ROS2-Forklift-Simulation/src/forklift_robot" ]] || missing+=("src/third_party/ROS2-Forklift-Simulation")
  [[ -d "${WORKSPACE_DIR}/src/third_party/aws-robomaker-small-warehouse-world" ]] || missing+=("src/third_party/aws-robomaker-small-warehouse-world")

  if [[ ${#missing[@]} -eq 0 ]]; then
    return 0
  fi

  echo "Missing required repository content:" >&2
  printf '  - %s\n' "${missing[@]}" >&2
  echo >&2
  echo "If this is a fresh clone, initialize submodules first:" >&2
  echo "  git submodule update --init --recursive" >&2
  exit 1
}

source_ros_environment() {
  local nounset_was_enabled=0
  case $- in
    *u*) nounset_was_enabled=1; set +u ;;
  esac

  source /opt/ros/humble/setup.bash
  if [[ -f /usr/share/gazebo/setup.sh ]]; then
    source /usr/share/gazebo/setup.sh
  fi

  if [[ ${nounset_was_enabled} -eq 1 ]]; then
    set -u
  fi
}

rebuild_selected_packages_if_sources_newer() {
  local stale_packages=()
  local package_name

  for package_name in "$@"; do
    local source_dir="${WORKSPACE_DIR}/src/${package_name}"
    local install_share_dir="${INSTALL_BASE}/${package_name}/share/${package_name}"

    if [[ ! -d "${source_dir}" ]]; then
      continue
    fi

    if [[ ! -d "${install_share_dir}" ]]; then
      stale_packages+=("${package_name}")
      continue
    fi

    if find "${source_dir}" -type f -newer "${install_share_dir}" -print -quit | grep -q .; then
      stale_packages+=("${package_name}")
    fi
  done

  if [[ ${#stale_packages[@]} -eq 0 ]]; then
    return 0
  fi

  echo "Detected newer source files than the built overlay. Rebuilding packages:" >&2
  printf '  - %s\n' "${stale_packages[@]}" >&2

  mkdir -p "${BUILD_BASE}" "${INSTALL_BASE}" "${LOG_BASE}"
  colcon \
    --log-base "${LOG_BASE}" \
    build \
    --symlink-install \
    --build-base "${BUILD_BASE}" \
    --install-base "${INSTALL_BASE}" \
    --packages-select "${stale_packages[@]}"
}

ensure_workspace_built() {
  if [[ -f "${INSTALL_BASE}/setup.bash" ]]; then
    return 0
  fi

  echo "Workspace install overlay was not found at:" >&2
  echo "  ${INSTALL_BASE}/setup.bash" >&2
  echo >&2
  echo "Running ./tools/build_workspace.sh to create it..." >&2
  "${WORKSPACE_DIR}/tools/build_workspace.sh"

  if [[ ! -f "${INSTALL_BASE}/setup.bash" ]]; then
    echo "Build finished but ${INSTALL_BASE}/setup.bash is still missing." >&2
    exit 1
  fi
}

source_workspace_environment() {
  ensure_required_sources
  source_ros_environment
  ensure_workspace_built
  local nounset_was_enabled=0
  case $- in
    *u*) nounset_was_enabled=1; set +u ;;
  esac
  source "${INSTALL_BASE}/setup.bash"
  if [[ ${nounset_was_enabled} -eq 1 ]]; then
    set -u
  fi
}
