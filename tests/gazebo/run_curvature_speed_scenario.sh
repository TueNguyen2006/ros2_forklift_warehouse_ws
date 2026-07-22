#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
HEADLESS="${HEADLESS:-1}"
cleanup() { pkill -x gzserver 2>/dev/null || true; pkill -x gzclient 2>/dev/null || true; pkill -x rviz2 2>/dev/null || true; }
trap cleanup EXIT INT TERM
cd "${REPO_ROOT}"
source tools/common.sh
source_workspace_environment
rebuild_selected_packages_if_sources_newer warehouse_visual_localization
gui=true; rviz=true; headless=false
if [[ "${HEADLESS}" == "1" ]]; then gui=false; rviz=false; headless=true; fi
ros2 launch warehouse_visual_localization nav_with_estimated_pose.launch.py gui:="${gui}" rviz:="${rviz}" headless:="${headless}" canonical_monitor:=true curvature_speed_limit:=true corridor_monitor:=true rollover_monitor:=true
