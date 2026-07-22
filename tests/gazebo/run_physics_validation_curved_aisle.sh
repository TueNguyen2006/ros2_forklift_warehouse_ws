#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cleanup() { pkill -x gzserver 2>/dev/null || true; pkill -x gzclient 2>/dev/null || true; pkill -x rviz2 2>/dev/null || true; }
trap cleanup EXIT INT TERM
cd "${REPO_ROOT}"
source tools/common.sh
source_workspace_environment
ros2 launch warehouse_visual_localization physics_validation.launch.py headless:="${HEADLESS:-0}" gui:="${GUI:-true}" rviz:="${RVIZ:-true}"
