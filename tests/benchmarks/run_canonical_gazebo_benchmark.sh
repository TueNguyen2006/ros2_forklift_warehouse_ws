#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
HEADLESS="${HEADLESS:-1}"

cleanup() {
  pkill -x gzserver 2>/dev/null || true
  pkill -x gzclient 2>/dev/null || true
  pkill -x rviz2 2>/dev/null || true
}
trap cleanup EXIT INT TERM

cd "${REPO_ROOT}"
source tools/common.sh
source_workspace_environment
rebuild_selected_packages_if_sources_newer warehouse_visual_localization

# The Python runner produces deterministic preflight artifacts. Gazebo runs are
# launched one-at-a-time and require an external deterministic goal publisher.
python3 tests/benchmarks/canonical_benchmark_runner.py
for configuration in baseline curvature_speed_only curvature_speed_plus_ssm curvature_speed_plus_cbf all_navigation_safety; do
  curvature=false; human=false; cbf=false
  [[ "${configuration}" == *curvature* || "${configuration}" == all_navigation_safety ]] && curvature=true
  [[ "${configuration}" == *ssm* || "${configuration}" == all_navigation_safety ]] && human=true
  [[ "${configuration}" == *cbf* || "${configuration}" == all_navigation_safety ]] && cbf=true
  gui=true; rviz=true; headless=false
  if [[ "${HEADLESS}" == "1" ]]; then gui=false; rviz=false; headless=true; fi
  timeout 90s ros2 launch warehouse_visual_localization nav_with_estimated_pose.launch.py gui:="${gui}" rviz:="${rviz}" headless:="${headless}" canonical_monitor:=true curvature_speed_limit:="${curvature}" human_ssm:="${human}" cbf_filter:="${cbf}" benchmark_logger:=true || true
  cleanup
done
