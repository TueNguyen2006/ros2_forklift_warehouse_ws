#!/usr/bin/env bash
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
source_workspace_environment
set -u

LOG_FILE="${TMPDIR:-/tmp}/forklift_cam_verify.log"

timeout 40s ros2 launch forklift_nav_bringup warehouse_nav_lattice_v2.launch.py \
  gui:=false \
  rviz:=false \
  use_amcl:=false \
  use_initial_pose_publisher:=false \
  >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill "${LAUNCH_PID}" >/dev/null 2>&1 || true
  wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep 18

echo "TOPICS"
ros2 topic list | grep camera || true

echo
echo "RGB INFO"
ros2 topic info /rgb_camera/image_raw || true

echo
echo "DEPTH INFO"
ros2 topic info /depth_camera/image_raw || true

echo
echo "POINTS INFO"
ros2 topic info /depth_camera/points || true

echo
echo "RGB SAMPLE"
timeout 8s ros2 topic echo /rgb_camera/camera_info --once 2>/dev/null | sed -n '1,12p' || true

echo
echo "DEPTH SAMPLE"
timeout 8s ros2 topic echo /depth_camera/camera_info --once 2>/dev/null | sed -n '1,12p' || true
