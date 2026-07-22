#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source install/setup.bash

timeout 60s ros2 launch forklift_simulation simulation.launch.py \
  simulation_mode:=planar \
  headless:=true \
  use_rviz:=false \
  gui:=false || test "$?" -eq 124

timeout 60s ros2 launch forklift_simulation simulation.launch.py \
  simulation_mode:=physics \
  headless:=true \
  use_rviz:=false \
  gui:=false || test "$?" -eq 124

timeout 60s ros2 launch forklift_rl rl_training_simulation.launch.py \
  headless:=true || test "$?" -eq 124
