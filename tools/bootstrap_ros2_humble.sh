#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo "Run this script as a normal user with sudo privileges, not as root."
  exit 1
fi

sudo apt-get update
sudo apt-get install -y curl gnupg2 lsb-release software-properties-common

if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
fi

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  python3-yaml \
  ros-humble-ackermann-steering-controller \
  ros-humble-controller-manager \
  ros-humble-gazebo-plugins \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-joint-state-broadcaster \
  ros-humble-nav2-amcl \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-bringup \
  ros-humble-nav2-collision-monitor \
  ros-humble-nav2-controller \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-map-server \
  ros-humble-nav2-planner \
  ros-humble-nav2-simple-commander \
  ros-humble-nav2-velocity-smoother \
  ros-humble-nav2-waypoint-follower \
  ros-humble-position-controllers \
  ros-humble-robot-state-publisher \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-rviz2 \
  ros-humble-slam-toolbox \
  ros-humble-xacro

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

if ! grep -q "source /opt/ros/humble/setup.bash" "${HOME}/.bashrc"; then
  echo "source /opt/ros/humble/setup.bash" >> "${HOME}/.bashrc"
fi

echo "ROS 2 Humble bootstrap complete."
