#!/usr/bin/env bash
set -Eeuo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-noetic}"
ROS_PREFIX="${ROS_PREFIX:-/opt/ros}"
ROS_SETUP="${ROS_PREFIX}/${ROS_DISTRO_NAME}/setup.bash"
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

APT_PACKAGES=(
  build-essential
  cmake
  git
  python3-catkin-tools
  python3-rosdep
  python3-rospkg
  python3-pyqt5
  ros-noetic-osqp-vendor
  ros-noetic-osqp
  ros-noetic-gazebo-ros
  ros-noetic-gazebo-plugins
  ros-noetic-hector-gazebo-plugins
  ros-noetic-rviz
  ros-noetic-rviz-plugin-tutorials
  ros-noetic-xacro
  ros-noetic-robot-state-publisher
)

log()
{
  printf '\n[install] %s\n' "$*"
}

die()
{
  printf '\n[install:error] %s\n' "$*" >&2
  exit 1
}

need_command()
{
  command -v "$1" >/dev/null 2>&1 || die "Missing required command: $1"
}

if [[ ! -f "${ROS_SETUP}" ]]; then
  die "ROS ${ROS_DISTRO_NAME} was not found at ${ROS_SETUP}. Install ROS Noetic first."
fi

if [[ ! -d "${WS_ROOT}/src" ]]; then
  die "Workspace source directory not found: ${WS_ROOT}/src"
fi

need_command sudo
need_command apt-get

log "Workspace: ${WS_ROOT}"
log "Checking sudo access"
sudo -v

log "Updating apt package index"
sudo apt-get update

log "Installing apt/ROS dependencies"
sudo apt-get install -y "${APT_PACKAGES[@]}"

need_command rosdep
need_command catkin

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  log "Initializing rosdep"
  sudo rosdep init
else
  log "rosdep is already initialized"
fi

log "Updating rosdep database"
rosdep update

log "Loading ROS ${ROS_DISTRO_NAME} environment"
# shellcheck disable=SC1090
source "${ROS_SETUP}"

if [[ -d "${WS_ROOT}/build" || -d "${WS_ROOT}/devel" || -d "${WS_ROOT}/logs" ]]; then
  log "Cleaning stale catkin build artifacts"
  catkin clean --workspace "${WS_ROOT}" -y --force --all-profiles -b -d -L
fi

log "Installing package dependencies with rosdep"
rosdep install --from-paths "${WS_ROOT}/src" --ignore-src --rosdistro "${ROS_DISTRO_NAME}" -y

log "Building catkin workspace"
catkin build --workspace "${WS_ROOT}" --force-cmake

cat <<EOF

[install] Done.

Run this in your current shell:
  source ${WS_ROOT}/devel/setup.bash

Recommended launch command:
  roslaunch ackermann_car_gazebo coverage_navigation_gui.launch

EOF
