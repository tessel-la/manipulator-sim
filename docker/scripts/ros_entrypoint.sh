#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

WORKSPACE="${MOVEIT_WS:-/home/${USERNAME}/moveit_ws}"
DEFAULT_PACKAGES="moveit2_tutorials custom_servo_demo manipulator_action_interfaces manipulator_actions"
PACKAGES="${COLCON_PACKAGES:-${DEFAULT_PACKAGES}}"

build_workspace() {
  if [ "${AUTO_BUILD_WORKSPACE:-0}" != "1" ]; then
    return
  fi

  if [ ! -d "${WORKSPACE}/src" ]; then
    echo "Workspace source directory not found: ${WORKSPACE}/src"
    return
  fi

  echo "Building ROS workspace packages: ${PACKAGES}"
  cd "${WORKSPACE}"
  colcon build \
    --symlink-install \
    --packages-select ${PACKAGES} \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
}

build_workspace

if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source "${WORKSPACE}/install/setup.bash"
fi

exec "$@"
