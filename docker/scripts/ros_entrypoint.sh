#!/bin/bash
set -e

RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

export RMW_IMPLEMENTATION
export ROS_LOCALHOST_ONLY

if [ "${RMW_IMPLEMENTATION}" = "rmw_fastrtps_cpp" ]; then
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
fi

if [ "${RMW_IMPLEMENTATION}" = "rmw_cyclonedds_cpp" ] && [ -z "${CYCLONEDDS_URI:-}" ] && [ -f /etc/cyclonedds/config.xml ]; then
  export CYCLONEDDS_URI="file:///etc/cyclonedds/config.xml"
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"

echo "DDS middleware: ${RMW_IMPLEMENTATION}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}; ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY}"
if [ "${RMW_IMPLEMENTATION}" = "rmw_cyclonedds_cpp" ]; then
  echo "CYCLONEDDS_URI: ${CYCLONEDDS_URI:-<unset>}"
fi

WORKSPACE="${MOVEIT_WS:-/home/${USERNAME}/moveit_ws}"
DEFAULT_PACKAGES="custom_servo_demo manipulator_action_interfaces manipulator_actions"
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
