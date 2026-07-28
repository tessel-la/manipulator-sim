#!/bin/bash
set -e

RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-}"

export RMW_IMPLEMENTATION
export ROS_AUTOMATIC_DISCOVERY_RANGE
export ROS_STATIC_PEERS

if [ "${RMW_IMPLEMENTATION}" = "rmw_fastrtps_cpp" ]; then
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
fi

if [ "${RMW_IMPLEMENTATION}" = "rmw_cyclonedds_cpp" ] && [ -z "${CYCLONEDDS_URI:-}" ] && [ -f /etc/cyclonedds/config.xml ]; then
  export CYCLONEDDS_URI="file:///etc/cyclonedds/config.xml"
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"

echo "DDS middleware: ${RMW_IMPLEMENTATION}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}; ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-<unset>}; ROS_AUTOMATIC_DISCOVERY_RANGE: ${ROS_AUTOMATIC_DISCOVERY_RANGE}"
if [ -n "${ROS_STATIC_PEERS}" ]; then
  echo "ROS_STATIC_PEERS: ${ROS_STATIC_PEERS}"
fi
if [ "${RMW_IMPLEMENTATION}" = "rmw_cyclonedds_cpp" ]; then
  echo "CYCLONEDDS_URI: ${CYCLONEDDS_URI:-<unset>}"
fi

WORKSPACE="${MOVEIT_WS:-/home/${USERNAME}/moveit_ws}"
DEFAULT_PACKAGES="custom_servo_demo manipulator_action_interfaces manipulator_actions"
PACKAGES="${COLCON_PACKAGES:-${DEFAULT_PACKAGES}}"
BUILD_MODE="${AUTO_BUILD_WORKSPACE:-auto}"
BUILD_STAMP="${WORKSPACE}/install/.manipulator_sim_build_stamp"
COLCON_EXECUTOR="${COLCON_EXECUTOR:-sequential}"

workspace_needs_build() {
  if [ ! -f "${WORKSPACE}/install/setup.bash" ] || [ ! -f "${BUILD_STAMP}" ]; then
    return 0
  fi

  local package
  for package in ${PACKAGES}; do
    if [ -d "${WORKSPACE}/src/${package}" ] && \
      find "${WORKSPACE}/src/${package}" -type f -newer "${BUILD_STAMP}" -print -quit | grep -q .; then
      return 0
    fi
  done

  return 1
}

build_workspace() {
  if [ ! -d "${WORKSPACE}/src" ]; then
    echo "Workspace source directory not found: ${WORKSPACE}/src"
    return
  fi

  case "${BUILD_MODE,,}" in
    0|false|no|off)
      echo "Workspace auto-build disabled."
      return
      ;;
    auto)
      if ! workspace_needs_build; then
        echo "Workspace is up to date; skipping colcon build."
        return
      fi
      ;;
    1|true|yes|on)
      ;;
    *)
      echo "AUTO_BUILD_WORKSPACE must be auto, 1, or 0; got '${BUILD_MODE}'." >&2
      exit 2
      ;;
  esac

  echo "Building ROS workspace packages: ${PACKAGES}"
  cd "${WORKSPACE}"
  colcon build \
    --symlink-install \
    --executor "${COLCON_EXECUTOR}" \
    --packages-select ${PACKAGES} \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
  touch "${BUILD_STAMP}"
}

build_workspace

if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source "${WORKSPACE}/install/setup.bash"
fi

exec "$@"
