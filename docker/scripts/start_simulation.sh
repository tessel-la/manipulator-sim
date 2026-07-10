#!/bin/bash
set -e

SESSION_NAME="${TMUXINATOR_PROJECT:-manipulator_simulation_setup}"
WORKSPACE="${MOVEIT_WS:-/home/${USERNAME}/moveit_ws}"
RESTART_SESSION=false

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Start or attach to the manipulator simulation tmux session.

Options:
  --arm-count COUNT        Number of arms to launch (default: ${ARM_COUNT:-1})
  --arm-prefix PREFIX      Namespace prefix for arms (default: ${ARM_PREFIX:-arm})
  --scene-config PATH      Pick-place scene YAML for Gazebo cameras
  --no-gazebo-camera       Disable Gazebo wrist cameras
  --scene-camera           Enable the fixed overhead scene camera
  --no-scene-camera        Disable the fixed overhead scene camera (default)
  --restart                Restart the tmux session before launching
  -h, --help               Show this help

Environment variables are still supported:
  ARM_COUNT, ARM_PREFIX, USE_JOY_TELEOP, USE_POSE_STAMPED_CONTROL,
  LAUNCH_ACTION_SERVERS, PREPARE_SERVO, USE_GAZEBO_CAMERA,
  LAUNCH_SCENE_CAMERA, PICK_PLACE_SCENE_CONFIG, BEHAVIOR_TREE_NAME,
  BEHAVIOR_TREE_TIMEOUT, WRIST_CAMERA_OFFSET_ROLL, WRIST_CAMERA_OFFSET_PITCH,
  WRIST_CAMERA_OFFSET_YAW, WRIST_CAMERA_LOOK_AT_FRAME
EOF
}

require_value() {
  local option="$1"
  local value="${2:-}"

  if [[ -z "${value}" || "${value}" == --* ]]; then
    echo "Missing value for ${option}." >&2
    usage >&2
    exit 2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --arm-count)
      require_value "$1" "${2:-}"
      ARM_COUNT="$2"
      shift 2
      ;;
    --arm-count=*)
      ARM_COUNT="${1#*=}"
      require_value "--arm-count" "${ARM_COUNT}"
      shift
      ;;
    --arm-prefix)
      require_value "$1" "${2:-}"
      ARM_PREFIX="$2"
      shift 2
      ;;
    --arm-prefix=*)
      ARM_PREFIX="${1#*=}"
      require_value "--arm-prefix" "${ARM_PREFIX}"
      shift
      ;;
    --scene-config)
      require_value "$1" "${2:-}"
      PICK_PLACE_SCENE_CONFIG="$2"
      shift 2
      ;;
    --scene-config=*)
      PICK_PLACE_SCENE_CONFIG="${1#*=}"
      require_value "--scene-config" "${PICK_PLACE_SCENE_CONFIG}"
      shift
      ;;
    --no-gazebo-camera)
      USE_GAZEBO_CAMERA=false
      shift
      ;;
    --no-scene-camera)
      LAUNCH_SCENE_CAMERA=false
      shift
      ;;
    --scene-camera)
      LAUNCH_SCENE_CAMERA=true
      shift
      ;;
    --restart)
      RESTART_SESSION=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! [[ "${ARM_COUNT:-1}" =~ ^[1-9][0-9]*$ ]]; then
  echo "ARM_COUNT must be a positive integer; got '${ARM_COUNT}'." >&2
  exit 2
fi

export ARM_COUNT="${ARM_COUNT:-1}"
export ARM_PREFIX="${ARM_PREFIX:-arm}"
export USE_JOY_TELEOP="${USE_JOY_TELEOP:-true}"
export USE_POSE_STAMPED_CONTROL="${USE_POSE_STAMPED_CONTROL:-true}"
export LAUNCH_ACTION_SERVERS="${LAUNCH_ACTION_SERVERS:-true}"
export PREPARE_SERVO="${PREPARE_SERVO:-true}"
export USE_GAZEBO_CAMERA="${USE_GAZEBO_CAMERA:-true}"
export LAUNCH_SCENE_CAMERA="${LAUNCH_SCENE_CAMERA:-false}"
export PICK_PLACE_SCENE_CONFIG="${PICK_PLACE_SCENE_CONFIG:-${WORKSPACE}/src/custom_servo_demo/config/pick_place_scene.yaml}"
export WRIST_CAMERA_OFFSET_ROLL="${WRIST_CAMERA_OFFSET_ROLL:-0.0}"
export WRIST_CAMERA_OFFSET_PITCH="${WRIST_CAMERA_OFFSET_PITCH:--1.5708}"
export WRIST_CAMERA_OFFSET_YAW="${WRIST_CAMERA_OFFSET_YAW:-0.0}"
export WRIST_CAMERA_LOOK_AT_FRAME="${WRIST_CAMERA_LOOK_AT_FRAME:-pick_place_table}"
export BEHAVIOR_TREE_NAME="${BEHAVIOR_TREE_NAME:-none}"
export BEHAVIOR_TREE_TIMEOUT="${BEHAVIOR_TREE_TIMEOUT:-60.0}"

case "${USE_GAZEBO_CAMERA,,}" in
  false|0|no|off)
    ;;
  *)
    if [[ "${ARM_COUNT}" -gt 3 ]]; then
      echo "Gazebo wrist cameras support up to 3 arms; use --no-gazebo-camera for higher counts." >&2
      exit 2
    fi
    ;;
esac

cd "${WORKSPACE}"

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  if [[ "${RESTART_SESSION}" == true ]]; then
    echo "Restarting tmux session '${SESSION_NAME}'..."
    tmux kill-session -t "${SESSION_NAME}"
  else
    echo "Tmux session '${SESSION_NAME}' is already running."
    echo "Attach only; restart with --restart to apply a different arm count."
  fi
fi

if ! tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo "Starting tmuxinator project '${SESSION_NAME}'..."
  echo "Launching ${ARM_COUNT} robot(s) with prefix '${ARM_PREFIX}'."
  tmuxinator start "${SESSION_NAME}"
else
  echo "Current requested settings: ARM_COUNT=${ARM_COUNT}, ARM_PREFIX=${ARM_PREFIX}."
fi

tmux attach-session -t "${SESSION_NAME}"
