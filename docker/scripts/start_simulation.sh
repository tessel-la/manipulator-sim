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
  --no-gazebo-camera       Disable Gazebo wrist cameras
  --restart                Restart the tmux session before launching
  -h, --help               Show this help

Environment variables are still supported:
  ARM_COUNT, ARM_PREFIX, USE_JOY_TELEOP, USE_POSE_STAMPED_CONTROL,
  LAUNCH_ACTION_SERVERS, PREPARE_SERVO, USE_GAZEBO_CAMERA
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
    --no-gazebo-camera)
      USE_GAZEBO_CAMERA=false
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
