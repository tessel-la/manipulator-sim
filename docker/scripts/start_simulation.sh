#!/bin/bash
set -e

SESSION_NAME="${TMUXINATOR_PROJECT:-manipulator_simulation_setup}"
WORKSPACE="${MOVEIT_WS:-/home/${USERNAME}/moveit_ws}"

cd "${WORKSPACE}"

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo "Tmux session '${SESSION_NAME}' is already running."
else
  echo "Starting tmuxinator project '${SESSION_NAME}'..."
  tmuxinator start "${SESSION_NAME}"
fi

tmux attach-session -t "${SESSION_NAME}"
