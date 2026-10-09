#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP_PATH="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
export ACKERMANN_ROBOT_WS="${ACKERMANN_ROBOT_WS:-${PROJECT_ROOT}}"
RUN_UI_REQUESTED_ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-}"

source_setup() {
  local setup_file="$1"
  # ROS setup scripts read optional environment variables without guarding
  # unset values. Keep nounset enabled everywhere else in this launcher.
  set +u
  source "${setup_file}"
  local source_status=$?
  set -u
  return "${source_status}"
}

if [[ ! -f "${ROS_SETUP_PATH}" ]]; then
  echo "ERROR: ROS setup file not found: ${ROS_SETUP_PATH}" >&2
  echo "Set ROS_SETUP to the installed ROS distribution setup.bash." >&2
  exit 1
fi
source_setup "${ROS_SETUP_PATH}"

if [[ -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
  source_setup "${PROJECT_ROOT}/install/setup.bash"
elif [[ -f "${PROJECT_ROOT}/third_party/RobotPilot/ros2/install/setup.bash" ]]; then
  source_setup "${PROJECT_ROOT}/third_party/RobotPilot/ros2/install/setup.bash"
else
  echo "ERROR: no built ROS workspace found. Build the AckermannRobot workspace first." >&2
  exit 1
fi

# A workspace environment hook may set its own ROS_DOMAIN_ID. Respect an
# explicit value supplied to this launcher so isolated UI runs stay isolated.
if [[ -n "${RUN_UI_REQUESTED_ROS_DOMAIN_ID}" ]]; then
  export ROS_DOMAIN_ID="${RUN_UI_REQUESTED_ROS_DOMAIN_ID}"
fi

if ! ros2 pkg prefix robotpilot_ui_package >/dev/null 2>&1; then
  echo "ERROR: robotpilot_ui_package is not available in the sourced ROS workspace." >&2
  exit 1
fi

exec ros2 launch robotpilot_ui_package new_ui_launch.py
