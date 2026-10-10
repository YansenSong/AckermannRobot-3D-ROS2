#!/usr/bin/env bash
# Reproducible S0 static/unit checks. Live Gazebo scenarios remain manual.
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "BLOCKED: ROS Humble is missing" >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
if [[ -f "$repo_root/install/setup.bash" ]]; then
  source "$repo_root/install/setup.bash"
fi

if [[ ! -x "$repo_root/third_party/RobotPilot/web/node_modules/.bin/vitest" ]]; then
  echo "BLOCKED: run npm ci in third_party/RobotPilot/web first" >&2
  exit 2
fi

check_root="${S0_CHECK_ROOT:-/tmp/alpha-s0-check}"
colcon --log-base "$check_root/log" build \
  --base-paths "$repo_root/src" \
    "$repo_root/third_party/RobotPilot/ros2/src/robotpilot_ui_msgs" \
    "$repo_root/third_party/RobotPilot/ros2/src/robotpilot_ui_package" \
  --packages-up-to mission_manager robotpilot_ui_package \
  --symlink-install --build-base "$check_root/build" --install-base "$check_root/install"
source "$check_root/install/local_setup.bash"

# The navigation launch composition must resolve the newly installed monitor.
# Other bringup dependencies are already supplied by the main workspace.
colcon --log-base "$check_root/bringup-log" build \
  --base-paths "$repo_root/src" \
  --packages-select robot_bringup \
  --symlink-install --build-base "$check_root/build" --install-base "$check_root/install"
source "$check_root/install/local_setup.bash"

python3 -m pytest -q \
  "$repo_root/third_party/RobotPilot/ros2/src/robotpilot_ui_package/test" \
  "$repo_root/src/extension/mission_manager/test" \
  "$repo_root/src/extension/area_rules/test" \
  "$repo_root/src/control/vehicle_control/test/test_cmd_vel_mux_hold.py"
(
  cd "$repo_root/third_party/RobotPilot/web"
  npm test
  npm run build
)
git -C "$repo_root" diff --check
echo "PASS: S0 build, unit tests, frontend tests/build, diff check"
echo "SKIPPED: Gazebo runtime, ROS graph, network outage, physical hardware"
