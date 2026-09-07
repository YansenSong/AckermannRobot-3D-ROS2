#!/usr/bin/env bash
# Start the first workflow step: map the Gazebo mini.world with LIO-SAM.

# ROS 2 setup scripts intentionally read optional, unset environment variables.
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f "$PROJECT_DIR/install/setup.bash" ]]; then
  echo "Workspace is not built. Run: colcon build --symlink-install" >&2
  exit 1
fi

cd "$PROJECT_DIR"
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/install/setup.bash"

echo "Starting mini.world mapping with LIO-SAM..."
echo "Map exports will be written to: $PROJECT_DIR/maps/YYYYMMDD_HHMMSS/"
ros2 launch ackermann_bringup mapping.launch.py &
MAPPING_PID=$!

cleanup() {
  kill "$MAPPING_PID" 2>/dev/null || true
  wait "$MAPPING_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Waiting for Gazebo to initialize..."
sleep 5
echo "Starting keyboard control in a separate terminal..."
ros2 launch ackermann_control keyboard_control.launch.py
