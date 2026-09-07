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
# Place the whole launch tree in its own process group.  Gazebo, RViz and the
# included launch files otherwise outlive the parent `ros2 launch` process.
setsid ros2 launch ackermann_bringup mapping.launch.py &
MAPPING_PID=$!

cleanup() {
  trap - EXIT INT TERM
  # Terminate the complete mapping launch tree, rather than only its launcher.
  kill -TERM -- "-$MAPPING_PID" 2>/dev/null || true
  wait "$MAPPING_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Waiting for Gazebo to initialize..."
sleep 5
echo "Keyboard control is active in this terminal (arrow keys; Q exits)."
ros2 run ackermann_control arrow_key_control.py
