#!/usr/bin/env bash
# Start the isolated Smac DUBIN + MPPI Ackermann Nav2 stack.
#
# Usage:
#   bash scripts/run_ackermann_nav.sh maps/mini

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAPS_DIR="$PROJECT_DIR/maps"

if [[ $# -ne 1 ]]; then
  echo "Usage: bash scripts/run_ackermann_nav.sh <maps/map_name>" >&2
  echo "Example: bash scripts/run_ackermann_nav.sh maps/mini" >&2
  exit 1
fi

if [[ "$1" = /* ]]; then
  echo "Map directory must be relative to the project, for example: maps/mini" >&2
  exit 1
fi

MAP_DIR="$(realpath -m "$PROJECT_DIR/$1")"
if [[ "$MAP_DIR" != "$MAPS_DIR"/* ]]; then
  echo "Map directory must be under maps/, for example: maps/mini" >&2
  exit 1
fi

for required_file in GlobalMap.pcd map.pgm map.yaml; do
  if [[ ! -f "$MAP_DIR/$required_file" ]]; then
    echo "Missing required map file: $MAP_DIR/$required_file" >&2
    exit 1
  fi
done

if [[ ! -f "$PROJECT_DIR/install/setup.bash" ]]; then
  echo "Workspace is not built: $PROJECT_DIR/install/setup.bash is missing" >&2
  echo "Build ackermann_nav first with colcon." >&2
  exit 1
fi

# ROS 2 Humble's generated setup scripts reference optional environment
# variables directly and are not nounset-safe.  Relax only nounset while
# sourcing them, then restore strict variable checking for the launcher.
set +u
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/install/setup.bash"
set -u

for node_name in \
  /cmd_vel_mux \
  /ackermann_smac_bridge \
  /neupan_node \
  /neupan_ackermann_adapter \
  /controller_server \
  /planner_server \
  /bt_navigator; do
  if ros2 node list 2>/dev/null | grep -Fxq "$node_name"; then
    echo "A conflicting navigation node is already running: $node_name" >&2
    echo "Stop the other navigation stack before starting Ackermann Nav2." >&2
    exit 2
  fi
done

echo ""
echo "=============================================="
echo "  Ackermann Nav2"
echo "  Smac DUBIN + MPPI Ackermann + /scan obstacle costmaps"
echo "  Forward-only; NeuPAN is NOT started"
echo "  Map: $MAP_DIR/map.yaml"
echo "=============================================="

exec ros2 launch ackermann_nav navigation_sim.launch.py \
  map:="$MAP_DIR/map.yaml" \
  globalmap_pcd:="$MAP_DIR/GlobalMap.pcd"
