#!/usr/bin/env bash
# Start the isolated Nav2 navigation system only.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"

MAP_FILE="${1:-${WORKSPACE_DIR}/maps/mini/map.yaml}"
GLOBALMAP_PCD="${2:-${WORKSPACE_DIR}/maps/mini/GlobalMap.pcd}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "错误：未找到 ROS 2 Humble 环境：${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "错误：工作区尚未构建，请先运行：" >&2
  echo "  cd ${WORKSPACE_DIR} && colcon build --packages-select ackermann_nav2 --symlink-install" >&2
  exit 1
fi

if [[ ! -f "${MAP_FILE}" ]]; then
  echo "错误：未找到二维地图：${MAP_FILE}" >&2
  exit 1
fi

if [[ ! -f "${GLOBALMAP_PCD}" ]]; then
  echo "错误：未找到点云地图：${GLOBALMAP_PCD}" >&2
  exit 1
fi

# ROS 2 Humble setup files probe optional variables that may be unset. Keep
# strict nounset checking for this script, but disable it while sourcing them.
set +u
# shellcheck disable=SC1091
source "${ROS_SETUP}"
# shellcheck disable=SC1091
source "${WORKSPACE_SETUP}"
set -u

echo "启动独立 Nav2 导航体系"
echo "二维地图：${MAP_FILE}"
echo "点云地图：${GLOBALMAP_PCD}"

exec ros2 launch ackermann_nav2 nav2_navigation_sim.launch.py \
  map:="${MAP_FILE}" \
  globalmap_pcd:="${GLOBALMAP_PCD}" \
  use_sim_time:=true \
  nav2_use_rviz:=true
