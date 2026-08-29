#!/bin/bash
# hdl_localization 3D NDT 定位 + Hybrid A* 全局规划 + NeuPAN 导航
#
# 前置: 需先用 LIO-SAM 建好 GlobalMap.pcd + 转为 map.pgm/map.yaml
#
# 用法:
#   终端 1: bash scripts/nav_hdl_neupan.sh
#   终端 2: bash scripts/run_neupan.sh

set -e

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source "$PROJECT_DIR/install/setup.bash"

echo "=== Gazebo + 机器人 ==="
ros2 launch ackermann_robot gazebo.launch.py publish_ekf_tf:=true &
sleep 5

echo "=== hdl_localization + Hybrid A* + NeuPAN ==="
ros2 launch robot_slam navigation_hdl.launch.py \
    map:="$PROJECT_DIR/src/gazebo_worlds/worlds/mini/maps/map.yaml" \
    map_pgm:="$PROJECT_DIR/src/gazebo_worlds/worlds/mini/maps/map.pgm" \
    globalmap_pcd:="$PROJECT_DIR/src/gazebo_worlds/worlds/mini/maps/GlobalMap.pcd"

echo ""
echo "=============================================="
echo "  hdl_localization + Hybrid A* 已启动"
echo "  NeuPAN 在另一个终端启动:"
echo "    cd $PROJECT_DIR"
echo "    bash scripts/run_neupan.sh"
echo "=============================================="
