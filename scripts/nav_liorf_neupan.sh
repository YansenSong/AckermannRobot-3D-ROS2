#!/bin/bash
# liorf prior-map localization + Nav2 Smac 全局规划 + NeuPAN 导航
#
# 前置: 需先用 LIO-SAM 建好 GlobalMap.pcd + 转为 map.pgm/map.yaml
#
# 用法:
#   终端 1: bash scripts/nav_liorf_neupan.sh <maps/地图目录>
#   终端 2: bash scripts/run_neupan.sh

set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAPS_DIR="$PROJECT_DIR/maps"

if [[ $# -ne 1 ]]; then
    echo "Usage: bash scripts/nav_liorf_neupan.sh <maps/地图目录>" >&2
    echo "Example: bash scripts/nav_liorf_neupan.sh maps/mini" >&2
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

test -f "$MAP_DIR/GlobalMap.pcd"
test -f "$MAP_DIR/map.pgm"
test -f "$MAP_DIR/map.yaml"
source "$PROJECT_DIR/install/setup.bash"

for required_package in robot_bringup nav_status mission_manager area_rules; do
    if ! ros2 pkg prefix "$required_package" >/dev/null 2>&1; then
        echo "缺少 ROS 包 $required_package。请先在项目根目录构建并重新加载环境：" >&2
        echo "  source /opt/ros/humble/setup.bash" >&2
        echo "  colcon build --packages-up-to robot_bringup mission_manager area_rules" >&2
        echo "  source install/setup.bash" >&2
        exit 1
    fi
done

echo ""
echo "=============================================="
echo "  即将启动 liorf 定位 + Nav2 Smac 全局规划"
echo "  NeuPAN 在另一个终端启动:"
echo "    cd $PROJECT_DIR"
echo "    bash scripts/run_neupan.sh"
echo "  网页相机需要 UI 后端与 web_video_server，在第三个终端启动:"
echo "    bash scripts/run_ros2_ui.sh"
echo "=============================================="

exec ros2 launch robot_bringup navigation_sim.launch.py \
    map:="$MAP_DIR/map.yaml" \
    map_pgm:="$MAP_DIR/map.pgm" \
    globalmap_pcd:="$MAP_DIR/GlobalMap.pcd"
