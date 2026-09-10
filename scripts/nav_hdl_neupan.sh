#!/bin/bash
# hdl_localization 3D NDT 定位 + Nav2 Smac 全局规划 + NeuPAN 导航
#
# 前置: 需先用 LIO-SAM 建好 GlobalMap.pcd + 转为 map.pgm/map.yaml
#
# 用法:
#   终端 1: bash scripts/nav_hdl_neupan.sh <maps/地图目录>
#   终端 2: bash scripts/run_neupan.sh

set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAPS_DIR="$PROJECT_DIR/maps"

if [[ $# -ne 1 ]]; then
    echo "Usage: bash scripts/nav_hdl_neupan.sh <maps/地图目录>" >&2
    echo "Example: bash scripts/nav_hdl_neupan.sh maps/mini" >&2
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

echo ""
echo "=============================================="
echo "  hdl_localization + Nav2 Smac 全局规划已启动"
echo "  NeuPAN 在另一个终端启动:"
echo "    cd $PROJECT_DIR"
echo "    bash scripts/run_neupan.sh"
echo "=============================================="

exec ros2 launch ackermann_bringup navigation_sim.launch.py \
    map:="$MAP_DIR/map.yaml" \
    map_pgm:="$MAP_DIR/map.pgm" \
    globalmap_pcd:="$MAP_DIR/GlobalMap.pcd"
