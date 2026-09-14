#!/bin/bash
#==========================================
# 实车启动脚本 (ROS 2 Humble)
#
#   lidar  启动 Hesai LiDAR (lidar_driver) + rviz2 可视化
#   bridge 仅启动运动控制桥 (motion_control)
#   all    全部启动 (LiDAR + rviz2 + 运动控制桥)
#
# 参考: Sensors/start_hesai.sh（已适配本工作区包名并精简）
#==========================================

set -e

# ---- 环境配置（路径由脚本位置推导，避免硬编码）----
ROS2_DISTRO="humble"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
VEHICLE_CONFIG="${PROJECT_DIR}/src/vehicle_config/config/real_vehicle.yaml"
BRIDGE_CONFIG="${PROJECT_DIR}/src/motion_control/config/bridge_params.yaml"

# ---- 颜色输出 ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info()  { echo -e "${GREEN}[INFO]${NC}  $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# ---- 解析子命令 ----
MODE="${1:-help}"
case "$MODE" in
    lidar)  WITH_LIDAR=true;  WITH_BRIDGE=false; WITH_NAV=false ;;
    bridge) WITH_LIDAR=false; WITH_BRIDGE=true;  WITH_NAV=false ;;
    nav)    WITH_LIDAR=true;  WITH_BRIDGE=true;  WITH_NAV=true  ;;
    all)    WITH_LIDAR=true;  WITH_BRIDGE=true;  WITH_NAV=false ;;
    help|-h|--help)
        echo "用法: $0 <lidar|bridge|nav|all> [map_pgm]"
        echo ""
        echo "  lidar   启动 Hesai LiDAR (lidar_driver) + rviz2 可视化"
        echo "  bridge  仅启动运动控制桥 (motion_control)"
        echo "  nav     完整导航链 (LiDAR + bridge + pcl→scan + Hybrid A*)"
        echo "          注意: NeuPAN 需在另一个终端手动启动: bash scripts/run_neupan.sh"
        echo "  all     全部启动 (LiDAR + rviz2 + 运动控制桥)"
        echo ""
        echo "示例:"
        echo "  $0 lidar                    # LiDAR + rviz2"
        echo "  $0 bridge                   # 仅运动控制桥"
        echo "  $0 nav /path/to/map.pgm     # 完整导航链"
        echo "  $0 all                      # 全部启动"
        exit 0
        ;;
    *)
        log_error "未知参数: $MODE"
        echo "用法: $0 <lidar|bridge|all>"
        exit 1
        ;;
esac

# ---- 检查 ROS 2 环境 ----
if [ -f "/opt/ros/${ROS2_DISTRO}/setup.bash" ]; then
    log_info "加载 ROS 2 ${ROS2_DISTRO} 环境..."
    source "/opt/ros/${ROS2_DISTRO}/setup.bash"
else
    log_error "未找到 ROS 2 ${ROS2_DISTRO}，请确认安装路径: /opt/ros/${ROS2_DISTRO}/setup.bash"
    exit 1
fi

# ---- 检查工作空间 ----
if [ -f "${PROJECT_DIR}/install/setup.bash" ]; then
    log_info "加载工作空间环境..."
    source "${PROJECT_DIR}/install/setup.bash"
else
    log_error "未找到工作空间 setup.bash，请先执行 colcon build"
    exit 1
fi

# ---- 启动组件 ----
echo ""
log_info "=============================="
log_info "  启动组件"
log_info "=============================="
echo ""

# 收集需要启动的进程 PID，用于 trap 清理
PIDS=()
cleanup() {
    log_warn "正在停止所有组件..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    log_info "所有组件已停止"
}
trap cleanup EXIT INT TERM

# --- LiDAR (lidar_driver + rviz2) ---
if $WITH_LIDAR; then
    if [ -f "${VEHICLE_CONFIG}" ]; then
        log_info "实车与 LiDAR 统一配置: ${VEHICLE_CONFIG}"
    else
        log_warn "实车统一配置不存在: ${VEHICLE_CONFIG}"
    fi

    log_info "启动 Hesai LiDAR + rviz2 ..."
    ros2 launch lidar_driver start.py &
    PIDS+=($!)
fi

# --- Motion Control Bridge ---
if $WITH_BRIDGE; then
    if [ -f "${BRIDGE_CONFIG}" ]; then
        log_info "Bridge 配置文件: ${BRIDGE_CONFIG}"
    else
        log_warn "Bridge 配置文件不存在: ${BRIDGE_CONFIG}"
    fi

    log_info "启动运动控制桥接节点（加载 YAML 参数）..."
    ros2 launch motion_control bridge.launch.py &
    PIDS+=($!)
fi

# --- Navigation Infrastructure (pcl→scan + Hybrid A*) ---
if $WITH_NAV; then
    MAP_PGM="${2:-}"
    log_info "启动导航基础设施..."

    if [ -z "$MAP_PGM" ] || [ ! -f "$MAP_PGM" ]; then
        log_warn "未指定 PGM 地图或文件不存在: ${MAP_PGM:-<未指定>}"
        log_warn "启动导航基础设施不带地图参数（Hybrid A* 不会发布 /plan）..."
        ros2 launch neupan_ros2 navigation.launch.py &
    else
        log_info "使用地图: $MAP_PGM"
        MAP_DIR="$(dirname "$MAP_PGM")"
        MAP_YAML="${MAP_DIR}/map.yaml"
        if [ -f "$MAP_YAML" ]; then
            ros2 launch neupan_ros2 navigation.launch.py \
                map_pgm:="$MAP_PGM" map_yaml:="$MAP_YAML" &
        else
            ros2 launch neupan_ros2 navigation.launch.py \
                map_pgm:="$MAP_PGM" &
        fi
    fi
    PIDS+=($!)

    echo ""
    log_info "=============================================="
    log_info "  导航基础设施已启动"
    log_info "  在另一个终端启动 NeuPAN:"
    log_info "    cd $PROJECT_DIR"
    log_info "    bash scripts/run_neupan.sh"
    log_info "=============================================="
    echo ""
fi

# ---- 等待所有后台进程 ----
if [ ${#PIDS[@]} -gt 0 ]; then
    log_info "所有组件已启动 (PID: ${PIDS[*]})，按 Ctrl+C 停止..."
    wait
fi
