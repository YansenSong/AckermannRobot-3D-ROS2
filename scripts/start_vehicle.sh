#!/usr/bin/env bash
#==========================================
# 实车启动脚本 (ROS 2 Humble)
#
#   lidar   启动 Hesai LiDAR + RViz2
#   imu     仅启动 LPMS-IG1 IMU
#   bridge  仅启动运动控制桥 (motion_control)
#   all     启动 LiDAR + RViz2 + 运动控制桥（不自动启动 IMU）
#   nav     启动 LiDAR + 运动控制桥 + 定位/规划/导航基础设施
#           NeuPAN 需在另一个终端运行 scripts/run_neupan.sh
#==========================================

set -eo pipefail

ROS2_DISTRO="${ROS2_DISTRO:-humble}"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VEHICLE_CONFIG="${VEHICLE_CONFIG:-${PROJECT_DIR}/config/vehicle.yaml}"
LIDAR_CONFIG="${LIDAR_CONFIG:-${PROJECT_DIR}/src/lidar/config/config.yaml}"
IMU_PORT="${IMU_PORT:-/dev/ttyUSB0}"
BRIDGE_CONFIG="${PROJECT_DIR}/src/motion_control/config/bridge_params.yaml"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

usage() {
    cat <<EOF
用法: $0 <lidar|imu|bridge|all|nav> [map]

  lidar   启动 Hesai LiDAR + RViz2
  imu     仅启动 LPMS-IG1 IMU
  bridge  仅启动运动控制桥
  all     启动 LiDAR + RViz2 + 运动控制桥（不自动启动 IMU）
  nav     启动 LiDAR + bridge + 定位/规划/导航基础设施
          map 可传地图目录、map.yaml 或 map.pgm
          地图目录需包含 map.yaml、map.pgm、GlobalMap.pcd
          NeuPAN 需另开终端执行: bash scripts/run_neupan.sh

环境变量:
  VEHICLE_CONFIG 项目级车辆参数，默认: config/vehicle.yaml
  LIDAR_CONFIG   Hesai 配置文件，默认: src/lidar/config/config.yaml
  IMU_PORT       LPMS 串口，默认: /dev/ttyUSB0
  ROS2_DISTRO    ROS 发行版，默认: humble

示例:
  $0 lidar
  $0 imu
  $0 bridge
  $0 all
  $0 nav maps/my_map
  VEHICLE_CONFIG=/path/to/vehicle.yaml $0 bridge
  LIDAR_CONFIG=/path/to/hesai.yaml $0 lidar
  IMU_PORT=/dev/ttyUSB1 $0 imu
EOF
}

MODE="${1:-help}"
ENABLE_LIDAR=false
LIDAR_RVIZ=false
ENABLE_IMU=false
ENABLE_CONTROL=false
ENABLE_NAVIGATION=false
NAVIGATION_RVIZ=false

case "$MODE" in
    lidar)
        ENABLE_LIDAR=true
        LIDAR_RVIZ=true
        ;;
    imu)
        ENABLE_IMU=true
        ;;
    bridge)
        ENABLE_CONTROL=true
        ;;
    all)
        ENABLE_LIDAR=true
        LIDAR_RVIZ=true
        ENABLE_CONTROL=true
        ;;
    nav)
        ENABLE_LIDAR=true
        ENABLE_CONTROL=true
        ENABLE_NAVIGATION=true
        NAVIGATION_RVIZ=true
        ;;
    help|-h|--help)
        usage
        exit 0
        ;;
    *)
        log_error "未知参数: $MODE"
        usage
        exit 1
        ;;
esac

ROS_SETUP="/opt/ros/${ROS2_DISTRO}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
    log_error "未找到 ROS 2 ${ROS2_DISTRO}: ${ROS_SETUP}"
    exit 1
fi

WORKSPACE_SETUP="${PROJECT_DIR}/install/setup.bash"
if [[ ! -f "$WORKSPACE_SETUP" ]]; then
    log_error "未找到工作空间环境: ${WORKSPACE_SETUP}"
    log_error "请先在仓库根目录执行 colcon build --symlink-install"
    exit 1
fi

source "$ROS_SETUP"
source "$WORKSPACE_SETUP"

if $ENABLE_CONTROL || $ENABLE_NAVIGATION; then
    if [[ ! -f "$VEHICLE_CONFIG" ]]; then
        log_error "车辆参数文件不存在: ${VEHICLE_CONFIG}"
        exit 1
    fi
    log_info "车辆参数: ${VEHICLE_CONFIG}"
fi

if $ENABLE_LIDAR; then
    if [[ ! -f "$LIDAR_CONFIG" ]]; then
        log_error "LiDAR 配置文件不存在: ${LIDAR_CONFIG}"
        exit 1
    fi
    log_info "LiDAR 配置: ${LIDAR_CONFIG}"
    if grep -Eq 'device_ip_address:[[:space:]]*""|udp_port:[[:space:]]*0([[:space:]]|$)' "$LIDAR_CONFIG"; then
        log_warn "LiDAR 配置仍包含空 IP 或 udp_port=0；请确认已填写实车参数。"
    fi
fi

if $ENABLE_IMU; then
    log_info "IMU 串口: ${IMU_PORT}"
fi

if $ENABLE_CONTROL; then
    if [[ ! -f "$BRIDGE_CONFIG" ]]; then
        log_error "运动控制配置不存在: ${BRIDGE_CONFIG}"
        exit 1
    fi
    log_info "Motion bridge 配置: ${BRIDGE_CONFIG}"
fi

MAP_YAML=""
MAP_PGM=""
GLOBALMAP_PCD=""

if $ENABLE_NAVIGATION; then
    MAP_ARG="${2:-}"
    if [[ -z "$MAP_ARG" ]]; then
        log_error "nav 模式需要地图目录、map.yaml 或 map.pgm"
        usage
        exit 1
    fi

    if [[ "$MAP_ARG" != /* ]]; then
        MAP_ARG="${PROJECT_DIR}/${MAP_ARG}"
    fi

    if [[ -d "$MAP_ARG" ]]; then
        MAP_DIR="$(realpath -m "$MAP_ARG")"
    elif [[ -f "$MAP_ARG" ]]; then
        MAP_DIR="$(dirname "$(realpath -m "$MAP_ARG")")"
    else
        log_error "地图路径不存在: ${MAP_ARG}"
        exit 1
    fi

    MAP_YAML="${MAP_DIR}/map.yaml"
    MAP_PGM="${MAP_DIR}/map.pgm"
    GLOBALMAP_PCD="${MAP_DIR}/GlobalMap.pcd"

    for required in "$MAP_YAML" "$MAP_PGM" "$GLOBALMAP_PCD"; do
        if [[ ! -f "$required" ]]; then
            log_error "缺少导航地图文件: ${required}"
            exit 1
        fi
    done

    log_info "导航地图目录: ${MAP_DIR}"
fi

echo ""
log_info "=============================="
log_info "  实车启动模式: ${MODE}"
log_info "=============================="

LAUNCH_CMD=(
    ros2 launch ackermann_bringup real_vehicle.launch.py
    "vehicle_config:=${VEHICLE_CONFIG}"
    "enable_lidar:=${ENABLE_LIDAR}"
    "lidar_config:=${LIDAR_CONFIG}"
    "lidar_rviz:=${LIDAR_RVIZ}"
    "enable_imu:=${ENABLE_IMU}"
    "imu_port:=${IMU_PORT}"
    "enable_control:=${ENABLE_CONTROL}"
    "enable_navigation:=${ENABLE_NAVIGATION}"
    "navigation_rviz:=${NAVIGATION_RVIZ}"
)

if $ENABLE_NAVIGATION; then
    LAUNCH_CMD+=(
        "map:=${MAP_YAML}"
        "map_pgm:=${MAP_PGM}"
        "globalmap_pcd:=${GLOBALMAP_PCD}"
    )

    echo ""
    log_info "NeuPAN 不由本脚本自动启动。另开终端执行:"
    log_info "  cd ${PROJECT_DIR}"
    log_info "  bash scripts/run_neupan.sh"
fi

echo ""
log_info "启动 ROS 2 实车 bringup；按 Ctrl+C 停止。"
exec "${LAUNCH_CMD[@]}"
