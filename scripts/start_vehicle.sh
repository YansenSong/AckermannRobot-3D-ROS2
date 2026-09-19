#!/usr/bin/env bash
#==========================================
# 实车启动脚本 (ROS 2 Humble)
#
#   lidar   启动 Hesai LiDAR + RViz2
#   imu     仅启动 LPMS-IG1 IMU
#   bridge  仅启动 STM32 运动接口后端 (motion_interface)
#   status  仅启动 sta__ 状态接收（不发任何控制帧，用于台架联调）
#   all     启动 LiDAR + RViz2 + STM32 运动接口（不自动启动 IMU）
#   nav     启动 Smac Hybrid-A* + NeuPAN 实车导航基础设施
#           NeuPAN 需在另一个终端运行 scripts/run_neupan.sh
#   nav2    启动独立 ackermann_nav：Smac Hybrid-A* + MPPI + Nav2 BT
#==========================================

set -eo pipefail

ROS2_DISTRO="${ROS2_DISTRO:-humble}"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VEHICLE_CONFIG="${VEHICLE_CONFIG:-${PROJECT_DIR}/config/vehicle.yaml}"
LIDAR_CONFIG="${LIDAR_CONFIG:-${PROJECT_DIR}/src/sensors/lidar/config/config.yaml}"
IMU_INTERFACE="${IMU_INTERFACE:-can0}"
IMU_NODE_ID="${IMU_NODE_ID:-5}"
IMU_PARAMS_FILE="${IMU_PARAMS_FILE:-${PROJECT_DIR}/src/sensors/lpms_ig1_ros2/config/lpms_ig1_calibration.yaml}"
INTERFACE_CONFIG="${PROJECT_DIR}/src/motion_interface/config/bridge_params.yaml"
STATUS_CONFIG="${PROJECT_DIR}/src/motion_interface/config/status_params.yaml"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

usage() {
    cat <<EOF
用法: $0 <lidar|imu|bridge|status|all|nav|nav2> [map]

  lidar   启动 Hesai LiDAR + RViz2
  imu     仅启动 LPMS-IG1 IMU
  bridge  仅启动 motion_interface 的 STM32 后端
  status  仅启动 sta__ 状态接收，不下发任何控制帧（台架联调用）
  all     启动 LiDAR + RViz2 + STM32 运动接口（不自动启动 IMU）
  nav     启动 Smac Hybrid-A* + NeuPAN 导航基础设施
          map 可传地图目录、map.yaml 或 map.pgm
          NeuPAN 需另开终端执行: bash scripts/run_neupan.sh
  nav2    启动 ackermann_nav 完整 Nav2 栈（Smac Hybrid-A* + MPPI + BT）
          map 可传地图目录、map.yaml 或 map.pgm

  nav/nav2 地图目录均需包含 map.yaml、map.pgm、GlobalMap.pcd
  两套导航栈不要同时启动。

环境变量:
  VEHICLE_CONFIG 项目级车辆参数，默认: config/vehicle.yaml
  LIDAR_CONFIG   Hesai 配置文件，默认: src/sensors/lidar/config/config.yaml
  IMU_INTERFACE  LPMS SocketCAN 接口，默认: can0
  IMU_NODE_ID    LPMS CANopen 节点 ID，默认: 5
  IMU_PARAMS_FILE LPMS 软件标定文件，默认使用项目内标定结果
  ROS2_DISTRO    ROS 发行版，默认: humble

示例:
  $0 lidar
  $0 imu
  $0 bridge
  $0 status
  $0 all
  $0 nav maps/my_map
  $0 nav2 maps/my_map
  VEHICLE_CONFIG=/path/to/vehicle.yaml $0 bridge
  LIDAR_CONFIG=/path/to/hesai.yaml $0 lidar
  IMU_INTERFACE=can0 IMU_NODE_ID=5 $0 imu
EOF
}

MODE="${1:-help}"
ENABLE_LIDAR=false
LIDAR_RVIZ=false
ENABLE_IMU=false
ENABLE_CONTROL=false
ENABLE_STATUS=false
ENABLE_NAVIGATION=false
NAVIGATION_RVIZ=false
NAV_STACK=""

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
    status)
        ENABLE_STATUS=true
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
        NAV_STACK="neupan"
        ;;
    nav2)
        ENABLE_LIDAR=true
        ENABLE_CONTROL=true
        ENABLE_NAVIGATION=true
        NAVIGATION_RVIZ=true
        NAV_STACK="nav2"
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
    log_info "IMU SocketCAN: ${IMU_INTERFACE}, node ID: ${IMU_NODE_ID}"
fi

if $ENABLE_CONTROL; then
    if [[ ! -f "$INTERFACE_CONFIG" ]]; then
        log_error "motion_interface 配置不存在: ${INTERFACE_CONFIG}"
        exit 1
    fi
    log_info "motion_interface 配置: ${INTERFACE_CONFIG}"
fi

# The bridge also starts the sta__ receiver (real_vehicle.launch.py leaves
# enable_status_receiver at its default), so this file is needed whenever
# either the bridge or the receiver is launched.
if $ENABLE_CONTROL || $ENABLE_STATUS; then
    if [[ ! -f "$STATUS_CONFIG" ]]; then
        log_error "sta__ 接收配置不存在: ${STATUS_CONFIG}"
        exit 1
    fi
    log_info "sta__ 接收配置: ${STATUS_CONFIG}"
fi

MAP_YAML=""
MAP_PGM=""
GLOBALMAP_PCD=""

if $ENABLE_NAVIGATION; then
    MAP_ARG="${2:-}"
    if [[ -z "$MAP_ARG" ]]; then
        log_error "${MODE} 模式需要地图目录、map.yaml 或 map.pgm"
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
    log_info "导航栈: ${NAV_STACK}"
fi

echo ""
log_info "=============================="
log_info "  实车启动模式: ${MODE}"
log_info "=============================="

if [[ "$MODE" == "nav2" ]]; then
    LAUNCH_CMD=(
        ros2 launch ackermann_nav navigation.launch.py
        "vehicle_config:=${VEHICLE_CONFIG}"
        "map:=${MAP_YAML}"
        "globalmap_pcd:=${GLOBALMAP_PCD}"
        "points_topic:=/lidar_points"
        "start_hardware:=true"
        "lidar_config:=${LIDAR_CONFIG}"
        "enable_imu:=${ENABLE_IMU}"
        "imu_interface:=${IMU_INTERFACE}"
        "imu_node_id:=${IMU_NODE_ID}"
        "imu_params_file:=${IMU_PARAMS_FILE}"
        "use_sim_time:=false"
        "rviz:=${NAVIGATION_RVIZ}"
    )
elif [[ "$MODE" == "status" ]]; then
    # Bench bring-up only: receive and validate sta__ while commanding
    # nothing. Goes straight to motion_interface rather than real_vehicle so
    # that gate and bridge are unambiguously off.
    LAUNCH_CMD=(
        ros2 launch motion_interface motion_interface.launch.py
        "enable_command_gate:=false"
        "enable_stm32_bridge:=false"
        "enable_status_receiver:=true"
        "status_params_file:=${STATUS_CONFIG}"
    )
else
    LAUNCH_CMD=(
        ros2 launch ackermann_bringup real_vehicle.launch.py
        "vehicle_config:=${VEHICLE_CONFIG}"
        "enable_lidar:=${ENABLE_LIDAR}"
        "lidar_config:=${LIDAR_CONFIG}"
        "lidar_rviz:=${LIDAR_RVIZ}"
        "enable_imu:=${ENABLE_IMU}"
        "imu_interface:=${IMU_INTERFACE}"
        "imu_node_id:=${IMU_NODE_ID}"
        "imu_params_file:=${IMU_PARAMS_FILE}"
        "enable_control:=${ENABLE_CONTROL}"
        "enable_navigation:=${ENABLE_NAVIGATION}"
        "navigation_rviz:=${NAVIGATION_RVIZ}"
        "bridge_params_file:=${INTERFACE_CONFIG}"
        "status_params_file:=${STATUS_CONFIG}"
    )

    if $ENABLE_NAVIGATION; then
        LAUNCH_CMD+=(
            "map:=${MAP_YAML}"
            "map_pgm:=${MAP_PGM}"
            "globalmap_pcd:=${GLOBALMAP_PCD}"
        )
    fi
fi

if [[ "$MODE" == "nav" ]]; then
    echo ""
    log_info "NeuPAN 不由本脚本自动启动。另开终端执行:"
    log_info "  cd ${PROJECT_DIR}"
    log_info "  bash scripts/run_neupan.sh"
fi

echo ""
log_info "启动 ROS 2 实车栈；按 Ctrl+C 停止。"
exec "${LAUNCH_CMD[@]}"
