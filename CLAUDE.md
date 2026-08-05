# CLAUDE.md

## 项目定位

`real-vehicle-deployment` 分支仅用于真实阿克曼车辆，不包含 Gazebo 世界、仿真
车模或仿真启动链路。运行环境为 ROS 2 Humble，硬件为 RK3588 工控机、Hesai
PandarXT-16 和 STM32 运控板。

## 参数约定

`src/vehicle_config/config/real_vehicle.yaml` 是实车物理尺寸、运动学限制和传感器
参数的唯一数据源。其他包只保留自身算法或通信配置，并在 launch 时接收统一参数。
新增实车参数时优先扩展 `vehicle_config`，不要在多个功能包中复制常量。

当前车辆尺寸：长 1.34 m、宽 0.91 m、轴距 0.97 m、轮半径 0.205 m。
`base_link` 原点位于后轴中心，采用 ROS 车体坐标约定：X 轴向前、Y 轴向左、Z 轴向上；
因此 `rear_axle_offset_x` 固定为 `0.0`。
LiDAR 型号为 Hesai PandarXT-16。IMU 尚未安装，禁止假定其话题、位姿和噪声参数
已经有效。

## 常用命令

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

bash scripts/start_vehicle.sh lidar
bash scripts/start_vehicle.sh bridge
bash scripts/start_vehicle.sh all
bash scripts/start_vehicle.sh nav /path/to/map.pgm
bash scripts/run_neupan.sh
```

NeuPAN 使用独立的 `neupan` conda 环境；`scripts/run_neupan.sh` 先加载 ROS 工作区，
再激活 conda 环境，以避免 Python 和动态库解析错误。

## 导航链路

```text
Hesai → /lidar_points → pointcloud_to_laserscan → /scan → NeuPAN
PGM → hybrid_astar_planner → /plan ───────────────────────────┘
NeuPAN → /cmd_vel → motion_control → UDP → STM32
```

主要功能包：

| 包 | 职责 |
|---|---|
| `vehicle_config` | 实车公共参数唯一数据源及 launch 参数映射 |
| `lidar_driver` | Hesai 点云驱动 |
| `motion_control` | Twist 到 STM32 UDP 协议桥 |
| `hybrid_astar_planner` | 基于 PGM 的全局路径规划 |
| `neupan_ros2` | 局部规划与实车导航启动编排 |
| `hdl_localization` | 可选的 3D NDT 定位 |
| `LIO-SAM` | 安装并标定 IMU 后使用的建图模块 |
| `pcd2pgm` | PCD 到 PGM 地图转换 |

## 安全约定

实车测试必须从低速开始，优先悬空车轮，并确保硬件急停可用。运控桥的软件超时
停车不能替代硬件安全机制。
