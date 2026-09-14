# AckermannRobot-3D 实车导航

本分支仅用于真实阿克曼车辆部署，运行环境为 ROS 2 Humble。车辆通过 Hesai
PandarXT-16 获取点云，通过以太网 UDP 向 STM32 运控板发送速度与转向指令。

## 目录

```text
src/
├── vehicle_config/       # 实车尺寸、运动学限制、传感器参数的唯一数据源
├── lidar_driver/         # Hesai 激光雷达驱动
├── lpms_ig1/             # LPMS-IG1-RS485 IMU 驱动（串口 / RS485 / CAN）
├── motion_control/       # /cmd_vel → STM32 UDP 运控桥
├── neupan_ros2/          # NeuPAN 局部规划及实车启动编排
├── hybrid_astar_planner/ # Hybrid A* 全局规划
├── hdl_localization/     # 3D NDT 定位
├── LIO-SAM/              # LiDAR-IMU 建图（安装并标定 IMU 后使用）
├── pcd2pgm/              # PCD → PGM 地图转换工具
└── maps/                 # 实车地图
```

## 统一参数

实车尺寸、运动学限制、LiDAR 网络配置、话题、扫描处理参数和安装位姿统一在：

```text
src/vehicle_config/config/real_vehicle.yaml
```

相关启动文件会在运行时把这些参数注入 `lidar_driver`、`motion_control`、
`lpms_ig1`、Hybrid A* 和 NeuPAN。不要再在各功能包中重复定义这些值。IMU 为
**LPMS-IG1-RS485**（已装车并标定陀螺仪零偏），串口、波特率、话题和采样率同样
在 `sensors.imu` 下统一配置。

## 编译

```bash
cd ~/AckermannRobot-3D
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

NeuPAN 核心包需要单独安装在 `neupan` conda 环境中。

`lpms_ig1` 依赖 LPMS OpenSource 库，构建前需先安装其 `.deb`（源码构建见
[src/lpms_ig1/README.md](src/lpms_ig1/README.md)）：

```bash
sudo dpkg -i libLpmsIG1_OpenSource-0.3.3-Linux.deb
```

## 启动

```bash
# 仅雷达和 RViz
bash scripts/start_vehicle.sh lidar

# 仅运控桥
bash scripts/start_vehicle.sh bridge

    # 直行 0.2 m/s（推荐从低速起步）
    ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, 
    angular: {z: 0.0}}"

    # 左转 ~17°（0.3 弧度）
    ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, 
    angular: {z: 0.3}}"

    # 右转
    ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, 
    angular: {z: -0.3}}"

    # 停车（Ctrl+C 停掉 pub 即可，0.5s 无指令自动发零速帧；或显式发零）
    ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, 
    angular: {z: 0.0}}"


# 雷达、运控桥和 RViz
bash scripts/start_vehicle.sh all

# 雷达、点云转扫描、Hybrid A* 和运控桥
bash scripts/start_vehicle.sh nav /绝对路径/map.pgm

# 另一个终端启动 NeuPAN
bash scripts/run_neupan.sh
```

IMU 驱动（LPMS-IG1-RS485）在所有模式下都会随脚本无条件启动，参数来自
`real_vehicle.yaml`，发布 `/imu/data`（约 167 Hz）：

```bash
ros2 topic echo /imu/data --once   # 查看 IMU 姿态与角速度
ros2 topic hz /imu/data            # 验证发布频率
```

完整导航链：

```text
Hesai LiDAR → /lidar_points → pointcloud_to_laserscan → /scan → NeuPAN
PGM 地图 → Hybrid A* → /plan ────────────────────────────────┘
NeuPAN → /cmd_vel → motion_control → UDP → STM32
```

## 安全

- 首次测试前悬空车轮或确保车辆四周无人无障碍物。
- 从 `0.1~0.2 m/s` 开始限速测试。
- 确认 LiDAR、工控机和运控板网段正确后再启动。
- `motion_control` 在指令超时后自动发送停车帧，但不能替代硬件急停。

## 文档

- [实车参数核对表](REAL_VEHICLE_PARAMS.md)
- [IMU 安装位姿集成（静态 TF + LIO-SAM 外参）](docs/imu_lio_sam_integration.md)
- [转向 EPS 字段刻度排障](docs/steering_eps_scale_troubleshooting.md)
- [NeuPAN 调参指南](docs/neupan_tuning.md)
- [运控 UDP 协议](docs/普通车运控_以太网UDP通信协议说明_V1.0.md)
- [Hesai 驱动说明](src/lidar_driver/README.md)
- [LPMS-IG1 IMU 驱动说明](src/lpms_ig1/README.md)
