# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概览

小型阿克曼机器人（0.70m × 0.52m，轴距 0.593m）的自主导航系统，基于 **ROS 2 Humble + Gazebo Classic 11**。仓库同时包含两条相对独立的链路：

- **Gazebo 仿真导航**（main 分支主线）：无 Nav2 依赖，Hybrid A*（全局）+ NeuPAN（局部）+ hdl_localization（3D NDT 定位）。
- **实车部署**（`real-vehicle-deployment` 分支）：RK3588 工控机 + Hesai LiDAR + STM32 运控板，见下文「实车部署」。

## 常用命令

### 编译

```bash
cd ~/AckermannRobot-3D-ROS2
source /opt/ros/humble/setup.bash
colcon build --symlink-install          # 全部
colcon build --symlink-install --packages-select <pkg>   # 单包
source install/setup.bash

# 清除重编
rm -rf build install log && colcon build --symlink-install
```

### 仿真导航

```bash
ros2 launch ackermann_robot gazebo.launch.py   # 单独启动 Gazebo
ros2 launch ackermann_robot review.launch.py   # RViz 模型预览

# 完整导航（两终端）
bash scripts/nav_hdl_neupan.sh    # 终端1：Gazebo + hdl_localization + Hybrid A* + rviz
bash scripts/run_neupan.sh        # 终端2：NeuPAN（需 conda neupan 环境）
```

### 实车（`real-vehicle-deployment` 分支）

```bash
bash scripts/start_vehicle.sh <lidar|bridge|all>
# lidar  = Hesai LiDAR + rviz2；bridge = 仅运动控制桥；all = 全部

# 测试运动控制桥（另开终端发布速度）
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

### SLAM 建图与地图转换

```bash
ros2 launch lio_sam run.launch.py     # LIO-SAM 建图（3D LiDAR-IMU）
# 保存 GlobalMap.pcd 后转 2D PGM（pcd2pgm 是独立 cmake 包，非 ROS 包）：
./src/pcd2pgm/build/pcd2gridmap src/maps/GlobalMap.pcd -o src/maps/map
```

### NeuPAN 依赖

NeuPAN 运行需要独立的 **conda `neupan` 环境**（NeuPAN 核心包来自 `https://github.com/hanruihua/NeuPAN`，`pip install -e` 安装）。`scripts/run_neupan.sh` 里顺序很关键：先 source ROS 2 工作空间，再 `conda activate neupan`，并前置 `$CONDA_PREFIX` 到 `PYTHONPATH`/`LD_LIBRARY_PATH`，否则 `ros2 run` 会用错 Python。该脚本绕过了入口脚本，直接 `python3 -c "from neupan_ros2.neupan_node import main; main()"`（规避 importlib.metadata 问题）。

## 导航架构（仿真）

数据流：

```
PGM map → hybrid_astar_planner → /plan (Path) ─────────→ NeuPAN
                               → /map (OccupancyGrid) → RViz

GlobalMap.pcd → hdl_localization → map→odom TF
/odom_wheel + /imu/data → EKF → odom→base_link TF + /odometry/filtered

/points_raw → pointcloud_to_laserscan → /scan → NeuPAN → /neupan_cmd_vel (Twist)
                                                           ↓
                                                   cmd_vel_mux → TwistStamped
                                                           ↓
                                            /ackermann_steering_controller/reference
```

TF 树：`map ←(hdl NDT)← odom ←(EKF)← base_link ←(URDF)← laser_link`

各包职责：

| 包 | 角色 |
|----|------|
| `ackermann_robot` | 机器人模型（xacro/URDF + ros2_control 阿克曼控制器 + EKF + cmd_vel_mux 脚本） |
| `hybrid_astar_planner` | C++ 全局规划器（Reeds-Shepp），独立节点，加载 PGM 栅格图发布 `/plan`、`/map` |
| `neupan_ros2` | NeuPAN 神经网络 MPC 局部规划器（Python/conda），订阅 `/scan`、`/plan`，发布 `/neupan_cmd_vel` |
| `robot_slam` | 导航 launch 编排（`navigation_hdl.launch.py`） |
| `hdl_localization` | 3D NDT + UKF 定位（内含 vendored ndt_omp），发布 `map→odom` |
| `LIO-SAM` | 3D LiDAR-IMU SLAM 建图 |
| `pcd2pgm` | 3D PCD → 2D PGM 工具（独立 cmake） |
| `gazebo_worlds` | Gazebo 仿真世界与地图（`worlds/mini/maps/`） |

### 关键约定

- **里程计重映射**：ros2_control 原始里程计 `odom` → `/odom_wheel`；EKF（`robot_localization`）融合 `/odom_wheel` + `/imu/data` 输出 `/odometry/filtered`。
- **控制链路**：`ackermann_steering_controller` 接收 `TwistStamped`（`use_stamped_vel: true`），`cmd_vel_mux` 把 NeuPAN 的 `Twist` 包装成 `TwistStamped`。切换来源：`ros2 param set /cmd_vel_mux active_planner <dwb|neupan>`。
- **IMU 话题统一** `/imu/data`；**`/scan`** 由 `pointcloud_to_laserscan` 从 3D LiDAR `/points_raw` 转换而来，供 NeuPAN 避障。
- 仿真全部节点开 `use_sim_time: true`。
- NeuPAN 配置按机器人分目录：`src/neupan_ros2/config/robots/<robot>/` 下 `robot.yaml`（几何/话题/LiDAR）与 `planner.yaml`（MPC 参数），模型权重为 `models/dune_model_5000.pth`。调参方法论见 `docs/neupan_tuning.md`（代价函数按机器人特征尺度 L₀/v₀ 归一化）。

## 实车部署（real-vehicle-deployment 分支）

### 硬件链路

```
RK3588 工控机
  enp5s0 → STM32 运控板 (UDP 192.168.1.50:5000) → CAN → RT49 驱动 / EPS 转向 / SEB 制动
  Hesai LiDAR (192.168.1.201, 2368/9347 端口) 经以太网直连
```

### `motion_control` 包（实车运控桥）

Python 节点，订阅 `/cmd_vel`（`Twist`），按阿克曼模型换算转向角，打包 26 字节 `cmd__` 协议帧，UDP 发送到 STM32。参数在 `src/motion_control/config/bridge_params.yaml`：

- `udp_host: 192.168.1.50`、`udp_port: 5000`、`bind_device: enp5s0`（SO_BINDTODEVICE 强制走指定网卡，多网卡主机必须）
- `wheelbase: 0.97`（实车轴距，与仿真 0.593 不同）、`max_steer_deg: 30.0`、`publish_rate: 20.0`
- **安全机制**：`command_timeout: 0.5s` 无指令自动发零速帧；速度/转角限幅；STM32 侧 500ms 超时安全状态

转向换算：`δ = atan2(ω × wheelbase, |v| + ε)`。帧格式为 26 字节大端（header `cmd__`、V_RAW、EPS_RAW、checksum 为前 22 字节 uint32 和），协议细节见 `docs/普通车运控_以太网UDP通信协议说明_V1.0.md`。

### `lidar_driver` 包（实车激光雷达）

Hesai PandarXT-16 驱动（原 `hesai_ros_driver`）。配置文件 `src/lidar_driver/config/config.yaml` 中 correction/firetimes 为**相对路径**，节点在编译期通过 `PROJECT_PATH` 宏定位包根目录解析（勿改成绝对路径）。话题：`/lidar_points`、`/lidar_packets`、`/lidar_imu`。

### 安全须知

实车测试前悬空车轮或限速，最大速度从 0.1~0.2 m/s 起步。运控板 UDP 网段须与 enp5s0 一致。

## 文档索引

| 文档 | 内容 |
|------|------|
| `README.md` | 完整编译/运行/架构说明（仿真 + 实车） |
| `docs/neupan_tuning.md` | NeuPAN MPC 代价参数调优方法论 |
| `docs/普通车运控_以太网UDP通信协议说明_V1.0.md` | STM32 26 字节 UDP 协议 |
| `src/lidar_driver/README.md` | Hesai LiDAR SDK 使用说明 |
