# AckermannRobot-3D

小型阿克曼机器人仿真与自主导航系统（0.70m × 0.52m，轴距 0.593m），基于 ROS 2 Humble + Gazebo Classic 11。

## 包结构

```
AckermannRobot-3D/
├── scripts/              # 一键启动脚本
├── src/
│   ├── ackermann_robot/  # 机器人模型、ros2_control、cmd_vel_mux
│   ├── neupan_ros2/      # NeuPAN 神经网络局部规划器（conda 环境）
│   ├── hybrid_astar_planner/  # Hybrid A* 全局路径规划（独立节点，无 Nav2 依赖）
│   ├── robot_slam/       # hdl_localization 3D NDT 定位 + 启动文件
│   ├── LIO-SAM/          # 3D LiDAR-IMU SLAM 建图
│   ├── hdl_localization/ # NDT 3D 点云定位 (ndt_omp vendored)
│   ├── pcd2pgm/          # 3D PCD → 2D PGM 地图转换工具
│   ├── gazebo_worlds/    # Gazebo 仿真世界
│   └── maps/             # 地图存储目录
├── install/              # colcon build 产物
├── build/                # 编译中间文件
└── log/                  # 运行日志
```

## 编译与环境设置

### 依赖安装

```bash
sudo apt update && sudo apt install -y \
  ros-humble-topic-tools \
  ros-humble-robot-localization \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-twist-stamper \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ackermann-steering-controller \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2 \
  libeigen3-dev \
  libomp-dev \
  libpcl-dev \
  libompl-dev \
  libceres-dev
```

**NeuPAN 额外依赖** (conda 环境):

```bash
# 1. 克隆并安装 NeuPAN 核心包（神经网络 MPC 规划器）
git clone https://github.com/hanruihua/NeuPAN ~/NeuPAN
conda create -n neupan python=3.10
conda activate neupan
pip install -e ~/NeuPAN

# 2. 编译 neupan_ros2 ROS 2 桥接包
cd ~/AckermannRobot-3D-ROS2
colcon build --symlink-install --packages-select neupan_ros2
source install/setup.bash
```

### 编译

```bash
cd ~/AckermannRobot-3D
colcon build --symlink-install
source install/setup.bash
```

### 清除与重新编译

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## 重要注意事项

1. **里程计重映射：** ros2_control 原始里程计已重映射 `odom` → `odom_wheel`，EKF 融合后输出 `/odometry/filtered`。
2. **控制接口：** `ackermann_steering_controller` 接收 `TwistStamped`，`cmd_vel_mux` 将 NeuPAN 的 `Twist` 转为 `TwistStamped`。
3. **IMU 话题：** 统一使用 `/imu/data`。
4. **/scan：** 由 `pointcloud_to_laserscan` 将 3D LiDAR (`/points_raw`) 转成 2D LaserScan，供 NeuPAN 避障。

---

## 运行与演示

### 模型预览

```bash
ros2 launch ackermann_robot review.launch.py        # RViz 预览
ros2 launch ackermann_robot gazebo.launch.py        # Gazebo 仿真
```

### 键盘控制

```bash
ros2 launch ackermann_robot keyboard_control.launch.py
# 或用 ros2 run
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/neupan_cmd_vel
```

---

## 3D SLAM 建图（LIO-SAM）

```bash
# 1. 启动 Gazebo
ros2 launch ackermann_robot gazebo.launch.py

# 2. 启动 LIO-SAM 建图
ros2 launch lio_sam run.launch.py

# 3. 键盘控制小车移动建图
ros2 launch ackermann_robot keyboard_control.launch.py

# 4. 保存地图：
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: /AckermannRobot-3D/src/maps/}"
# → GlobalMap.pcd

# 5. 转为 2D PGM (给 Hybrid A* 用)：
cd ~/AckermannRobot-3D
./src/pcd2pgm/build/pcd2gridmap src/maps/GlobalMap.pcd -o src/maps/map
# → src/maps/map.pgm + src/maps/map.yaml
```

---

## 导航

使用 **hdl_localization** (3D NDT) 定位 + **Hybrid A\*** 全局规划 + **NeuPAN** 局部规划。

### hdl + NeuPAN

```bash
# 终端 1：Gazebo + hdl_localization + Hybrid A*
bash scripts/nav_hdl_neupan.sh

# 终端 2：NeuPAN
bash scripts/run_neupan.sh
```

### RViz 操作

1. 用 **"2D Pose Estimate"** 设置初始位姿（hdl_localization 需要）
2. 等待 hdl_localization 收敛（NDT 扫描匹配 score 应下降）
3. 用 **"2D Goal Pose"** 设置导航目标 → Hybrid A* 规划全局路径 → NeuPAN 执行

---

## 架构

### 导航数据流

```
PGM map → hybrid_astar_planner → /plan (Path) → NeuPAN
                               → /map (OccupancyGrid) → RViz

GlobalMap.pcd → hdl_localization → map→odom TF
                                  → /odom (Odometry)

/odom_wheel + /imu/data → EKF → odom→base_link TF
                               → /odometry/filtered

/points_raw → pointcloud_to_laserscan → /scan → NeuPAN → /neupan_cmd_vel (Twist)
                                                          ↓
                                                  cmd_vel_mux → TwistStamped
                                                          ↓
                                          /ackermann_steering_controller/reference
```

### TF 树

```
map ←(hdl NDT)← odom ←(EKF)← base_link ←(URDF)← laser_link
```

| TF | 发布者 | 来源 |
|----|--------|------|
| `map→odom` | hdl_localization | NDT 3D 点云匹配 |
| `odom→base_link` | EKF (`robot_localization`) | `/odom_wheel` + `/imu/data` 融合 |
| `base_link→laser_link` | `robot_state_publisher` | URDF 静态变换 |

### 关键参数

| 参数 | 值 |
|------|-----|
| 车体尺寸 | 0.70m × 0.52m |
| 轴距 | 0.593m |
| 最小转弯半径 | 1.05m |
| 最大转向角 | 0.52 rad (~30°) |
| 最大线速度 | 1.5 m/s 前进, -0.5 m/s 后退 |
| LiDAR | 16线, 360°水平, ±15°垂直, 20Hz |
| 全局规划器 | Hybrid A* (Reeds-Shepp, 独立节点) |
| 局部规划器 | NeuPAN (神经网络 MPC) |
| 定位 | hdl_localization (3D NDT + UKF) |
| ndt_resolution | 1.0m |
| downsample_resolution | 0.4m |

### 定位方式对比

| | hdl_localization | AMCL (已废弃) |
|---|---|---|
| 算法 | 3D NDT 扫描匹配 + UKF | 2D 粒子滤波 + 似然场 |
| 输入 | `/points_raw` (3D PointCloud2) | `/scan` (2D LaserScan) |
| 初始位姿 | RViz 手动设置 | 必须手动 "2D Pose Estimate" |
| 输出 | `map→odom` TF | `map→odom` TF |
| 鲁棒性 | ★★★★★ | ★★ |
