# AckermannRobot-3D-ROS2

一个基于 **ROS 2 Humble + Gazebo Classic 11** 的小型阿克曼机器人 3D 建图、定位与自主导航项目。

与 2D 版本不同，这个仓库以 **3D LiDAR + LIO-SAM + NDT 定位** 为主：建图阶段由 LIO-SAM 生成 PCD 地图；导航阶段使用 hdl_localization 在三维点云地图中定位，同时将 PCD 投影 / 转换为 2D OccupancyGrid，供 Hybrid A* 规划全局路径，再由 NeuPAN 执行局部避障与车辆控制。

## 系统架构

```text
                         ┌────────────────────┐
                         │  Gazebo Ackermann  │
                         └─────────┬──────────┘
                                   │
                 ┌─────────────────┼───────────────────┐
                 ▼                 ▼                   ▼
            /points_raw        /odom_wheel         /imu/data
                 │                 │                   │
                 │                 └────────┬──────────┘
                 │                          ▼
                 │                         EKF
                 │                          │ odom→base_link
                 │                          ▼
                 │                   /odometry/filtered
                 │
        ┌────────┴─────────────┐
        ▼                      ▼
gazebo_lidar_adapter   pointcloud_to_laserscan
        │ /points_lio          │ /scan
        ▼                      ▼
     LIO-SAM                 NeuPAN
        │                      ▲
        │ GlobalMap.pcd        │ /plan
        ▼                      │
  3D Mapping / Save            │
                               │
GlobalMap.pcd ──► hdl_localization ──► map→odom
        │
        └─► PCD → PGM/YAML ──► Hybrid A* ──► /plan
                                                │
                                                ▼
                                             NeuPAN
                                                │
                                                ▼
                                          cmd_vel_mux
                                                │
                                                ▼
                                  Ackermann Steering Controller
```

NeuPAN 算法源码不参与 ROS 工作空间构建，位于 `third_party/NeuPAN/`；`src/neupan_ros2` 是其 ROS 2 集成层。

## 主要能力

- 3D LiDAR + IMU 仿真；
- LIO-SAM 三维 SLAM 建图；
- Gazebo LiDAR 点云 `ring/time` 字段适配；
- PCD 地图保存；
- PCD → 2D PGM / YAML 地图转换；
- hdl_localization / NDT 三维定位；
- wheel odometry + IMU EKF 融合；
- Hybrid A* 全局规划；
- NeuPAN 局部避障；
- 阿克曼运动学控制；
- RViz 初始位姿与目标点交互；
- 一键建图 / 导航脚本。

## 车辆参数

| 参数 | 值 |
|---|---:|
| 运动模型 | Ackermann |
| 车体尺寸 | 0.70 m × 0.52 m |
| 轴距 | 0.593 m |
| 最大转向角 | ±0.52 rad（约 30°） |
| 最小转弯半径 | 1.05 m |
| 最大前进速度 | 1.5 m/s |
| 最大后退速度 | -0.5 m/s |
| LiDAR | 16 线、360°、±15°、20 Hz |
| 全局规划 | Hybrid A* / Reeds-Shepp |
| 局部规划 | NeuPAN |
| 3D 定位 | hdl_localization / NDT |

## 项目结构

```text
AckermannRobot-3D-ROS2/
├── scripts/                      # 一键启动脚本
└── src/
    ├── ackermann_simulation/     # URDF/Xacro、Gazebo、worlds、meshes、传感器仿真
    ├── ackermann_control/        # ros2_control、cmd_vel 与键盘控制
    ├── ackermann_bringup/        # 建图、定位、规划与导航组合 launch
    ├── LIO-SAM/                  # LiDAR-Inertial SLAM
    ├── hdl_localization/         # NDT 点云定位
    ├── hybrid_astar_planner/     # 独立 Hybrid A* 全局规划器
    ├── neupan_ros2/              # NeuPAN ROS 2 封装
    └── maps/                     # 地图资源
```

> 如果仓库使用 Git submodule，请克隆后执行 `git submodule update --init --recursive`，确保算法依赖完整。

## 环境要求

推荐基线：

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- `colcon`
- PCL / Eigen / Ceres / OpenMP
- NeuPAN 对应 Python / conda 环境

安装主要系统依赖：

```bash
sudo apt update
sudo apt install -y \
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

NeuPAN 推荐独立环境：

```bash
conda create -n neupan python=3.10
conda activate neupan
pip install torch numpy
```

其余 NeuPAN 依赖以仓库内对应源码 / requirements 为准。

## 编译

```bash
git clone <repository-url>
cd AckermannRobot-3D-ROS2

git submodule update --init --recursive
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

彻底重新编译：

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

每个新终端都需要 source：

```bash
source /opt/ros/humble/setup.bash
source ~/AckermannRobot-3D-ROS2/install/setup.bash
```

## 最快体验

### RViz 模型预览

```bash
ros2 launch ackermann_simulation display.launch.py
```

### Gazebo 仿真

```bash
ros2 launch ackermann_simulation gazebo.launch.py
```

### 键盘控制

```bash
ros2 launch ackermann_control keyboard_control.launch.py
```

也可以直接发送到 NeuPAN 控制入口：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/neupan_cmd_vel
```

## 3D SLAM 建图

### 1. 启动 Gazebo + LIO-SAM

推荐入口：

```bash
ros2 launch ackermann_bringup mapping.launch.py
```

建图模式会处理 TF 发布关系，避免 EKF 与 LIO-SAM 同时争抢同一条变换。

### 2. 控制机器人覆盖环境

新终端：

```bash
source /opt/ros/humble/setup.bash
source ~/AckermannRobot-3D-ROS2/install/setup.bash
ros2 launch ackermann_control keyboard_control.launch.py
```

建图时建议：

- 低速移动；
- 让 LiDAR 对环境保持足够重叠观测；
- 避免长时间原地高速旋转；
- 尽量覆盖未来导航会经过的区域。

### 3. 保存 PCD 地图

先准备输出目录，例如：

```bash
mkdir -p "$PWD/src/ackermann_simulation/worlds/mini/maps"
```

再调用 LIO-SAM 保存服务，并把 `destination` 换成你机器上的**绝对路径**：

```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
  "{resolution: 0.2, destination: /absolute/path/to/AckermannRobot-3D-ROS2/src/ackermann_simulation/worlds/mini/maps/}"
```

输出通常包括：

```text
GlobalMap.pcd
```

> 不要直接复制 README 中其他机器的 `/home/...` 路径；ROS service 参数应使用你当前机器的真实绝对路径。

## 生成 2D 地图

Hybrid A* 使用 2D Occupancy Map，因此需要把三维 PCD 转为 PGM / YAML。

项目中提供独立 PCL 工具，不参与 `colcon build`。先单独编译：

```bash
cmake -S tools/pcd2pgm -B tools/pcd2pgm/build
cmake --build tools/pcd2pgm/build -j
```

随后调用：

```bash
./tools/pcd2pgm/build/pcd2gridmap \
  src/ackermann_simulation/worlds/mini/maps/GlobalMap.pcd \
  -o src/ackermann_simulation/worlds/mini/maps/map
```

输出：

```text
map.pgm
map.yaml
```

导航脚本默认会使用与当前 world 对应的地图资源。

## 自主导航

导航组合：

```text
GlobalMap.pcd
     │
     ▼
hdl_localization
     │ map→odom
     ▼
Robot Pose

map.pgm / map.yaml
     │
     ▼
Hybrid A*
     │ /plan
     ▼
NeuPAN
     │ /neupan_cmd_vel
     ▼
Ackermann Robot
```

### 终端 1：Gazebo + NDT + Hybrid A*

```bash
bash scripts/nav_hdl_neupan.sh
```

### 终端 2：NeuPAN

```bash
bash scripts/run_neupan.sh
```

## RViz 操作

导航启动后：

1. 使用 **2D Pose Estimate** 给 hdl_localization 提供初始位姿；
2. 等待 NDT 匹配收敛；
3. 使用 **2D Goal Pose** 设置目标；
4. Hybrid A* 生成全局路径；
5. NeuPAN 根据路径与激光数据执行局部避障。

如果初始位姿偏差太大，NDT 可能无法可靠收敛。

## 坐标系与参考点

导航状态参考点统一使用：

```text
rear_axle_link
```

这比以车体几何中心作为阿克曼规划参考更符合车辆运动学。

TF 关系：

```text
map
 └─ odom               ← hdl_localization / NDT
     └─ base_link      ← EKF
         ├─ rear_axle_link
         └─ laser_link
```

| TF | 发布者 | 数据来源 |
|---|---|---|
| `map → odom` | hdl_localization | 3D NDT 匹配 |
| `odom → base_link` | robot_localization EKF | `/odom_wheel` + `/imu/data` |
| `base_link → rear_axle_link` | robot_state_publisher | URDF |
| `base_link → laser_link` | robot_state_publisher | URDF |

## 点云数据链路

Gazebo 原始 3D LiDAR：

```text
/points_raw
```

LIO-SAM 需要 `ring` / `time` 等字段，因此项目加入适配器：

```text
/points_raw
    │
    ▼
gazebo_lidar_adapter
    │
    ▼
/points_lio
    │
    ▼
LIO-SAM
```

同时，为 NeuPAN 生成二维 LaserScan：

```text
/points_raw
    │
    ▼
pointcloud_to_laserscan
    │
    ▼
/scan
    │
    ▼
NeuPAN
```

## 控制链路

NeuPAN 对阿克曼模型输出：

```text
[v, steering_angle]
```

ROS 2 节点会根据轴距换算车体角速度：

```text
ω = v · tan(steering_angle) / wheelbase
```

然后由 `cmd_vel_mux` 转为底盘控制器需要的消息。

## 常见问题

### LIO-SAM 没有点云 / 报字段错误

检查：

```bash
ros2 topic echo /points_raw --once
ros2 topic echo /points_lio --once
```

如果 `/points_raw` 有数据而 `/points_lio` 无数据，优先检查 `gazebo_lidar_adapter`。

### TF 抖动或出现 multiple authority

建图时最常见原因是 EKF 与 LIO-SAM 同时发布重叠 TF。优先使用仓库提供的 `mapping.launch.py`，不要随意把多个定位 / SLAM launch 叠加启动。

### hdl_localization 不收敛

检查：

- `GlobalMap.pcd` 是否与当前 world 对应；
- 初始位姿是否接近真实位姿；
- 点云 frame 是否一致；
- NDT resolution / downsample 参数是否合理；
- 是否有足够结构特征用于匹配。

### Hybrid A* 没有路径

检查：

- `map.pgm` / `map.yaml` 与 PCD 是否来自同一场景；
- 起终点是否在可行驶区域；
- 地图 origin / resolution；
- 车辆最小转弯半径；
- 目标姿态是否可达。

### NeuPAN 不运动

确认：

- `/plan` 有路径；
- `/scan` 有数据；
- odometry 有效；
- NeuPAN conda 环境依赖完整；
- 控制 topic 与 mux 配置一致。

## 调参与深入文档

NeuPAN 的仿真 / 实车参数与故障判断见：

```text
docs/neupan_tuning.md
```

调试 3D 导航时建议按层排查：

```text
1. Gazebo / Controller
2. LiDAR / IMU
3. EKF / TF
4. LIO-SAM or hdl_localization
5. 2D map conversion
6. Hybrid A*
7. NeuPAN
8. Ackermann control output
```

不要一开始就把所有模块一起怀疑。

## 与 2D 版本的区别

同账户下的 `AckermannRobot-2D-ROS2` 使用：

```text
2D LiDAR + Cartographer
```

本仓库使用：

```text
3D LiDAR + LIO-SAM + hdl_localization
```

如果只是验证阿克曼路径规划、2D SLAM 和 NeuPAN，2D 版本更轻；如果重点是三维感知、LiDAR-Inertial SLAM 与 NDT 定位，使用本仓库。

## License

请以仓库中的第三方子模块 / 组件许可证为准。若根目录后续对整体项目增加统一 License，应同时保留各第三方组件的原始许可证与版权声明。
