# 铁马无人驾驶平台 — 项目整体架构

## 一、项目概述

**铁马（Iron Horse）** 是一个基于 ROS1 Noetic 的阿克曼底盘无人驾驶平台，包含完整的**传感器驱动 → SLAM 建图 → 多传感器定位 → 全局/局部路径规划 → CAN 总线底盘控制**全链路。

### 1.1 项目目录结构

```
navigation/
├── control/                          # CAN 总线底盘控制（独立 catkin 工作区）
│   └── src/can_analyzer/             #   9 .cpp + 8 .h + 5 .msg
│
├── LIO_SAM_WS/                       # LIO-SAM SLAM 建图（独立 catkin 工作区）
│   └── src/liosam_liauto-main/       #   9 .cpp（定制化修改版）
│
├── navigation_location/              # 定位 + 导航（最大工作区）
│   └── src/                          #   18 个 ROS 包
│       ├── nav_pkg/                  #     导航框架整合
│       ├── hybrid_astar_planner/     #     全局规划器（move_base 插件）
│       ├── init_location/            #     GNSS+LiDAR 初始位姿估计
│       ├── mutitarget_nav/           #     多点航点导航
│       ├── hdl_localization/         #     实时 NDT 定位（nodelet）
│       ├── hdl_global_localization/  #     全局重定位
│       ├── fast_gicp/                #     GICP 点云配准
│       ├── ndt_omp/                  #     NDT 配准（OMP 加速）
│       ├── liorf_localization/       #     因子图优化定位
│       ├── comnav_gnss/              #     GNSS 驱动
│       ├── velodyne-master/          #     激光雷达驱动
│       ├── pointcloud_to_laserscan/  #     点云转伪激光
│       ├── pcd2pgm/                  #     3D→2D 地图转换
│       └── ...                       #     其余辅助包
│
├── neupan_local_control/             # 神经网络局部规划器（独立工作区）
│   └── src/neupan_ros/               #   19 .py（DUNE 算法）
│
├── sensors/                          # 传感器驱动（独立 catkin 工作区）
│   └── src/
│       ├── comnav_gnss/              #     GNSS 驱动（另一份拷贝）
│       ├── velodyne-master/          #     LiDAR 驱动（另一份拷贝）
│       └── ...
│
└── start_scripts/                    # 一键启动脚本
    ├── navigation.bash               #   定位 + 导航
    ├── control.bash                  #   底盘控制
    └── local_planner.bash            #   局部规划
```

---

## 二、系统四层架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           人机交互层                                         │
│                                                                             │
│  ┌────────────────────┐   ┌──────────────────────┐   ┌─────────────────┐  │
│  │  RViz 可视化        │   │  多点航点交互          │   │  操作服务调用    │  │
│  │  - 定位状态          │   │  - 在 RViz 中点击设点   │   │  /waypoint_    │  │
│  │  - 规划轨迹          │   │  - 球体+编号可视化      │   │  planner/plan  │  │
│  │  - 代价地图          │   │  - 折线连接轨迹        │   │  /waypoint_    │  │
│  │  - 机器人模型        │   │                      │   │  planner/clear │  │
│  └────────────────────┘   └──────────────────────┘   └─────────────────┘  │
├─────────────────────────────────────────────────────────────────────────────┤
│                            规划层                                           │
│                                                                             │
│  ┌─────────────────────────┐   ┌─────────────────────────────────────┐     │
│  │  全局规划                 │   │  局部规划                            │     │
│  │  Hybrid A*               │   │  Neupan DUNE（神经网络 + MPC）      │     │
│  │  - A* 搜索（3D 状态空间）  │   │  - 阿克曼约束轨迹生成                │     │
│  │  - Reeds-Shepp 曲线连接   │   │  - 实时避障                        │     │
│  │  - 支持倒车模式           │   │  - 轨迹跟踪                        │     │
│  │  - nav_core 插件          │   │  - 输入: /scan + /stitched_plan    │     │
│  │  - OMPL 优化             │   │  - 输出: /neupan_cmd_vel           │     │
│  └───────────┬─────────────┘   └───────────────┬─────────────────────┘     │
│              │                                  │                          │
│              └──────────┬───────────────────────┘                          │
│                         ▼                                                   │
│              /neupan_cmd_vel (Twist)                                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                            定位层                                           │
│                                                                             │
│  ┌────────────────────────────────────────────────────────────────────────┐│
│  │ 多传感器融合定位                                                        ││
│  │                                                                        ││
│  │  ① 初始位姿估计（init_location）                                        ││
│  │     GNSS 粗定位 ──→ NDT 精对齐（多角度旋转尝试） ──→ /initialpose       ││
│  │                                                                        ││
│  │  ② 实时定位追踪（hdl_localization）                                     ││
│  │     NDT_OMP 帧图匹配 ──→ IMU 辅助预测 ──→ map→odom TF                  ││
│  │                                                                        ││
│  │  ③ 全局重定位（hdl_global_localization）                                ││
│  │     当定位丢失时，全局搜索找回位置                                        ││
│  │                                                                        ││
│  │  ④ 因子图优化定位（liorf_localization）                                 ││
│  │     GTSAM 因子图，融合 LiDAR + IMU                                      ││
│  └────────────────────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────────────────────┤
│                          传感器层                                           │
│                                                                             │
│  ┌────────────────┐   ┌──────────────────┐   ┌──────────────────────┐     │
│  │  Velodyne VLP-16│   │  GNSS 组合导航    │   │  IMU lpms_ig1       │     │
│  │  激光雷达        │   │  comnav_gnss     │   │  惯性测量单元        │     │
│  │  /velodyne_    │   │  /gps/fix        │   │  /imu/data          │     │
│  │  points        │   │  /gps/heading    │   │                     │     │
│  └────────────────┘   └──────────────────┘   └──────────────────────┘     │
├─────────────────────────────────────────────────────────────────────────────┤
│                           控制层                                            │
│                                                                             │
│  ┌────────────────────────────────────────────────────────────────────────┐│
│  │  CAN 总线通信（can_analyzer）                                           ││
│  │                                                                        ││
│  │  输入: /neupan_cmd_vel (geometry_msgs/Twist)                           ││
│  │        - linear.x  → 目标速度（m/s）                                    ││
│  │        - angular.z → 目标转向角（rad）                                  ││
│  │                                                                        ││
│  │  节点:                                                                  ││
│  │  ├─ can_receive_test: CAN 报文接收、解析、执行                         ││
│  │  │   - 订阅 /control_mode 切换控制模式                                 ││
│  │  │   - 订阅 /AD_101~103 CAN 报文（车辆状态反馈）                        ││
│  │  └─ lonControlTable: 纵横向 PID 控制                                   ││
│  │      - 横向: 目标转向角 → PID → 转向执行器                             ││
│  │      - 纵向: 目标速度 → PID → 油门/刹车                                ││
│  │      - 订阅: /waterplus/navi_result (导航状态)                          ││
│  │      - 发布: /control_mode, /curent_speed, /target_speed              ││
│  │                                                                        ││
│  │  输出: CAN 总线报文 → 底盘 VCU（整车控制器）                             ││
│  └────────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 三、两阶段运行流程

### 阶段 A：离线准备

#### A.1 构建 3D 点云地图（LIO-SAM SLAM）

```
启动流程:
┌───────────────────────────────────────────┐
│ 终端1: IMU 启动                            │
│   cd sensors && source devel/setup.bash    │
│   roslaunch lpms_ig1 lpmsig1_rs485.launch  │
├───────────────────────────────────────────┤
│ 终端2: LiDAR 启动                          │
│   cd sensors && source devel/setup.bash    │
│   roslaunch velodyne_pointcloud            │
│            VLP16_points.launch             │
├───────────────────────────────────────────┤
│ 终端3: LIO-SAM 启动                        │
│   cd LIO_SAM_WS && source devel/setup.bash │
│   roslaunch lio_sam run.launch             │
│   → 遥控车辆扫描环境                       │
│   → Ctrl+C 自动保存地图                    │
└───────────────────────────────────────────┘
         ↓
  GlobalMap.pcd（3D 点云地图）
```

#### A.2 3D 点云地图 → 2D 栅格地图（pcd2pgm）

```
GlobalMap.pcd
      ↓
 ┌─────────────────────────────────────────────────────┐
 │ 1. RANSAC 地面分割                                   │
 │    SACMODEL_PLANE + 距离阈值 0.15m                   │
 │    → 分离 地面点 和 非地面点                          │
 ├─────────────────────────────────────────────────────┤
 │ 2. 高度滤波                                          │
 │    保留距地面 [0.1m, 1.0m] 的点                      │
 │    （过滤掉路面、低矮障碍和树冠/天花板）                │
 ├─────────────────────────────────────────────────────┤
 │ 3. 半径离群点去除                                     │
 │    半径 0.1m 内少于 10 个邻居 → 视为噪声删除           │
 ├─────────────────────────────────────────────────────┤
 │ 4. 投影到 2D 栅格                                    │
 │    丢弃 z 坐标，只取 (x, y) 映射到栅格                │
 │    分辨率 0.05m/格                                   │
 │    栅格落点 → 100（占据）                             │
 │    空栅格 → 0（空闲）                                 │
 └─────────────────────────────────────────────────────┘
      ↓
  map.pgm（2D 灰度图） + map.yaml（地图元数据）
      ↓
  rosrun map_server map_saver（保存）
```

**关键文件路径：**
- 3D 点云: `nav_pkg/config/pcd/GlobalMap.pcd`
- 2D 地图: `nav_pkg/config/map/map.yaml`

#### A.3 局部规划器模型训练（Neupan DUNE）

```
配置车辆参数（轴距 1.27m、车长 1.78m、车宽 0.85m）
      ↓
执行训练: python3 dune_train_limo.py
      ↓
输出模型: IronHorse2.pth → neupan_ros/model/
```

---

### 阶段 B：在线运行

#### 完整启动链路

```
┌──────────────────────────────────────────────────────────────────────────┐
│                         启动顺序（4 个终端）                              │
│                                                                          │
│  ◆ 终端 1 — 传感器层                                                    │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  IMU 启动               Velodyne LiDAR 启动                     │    │
│  │  roslaunch lpms_ig1     roslaunch velodyne_pointcloud          │    │
│  │  lpmsig1_rs485.launch   VLP16_points.launch                   │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│           │                         │                                     │
│           ▼                         ▼                                     │
│      /imu/data              /velodyne_points                              │
│                                                                          │
│  ◆ 终端 2 — 控制层 (control.bash)                                       │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  roslaunch can_analyzer Acmontion_control.launch               │    │
│  │  ├─ can_receive_test (CAN 报文接收解析)                        │    │
│  │  └─ lonControlTable (PID 纵横向控制)                           │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│           │ 等待 /neupan_cmd_vel (Twist)                                 │
│           ▼                                                               │
│      CAN 总线 → 底盘执行器                                                │
│                                                                          │
│  ◆ 终端 3 — 导航层 (navigation.bash)                                    │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  ① roslaunch nav_pkg muti_nav.launch                           │    │
│  │     ├─ move_base + HybridAStar 全局规划器                      │    │
│  │     ├─ map_server（加载 2D 栅格地图）                          │    │
│  │     ├─ hdl_localization（NDT 实时定位）                        │    │
│  │     ├─ hdl_global_localization（全局重定位）                   │    │
│  │     ├─ muti_segment_planner（多点航点拼接）                    │    │
│  │     ├─ static TF（base_link ↔ velodyne）                       │    │
│  │     └─ RViz（导航可视化）                                      │    │
│  │                                                               │    │
│  │  ② rosrun init_location data_pretreat_node                    │    │
│  │     （GNSS 引导 + NDT 多角度搜索 → /initialpose）             │    │
│  │                                                               │    │
│  │  ③ rosrun comnav_gnss gps_get（GNSS 数据驱动）               │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│           │                                                               │
│           ▼                                                               │
│     定位初始化完成 → TF 树全链路打通                                      │
│                                                                          │
│  ◆ 终端 4 — 局部规划层 (local_planner.bash)                             │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  conda activate neupan                                        │    │
│  │  roslaunch neupan_ros neupan_local_planner.launch             │    │
│  │  └─ neupan_node.py (DUNE 局部规划器)                          │    │
│  │     - 订阅: /scan, /waypoint_planner/stitched_plan            │    │
│  │     - 发布: /neupan_cmd_vel → 控制层                          │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                          │
│  ◆ 用户操作（在 RViz 中）：                                             │
│     1. 在 RViz 中点击多个导航目标点                                     │
│     2. rosservice call /waypoint_planner/plan                          │
│     3. 车辆按规划路径自主行驶                                            │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 四、核心数据流

```
LiDAR ──→ /velodyne_points ──→ hdl_localization（NDT 帧图匹配）
                                  │
                                  ├──→ map → odom TF（定位坐标变换）
                                  │
GNSS ──→ /gps/fix ──────────→ init_location（初始位姿估计）
                                  │
                                  └──→ /initialpose（定位初始化）

用户点击 RViz ──→ /waypoint_planner/goal_raw ──→ muti_segment_planner
                                                      │
                                                      ↓
                                           /waypoint_planner/stitched_plan
                                                      │
                                                      ↓
LiDAR ──→ /velodyne_points ──→ pcl_to_laserscan ──→ /scan ──→ neupan DUNE
 (可选多线转伪激光)                                              │
                                                    ┌──────────┘
                                                    ↓
                                           /neupan_cmd_vel (Twist)
                                                    │
                                                    ↓
                                           can_analyzer（PID 控制）
                                                    │
                                                    ↓
                                           CAN 总线 → 底盘执行器
```

---

## 五、各包详表

### 5.1 自定义包

| 包名 | 目录 | 语言 | 文件数 | 作用 | 依赖 |
|------|------|------|--------|------|------|
| **can_analyzer** | control/ | C++ | 9 .cpp + 8 .h | CAN 总线收发、PID 纵横向控制 | roscpp, std_msgs |
| **comnav_gnss** | sensors/ | C++ | 2 .cpp + 2 .h | GNSS 串口驱动，NMEA 解析 | roscpp, serial, std_msgs |
| **nav_pkg** | navigation_location/ | C++ | 5 .cpp | 导航配置整合、点云转激光 | move_base, actionlib, pcl_ros |
| **init_location** | navigation_location/ | C++ | 7 .cpp + 8 .hpp | GNSS+LiDAR 初始位姿估计 | roscpp, PCL, Eigen |
| **mutitarget_nav** | navigation_location/ | C++ | 1 .cpp | 多点航点管理、路径拼接 | actionlib, tf2_ros, move_base_msgs |
| **hybrid_astar_planner** | navigation_location/ | C++ | 12 .cpp + 12 .h | Hybrid A* 全局规划器（nav_core 插件） | nav_core, pluginlib, OMPL, tf |
| **neupan_ros** | neupan_local_control/ | Python | 19 .py | DUNE 神经网络局部规划器 | rospy, std_msgs |
| **pcd2pgm** | navigation_location/ | C++ | 3 .cpp | 3D 点云 → 2D 栅格地图 | PCL, nav_msgs |

### 5.2 第三方包

| 包名 | 来源 | 作用 | 在 ROS2 下的替换策略 |
|------|------|------|-------------------|
| **velodyne** | ROS 官方 | Velodyne LiDAR 驱动 | 官方已支持 ROS2 |
| **lio_sam** | Tixiao Shan | LiDAR-IMU SLAM 建图 | 官方有 ROS2 分支 |
| **hdl_localization** | Koide3 | NDT 实时定位 | 需自行移植或找替代 |
| **hdl_global_localization** | Koide3 | 全局重定位 | 需自行移植或找替代 |
| **fast_gicp** | Koide3 | GICP 点云配准 | 有 ROS2 分支 |
| **ndt_omp** | Koide3 | NDT 配准（OMP） | 有 ROS2 分支 |
| **liorf_localization** | 开源 | 因子图优化定位 | 需检查 ROS2 支持 |
| **pointcloud_to_laserscan** | ROS 官方 | 点云转伪激光 | 官方已支持 ROS2 |

---

## 六、配置文件体系

### 6.1 导航代价地图配置

```
nav_pkg/config/
├── costmap_common_params.yaml     # 通用配置（机器人轮廓、障碍物层、膨胀层）
├── global_costmap_params.yaml     # 全局代价地图（静态地图模式）
├── local_costmap_params.yaml      # 局部代价地图（滚动窗口 5.5m×5.5m）
├── move_base_params.yaml          # move_base 参数
├── dwa_local_planner_params.yaml  # DWA 局部规划器参数（备选）
├── teb_local_planner_params.yaml  # TEB 局部规划器参数（备选）
├── local_planner_params.yaml      # 局部规划器参数
├── costmap_converter_params.yaml  # 代价地图转换参数
├── pcd/
│   └── GlobalMap.pcd             # 3D 点云地图
└── map/
    ├── map.yaml                   # 2D 栅格地图描述文件
    └── *.pgm                      # 2D 栅格地图图像
```

### 6.2 机器人物理参数（costmap_common_params.yaml）

```yaml
footprint:                      # 机器人轮廓（阿克曼底盘）
  - [ 0.67,  0.43]   # 前右
  - [ 0.67, -0.43]   # 前左
  - [-0.67, -0.43]   # 后左
  - [-0.67,  0.43]   # 后右

inflation_layer:
  inflation_radius: 0.5         # 膨胀半径
  cost_scaling_factor: 5.0      # 代价衰减系数

obstacle_layer:
  obstacle_range: 3.0           # 障碍物检测范围
  raytrace_range: 3.5           # 光线追踪范围
```

### 6.3 定位配置

```yaml
# hdl_localization 关键参数
reg_method: NDT_OMP              # 配准方法
ndt_resolution: 1.0              # NDT 体素分辨率
ndt_neighbor_search_method: DIRECT7  # 搜索方法
use_imu: true                    # 使用 IMU 辅助预测
specify_init_pose: true          # 使用初始位姿估计
```

---

## 七、TF 坐标变换树

```
map
 ↑  (hdl_localization 发布的 map→odom 变换)
odom
 ↑  (odometry 或 robot_pose_ekf)
base_link
 ├── velodyne（LiDAR 中心，静态变换 0 0 0 0 0 0）
 ├── rslidar（备选雷达，静态变换 0 0 0 0 0 0）
 └── laser_link（激光扫描坐标系，用于局部规划）
```

---

## 八、关键技术特点

### 8.1 混合定位策略

```
启动时: GNSS 粗定位 → 读取 /gps/fix 获取经纬度 → 转 UTM 局部坐标
         ↓
        多角度 NDT 搜索（72 等分，每 5° 搜索一次）
         ↓
        取拟合分数最低的位姿 → 发布 /initialpose
         ↓
运行时: hdl_localization（NDT_OMP）持续帧图匹配
         ↓
        若长时间匹配分数过高 → hdl_global_localization 全局重定位
```

### 8.2 阿克曼约束全局规划

Hybrid A* 算法在 A* 搜索基础上增加：
- **车辆运动学约束**：搜索空间从 (x, y) 扩展到 (x, y, θ)，每个节点携带航向
- **Reeds-Shepp 曲线**：节点间用 RS 曲线连接，保证路径可行驶
- **支持倒车模式**：`reverse = true` 时规划器可输出倒车路径

### 8.3 多点航点导航（mutitarget_nav）

```
用户点击多个 2D Nav Goal → 存入 waypoints[] 列表
         ↓
调用 /waypoint_planner/plan 服务
         ↓
遍历相邻航点对:
  (起点 → 航点1) → 调 /move_base/HybridAStarPlanner/make_plan 服务
  (航点1 → 航点2) → 同上调用
  (航点2 → 航点3) → 同上调用
         ↓
路径拼接 → 合并为一条完整 Path → 发布 /waypoint_planner/stitched_plan
         ↓
局部规划器沿此路径行驶
```

### 8.4 三维点云转二维代价地图的处理管线

```
原始3D点云（含地面、树木、建筑、车辆等）
      ↓
① RANSAC 地面分割 → 去掉地面点
② 高度滤波 → 保留地面以上 [0.1m, 1.0m] 的障碍物点
③ 半径去噪 → 去掉稀疏噪声点
④ XY 投影 → 丢弃 Z 坐标，按 (X,Y) 填入栅格
      ↓
最终栅格地图保留的是 "位于地面以上若干厘米的障碍物" 的二维投影
```

---

## 九、项目规模统计

| 指标 | 数值 |
|------|------|
| 自定义包 | 8 个 |
| 第三方包 | 9 个 |
| C++ 源文件 | ~165 个 |
| Python 源文件 | ~29 个 |
| Launch 文件 | 62 个 |
| 自定义消息(.msg) | 17 个 |
| 自定义服务(.srv) | 5 个 |
| 配置文件(.yaml) | 70+ 个 |
| 独立 catkin 工作区 | 5 个 |
| 启动脚本 | 3 个 |
| 机器人类型 | 阿克曼底盘 |
| 传感器 | Velodyne VLP-16 + IMU + GNSS |
| 操作系统 | Ubuntu 20.04 + ROS1 Noetic |
