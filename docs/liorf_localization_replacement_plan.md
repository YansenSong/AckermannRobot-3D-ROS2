# AckermannRobot：用 `liorf_localization` 替换 `src/hdl_localization` 的执行计划

> 目标工作区：`/home/young/Project/AckermannRobot`  
> liorf 源码工作区：`/home/young/Project/liorf_localization`  
> 目标系统：Ubuntu 22.04 + ROS 2 Humble  
> 计划用途：交给 Codex 按阶段实施、编译、验证和清理

---

## 0. 结论先行

本次替换不是简单的“删除 `src/hdl_localization`，把另一个目录复制进来”。正确做法是：

1. 保持现有 **LIO-SAM 建图链路不变**，继续生成 `maps/<map_name>/GlobalMap.pcd`。
2. 在导航阶段，将当前：

```text
GlobalMap.pcd + /points_raw + /imu/data
        ↓
hdl_localization
(NDT_OMP + UKF)
        ↓
map -> base_link
```

替换为：

```text
GlobalMap.pcd
      │
      │
/points_raw ── gazebo_lidar_adapter ──> /points_lio (ring + time)
                                             │
/imu/data ───────────────────────────────────┤
                                             ▼
                                  liorf ImageProjection
                                  + IMU deskew
                                             │
                                             ▼
                                  liorf mapOptimization
                                  prior-map scan-to-map
                                             │
                                             ▼
                                  IMU Preintegration
                                  + TransformFusion
                                             │
                                             ▼
                                      /odom
                                  odom -> base_link
                                             │
map -> odom (identity) ──────────────────────┘
```

3. **定位阶段的 LiDAR 输入必须从 `/points_raw` 改为 `/points_lio`。** 当前项目已经有 `gazebo_lidar_adapter.py`，它专门为 LIO-SAM 添加 `ring` 和逐点 `time` 字段，而 `liorf_localization` 的 ROS2 `ImageProjection` 正需要这种 Velodyne/LIO-SAM 风格点云。因此不要新写转换节点。
4. **导航模式必须关闭 robot_localization EKF 对 `odom -> base_link` 的 TF 发布。** liorf 的 `TransformFusion` 应成为该动态 TF 的唯一发布者。
5. 不直接使用 liorf 上游自带的 `run_localization.launch.py` 作为 Ackermann 总启动文件；由 `ackermann_bringup` 继续统一编排定位、规划、RViz、NeuPAN 周边组件。
6. liorf 上游 ROS2 分支只明确声称支持 Foxy/Galactic，因此在 Humble 下需要先做兼容性验证，并修补几个已从源码确认的集成问题。
7. 最终删除 `src/hdl_localization`，但**必须在 liorf 完成独立编译和导航烟雾测试之后再删除**，方便对照和回滚。

---

# 1. 本计划所依据的源码版本

## 1.1 AckermannRobot

已按 GitHub 当前 `main` 分支分析：

```text
repository: YansenSong/AckermannRobot-3D-ROS2
commit:     e9e4b6f9f056b02f8e1a6584c7bcbebcc520ec31
commit time: 2026-09-10
```

重点检查过：

```text
README.md
scripts/nav_hdl_neupan.sh
scripts/run_neupan.sh
src/ackermann_bringup/launch/localization.launch.py
src/ackermann_bringup/launch/navigation.launch.py
src/ackermann_bringup/launch/navigation_sim.launch.py
src/ackermann_bringup/launch/planning.launch.py
src/ackermann_bringup/package.xml
src/ackermann_simulation/gazebo/launch/gazebo.launch.py
src/ackermann_simulation/gazebo/scripts/gazebo_lidar_adapter.py
src/ackermann_simulation/robot/xacro/sensors.xacro
src/lio-sam/config/params.yaml
src/neupan_ros2/config/robots/ackermann_robot/robot.yaml
src/nav_status/config/nav_status.yaml
docs/topic_info.md
src/hdl_localization/...
```

## 1.2 liorf_localization

必须基于其 **ROS2 分支**：

```text
repository: YJZLuckyBoy/liorf_localization
branch:     liorf_localization-ros2
commit:     ad592cebc397209273245b4e669327399a591c9d
```

上游 README 明确说明：

- 这是基于 LIO-SAM framework 的 prior-map localization；
- 可使用 LIO-SAM 等算法生成的先验地图；
- 地图文件名为 `GlobalMap.pcd`；
- ROS2 分支注明 Foxy/Galactic；
- 初始化使用 RViz `2D Pose Estimate`。

### Codex 开始执行时必须再次确认本地版本

本计划无法直接读取用户机器上的 `/home/young/Project/...` 文件系统，因此 Codex 必须优先以本地 checkout 为准，并记录差异：

```bash
PROJECT=/home/young/Project/AckermannRobot
LIORF=/home/young/Project/liorf_localization

git -C "$PROJECT" status --short
git -C "$PROJECT" branch --show-current
git -C "$PROJECT" rev-parse HEAD

git -C "$LIORF" status --short
git -C "$LIORF" branch --show-current
git -C "$LIORF" rev-parse HEAD
```

如果本地 AckermannRobot 比上述 commit 更新，以**本地最新源码**为准；如果 liorf 本地有用户修改，不得覆盖或丢弃。

---

# 2. 当前系统的定位边界

当前项目中，建图和导航已经是解耦的：

## 2.1 建图阶段

```text
/points_raw
    ↓
gazebo_lidar_adapter
    ↓
/points_lio
    ↓
LIO-SAM + /imu/data
    ↓
GlobalMap.pcd
```

当前 LIO-SAM 参数已经与仿真机器人匹配：

```text
pointCloudTopic = /points_lio
imuTopic        = /imu/data
lidarFrame      = laser_link
baselinkFrame   = base_link
odometryFrame   = odom
mapFrame        = map
sensor          = velodyne
N_SCAN          = 16
Horizon_SCAN    = 1800
```

因此**建图部分无需换成 liorf，也不要修改当前 `src/lio-sam` 的建图行为**。

## 2.2 当前导航定位阶段

`ackermann_bringup/launch/localization.launch.py` 当前启动：

```text
hdl_localization_map_server
hdl_localization_node
```

其中 HDL：

```text
/points_raw + /imu/data + GlobalMap.pcd
        ↓
NDT_OMP + UKF
        ↓
/odom + TF
```

而 `navigation.launch.py` 的其他模块主要依赖 TF，而不是依赖 HDL 私有接口：

- Hybrid A* bridge：需要 `map -> rear_axle_link`；
- NeuPAN：明确期望 `map -> odom -> base_link -> rear_axle_link`；
- `nav_status` 使用的是 `/odom_wheel`，不是 HDL `/odom`；
- 2D 障碍物链路继续使用 `/points_raw -> pointcloud_to_laserscan -> /scan`。

所以替换定位模块的影响边界较清晰：**重点是定位 launch、TF 所有权、`/odom` 兼容和地图加载接口；规划/NeuPAN/控制算法本身无需修改。**

---

# 3. 目标架构与接口契约

最终必须形成下面的 TF 树：

```text
map
 └── odom                       # ackermann_bringup: static identity
      └── base_link             # liorf TransformFusion: dynamic
           ├── laser_link       # robot_state_publisher: fixed joint
           ├── gyro_link        # robot_state_publisher: fixed joint
           └── rear_axle_link   # robot_state_publisher: robot geometry
```

## TF 所有权必须唯一

| TF | 唯一发布者 | 说明 |
|---|---|---|
| `map -> odom` | `static_transform_publisher` | liorf 上游设计本身采用 identity；保持与 NeuPAN 现有 TF 契约一致 |
| `odom -> base_link` | liorf `TransformFusion` | 导航定位的动态姿态 |
| `base_link -> laser_link` | `robot_state_publisher` | Xacro 固定外参 |
| `base_link -> gyro_link` | `robot_state_publisher` | IMU 当前就在 base 原点 |
| `base_link -> rear_axle_link` | `robot_state_publisher` | 供 Smac/NeuPAN 使用 |

### 禁止

不得同时让以下两个模块发布 `odom -> base_link`：

```text
robot_localization EKF
liorf TransformFusion
```

导航模式必须：

```text
publish_ekf_tf = false
```

EKF 节点可以继续运行并发布 `/odometry/filtered` 做诊断，但不能拥有 TF。

---

# 4. liorf 与当前项目之间已经确认的关键兼容点

## 4.1 点云格式：可以直接复用现有 `/points_lio`

当前 `gazebo_lidar_adapter.py` 输出的点字段是：

```text
x           FLOAT32
y           FLOAT32
z           FLOAT32
intensity   FLOAT32
ring        UINT16
time        FLOAT32
```

liorf ROS2 `imageProjection.cpp` 的 Velodyne 输入结构正是：

```text
x y z intensity ring(uint16) time(float)
```

因此迁移后：

```text
liorf pointCloudTopic = /points_lio
```

不要把 liorf 接到 `/points_raw`，否则缺少 `ring/time`，去畸变和投影会失败或严重退化。

## 4.2 IMU：直接复用 `/imu/data`

当前仿真 IMU：

```text
topic: /imu/data
frame: gyro_link
rate: 100 Hz
```

当前 LIO-SAM 已经使用这一路 IMU 正常建图，因此 liorf 的首轮参数应直接复用已验证过的 IMU 噪声、重力和外参配置，而不是使用上游示例值。

## 4.3 先验地图：继续使用原有 `GlobalMap.pcd`

无需改地图生产链：

```text
LIO-SAM /lio_sam/save_map
      ↓
maps/<name>/GlobalMap.pcd
```

但需要修改 liorf 的地图路径接口，见后文“必须修补项”。

## 4.4 `/initialpose`：现有 RViz 流程可以继续

liorf `mapOptmization.cpp` 已订阅：

```text
/initialpose
```

并在首次定位时使用：

```text
用户粗略初始位姿
    ↓
当前 deskewed LiDAR scan
    ↓
ICP against GlobalMap.pcd
    ↓
fitness < 0.3 才认为初始化成功
```

因此用户操作仍可保持：

```text
RViz -> 2D Pose Estimate -> 等定位稳定 -> 2D Goal Pose
```

注意：上游 liorf 的 `/initialpose` 实现更接近“LiDAR 初始姿态”语义，而 RViz 通常表达 `base_link` 初始姿态。当前 `base_link -> laser_link` 有约 `(-0.0778, 0, 0.384)` 的平移。首轮集成可以依赖 ICP 修正这点小偏差；完成基本迁移后，应把“将 base 初始位姿转换为 lidar 初始位姿”列为推荐兼容增强，避免留下坐标语义歧义。

---

# 5. liorf 上游代码中必须处理的集成问题

这部分是本次替换最重要的代码工作，不应跳过。

## 5.1 必须新增 `globalmap_pcd` 绝对路径参数

### 上游现状

`mapOptmization.cpp::loadGlobalMap()` 当前采用：

```cpp
std::string global_map = std::getenv("HOME") + savePCDDirectory;
pcl::io::loadPCDFile<PointType>(global_map + "GlobalMap.pcd", ...);
```

这与 Ackermann 当前接口不兼容。当前启动脚本已经把：

```text
/home/young/Project/AckermannRobot/maps/mini/GlobalMap.pcd
```

作为 `globalmap_pcd` 明确传入。

### 目标修改

在 liorf 参数层增加：

```text
globalmap_pcd: ""
```

建议在 `ParamServer` 中声明：

```cpp
std::string globalmapPcd;
```

然后 `loadGlobalMap()`：

1. `globalmap_pcd` 非空：直接使用这个文件；
2. 空时才保留原来的 `HOME + savePCDDirectory + GlobalMap.pcd` 作为兼容 fallback；
3. 检查 `pcl::io::loadPCDFile()` 返回值；
4. 打印实际绝对路径；
5. 地图不存在、读取失败、点数不足时给明确 `RCLCPP_ERROR`，不得继续输出假定位。

这样可以保持现有外部脚本接口：

```bash
globalmap_pcd:="$MAP_DIR/GlobalMap.pcd"
```

## 5.2 必须修复 TransformFusion 的 ROS2 时间戳溢出风险

### 上游现状

`imuPreintegration.cpp` 的 `TransformFusion::imuOdometryHandler()` 中存在类似：

```cpp
rclcpp::Time t(static_cast<uint32_t>(lidarOdomTime * 1e9));
```

纳秒时间先被压成 `uint32_t`，数秒后就会溢出。这对 NeuPAN 尤其危险，因为 NeuPAN 当前配置明确会拒绝 stale TF。

### 目标修改

不要从 `double seconds -> uint32 nanoseconds` 重建时间。

使用当前消息的原生 ROS stamp，例如：

```cpp
rclcpp::Time tf_stamp(laserOdometry.header.stamp);
```

或直接使用最新 `imuOdomQueue.back().header.stamp`。

确保发布的 `odom -> base_link` TF 时间与对应融合里程计时间一致且单调递增。

### 验收重点

必须运行超过 5 分钟，确认：

```text
无 TF 时间跳回
无 extrapolation into the past/future 连续报错
NeuPAN 不持续报告 stale odom-to-lidar transform
```

## 5.3 必须解决 mapOptimization 额外发布 `odom -> lidar_link` 的问题

上游 `mapOptmization.cpp::publishOdometry()` 额外广播：

```text
odometryFrame -> "lidar_link"
```

而当前机器人真实 LiDAR frame 是：

```text
laser_link
```

同时，liorf `TransformFusion` 又会发布：

```text
odom -> base_link
```

再由 `robot_state_publisher` 发布：

```text
base_link -> laser_link
```

### 目标策略

**不要简单把硬编码 `lidar_link` 改成 `laser_link`。**

那样会让 `laser_link` 同时拥有：

```text
odom -> laser_link
base_link -> laser_link
```

造成多父节点/TF 冲突。

推荐做法：

- 删除或通过参数关闭 `mapOptimization` 里的这个 TF broadcaster；
- `mapOptimization` 继续发布其 odometry topic；
- 由 `TransformFusion` 唯一发布 `odom -> base_link`；
- `robot_state_publisher` 唯一发布 `base_link -> laser_link`。

## 5.4 必须让最终 `/odom` 消息 frame 语义正确

建议把 liorf 的公共输出约定成：

```text
topic:          /odom
header.frame_id = odom
child_frame_id  = base_link
```

`TransformFusion` 发布最终融合里程计时，显式保证上述字段，而不要保留内部 `odom_imu` 等 child frame 名。

内部 liorf topic 可继续保留：

```text
/odom_incremental
liorf_localization/mapping/odometry
liorf_localization/mapping/odometry_incremental
```

## 5.5 必须让 lidar↔base 静态 TF 查询具备启动鲁棒性

上游 `TransformFusion` 在构造函数中，如果：

```text
lidarFrame != baselinkFrame
```

会立即只查询一次 TF。当前项目：

```text
lidarFrame    = laser_link
baselinkFrame = base_link
```

启动时若 `robot_state_publisher` 的静态 TF 尚未进入 buffer，会查询失败，而上游没有可靠重试。

### 目标修改

改为以下任一种健壮方案：

- 首次真正需要融合时再查询，失败则下一条消息重试；或
- 用短周期 timer 非阻塞重试直到成功；
- 成功后缓存固定变换。

不得通过硬编码当前机器人外参来绕过 TF 系统。

## 5.6 推荐：全局地图发布改为 Transient Local

上游 map publisher 使用 volatile QoS，而且 `publishCloud()` 只有“当时已有订阅者”才 publish。这样如果 RViz 晚于 mapOptimization 启动，可能看不到全局地图。

推荐对：

```text
liorf_localization/localization/global_map
```

使用：

```text
KeepLast(1) + Reliable + TransientLocal
```

并在地图加载成功后无条件发布一次。

同时删除 `loadGlobalMap()` 中为了等待订阅者而写的 `sleep(3)`，避免阻塞式启动技巧。

这不是定位算法修改，只是 ROS2 工程化修复。

## 5.7 推荐但可放到第二轮：明确 `/initialpose` 的 base_link 语义

为了和当前 HDL/RViz 使用体验完全一致，最终应把 RViz `/initialpose` 视为：

```text
T_map_base
```

在 liorf 内部初始化 ICP 之前，通过 TF：

```text
T_map_lidar = T_map_base * T_base_lidar
```

再作为当前 LiDAR scan 的粗初值。

但该项不应阻塞第一轮迁移；先验证原版初始化链路，确认是否确实产生可观测偏差，再做此兼容补丁。

---

# 6. 推荐的 liorf 项目参数

不要直接使用上游 `config/localization.yaml` 的 UrbanNav 示例参数。

建议新建 Ackermann 专用配置：

```text
src/ackermann_bringup/config/liorf_localization.yaml
```

这样：

- vendored liorf 源码尽量保留上游结构；
- Ackermann 传感器/仿真参数归 `ackermann_bringup` 管理；
- 未来真实车和 Gazebo 可以再拆成不同参数文件。

首轮配置建议：

```yaml
/**:
  ros__parameters:
    history_policy: "history_keep_last"
    reliability_policy: "reliability_reliable"

    # Inputs / public output prefix
    pointCloudTopic: "/points_lio"
    imuTopic: "/imu/data"
    odomTopic: "/odom"
    gpsTopic: "/odometry/gpsz"

    # Frames
    lidarFrame: "laser_link"
    baselinkFrame: "base_link"
    odometryFrame: "odom"
    mapFrame: "map"

    # New integration parameter; launch overrides this
    globalmap_pcd: ""

    # No GPS in current simulation localization path
    useImuHeadingInitialization: false
    useGpsElevation: false
    gpsCovThreshold: 2.0
    poseCovThreshold: 25.0

    # LiDAR: align with the already-working LIO-SAM mapping config
    sensor: "velodyne"
    N_SCAN: 16
    Horizon_SCAN: 1800
    downsampleRate: 1
    point_filter_num: 3
    lidarMinRange: 1.0
    lidarMaxRange: 1000.0

    # IMU: start by copying the already-validated LIO-SAM values
    imuType: 1
    imuRate: 100.0
    imuAccNoise: 3.9939570888238808e-03
    imuGyrNoise: 1.5636343949698187e-03
    imuAccBiasN: 6.4356659353532566e-05
    imuGyrBiasN: 3.5640318696367613e-05
    imuGravity: 9.80511
    imuRPYWeight: 0.01

    extrinsicTrans: [-0.078, 0.0, 0.384]
    extrinsicRot: [1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0]
    extrinsicRPY: [1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0]

    mappingSurfLeafSize: 0.2
    surroundingKeyframeMapLeafSize: 0.2

    # 20 Hz LiDAR；先允许每帧处理，再根据 CPU 实测调节
    mappingProcessInterval: 0.05
    numberOfCores: 4

    surroundingkeyframeAddingDistThreshold: 0.5
    surroundingkeyframeAddingAngleThreshold: 0.2
    surroundingKeyframeDensity: 2.0
    surroundingKeyframeSearchRadius: 15.0

    # Localization-only integration: no active loop closure thread
    loopClosureEnableFlag: false

    z_tollerance: 1000.0
    rotation_tollerance: 1000.0
```

### 参数说明

以上是“首轮可运行参数”，不是最终性能调优结论。

优先级应为：

```text
1. 当前已经成功建图的 LIO-SAM 传感器/IMU/外参参数
2. liorf 算法特有参数的上游默认值
3. 在 mini.world 上实测延迟、收敛和精度后再调
```

尤其不要在尚未跑通前同时大幅调整：

```text
point_filter_num
mappingSurfLeafSize
surroundingKeyframeMapLeafSize
mappingProcessInterval
surroundingKeyframeSearchRadius
ICP threshold
IMU noise
```

否则无法判断故障来自迁移还是调参。

---

# 7. 分阶段执行计划

## Phase 0：保护现场、建立基线

### 0.1 检查工作树

```bash
PROJECT=/home/young/Project/AckermannRobot
LIORF=/home/young/Project/liorf_localization

cd "$PROJECT"
git status --short
git branch --show-current
git rev-parse HEAD

cd "$LIORF"
git status --short
git branch --show-current
git rev-parse HEAD
```

不得 reset、clean 或覆盖任何未提交用户修改。

### 0.2 创建迁移分支

如果项目当前没有未处理的分支策略：

```bash
cd "$PROJECT"
git switch -c feat/liorf-localization
```

### 0.3 建立当前 HDL 基线

先确认当前工程仍可构建：

```bash
cd "$PROJECT"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果当前 main 本身编译失败，先记录失败，**不要把原有失败归因到 liorf**。

建议记录：

```bash
ros2 pkg list | grep -E 'lio_sam|hdl_localization|ackermann_bringup'
```

并在能运行时保存当前 TF / topic 基线：

```bash
ros2 topic list > /tmp/ackermann_topics_before_liorf.txt
ros2 run tf2_ros tf2_echo map base_link
```

---

## Phase 1：把 ROS2 liorf vendoring 进目标工程，但暂不删除 HDL

### 1.1 确认源码是 ROS2 分支

如果 `/home/young/Project/liorf_localization` 已经是：

```text
liorf_localization-ros2
```

且工作区状态可安全读取，直接作为来源。

如果本地当前不是 ROS2 分支，**不要在有未提交修改的工作树上强制 checkout**。推荐：

```bash
git -C "$LIORF" fetch origin

git -C "$LIORF" worktree add \
  /tmp/liorf-localization-ros2 \
  origin/liorf_localization-ros2
```

然后使用 `/tmp/liorf-localization-ros2` 作为 vendoring 来源。

### 1.2 复制到目标

最终目标目录：

```text
/home/young/Project/AckermannRobot/src/liorf_localization
```

推荐复制仓库内容而不嵌套其 `.git`：

```bash
rsync -a --exclude='.git' \
  /tmp/liorf-localization-ros2/ \
  "$PROJECT/src/liorf_localization/"
```

如果来源本来就在正确 ROS2 分支，则替换 rsync source 路径即可。

### 1.3 许可证

必须保留 liorf 上游 `LICENSE` 和原作者信息，不要把它改成 AckermannRobot 自己的 Apache license。

### 1.4 先只编 liorf

```bash
cd "$PROJECT"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select liorf_localization
```

如果失败：

- 先解决 Humble API/CMake 兼容；
- 不要为了“能编”大改算法；
- 优先最小兼容 patch；
- 记录每一处与上游 ROS2 分支的差异。

预计已有 `GTSAM` 依赖，因为当前项目 LIO-SAM 已经使用 GTSAM；仍应由构建结果确认，不要假设系统环境完整。

---

## Phase 2：完成 liorf 的项目级兼容修补

按本计划第 5 节依次处理：

```text
A. globalmap_pcd 绝对路径参数
B. TransformFusion 时间戳溢出
C. 禁用 mapOptimization 的 odom->lidar TF
D. 公共 /odom frame_id / child_frame_id
E. lidarFrame<->baseLink TF 查询重试
F. global map transient-local（推荐）
```

每完成一项至少执行：

```bash
colcon build --symlink-install --packages-select liorf_localization
```

不要等所有 patch 写完才第一次编译。

### Phase 2 完成验收

```text
[ ] liorf_localization 在 ROS2 Humble 下独立编译通过
[ ] 不依赖 hdl_localization / ndt_omp / hdl_global_localization
[ ] 地图路径可由 globalmap_pcd 直接传入
[ ] 不再发布 odom -> lidar_link/laser_link 的额外动态 TF
[ ] TransformFusion 仍发布 odom -> base_link
[ ] 公共 /odom 是 odom -> base_link 语义
```

---

## Phase 3：增加 Ackermann 专用 liorf 配置

创建：

```text
src/ackermann_bringup/config/liorf_localization.yaml
```

使用第 6 节参数作为初始值。

不要修改：

```text
src/lio-sam/config/params.yaml
```

除非运行实测证明当前 LIO-SAM 建图参数本身存在独立错误。本任务的目标是替换 localization，不是重新调 mapping。

---

## Phase 4：重写 `ackermann_bringup/launch/localization.launch.py`

当前文件是 HDL-specific，应整体改成 liorf-specific 编排。

### 新 localization.launch.py 的职责

它应只负责：

```text
1. map -> odom static identity
2. liorf_localization_imageProjection
3. liorf_localization_imuPreintegration
4. liorf_localization_mapOptmization
```

不要在这个 launch 内启动：

```text
RViz
Gazebo
Smac planner
NeuPAN
robot_localization EKF
```

这些仍由现有上层 launch 管理。

### 建议 launch 参数

保留：

```text
globalmap_pcd
use_sim_time
params_file
```

其中：

```text
params_file default = ackermann_bringup/config/liorf_localization.yaml
```

并对三个 liorf Node 同时传：

```python
parameters=[
    LaunchConfiguration('params_file'),
    {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'globalmap_pcd': LaunchConfiguration('globalmap_pcd'),
    },
]
```

### 初始位姿参数兼容

当前 `navigation.launch.py` 还声明过：

```text
specify_init_pose
init_pos_x/y/z
init_ori_w/x/y/z
```

但当前 `nav_hdl_neupan.sh` 并未使用这些参数，默认工作流就是 RViz `/initialpose`。

首轮替换建议：

- 删除这些 HDL-specific 参数；
- 统一由 `/initialpose` 初始化；
- README 明确要求先 `2D Pose Estimate`。

如果后续确实需要无 RViz 自动设置初值，再增加一个很薄的 one-shot initial-pose publisher，而不是把 HDL 参数机制硬塞进 liorf 核心算法。

---

## Phase 5：修改 `navigation.launch.py`

目标：外部导航结构尽可能不变，只替换 localization include 参数。

需要：

1. 去掉 HDL-only initial pose launch arguments；
2. 向新的 localization.launch.py 传：
   - `globalmap_pcd`；
   - `use_sim_time=true`；
   - 可选 `params_file`；
3. 保持以下节点完全不动，除非编译/TF 实测要求：
   - `planning.launch.py`；
   - `pointcloud_to_laserscan`；
   - `cmd_vel_mux`；
   - `nav_status`；
   - RViz。

### 重要

`pointcloud_to_laserscan` 继续：

```text
/points_raw -> /scan
```

不要为了 liorf 把它改成 `/points_lio`。

这样定位和障碍物感知各取最合适的数据：

```text
/points_raw -> 2D LaserScan -> NeuPAN
/points_lio -> liorf -> localization
```

---

## Phase 6：修改 `navigation_sim.launch.py` 的 TF 所有权

当前该 launch 启动 Gazebo 时是：

```python
'publish_ekf_tf': 'true'
```

必须改为：

```python
'publish_ekf_tf': 'false'
```

原因：liorf `TransformFusion` 接管 `odom -> base_link`。

不要直接删除 EKF 节点；它仍可保留 `/odometry/filtered` 供诊断。如果确认全项目完全不再需要它，再另开独立清理任务。

---

## Phase 7：修改 `ackermann_bringup/package.xml`

删除：

```xml
<exec_depend>hdl_localization</exec_depend>
```

增加：

```xml
<exec_depend>liorf_localization</exec_depend>
```

其余 planner / NeuPAN / pointcloud_to_laserscan 依赖保持不变。

---

## Phase 8：新增/重命名导航脚本

当前：

```text
scripts/nav_hdl_neupan.sh
```

建议最终改成：

```text
scripts/nav_liorf_neupan.sh
```

脚本对地图文件的检查继续保留：

```text
GlobalMap.pcd
map.pgm
map.yaml
```

调用接口也尽量保持：

```bash
bash scripts/nav_liorf_neupan.sh maps/mini
```

它仍传：

```text
map:=...
map_pgm:=...
globalmap_pcd:=...
```

### 兼容策略

二选一：

**方案 A（推荐最终清爽）**

直接删除 `nav_hdl_neupan.sh` 并更新文档。

**方案 B（迁移期更稳）**

旧脚本保留一小段 wrapper：

```bash
echo "Deprecated: use nav_liorf_neupan.sh"
exec bash .../nav_liorf_neupan.sh "$@"
```

确认所有文档和个人使用习惯迁移后再删除。

---

## Phase 9：先并存运行 liorf，再删除 HDL

在 `src/hdl_localization` 尚未删除时执行一次完整构建：

```bash
cd "$PROJECT"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ackermann_bringup
source install/setup.bash
```

此时 active launch 已经不应引用 HDL。

检查：

```bash
grep -R --line-number --exclude-dir=.git \
  'hdl_localization' \
  src/ackermann_bringup scripts README.md docs || true
```

允许剩余的只有：

- 历史说明；或
- 待下一阶段删除的旧脚本/文档。

不能有 active launch/package dependency。

---

# 8. 第一次运行的严格测试顺序

不要一上来直接给目标点跑完整导航。按以下顺序隔离问题。

## Test A：传感器输入

启动 Gazebo 后确认：

```bash
ros2 topic hz /points_raw
ros2 topic hz /points_lio
ros2 topic hz /imu/data
```

预期约：

```text
LiDAR: 20 Hz
IMU:   100 Hz
```

确认 `/points_lio` 的 PointCloud2 schema 包含：

```text
ring
time
```

如果 liorf 日志出现：

```text
Point cloud ring channel not available
Point cloud timestamp not available
```

先修点云输入，不要调 ICP/scan-to-map 参数。

## Test B：只运行定位，不运行规划/NeuPAN

建议给 `localization.launch.py` 单独启动能力：

```bash
ros2 launch ackermann_bringup localization.launch.py \
  globalmap_pcd:=/home/young/Project/AckermannRobot/maps/mini/GlobalMap.pcd \
  use_sim_time:=true
```

同时 Gazebo 已运行且 `publish_ekf_tf=false`。

检查：

```bash
ros2 topic list | grep liorf
ros2 topic hz /odom
```

初始化前，允许最终定位尚未稳定输出。

## Test C：RViz 初始化

RViz Fixed Frame：

```text
map
```

使用：

```text
2D Pose Estimate
```

在机器人真实初始位置附近给粗略 x/y/yaw。

日志应至少能区分：

```text
map loaded
waiting for initial pose
manual initialize pose
ICP initialize success / failure
```

若 ICP 失败：

1. 先检查地图与 scan 是否同坐标尺度；
2. 检查点云 frame；
3. 检查 initial pose 是否离真实位置过远；
4. 检查地图点数/下采样；
5. 最后才调 ICP 阈值。

不要第一反应把 `fitness < 0.3` 放宽到很大。

## Test D：TF 树

初始化成功后：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map rear_axle_link
ros2 run tf2_ros tf2_echo base_link laser_link
```

推荐额外：

```bash
ros2 run tf2_tools view_frames
```

验收：

```text
[ ] map -> odom 唯一且 identity
[ ] odom -> base_link 只有 liorf 在发布
[ ] base_link -> laser_link 是静态固定关节
[ ] map -> rear_axle_link 可连续查询
[ ] 没有 lidar_link 这种幽灵 frame 被 mapOptimization 动态挂到 odom 下
[ ] 没有同一 child 的两个 parent
```

## Test E：里程计语义

```bash
ros2 topic echo /odom --once
```

必须看到：

```text
header.frame_id: odom
child_frame_id: base_link
```

车辆移动后：

```bash
ros2 topic hz /odom
```

检查输出连续，不应只以低频 scan-to-map 频率跳变；TransformFusion 应提供 IMU 驱动的高频融合输出。

## Test F：持续运行测试

至少运行 5 分钟，并多次前进/转弯/倒车。

重点检查：

```text
TF 时间戳持续递增
没有数秒后突然 stale
没有 nan / inf
没有 IMU preintegration failure
没有 TF extrapolation 持续刷屏
定位没有周期性跳回初值
```

这是专门用于验证第 5.2 节时间戳修复。

## Test G：完整导航

确认定位稳定后再启动：

```bash
bash scripts/nav_liorf_neupan.sh maps/mini
```

另一个终端：

```bash
bash scripts/run_neupan.sh
```

流程：

```text
2D Pose Estimate
    ↓
liorf ICP init + continuous localization
    ↓
2D Goal Pose
    ↓
SmacPlannerHybrid
    ↓
/plan
    ↓
NeuPAN
    ↓
cmd_vel_mux
    ↓
Ackermann controller
```

验收 Hybrid A* / NeuPAN 无需知道 HDL 已被替换，它们只应感知到正确而稳定的 TF。

## Test H：失败场景

至少测试：

1. `globalmap_pcd` 路径错误：应明确报错，不应崩溃或静默继续；
2. 未给 `/initialpose`：应等待，不应发布明显错误的全局定位；
3. 给一个明显错误初值：ICP 应失败或不进入稳定定位，而不是“自信地输出垃圾”；
4. 再次给正确 `/initialpose`：应能够重新初始化；
5. 重启 localization 节点，不重启 Gazebo：应能重新接收地图/传感器并初始化。

---

# 9. 性能与精度调优顺序

只有功能测试全部通过后再调性能。

建议一次只改一个变量，顺序：

```text
1. mappingProcessInterval
2. point_filter_num
3. mappingSurfLeafSize
4. surroundingKeyframeMapLeafSize
5. surroundingKeyframeSearchRadius
6. ICP 初始化参数
7. IMU noise / bias（仅有证据时）
```

建议记录指标：

```text
CPU usage
scan-to-map frequency
/odom frequency
初始化 ICP fitness
初始化耗时
运行 5/10/30 min 漂移
转弯时轨迹抖动
静止时 pose jitter
Nav/NeuPAN 是否出现 TF stale
```

### 不要为了“看起来更强”提前打开 loop closure

这里使用的是固定先验地图定位，不是重新建图。首轮保持：

```text
loopClosureEnableFlag: false
```

并且不要改写 liorf 核心优化结构，先验证上游 localization 算法本身。

---

# 10. 删除 HDL 与最终清理

只有在 Test A~G 全部通过后才执行。

## 10.1 删除旧定位源码

最终删除：

```text
src/hdl_localization
```

其中嵌套的：

```text
ndt_omp
fast_gicp
hdl_global_localization
hdl_localization
```

都会随旧模块移除。

注意：当前 active Ackermann bringup 已经设置：

```text
use_global_localization = false
```

因此删除当前 HDL package 不会丢掉一个正在使用的 BBS/FPFH global relocalization 功能。liorf 本身目前仍依赖用户给粗略 `/initialpose`；不要在本任务里额外实现新的全局重定位算法。

## 10.2 清理旧构建产物

删除旧包残留，而不是盲目 `rm -rf build install log` 前先确认用户是否有其他环境依赖。

至少检查：

```bash
find build install -maxdepth 2 -iname '*hdl*' -o -iname '*ndt_omp*'
```

然后按包清理或完整重新构建。

## 10.3 全仓引用检查

```bash
cd "$PROJECT"
grep -R --line-number --exclude-dir=.git \
  -E 'hdl_localization|hdl_global_localization|NDT_OMP' \
  . || true
```

保留的引用必须只是“历史对比说明”；active code / launch / package.xml / startup script 不得再依赖 HDL。

---

# 11. 文档更新范围

至少更新：

```text
README.md
docs/topic_info.md
scripts/nav_hdl_neupan.sh -> scripts/nav_liorf_neupan.sh
```

README 中导航部分应变成：

```text
Gazebo + liorf localization + Smac Hybrid A* + NeuPAN
```

`docs/topic_info.md` 至少更新：

```text
/initialpose -> liorf_localization
/points_lio -> liorf ImageProjection
/imu/data -> liorf ImageProjection + IMUPreintegration
/odom -> liorf TransformFusion output
liorf_localization/... -> 调试/地图/轨迹话题
```

并删除：

```text
/globalmap 是 HDL Localization 使用
/odom 是 HDL 输出
```

这类过期说明。

---

# 12. 预期最终目录结构

重点相关部分应接近：

```text
AckermannRobot/
├── maps/
│   └── mini/
│       ├── GlobalMap.pcd
│       ├── map.pgm
│       └── map.yaml
├── scripts/
│   ├── mapping_mini.sh
│   ├── pcd_to_map.sh
│   ├── nav_liorf_neupan.sh
│   └── run_neupan.sh
└── src/
    ├── lio-sam/                    # mapping，保持
    ├── liorf_localization/         # 新 localization
    ├── ackermann_bringup/
    │   ├── config/
    │   │   └── liorf_localization.yaml
    │   └── launch/
    │       ├── mapping.launch.py
    │       ├── localization.launch.py
    │       ├── navigation.launch.py
    │       └── navigation_sim.launch.py
    ├── ackermann_simulation/
    ├── ackermann_smac_bridge/
    ├── nav2_smac_planner/
    ├── nav_status/
    └── neupan_ros2/
```

最终不再存在：

```text
src/hdl_localization/
```

---

# 13. 建议的 Git 提交拆分

不要做成一个巨大 commit。推荐：

```text
commit 1: vendor liorf_localization ROS2 source
commit 2: harden liorf for Humble / map-path / TF integration
commit 3: add Ackermann liorf config and switch bringup
commit 4: switch navigation script and validate full stack
commit 5: remove hdl_localization and stale dependencies
commit 6: update README and topic documentation
```

每个 commit 前至少保证相关 package 能编译。

---

# 14. 完成定义（Definition of Done）

只有以下全部满足，才认为“HDL -> liorf”替换完成：

```text
[ ] ROS2 Humble 下完整 colcon build 成功
[ ] active source/launch/package dependency 不再依赖 hdl_localization
[ ] src/hdl_localization 最终已删除
[ ] LIO-SAM mapping 行为未被破坏
[ ] 原有 GlobalMap.pcd 可直接被 liorf 从绝对路径加载
[ ] /points_lio 包含 ring/time，并被 liorf 正确消费
[ ] /imu/data 被 ImageProjection 和 IMUPreintegration 正确消费
[ ] RViz /initialpose 可以触发 ICP 初始化
[ ] 初始化成功后可持续 scan-to-map localization
[ ] /odom 为 odom -> base_link 语义
[ ] TF 唯一形成 map -> odom -> base_link -> rear_axle_link
[ ] robot_state_publisher 继续唯一拥有 base_link -> laser_link
[ ] EKF 导航模式 publish_tf=false
[ ] 无 mapOptimization 额外 odom -> lidar_link/laser_link TF
[ ] 连续运行至少 5 分钟无 TF 时间戳溢出/stale 问题
[ ] Smac Hybrid A* 能正常规划
[ ] NeuPAN 能正常获得 TF、/scan 和 /plan，并控制车辆
[ ] 地图错误、未初始化、错误初值均有可理解的失败行为
[ ] README / topic_info / 启动脚本已同步更新
```

---

# 15. 本次任务明确不做的事情

为了控制风险，Codex 不要顺手扩展任务范围：

```text
不重写 LIO-SAM mapping
不更换 SmacPlannerHybrid
不更换 NeuPAN
不修改 Ackermann controller 算法
不重新设计 map.pgm/map.yaml 生成流程
不引入新的 EKF 融合架构
不加入 Scan Context / FPFH / TEASER 全局重定位
不为了“大地图”重构 liorf 地图管理
不删除 robot_localization EKF 节点（只关闭其 TF ownership）
不主动调大 ICP threshold 掩盖初始化错误
```

如果完成基本迁移后要增加真正的自动全局重定位，应作为下一阶段独立任务。

---

# 16. 给 Codex 的直接执行指令

以下内容可作为实际执行约束：

> 在 `/home/young/Project/AckermannRobot` 中实施定位模块替换。读取 `/home/young/Project/liorf_localization` 作为 liorf 源码来源，但不要修改或破坏该来源仓库的用户工作树。先检查两个仓库的 branch、HEAD 和 dirty 状态；liorf 必须使用 `liorf_localization-ros2` 分支，如果来源工作树不便切换则创建临时 git worktree。先把 liorf vendoring 到 `AckermannRobot/src/liorf_localization` 并单独在 ROS2 Humble 下编译，通过后再做项目集成。不要一开始删除 `src/hdl_localization`。
>
> 保持现有 LIO-SAM 建图、GlobalMap.pcd、PCD->PGM、Smac Hybrid A*、NeuPAN、Ackermann controller 和 `/points_raw -> /scan` 链路不变。liorf 的 LiDAR 输入必须使用现有 `/points_lio`，IMU 使用 `/imu/data`。创建 `ackermann_bringup/config/liorf_localization.yaml`，传感器、frame、IMU noise 和 extrinsics 首先复用当前 `src/lio-sam/config/params.yaml` 中已验证值。
>
> 对 vendored liorf 做最小必要的 Humble/工程化 patch：新增 `globalmap_pcd` 绝对路径参数；修复 TransformFusion 中通过 `uint32_t` 构造纳秒时间的溢出问题；删除/禁用 mapOptimization 额外的 `odom -> lidar_link` TF；保证公共 `/odom` 是 `header.frame_id=odom`、`child_frame_id=base_link`；让 lidar/base 静态 TF 查询失败后可重试；推荐把 global map publisher 改成 Transient Local 并移除 `sleep(3)` 式等待。不要改变 liorf 的核心 scan-to-map/IMU preintegration 算法。
>
> 重写 `ackermann_bringup/launch/localization.launch.py`，由它启动 map->odom identity static TF 和 liorf 的三个可执行节点，不要使用 liorf 上游 launch 中自带的 RViz。修改 `navigation.launch.py` 接入新的 localization；修改 `navigation_sim.launch.py` 使 Gazebo 中 robot_localization 的 `publish_ekf_tf=false`，因为 `odom->base_link` 只能由 liorf TransformFusion 发布。修改 `ackermann_bringup/package.xml` 把 `hdl_localization` 依赖换成 `liorf_localization`。
>
> 新增 `scripts/nav_liorf_neupan.sh`，保持现有地图目录参数和 `GlobalMap.pcd/map.pgm/map.yaml` 检查逻辑。先完成独立定位测试、TF 测试、至少 5 分钟持续运行测试，再做完整 Smac+NeuPAN 导航测试。只有这些测试通过后才删除 `src/hdl_localization` 和过期依赖/文档。每个阶段都实际执行 colcon build，遇到编译错误时做最小兼容修复，并记录与上游 liorf 的差异。不要 reset/clean 用户未提交修改。

---

# 17. 迁移后的下一阶段建议（不属于本次实施）

当此次替换稳定后，再单独评估两个方向：

1. **全局重定位**：liorf 当前需要较合理的 `/initialpose`。如果需要机器人任意位置上电自动找回位置，可再增加 Scan Context / FPFH / TEASER / coarse GICP 等模块。
2. **大地图扩展**：上游 liorf README 对大场景能力有保守说明。若未来地图明显扩大，再考虑全局地图分块、按预测位姿裁剪局部先验地图、分层 KD-tree 等优化。

在 mini.world / 当前 Ackermann 仿真验证阶段，不应把这两项与基础替换混在一起。

