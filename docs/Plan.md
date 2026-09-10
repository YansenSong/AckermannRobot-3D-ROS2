# AckermannRobot-3D-ROS2

# 用 ROS2 Humble 官方 `nav2_smac_planner::SmacPlannerHybrid` 替换现有 `src/hybrid_astar_planner` 的执行计划

## 0. 任务目标

目标工作区：

```text
/home/young/Project/AckermannRobot
```

Navigation2 本地源码：

```text
/home/young/Project/navigation2
```

最终目标：

```text
删除：
AckermannRobot-3D-ROS2/src/hybrid_astar_planner

引入：
ROS2 Humble 官方 nav2_smac_planner

新增：
一个很薄的 Ackermann/ROS2 接口桥接节点

继续保持：
RViz /goal_pose
      ↓
SmacPlannerHybrid
      ↓
/plan
      ↓
NeuPAN
      ↓
Ackermann controller
```

本次工作是“替换全局规划器”，不是把整套 Nav2 Navigation Stack 引入项目。

不得替换 NeuPAN，不得替换现有 Ackermann controller，不得引入 Nav2 controller_server、BT navigator 等与本任务无关的组件。

---

# 1. 最重要的执行原则

## 1.1 不允许使用原 `src/hybrid_astar_planner` 中的规划参数作为新配置依据

原目录可以阅读，仅用于识别：

* 现有话题接口；
* 现有启动关系；
* 下游模块依赖；
* `/plan_path`；
* `/global_path_remaining_distance`；
* `/map`；
* RViz `/goal_pose`。

但禁止从原 planner 中抄取或参考：

```text
minimum_turning_radius
angle_quantization
max_iterations
max_on_approach_iterations
tolerance
allow_unknown
change_penalty
non_straight_penalty
reverse_penalty
cost_penalty
analytic_expansion_ratio
analytic_expansion_max_length
smoother 参数
```

新 Smac 参数的优先级必须是：

```text
第 1 优先级：
ROS2 Humble 官方 nav2_smac_planner 源码默认值

第 2 优先级：
Navigation2 官方文档对机器人几何参数的约束关系

第 3 优先级：
AckermannRobot-3D-ROS2 的 Xacro / ros2_control / TF 中的真实车辆参数

禁止：
从旧 src/hybrid_astar_planner 参数反推新参数
```

---

# 2. 第一阶段：版本检查，任何代码修改之前必须完成

项目明确运行：

```text
ROS2 Humble
Ubuntu 22.04
```

因此不能直接复制 Navigation2 当前 `main` 分支的 `nav2_smac_planner`。

当前 Navigation2 main 和 Humble 的接口已经不同。

执行：

```bash
PROJECT=/home/young/Project/AckermannRobot-3D-ROS2
NAV2=/home/young/Project/navigation2

git -C "$PROJECT" status --short
git -C "$PROJECT" rev-parse HEAD
git -C "$PROJECT" branch --show-current

git -C "$NAV2" status --short
git -C "$NAV2" rev-parse HEAD
git -C "$NAV2" branch --show-current

grep -A4 '<name>nav2_smac_planner' \
  "$NAV2/nav2_smac_planner/package.xml"
```

记录两个仓库的：

```text
branch
commit SHA
dirty/clean 状态
```

### 如果 `/home/young/Project/navigation2` 已经是 Humble-compatible

直接使用该源码。

Humble 版 `nav2_smac_planner/package.xml` 应属于 1.1.x 系列，而不是当前 main 的 1.5.x API。

### 如果 navigation2 当前位于 main / rolling / jazzy 等分支

不要直接修改或强制 checkout 一个有用户改动的工作树。

如果工作树干净，可以切换 Humble。

如果工作树不干净，创建独立 worktree：

```bash
git -C "$NAV2" fetch origin

git -C "$NAV2" worktree add \
  /tmp/navigation2-humble \
  origin/humble
```

之后所有 vendoring 操作从：

```text
/tmp/navigation2-humble/nav2_smac_planner
```

进行。

### 禁止

```text
为了让 main 版 Smac 在 Humble 编译，
手工删除 nav2_ros_common、
修改 GlobalPlanner API、
修改 createPlan() 函数签名。
```

正确方案是使用官方 Humble 源码，不是把新版源码人工降级。

---

# 3. 推荐的最终系统架构

不要重新实现一个 standalone `AStarAlgorithm<NodeHybrid>` wrapper。

采用下面的结构：

```text
                           map.yaml
                              │
                              ▼
                    ┌──────────────────┐
                    │ nav2_map_server  │
                    └────────┬─────────┘
                             │ /map
                             ▼
                ┌───────────────────────────┐
                │ nav2_planner             │
                │ PlannerServer            │
                │                          │
                │ ┌───────────────────────┐ │
                │ │ global_costmap        │ │
                │ │ StaticLayer           │ │
                │ │ InflationLayer        │ │
                │ └───────────┬───────────┘ │
                │             │             │
                │ ┌───────────▼───────────┐ │
                │ │ SmacPlannerHybrid     │ │
                │ │ official Humble code │ │
                │ └───────────┬───────────┘ │
                └─────────────┼─────────────┘
                              │
                              │ /plan
                              ▼
                           NeuPAN
                              │
                       /neupan_cmd_vel
                              ▼
                     existing cmd_vel_mux
                              ▼
              ackermann_steering_controller
```

用户接口：

```text
RViz /goal_pose
      │
      ▼
ackermann_smac_bridge
      │
      │ nav2_msgs/action/ComputePathToPose
      ▼
PlannerServer
```

同时：

```text
PlannerServer
   │
   ├── /plan ───────────────► NeuPAN
   │
   └── action result
            │
            ▼
   ackermann_smac_bridge
            │
            ├── /plan_path
            └── /global_path_remaining_distance
```

注意：

**不要让 bridge 再发布 `/plan`。**

Humble `PlannerServer` 自己已经发布：

```text
/plan
```

否则会造成 `/plan` 上出现两个 publisher。

---

# 4. 为什么必须保留 PlannerServer，而不是“只 new 一个 SmacPlannerHybrid”

官方 `SmacPlannerHybrid` 的接口依赖：

```text
nav2_core::GlobalPlanner
LifecycleNode
Costmap2DROS
TF buffer
pluginlib
```

configure 时会直接取得：

```cpp
_costmap = costmap_ros->getCostmap();

_collision_checker.setFootprint(
    costmap_ros->getRobotFootprint(),
    ...
);
```

因此 Smac 的很多核心优势本来就建立在 Nav2 Costmap 上：

```text
Cost-aware search
Obstacle heuristic
Inflation cost
SE(2) footprint collision
Costmap downsampling
Dynamic parameter system
官方 smoother
```

如果再次自己写 PGM adapter 并直接调用底层 A*，虽然“能跑”，但已经不是完整意义上的官方 SmacPlannerHybrid 行为。

因此本项目的策略是：

```text
只 vendor nav2_smac_planner 源码

但使用 ROS Humble 已安装的：
nav2_planner
nav2_costmap_2d
nav2_map_server
nav2_lifecycle_manager

作为运行时基础设施。
```

不要复制整套 Navigation2。

---

# 5. 检查系统 Nav2 运行时依赖

先：

```bash
source /opt/ros/humble/setup.bash

ros2 pkg prefix nav2_planner
ros2 pkg prefix nav2_costmap_2d
ros2 pkg prefix nav2_map_server
ros2 pkg prefix nav2_lifecycle_manager
ros2 pkg prefix nav2_core
ros2 pkg prefix nav2_util
```

如果存在，直接使用 `/opt/ros/humble` 的这些包。

如果缺包：

```text
不要自己重新实现 PlannerServer / Costmap。
报告缺失依赖。
优先安装与 Humble 匹配的 ROS2 Navigation2 二进制包。
```

---

# 6. Vendor 官方 Humble `nav2_smac_planner`

目标：

```text
AckermannRobot-3D-ROS2/src/nav2_smac_planner
```

彻底删除：

```text
AckermannRobot-3D-ROS2/src/hybrid_astar_planner
```

从 Humble-compatible Navigation2 工作树复制整个：

```text
nav2_smac_planner/
```

而不是只复制：

```text
a_star.cpp
node_hybrid.cpp
smac_planner_hybrid.cpp
```

整个 package 一起保留，包括：

```text
include/
src/
test/
lattice_primitives/
CMakeLists.txt
package.xml
smac_plugin_*.xml
```

原因：

保持官方代码结构，减少今后升级和代码对比困难。

### 原则

除非存在明确的 Humble build bug，否则：

```text
禁止修改 vendored nav2_smac_planner 算法源码。
```

所有机器人适配写到：

```text
ackermann_bringup/config/
ackermann_smac_bridge/
launch/
```

中。

建议新增：

```text
docs/nav2_smac_planner_upstream.md
```

记录：

```text
Upstream:
https://github.com/ros-navigation/navigation2

Branch:
humble

Commit:
<实际使用的 SHA>

Imported package:
nav2_smac_planner

Local algorithm modifications:
None
```

保留所有 Apache-2.0 copyright/license header。

---

# 7. 新增 `ackermann_smac_bridge`

新增 package：

```text
src/ackermann_smac_bridge
```

建议 C++ 实现，不使用 Python。

原因：

NeuPAN 已经需要特殊 Conda Python 环境，Smac bridge 不应再被 Python 环境污染。

依赖：

```text
rclcpp
rclcpp_action
nav2_msgs
nav_msgs
geometry_msgs
std_msgs
tf2
tf2_ros
```

节点名称建议：

```text
ackermann_smac_bridge
```

---

# 8. Bridge 必须承担的职责

## 8.1 接收 RViz Goal

订阅：

```text
/goal_pose
geometry_msgs/msg/PoseStamped
```

---

## 8.2 获取规划起点

TF：

```text
map -> rear_axle_link
```

Smac 规划状态统一采用：

```text
rear_axle_link
```

作为 Ackermann bicycle model 的参考点。

不要使用：

```text
base_link
```

作为 Smac 全局规划参考点后再直接把路径交给 NeuPAN。

NeuPAN 自己的 Ackermann state 已经是 rear-axle referenced，因此全局路径也应该与它一致。

---

## 8.3 调 PlannerServer

Action：

```text
/compute_path_to_pose
nav2_msgs/action/ComputePathToPose
```

发送：

```text
goal.start      = 当前 map -> rear_axle_link
goal.goal       = /goal_pose
goal.planner_id = "GridBased"
goal.use_start  = true
```

必须显式使用 rear axle start，避免 PlannerServer 和桥接节点使用不同机器人参考点。

---

# 9. `/goal_pose` 的语义

本次替换后明确规定：

```text
RViz /goal_pose 表示 rear_axle_link 的目标 Pose
```

这样：

```text
当前状态 reference
终点 reference
Smac footprint reference
NeuPAN state reference
```

全部统一为后轴中心。

不要在第一版中加入来自旧 planner 的：

```text
goal_pose_is_base_link
rear_axle_offset_x
```

逻辑。

如果以后确实希望 RViz 箭头表示 `base_link` 中心目标，再单独增加一个明确的“base_link goal → rear axle goal”转换功能。

第一版先避免隐式坐标语义转换。

---

# 10. Bridge 对规划结果的处理

PlannerServer 成功后：

```text
PlannerServer 自动发布 /plan
```

这直接供 NeuPAN 使用。

Bridge 从 Action result 中拿到同一条 Path：

```text
result.path
```

只额外发布：

```text
/plan_path
```

供：

```text
RViz
nav_status
```

使用。

### 非常重要

绝对不要修改：

```text
result.path.poses[i].pose.orientation
```

尤其禁止执行：

```text
yaw = atan2(y[i+1]-y[i], x[i+1]-x[i])
```

再覆盖 Smac 输出的 orientation。

Smac 在倒车段中：

```text
车辆 heading
```

与：

```text
轨迹移动方向
```

可以相反。

而 NeuPAN 正是利用这个关系识别：

```text
gear = -1
```

如果 bridge 把 orientation 重算成切线方向：

```text
Smac 的倒车信息将被破坏。
```

---

# 11. `/global_path_remaining_distance`

原 planner 还承担了一个与 Hybrid A* 算法无关、但系统需要的功能：

```text
/global_path_remaining_distance
std_msgs/msg/Float64
```

`nav_status_node` 使用该话题判断：

```text
MOVING -> ARRIVED
```

因此必须迁移到：

```text
ackermann_smac_bridge
```

而不是删除。

Bridge 保存最新成功的 Smac path：

```cpp
nav_msgs::msg::Path active_path_;
```

周期：

```text
10 Hz
```

查询：

```text
map -> rear_axle_link
```

将当前位置投影到最近的 path segment，然后计算：

```text
投影点 → path 终点
```

的累计弧长。

发布：

```text
/global_path_remaining_distance
```

这里可以重新实现同样的“沿路径剩余长度”数学定义，但不要把旧 planner 中任何调参值迁移过来。

---

# 12. 新目标与 Action 并发处理

Bridge 必须正确处理快速连续点击多个目标。

实现：

```text
goal generation counter
+
cancel previous action goal
```

逻辑：

```text
Goal A
  ↓
planning A

Goal B 到达
  ↓
cancel / invalidate A
  ↓
planning B

如果 A 的 result 晚到：
丢弃
```

新目标收到时：

```text
清空 active_path_
暂停发布 remaining distance
```

只有最新成功 path 才重新设置 active path。

规划失败：

```text
不要把旧路径当成当前新目标路径继续用于 remaining distance。
```

同时发布空 `/plan_path` 清除旧 RViz 路径。

---

# 13. `/plan` GetPlan Service

旧 standalone planner 同时提供了：

```text
/plan topic
/plan service
```

本次迁移之前，在本地仓库执行：

```bash
rg -n \
  'GetPlan|create_client.*plan|/plan' \
  /home/young/Project/AckermannRobot-3D-ROS2/src \
  /home/young/Project/AckermannRobot-3D-ROS2/scripts
```

已知 NeuPAN 使用的是：

```text
/plan topic
```

而不是 GetPlan service。

如果本地全文搜索确认没有消费者：

```text
删除旧 /plan GetPlan service，不重新实现。
```

不要为了历史兼容无意义地维护一套第二规划 API。

如果发现外部消费者，再用独立 compatibility wrapper 映射到 `ComputePathToPose`。

---

# 14. 车辆几何参数必须从 Xacro 重新推导

权威文件：

```text
src/ackermann_simulation/robot/xacro/chassis.xacro
```

不要使用旧 `hybrid_astar_planner` 参数。

Xacro 中：

```text
rear axle x = -0.30449 m
front steering axle x = +0.28930 m
```

因此真实轴距：

```text
L
= 0.28930 - (-0.30449)
= 0.59379 m
```

前轮中心横向位置：

```text
steering joint = ±0.163000
wheel offset   = ±0.091957
```

所以直行状态下前轮中心轮距：

```text
Tfront
= 2 × (0.163000 + 0.091957)
= 0.509914 m
```

Xacro 中两个 steering joint 的机械限位：

```text
-0.52 rad
+0.52 rad
```

ROS2 controller 配置中的：

```text
wheelbase ≈ 0.593
front_wheel_track ≈ 0.510
```

与 Xacro 基本一致。

算法参数应该使用 Xacro 精确值作为几何推导源。

---

# 15. Smac 最小转弯半径重新推导

不能简单使用：

```text
R = L / tan(0.52)
```

作为 rear axle center 的 Smac `minimum_turning_radius`。

因为：

```text
±0.52 rad
```

是左右实际前转向 joint 的硬限位，而 Smac 的 bicycle model 最小转弯半径对应车辆参考点/后轴中心的轨迹半径。

如果内侧轮达到最大：

```text
δ_inner = 0.52 rad
```

则：

```text
R_inner = L / tan(δ_inner)

R_center = R_inner + Tfront / 2
```

代入：

```text
L      = 0.59379
Tfront = 0.509914
δinner = 0.52
```

得到：

```text
L / tan(0.52)
≈ 1.03708 m

Tfront / 2
≈ 0.254957 m

Rrear-center
≈ 1.29203 m
```

因此第一版 Smac 配置：

```yaml
minimum_turning_radius: 1.292
```

不要使用旧 planner 的 1.05 m。

相应等效 bicycle steering angle：

```text
atan(0.59379 / 1.29203)
≈ 0.4308 rad
≈ 24.7°
```

---

# 16. 最小转弯半径必须做仿真验证

上述 1.292 m 是根据 Xacro joint limit 做的几何推导。

还必须用 ros2_control 验证实际 controller 语义。

项目当前控制链把 NeuPAN 的：

```text
[v, steering]
```

转换成：

```text
linear.x = v

angular.z =
v * tan(steering) / wheelbase
```

随后 `ackermann_steering_controller` 接收 body：

```text
linear velocity
angular velocity
```

并计算左右转向 joint command。

所以做一次低速稳态测试，例如：

```text
v = 0.3 m/s
R = 1.292 m
omega = v / R
```

检查：

```text
/joint_states
```

中：

```text
left_steering_joint
right_steering_joint
```

是否均满足：

```text
abs(angle) <= 0.52 rad
```

如果存在超限：

```text
增大 Smac minimum_turning_radius。
```

除非仿真数据能证明更小半径可被 controller 实际实现，否则不要降低 1.292 m。

---

# 17. Footprint 不得采用圆形 robot_radius

Smac Hybrid 应使用真实 polygon footprint：

```text
SE(2) polygon collision checking
```

不使用：

```text
robot_radius
```

Xacro 中：

```text
base_link body collision box:
0.7 × 0.3 m
```

但轮胎伸出车身，因此只用：

```text
0.7 × 0.3
```

作为 footprint 是不安全的。

还需要包含：

```text
rear wheels
front wheels
front wheels 在最大左右转角下的 swept envelope
steering link collision geometry
```

---

# 18. 第一版 footprint 初始值

按当前 Xacro 的 body + wheel collision primitives 推导，relative to：

```text
rear_axle_link
```

可以先采用保守矩形：

```yaml
footprint: "[[-0.10, -0.32],
              [ 0.74, -0.32],
              [ 0.74,  0.32],
              [-0.10,  0.32]]"
```

Nav2 Humble Costmap 默认：

```yaml
footprint_padding: 0.01
```

保留官方默认。

但 Codex 在写死该 footprint 前必须继续检查本地：

```text
chassis.xacro
steering_link collision mesh
front/rear wheel collision geometry
```

最好编写一个小型离线检查，比较：

```text
steering = 0
steering = maximum left
steering = maximum right
```

时全部 collision geometry 在 XY 平面上的包络。

如果任何 collision mesh 超出上述矩形：

```text
只允许向外扩大 footprint，
不能缩小。
```

不要使用视觉 mesh 的外观尺寸替代 collision geometry。

---

# 19. Global Costmap 策略

当前系统的全局规划本质上使用静态建图结果，而 NeuPAN 负责实时激光避障。

因此第一版不要突然把动态 `/scan` 加入 global costmap。

使用：

```text
StaticLayer
+
InflationLayer
```

即可。

这样既保留现有架构职责，又让 Smac 可以利用 cost-aware potential。

结构：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true

      global_frame: map
      robot_base_frame: rear_axle_link

      rolling_window: false

      resolution: <来自当前 map.yaml>

      track_unknown_space: true

      footprint: "[[-0.10,-0.32],
                    [0.74,-0.32],
                    [0.74,0.32],
                    [-0.10,0.32]]"

      footprint_padding: 0.01

      plugins:
        - static_layer
        - inflation_layer

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
```

第一版不要主动设置：

```text
inflation_radius
cost_scaling_factor
```

让 Humble `InflationLayer` 自己的源码默认值生效。

当前 Humble 源码默认：

```text
inflation_radius = 0.55
cost_scaling_factor = 10.0
```

等替换成功后，单独进行 cost-field 调参。

不要把“算法替换”和“costmap 性能调优”混成一次修改。

---

# 20. Map resolution

例如仓库当前：

```text
maps/mini/map.yaml
```

是：

```yaml
resolution: 0.05
```

但不要在程序中假设所有地图永远都是 0.05。

`planning.launch.py` 应继续读取：

```text
map.yaml
```

并将 resolution 传给 global costmap，或者让 StaticLayer 根据 map metadata 正确调整。

`map_pgm` 参数在新架构中不再是规划器必需参数。

为了不破坏：

```text
scripts/nav_hdl_neupan.sh
navigation_sim.launch.py
navigation.launch.py
```

第一版可以暂时继续接受：

```text
map_pgm
```

launch argument，但不再使用它。

后续可以单独清理。

---

# 21. Smac 参数基线：严格使用 Humble 源码默认值

新增：

```text
src/ackermann_bringup/config/smac_planner.yaml
```

Smac 部分以 Humble `smac_planner_hybrid.cpp` 中实际 declare default 为准，而不是旧项目参数，也不是网上随便找的 YAML。

第一版配置：

```yaml
planner_server:
  ros__parameters:
    use_sim_time: true

    planner_plugins:
      - GridBased

    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"

      downsample_costmap: false
      downsampling_factor: 1

      angle_quantization_bins: 72

      tolerance: 0.25

      allow_unknown: true

      max_iterations: 1000000

      max_on_approach_iterations: 1000

      smooth_path: true

      minimum_turning_radius: 1.292

      cache_obstacle_heuristic: false

      reverse_penalty: 2.0

      change_penalty: 0.0

      non_straight_penalty: 1.2

      cost_penalty: 2.0

      retrospective_penalty: 0.015

      analytic_expansion_ratio: 3.5

      analytic_expansion_max_length: 6.46

      max_planning_time: 5.0

      lookup_table_size: 20.0

      motion_model_for_search: "REEDS_SHEPP"

      smoother:
        tolerance: 1.0e-10
        max_iterations: 1000
        w_data: 0.2
        w_smooth: 0.3
        do_refinement: true
```

---

# 22. 哪几个参数允许因为 AckermannRobot 而修改

相对于官方 Humble Smac source default，仅允许第一版主动修改三个与本项目明确需求相关的值。

### `minimum_turning_radius`

官方默认：

```text
0.4 m
```

替换成根据 Xacro/Ackermann 几何重新推导的：

```text
1.292 m
```

### `motion_model_for_search`

官方源码默认：

```text
DUBIN
```

本项目明确允许倒车，NeuPAN 也能识别倒车方向，因此使用：

```text
REEDS_SHEPP
```

否则 Smac 不会规划倒车 manoeuvre。

### `analytic_expansion_max_length`

官方源码默认：

```text
3.0 m
```

官方 Smac 使用建议要求该长度结合 minimum turning radius 缩放，建议至少约：

```text
4～5 × minimum_turning_radius
```

采用：

```text
5 × 1.29203
≈ 6.460 m
```

所以第一版：

```yaml
analytic_expansion_max_length: 6.46
```

这是根据官方参数约束关系做的车辆适配，不是旧 planner 参数迁移。

---

# 23. 禁止迁移旧 planner 独有参数

如果旧 planner 里存在以下东西：

```text
gear_change_penalty
angle_quantization
motion_model
smooth_max_time
smooth_max_iterations
vehicle_length
vehicle_width
rear_axle_offset_x
goal_pose_is_base_link
```

不要机械地在新配置中寻找“一一对应”。

只使用官方 Smac 实际支持的参数。

尤其：

```text
angle_quantization
```

不要继续使用旧名字。

官方 Smac：

```text
angle_quantization_bins
```

---

# 24. Map Server

原 `hybrid_astar_planner` 自己发布 `/map`。

删除它后必须由：

```text
nav2_map_server
```

接管。

`planning.launch.py` 启动：

```text
map_server
```

并传：

```yaml
yaml_filename: <map launch argument>
use_sim_time: true
```

QoS 继续使用 transient-local，因此 RViz 和 StaticLayer 后启动仍可以收到地图。

---

# 25. Lifecycle Manager

启动：

```text
nav2_lifecycle_manager
```

只管理：

```text
map_server
planner_server
```

设置：

```text
autostart = true
```

不加入：

```text
controller_server
bt_navigator
behavior_server
waypoint_follower
```

生命周期顺序：

```text
map_server
→ planner_server
```

确保 StaticLayer 有地图可用。

---

# 26. 修改 `ackermann_bringup/launch/planning.launch.py`

当前 launch 中直接启动旧：

```text
hybrid_astar_planner_node
```

新的 `planning.launch.py` 应负责：

```text
1. 解析 map.yaml
2. 启动 nav2_map_server/map_server
3. 启动 nav2_planner/planner_server
4. 启动 nav2_lifecycle_manager/lifecycle_manager
5. 启动 ackermann_smac_bridge
```

参数文件：

```text
ackermann_bringup/config/smac_planner.yaml
```

map yaml 路径从原有：

```text
map
```

launch argument 获取。

继续支持：

```text
use_sim_time=true
```

---

# 27. `navigation.launch.py` 和 `navigation_sim.launch.py`

尽可能不改变它们对外 CLI。

现有：

```bash
ros2 launch ackermann_bringup navigation_sim.launch.py \
    map:=... \
    map_pgm:=... \
    globalmap_pcd:=...
```

第一版继续可用。

仅让：

```text
planning.launch.py
```

内部实现发生变化。

目标是：

```text
scripts/nav_hdl_neupan.sh
```

用户使用方式保持不变。

---

# 28. 修改 `ackermann_bringup/package.xml`

删除：

```xml
<exec_depend>hybrid_astar_planner</exec_depend>
```

新增必要 runtime dependencies：

```text
nav2_smac_planner
nav2_planner
nav2_map_server
nav2_lifecycle_manager
ackermann_smac_bridge
```

同时 bridge package 自己声明：

```text
nav2_msgs
rclcpp
rclcpp_action
tf2
tf2_ros
nav_msgs
geometry_msgs
std_msgs
```

---

# 29. NeuPAN 配置原则

下面这些现有 NeuPAN 配置保持：

```text
base_frame: rear_axle_link

plan_input_topic: /plan

refresh_initial_path: true

include_initial_path_direction: true

direct_goal_planning: false
```

不要为了 Smac 修改它们。

新的路径数据流应该自然满足现有 NeuPAN。

尤其：

```text
include_initial_path_direction=true
```

必须保留。

这是 Reeds-Shepp 倒车路径能够正确进入 NeuPAN 的关键。

---

# 30. `nav_status` 接口保持兼容

当前 nav_status 依赖：

```text
goal_topic:
  /goal_pose

plan_topic:
  /plan_path

remaining_distance_topic:
  /global_path_remaining_distance
```

因此 bridge 必须继续提供：

```text
/plan_path

/global_path_remaining_distance
```

不需要修改 nav_status。

---

# 31. 需要修改/新增/删除的文件

最终预期：

```text
DELETE
src/hybrid_astar_planner/**

ADD
src/nav2_smac_planner/**
    # 来自 Navigation2 Humble，尽量零修改

ADD
src/ackermann_smac_bridge/
├── CMakeLists.txt
├── package.xml
├── include/...
└── src/
    └── ackermann_smac_bridge.cpp

ADD
src/ackermann_bringup/config/smac_planner.yaml

ADD
docs/nav2_smac_planner_upstream.md

MODIFY
src/ackermann_bringup/launch/planning.launch.py

MODIFY
src/ackermann_bringup/package.xml

OPTIONAL DOCUMENTATION UPDATE
README.md
scripts/nav_hdl_neupan.sh
```

如果 `/plan_path` 保持不变：

```text
RViz config 不需要因为 planner 替换而改变。
```

---

# 32. 不要直接修改这些模块

除非编译证明存在接口问题，否则不要修改：

```text
src/neupan_ros2/**
src/ackermann_control/**
src/ackermann_simulation/**
src/hdl_localization/**
src/lio-sam/**
src/nav_status/**
```

特别是：

```text
neupan_node.py
planner.yaml
robot.yaml
cmd_vel_mux.py
ackermann_controllers.yaml
```

不是这次替换的目标。

---

# 33. Build 前清理旧 package 缓存

删除源码 package 后，旧 install/build 中仍可能残留：

```text
hybrid_astar_planner
```

需要清理对应 build/install 缓存。

避免无脑删除整个工作区。

例如：

```bash
cd /home/young/Project/AckermannRobot-3D-ROS2

rm -rf build/hybrid_astar_planner
rm -rf install/hybrid_astar_planner

rm -rf build/nav2_smac_planner
rm -rf install/nav2_smac_planner

rm -rf build/ackermann_smac_bridge
rm -rf install/ackermann_smac_bridge
```

然后：

```bash
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-up-to \
  nav2_smac_planner \
  ackermann_smac_bridge \
  ackermann_bringup
```

之后：

```bash
source install/setup.bash
```

---

# 34. 确认实际加载的是工作区内的 Smac

执行：

```bash
ros2 pkg prefix nav2_smac_planner
```

必须优先得到类似：

```text
/home/young/Project/AckermannRobot-3D-ROS2/install/nav2_smac_planner
```

而不是：

```text
/opt/ros/humble
```

否则说明 vendored package 没有进入 overlay。

---

# 35. 启动后检查生命周期

执行：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
```

期望：

```text
active
active
```

然后：

```bash
ros2 action list
```

必须出现：

```text
/compute_path_to_pose
```

---

# 36. 检查 Planner plugin

执行：

```bash
ros2 param get /planner_server planner_plugins
```

应包含：

```text
GridBased
```

再：

```bash
ros2 param get /planner_server GridBased.motion_model_for_search
ros2 param get /planner_server GridBased.minimum_turning_radius
ros2 param get /planner_server GridBased.angle_quantization_bins
```

期望：

```text
REEDS_SHEPP
1.292
72
```

---

# 37. 检查 `/plan` publisher 数量

没有 NeuPAN 时：

```bash
ros2 topic info /plan -v
```

期望：

```text
Publisher:
planner_server

不要出现：
ackermann_smac_bridge 同时发布 /plan
```

启动 NeuPAN 后：

```text
Subscriber:
neupan_node
```

必须出现。

---

# 38. 检查 `/map`

原 planner 被删后：

```bash
ros2 topic info /map -v
```

publisher 应变成：

```text
map_server
```

而不是旧：

```text
hybrid_astar_planner
```

RViz 必须正常显示地图。

---

# 39. 检查 global costmap

确认：

```text
/global_costmap/costmap
/global_costmap/published_footprint
```

正常存在。

RViz 添加：

```text
Map /global_costmap/costmap
Polygon /global_costmap/published_footprint
```

检查 footprint：

```text
位置以 rear_axle_link 为原点
覆盖车体
覆盖四个轮子
随车体旋转时不穿越障碍
```

---

# 40. 最重要的 Reeds-Shepp / 倒车验证

设计一个无法仅靠向前运动轻松完成的目标姿态。

获取：

```text
/plan
```

检查相邻点：

```text
segment direction
=
atan2(y[i+1]-y[i], x[i+1]-x[i])

vehicle heading
=
yaw(path[i].orientation)
```

对于倒车段应存在：

```text
cos(vehicle_heading - segment_direction) < 0
```

这正是 NeuPAN 识别：

```text
gear = -1
```

的条件。

如果 Smac 明明倒车，但该条件不存在：

```text
优先检查是否有 bridge / smoother 错误覆盖 path orientation。
```

不要修改 NeuPAN gear detection。

---

# 41. Smoother 验证

Smac 官方 smoother 保持开启：

```yaml
smooth_path: true
```

需要确认 smoother 后：

```text
车辆 heading 信息仍正确；
倒车段仍能被 NeuPAN 正确识别；
没有把 Reeds-Shepp cusp 错误平滑掉。
```

分别对比：

```text
/unsmoothed_plan
/plan
```

如果某个极端场景中 smoother 破坏换向信息，先形成可重复 testcase，再决定是否针对该场景关闭 smoother。

不要因为担心就直接默认关闭官方 smoother。

---

# 42. 最小转弯半径仿真验收

使用无障碍区域生成接近最大曲率的 Smac 路径。

统计路径离散曲率：

```text
κ
```

期望：

```text
|κ|max <= 1 / 1.292
≈ 0.774 1/m
```

同时观察：

```text
left_steering_joint
right_steering_joint
```

要求：

```text
|joint position| <= 0.52 rad
```

如果路径能规划但 controller 无法实现：

```text
增加 minimum_turning_radius。
```

Smac 参数应该服从实际车辆，而不是为了让规划更容易而违反转向极限。

---

# 43. NeuPAN 集成验收

启动原有工作流：

终端 1：

```bash
cd /home/young/Project/AckermannRobot-3D-ROS2

bash scripts/nav_hdl_neupan.sh maps/mini
```

终端 2：

```bash
cd /home/young/Project/AckermannRobot-3D-ROS2

bash scripts/run_neupan.sh
```

RViz：

```text
1. 2D Pose Estimate
2. 等待 hdl localization 稳定
3. 2D Goal Pose
```

数据链应为：

```text
/goal_pose
      ↓
ackermann_smac_bridge
      ↓
/compute_path_to_pose
      ↓
PlannerServer
      ↓
SmacPlannerHybrid
      ↓
/plan
      ↓
NeuPAN
      ↓
/neupan_cmd_vel
      ↓
cmd_vel_mux
      ↓
ackermann_steering_controller
```

---

# 44. nav_status 验收

新 goal：

```text
WAITING_FOR_GOAL
→ PLANNING
```

收到 `/plan_path`：

```text
PLANNING
→ MOVING
```

车辆接近终点，且：

```text
/global_path_remaining_distance
```

持续正确下降。

停车后：

```text
MOVING
→ ARRIVED
```

如果状态一直停留在 MOVING：

优先检查 bridge 的：

```text
active_path
map -> rear_axle_link TF
remaining distance calculation
```

而不是调整 Smac。

---

# 45. 第一版不要加入动态 ObstacleLayer

当前系统设计：

```text
Smac：
静态全局路径

NeuPAN：
LaserScan 实时局部避障
```

因此第一版 global costmap 只用：

```text
StaticLayer
InflationLayer
```

不要同时让 Smac 对 `/scan` 动态重规划。

否则会同时改变：

```text
全局规划策略
局部避障策略
```

很难判断迁移问题来自哪里。

后续如果需要可以单独做：

```text
Smac dynamic obstacle global replan
```

实验。

---

# 46. 第一版禁止做性能调参

替换成功之前禁止随意调整：

```text
reverse_penalty
change_penalty
non_straight_penalty
cost_penalty
retrospective_penalty
angle_quantization_bins
cache_obstacle_heuristic
inflation_radius
cost_scaling_factor
```

首先建立：

```text
“官方 Humble baseline + 车辆几何适配”
```

然后再另开 commit 做：

```text
AckermannRobot-specific tuning
```

否则后续无法区分：

```text
官方算法效果
vs
人工调参效果
```

---

# 47. 第一版成功后的第二阶段调优候选项

本次替换完成并验收后，才可以考虑：

```text
cache_obstacle_heuristic: true

更宽的 inflation potential field

cost_penalty 调优

reverse_penalty 调优

analytic_expansion_ratio 调优

angle_quantization_bins 对速度/质量影响

不同 minimum_turning_radius 的实车裕量
```

每次改变一个变量并记录：

```text
planning time
expanded nodes
path length
minimum clearance
max curvature
reverse distance
gear switch count
NeuPAN tracking error
```

这些不属于第一版替换任务。

---

# 48. 建议提交顺序

建议拆成四个 commit。

### Commit 1

```text
vendor Humble nav2_smac_planner
```

只做：

```text
删除旧 hybrid_astar_planner
引入官方 nav2_smac_planner
记录 upstream SHA
```

不要混入项目适配。

### Commit 2

```text
add Ackermann Smac goal bridge
```

加入：

```text
/goal_pose
ComputePathToPose
/plan_path
remaining distance
rear_axle TF
```

### Commit 3

```text
integrate PlannerServer and global costmap
```

修改：

```text
planning.launch.py
smac_planner.yaml
ackermann_bringup/package.xml
```

### Commit 4

```text
tests and documentation
```

加入：

```text
build verification
topic verification
Reeds-Shepp direction test
turning-radius test
README update
```

---

# 49. 最终验收条件

只有全部满足才认为替换完成：

```text
[ ] src/hybrid_astar_planner 已不存在

[ ] src/nav2_smac_planner 来自 Navigation2 Humble-compatible 源码

[ ] vendored Smac 算法源码没有 AckermannRobot-specific 魔改

[ ] ros2 pkg prefix nav2_smac_planner 指向本工作区 overlay

[ ] map_server active

[ ] planner_server active

[ ] /compute_path_to_pose active

[ ] Planner plugin 为 nav2_smac_planner/SmacPlannerHybrid

[ ] motion_model_for_search = REEDS_SHEPP

[ ] minimum_turning_radius 从 Xacro/控制器几何重新推导

[ ] 未引用旧 hybrid_astar_planner 参数

[ ] global costmap reference frame = rear_axle_link

[ ] 使用 polygon footprint 而不是 robot_radius

[ ] /map 正常

[ ] /goal_pose 正常

[ ] /plan 只有 PlannerServer 发布

[ ] NeuPAN 正常订阅 /plan

[ ] /plan_path 正常供 RViz/nav_status 使用

[ ] /global_path_remaining_distance 正常

[ ] Smac path pose orientation 未被 bridge 重写

[ ] Reeds-Shepp 倒车段可以被 NeuPAN 检测为 gear=-1

[ ] 最大曲率满足车辆转向限制

[ ] 左右 steering joint 均不超过 ±0.52 rad

[ ] nav_status 可以 PLANNING -> MOVING -> ARRIVED

[ ] 原 scripts/nav_hdl_neupan.sh 用户操作流程仍然有效
```

---

# 50. Codex 必须遵守的禁止事项

不要：

```text
1. 使用 Navigation2 main 版源码硬改到 Humble。

2. 从旧 src/hybrid_astar_planner 复制规划参数。

3. 重新写一套自定义 AStarAlgorithm/PGMMapAdapter 来代替官方
   PlannerServer + Costmap2DROS。

4. 修改官方 vendored Smac 搜索算法来适配机器人。

5. 使用 base_link 规划路径然后不转换就直接交给 rear-axle-based NeuPAN。

6. 把 Smac 路径 orientation 重算成路径切线。

7. 用圆形 robot_radius 代替车辆 polygon footprint。

8. 第一版就加入动态 obstacle layer。

9. 第一版就做 penalty / heuristic / inflation 性能调参。

10. 修改 NeuPAN、Ackermann controller 或定位系统来“迁就”Smac。

11. 在发现编译 API 不兼容后自行删除接口；
    先检查是不是错误使用了非 Humble Navigation2 源码。

12. 覆盖用户两个 Git 仓库中已有未提交修改。
```

---

# 51. Codex 执行结束时必须给出的报告

完成后不要只说“build successful”。

请输出：

```text
1. AckermannRobot repo 原始 commit
2. Navigation2 source commit
3. 使用的 Navigation2 branch
4. vendored nav2_smac_planner commit provenance
5. 修改文件列表
6. 删除文件列表
7. 最终 Smac 参数 dump
8. 车辆几何推导结果
9. 最终 footprint
10. minimum_turning_radius 的推导/验证
11. colcon build 结果
12. lifecycle 状态
13. /plan publisher/subscriber 信息
14. /compute_path_to_pose 状态
15. 至少一次 forward planning 测试结果
16. 至少一次 reverse/Reeds-Shepp 测试结果
17. steering joint 最大实测角
18. 是否通过全部验收条件
19. 尚未解决的问题
```

如果某一步因本地环境、依赖或运行时问题无法验证，要明确写：

```text
NOT VERIFIED
```

不要猜测为成功。

---

# 52. 最终目标状态

替换完成后的代码职责应该非常清晰：

```text
navigation2/nav2_smac_planner
    │
    └── 纯官方 Hybrid A* 算法
        不含本机器人私有逻辑

ackermann_bringup
    │
    ├── map server
    ├── PlannerServer
    ├── global costmap
    └── Smac 参数配置

ackermann_smac_bridge
    │
    ├── /goal_pose -> ComputePathToPose
    ├── /plan_path
    └── remaining path distance

neupan_ros2
    │
    └── /plan -> 局部轨迹优化 / 避障 / 控制

ackermann_control
    │
    └── 实际 Ackermann ros2_control

ackermann_simulation
    │
    └── 唯一车辆几何真值来源之一
```

这样以后如果要研究：

```text
Improved Smac Hybrid A*
Safe Corridor
Voronoi heuristic
new cost function
new analytic expansion
trajectory optimization
```

可以从一套干净的：

```text
official Smac baseline
```

继续改，而不是继续维护现有 standalone Hybrid A* 的历史代码。
