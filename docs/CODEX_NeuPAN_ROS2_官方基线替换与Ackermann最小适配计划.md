# Codex 执行计划：以官方 NeuPAN ROS2 重新替换并最小化适配 AckermannRobot

> **本文件用于直接交付给 Codex 执行。**
>
> 任务性质：源码迁移 / vendor reset / 最小适配  
> 目标项目：`/home/young/Project/AckermannRobot`  
> 官方 NeuPAN ROS2 本地克隆：`/home/young/Project/neupan_ros2`  
> 目标包：`/home/young/Project/AckermannRobot/src/neupan_ros2`
>
> 核心要求：
>
> 1. 用本地官方仓库 `/home/young/Project/neupan_ros2/src/neupan_ros2` **重新替换**项目中的 `src/neupan_ros2`。
> 2. **必须原样保留**项目现有 `src/neupan_ros2/config/dune_checkpoint` 中的所有权重文件。
> 3. 不把旧 `src/neupan_ros2` 中的大量自定义逻辑重新合并回来。
> 4. 以官方 `KevinLADLee/neupan_ros2` 为基线，只施加当前项目确实必需的 Ackermann / Smac 集成适配。
> 5. 本任务**不重置、不删除、不重写** `third_party/NeuPAN`；NeuPAN 核心回归官方必须另开任务。
> 6. 不修改当前 SmacPlannerHybrid / liorf / 导航状态机架构，除非为了连接新的 NeuPAN ROS2 接口做极小改动。
> 7. 不 `git push`，不自动提交，不修改远端仓库。
>
> 2026-09-10 对 GitHub 公共仓库的参考快照：
>
> - AckermannRobot-3D-ROS2：`5d6fb60666bf2ff9043c705d37993b7dd75a53d7`
> - KevinLADLee/neupan_ros2：`4ffb7ec2dc45ff7ee9024f64083813237906af98`
>
> **但 Codex 执行时必须以两个本地目录的实时内容为最终依据。不要因为本文件记录了 SHA 就强制 checkout。**

---

# 0. 任务目标与最终架构

迁移完成后目标链路：

```text
RViz /goal_pose
      │
      ▼
ackermann_smac_bridge
      │
      ▼
Nav2 PlannerServer + SmacPlannerHybrid
      │
      │ /plan
      ▼
官方 NeuPAN ROS2 wrapper
      │
      │ /neupan_cmd_vel_raw
      │ Twist.linear.x  = v
      │ Twist.angular.z = NeuPAN steering angle ψ
      ▼
ackermann_control/neupan_ackermann_adapter.py
      │
      │ /neupan_cmd_vel
      │ Twist.linear.x  = v
      │ Twist.angular.z = body yaw rate ω
      ▼
现有 cmd_vel_mux.py
      │
      ▼
/ackermann_steering_controller/reference
      │
      ▼
Gazebo Ackermann robot
```

其中：

```text
ω = v * tan(ψ) / wheelbase
wheelbase = 0.593 m
```

必须保持当前全局规划职责：

```text
SmacPlannerHybrid = 全局规划
NeuPAN           = 路径跟踪 + 局部避障 + 局部控制
```

不要让 NeuPAN 再直接接管 `/goal_pose` 并生成第二条全局路径。

---

# 1. 明确本任务“要改”和“不要改”

## 1.1 本任务允许修改

```text
src/neupan_ros2/
src/ackermann_control/CMakeLists.txt
src/ackermann_control/neupan_ackermann_adapter.py        # 新增
src/ackermann_bringup/launch/navigation.launch.py
scripts/run_neupan.sh                                    # 仅必要诊断/路径调整
docs/neupan_ros2_upstream.md                             # 新增
docs/neupan_ros2_migration_report.md                     # 执行完新增
```

如果确有必要，可新增：

```text
src/neupan_ros2/launch/ackermann_robot.launch.py
```

但不要为了“看起来完整”去大改其它 launch。

---

## 1.2 本任务禁止修改

除非出现明确 build blocker，否则不要修改：

```text
src/nav2_smac_planner/
src/ackermann_smac_bridge/
src/liorf_localization/
src/nav_status/
src/ackermann_simulation/
third_party/NeuPAN/
```

尤其禁止：

```text
删除 third_party/NeuPAN
切换到 pip NeuPAN
更新 NeuPAN 核心
重新实现 PAN / DUNE / NRMP
把旧 avoidance seed 重新移植进 ROS2 wrapper
把旧 scan stale / TF compensation 整套代码重新移植回来
把旧 command rate limiter 重新移植进 neupan_node.py
```

本次的目的不是“把旧功能重新拼回去”，而是先建立**接近官方的可解释基线**。

---

# 2. 当前项目现状：Codex 执行前必须重新确认

根据本计划编写时的项目状态，当前项目已经是：

```text
liorf localization
      ↓
Nav2 PlannerServer
      ↓
SmacPlannerHybrid (REEDS_SHEPP)
      ↓
/plan
      ↓
NeuPAN
```

当前 `smac_planner.yaml` 使用：

```text
robot_base_frame: rear_axle_link
motion_model_for_search: REEDS_SHEPP
minimum_turning_radius: 1.320 m
```

当前 `ackermann_smac_bridge`：

```text
接收 /goal_pose
调用 /compute_path_to_pose
保留 Smac Path 中原始 pose orientation
不重新计算 reverse segment 的 yaw
```

因此迁移 NeuPAN 时：

**不得破坏 `/plan` 中 Smac 原始 orientation。**

---

# 3. Phase 0：Preflight，任何删除操作前必须执行

Codex 必须首先运行：

```bash
set -euo pipefail

PROJECT=/home/young/Project/AckermannRobot
UPSTREAM=/home/young/Project/neupan_ros2
TARGET="$PROJECT/src/neupan_ros2"
SOURCE="$UPSTREAM/src/neupan_ros2"

test -d "$PROJECT/.git"
test -d "$UPSTREAM/.git"
test -d "$TARGET"
test -d "$SOURCE"

echo "=== TARGET PROJECT ==="
git -C "$PROJECT" branch --show-current
git -C "$PROJECT" rev-parse HEAD
git -C "$PROJECT" status --short

echo "=== UPSTREAM NEUPAN_ROS2 ==="
git -C "$UPSTREAM" branch --show-current
git -C "$UPSTREAM" rev-parse HEAD
git -C "$UPSTREAM" status --short
```

## 3.1 上游仓库规则

如果：

```bash
git -C "$UPSTREAM" status --short
```

非空：

**停止替换。**

不要：

```text
stash
reset
checkout
clean
git pull
```

不要擅自“修复”用户的官方 clone。

报告：

```text
官方 clone 不是 clean worktree，无法确认复制的是官方基线。
```

---

## 3.2 项目仓库规则

项目仓库允许有其它未提交修改，但：

- 不得 revert 用户其它修改；
- 不得 `git reset --hard`；
- 不得 `git clean -fd`；
- 不得覆盖与本任务无关的 dirty 文件。

把执行前状态保存：

```bash
mkdir -p /tmp/neupan_rebase_preflight

git -C "$PROJECT" status --short \
  > /tmp/neupan_rebase_preflight/project_status_before.txt

git -C "$PROJECT" diff \
  > /tmp/neupan_rebase_preflight/project_diff_before.patch

git -C "$UPSTREAM" status --short \
  > /tmp/neupan_rebase_preflight/upstream_status.txt
```

---

# 4. Phase 1：对旧 NeuPAN 做完整安全备份

这是**强制步骤**。

生成时间戳备份：

```bash
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="/home/young/Project/AckermannRobot_neupan_backup_$STAMP"

mkdir -p "$BACKUP"

cp -a "$TARGET" "$BACKUP/neupan_ros2_old"
```

同时备份后面允许修改的外围文件：

```bash
mkdir -p "$BACKUP/peripheral"

cp -a "$PROJECT/src/ackermann_control/CMakeLists.txt" \
  "$BACKUP/peripheral/"

cp -a "$PROJECT/src/ackermann_control/cmd_vel_mux.py" \
  "$BACKUP/peripheral/"

cp -a "$PROJECT/src/ackermann_bringup/launch/navigation.launch.py" \
  "$BACKUP/peripheral/"

cp -a "$PROJECT/scripts/run_neupan.sh" \
  "$BACKUP/peripheral/"
```

---

# 5. Phase 2：权重目录必须字节级保留

目标目录：

```text
$TARGET/config/dune_checkpoint
```

本计划编写时 GitHub 中能看到：

```text
scout_model_5000.pth
scout_model_5000_v2.pth
```

但执行时不要假设只有这两个文件。

先完整枚举：

```bash
find "$TARGET/config/dune_checkpoint" \
  -type f -print | sort
```

再保存 hash：

```bash
find "$TARGET/config/dune_checkpoint" \
  -type f -print0 \
  | sort -z \
  | xargs -0 sha256sum \
  > "$BACKUP/dune_checkpoint.sha256"
```

独立复制：

```bash
cp -a \
  "$TARGET/config/dune_checkpoint" \
  "$BACKUP/dune_checkpoint"
```

同时记录旧 `src/neupan_ros2` 内所有 `.pth`，避免误丢：

```bash
find "$TARGET" -type f -name '*.pth' -print | sort \
  > "$BACKUP/all_old_neupan_weights.txt"
```

注意：

> 用户明确要求最终必须保留的是
> `src/neupan_ros2/config/dune_checkpoint`。
>
> 旧的 `config/robots/ackermann_robot/models/...` 不属于“必须原位保留”的目录。
> 它已经存在完整的 `$BACKUP/neupan_ros2_old` 备份，因此不要把它偷偷混进新的官方 baseline。

---

# 6. Phase 3：真正替换 `src/neupan_ros2`

执行方式应当是：

```text
删旧包
复制官方包
恢复 dune_checkpoint
```

而不是：

```text
rsync 新旧混合
```

这样才能避免旧自定义文件残留。

执行：

```bash
rm -rf "$TARGET"

cp -a "$SOURCE" "$TARGET"

mkdir -p "$TARGET/config"

cp -a \
  "$BACKUP/dune_checkpoint" \
  "$TARGET/config/dune_checkpoint"
```

立刻验证权重：

```bash
find "$TARGET/config/dune_checkpoint" \
  -type f -print0 \
  | sort -z \
  | xargs -0 sha256sum \
  > "$BACKUP/dune_checkpoint_after_copy.sha256"

diff -u \
  "$BACKUP/dune_checkpoint.sha256" \
  "$BACKUP/dune_checkpoint_after_copy.sha256"
```

**只有 diff 为空才继续。**

---

# 7. Phase 4：先验证“纯官方包”能够构建

此时：

```text
不要先创建 ackermann_robot
不要先改 neupan_node.py
```

先确认官方包本身能在当前工作区构建。

建议清除旧 NeuPAN 构建产物：

```bash
rm -rf \
  "$PROJECT/build/neupan_ros2" \
  "$PROJECT/install/neupan_ros2"
```

然后：

```bash
cd "$PROJECT"

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select neupan_ros2
```

如果失败：

1. 先判断是 Python/ROS 环境还是源码问题；
2. 不得立即把旧 `neupan_node.py` 抄回来；
3. 不得为了通过编译修改算法逻辑；
4. 在 migration report 中记录完整错误。

纯官方包 build 成功后再继续。

---

# 8. Phase 5：建立新的 AckermannRobot 配置

从官方 Ranger 复制，而不是从旧 ackermann config 合并：

```bash
cp -a \
  "$TARGET/config/robots/ranger" \
  "$TARGET/config/robots/ackermann_robot"
```

这一步会得到官方 Ranger：

```text
robot.yaml
planner.yaml
models/dune_model_5000.pth
```

**第一版使用官方 Ranger checkpoint 作为 baseline。**

用户原有：

```text
config/dune_checkpoint/
```

继续保留，但第一轮不要自动切换过去。

原因：

```text
先确定“官方 ROS2 + 官方 Ackermann 示例 DUNE”的行为，
再做单变量 checkpoint A/B。
```

---

# 9. Phase 6：新的 `robot.yaml` 只做接口必要修改

目标：

```text
src/neupan_ros2/config/robots/ackermann_robot/robot.yaml
```

以官方 Ranger `robot.yaml` 为模板。

建议最终内容保持官方字段集合，不迁移旧自定义字段。

关键修改：

```yaml
neupan_node:
  ros__parameters:
    use_sim_time: true

    robot_type: 'ackermann_robot'
    robot_description: 'AckermannRobot Gazebo simulation'

    planner_config_file: 'planner.yaml'
    dune_checkpoint_file: 'models/dune_model_5000.pth'

    map_frame: 'map'
    base_frame: 'rear_axle_link'
    lidar_frame: 'laser_link'

    enable_visualization: true
    enable_dune_markers: true
    enable_nrmp_markers: true
    enable_robot_marker: true
    marker_size: 0.05
    marker_z: 1.0

    scan_angle_max: 3.14
    scan_angle_min: -3.14
    scan_downsample: 1

    # 机器人仿真 LiDAR 的物理最小距离
    scan_range_min: 0.15

    # 第一轮尽量保持官方 Ranger 感知范围
    scan_range_max: 5.0

    flip_angle: false
    refresh_initial_path: true

    # 后续通过最小 reverse gear patch 使用 Smac pose orientation
    include_initial_path_direction: true

    # 第一轮保持官方
    control_frequency: 50.0

    # NeuPAN 原始 Ackermann 控制量，不直接送 controller
    cmd_vel_topic: '/neupan_cmd_vel_raw'

    scan_topic: '/scan'
    plan_input_topic: '/plan'

    # 当前全局 goal 由 Smac bridge 处理；隔离 NeuPAN 自带 goal callback
    goal_topic: '/neupan_unused_goal'

    plan_output_topic: '/neupan_plan'
    ref_state_topic: '/neupan_ref_state'
    initial_path_topic: '/neupan_initial_path'
    dune_markers_topic: '/dune_point_markers'
    robot_marker_topic: '/robot_marker'
    nrmp_markers_topic: '/nrmp_point_markers'
```

---

# 10. 禁止把旧 `robot.yaml` 自定义字段迁回来

第一版不要出现：

```text
odom_frame
scan_tf_max_age
scan_data_timeout
direct_goal_planning
command_rate_limit
executor_threads
```

这些属于旧 fork 行为。

当前目标：

```text
官方 node + 标准配置
```

不是重建旧 node。

如果后续 runtime 证明某一个功能确实必需，再作为独立 patch 添加。

---

# 11. Phase 7：`planner.yaml` 从官方 Ranger 开始，只改硬件/运动学必须项

目标：

```text
src/neupan_ros2/config/robots/ackermann_robot/planner.yaml
```

第一轮建议：

```yaml
# MPC
receding: 15
step_time: 0.2

# 必须适配当前机器人的真实速度能力
ref_speed: 0.5

device: 'cpu'
time_print: False
collision_threshold: 0.01

robot:
  kinematics: 'acker'

  # 当前车轮速度上限决定不能继续使用官方 Ranger 2.0 m/s
  #
  # 第二维为 NeuPAN bicycle steering angle，不是实体内轮 joint angle。
  # 当前 Smac Rmin=1.320 m、wheelbase=0.593 m：
  # atan(0.593 / 1.320) ≈ 0.422 rad
  max_speed: [0.7, 0.422]

  # 第一轮保持官方 Ranger
  max_acce: [1.0, 0.328]

  # 第一轮使用 Ranger DUNE checkpoint，因此先保留 Ranger footprint
  length: 0.720
  width: 0.500

  # 必须适配当前机器人
  wheelbase: 0.593

ipath:
  interval: 0.03
  curve_style: 'dubins'

  # 与当前 Smac / 实车可执行最小半径统一。
  # 在外部 /plan 模式下它主要是 fallback，但不留下明显错误物理值。
  min_radius: 1.320

  loop: False
  arrive_threshold: 0.5
  close_threshold: 0.05
  arrive_index_threshold: 3

pan:
  iter_num: 2
  dune_max_num: 200
  nrmp_max_num: 10
  dune_checkpoint: None
  iter_threshold: 0.1

adjust:
  # 第一轮全部保持官方 Ranger
  q_s: 0.1
  p_u: 0.5
  eta: 15.0
  d_max: 0.1
  d_min: 0.01
```

---

# 12. 第一轮 planner 禁止重新加入的参数

不要迁移：

```text
avoidance_seed_enabled
avoidance_seed_distance
avoidance_seed_clearance
avoidance_seed_steer
min_speed
ind_range: 0
```

说明：

- `third_party/NeuPAN` 当前可能仍支持 `min_speed` 和 avoidance seed；
- 本任务不删除这些核心扩展；
- 但 baseline YAML **不启用**它们；
- 如果 `min_speed` 不写，当前本地核心应回落为对称速度边界；
- 后续要验证非对称倒车限速时，再单独做实验。

---

# 13. 当前速度/转向适配依据

Codex 执行时必须从本地源码重新核对，不可只信本文数字。

检查：

```text
src/ackermann_simulation/robot/xacro/chassis.xacro
src/ackermann_simulation/robot/xacro/control.xacro
src/ackermann_control/config/ackermann_controllers.yaml
src/ackermann_bringup/config/smac_planner.yaml
```

至少确认：

```text
controller wheelbase ≈ 0.593 m
wheel radius ≈ 0.093 m
rear wheel command max ≈ 10 rad/s
steering joint limit ≈ ±0.52 rad
Smac minimum_turning_radius ≈ 1.320 m
```

若本地文件已改变：

**以本地实际参数重新计算 NeuPAN `max_speed[1]`。**

公式：

```text
ψ_max = atan(wheelbase / Smac_minimum_turning_radius)
```

不要直接：

```text
ψ_max = physical_inner_wheel_joint_limit
```

---

# 14. Phase 8：只迁移一个必要的 NeuPAN ROS2 源码补丁——reverse gear

官方 ROS2 `path_callback()` 即使：

```text
include_initial_path_direction=true
```

也只会使用 Path pose orientation 作为 theta，
但 initial path 的 gear 仍默认全部为 `+1`。

当前 SmacPlannerHybrid：

```text
motion_model_for_search = REEDS_SHEPP
```

所以项目需要倒车段。

这是本次唯一计划内的 `neupan_node.py` 算法接口 patch。

---

# 15. reverse gear patch 的严格范围

修改：

```text
src/neupan_ros2/neupan_ros2/neupan_node.py
```

仅在：

```python
path_callback()
```

中进行。

当：

```python
include_initial_path_direction == True
```

时：

1. 从 `Path.pose.orientation` 读取车辆 heading `theta_i`；
2. 对相邻路径点计算运动方向：

```python
segment_i = atan2(y[i+1]-y[i], x[i+1]-x[i])
```

3. 判断：

```python
cos(theta_i - segment_i) >= 0
```

则：

```text
gear = +1
```

否则：

```text
gear = -1
```

4. 最后一个点继承倒数第二个点 gear；
5. 生成：

```text
[x, y, theta, gear]
```

交给：

```python
self.neupan_planner.set_initial_path(...)
```

---

# 16. reverse patch 中禁止顺便做的事

不要把旧 fork 的这些逻辑一起移植：

```text
path update 后 reset stop/arrive 的自定义代码
odom-frame compensation
scan timestamp fallback
scan timeout
sensor fault stop
command rate limiter
executor_threads
Ackermann yaw-rate conversion
avoidance_seed logging
```

如果基线运行后出现对应问题：

**一个问题一个 patch。**

---

# 17. Smac path orientation 验证

当前 `ackermann_smac_bridge` 的设计已经是：

```text
保留 Smac 原 path orientation
不重新计算 yaw
```

本任务默认不修改它。

但 runtime 必须验证：

```bash
ros2 topic echo /plan --once
```

对于明确出现 Reeds-Shepp reverse 的路径：

```text
Pose orientation = vehicle heading
segment direction = travel direction
```

在倒车段两者应近似相反。

如果 `/plan` 本身已经丢失 reverse heading：

不要在 NeuPAN 中“猜”修复。

先报告 Smac path upstream 数据问题。

---

# 18. Phase 9：不要改官方 NeuPAN 的 Ackermann Twist 语义

官方 `neupan_node.py` 对 Ackermann 输出的第二维是：

```text
steering angle ψ
```

而它会暂存在：

```text
Twist.angular.z
```

本任务不修改这段官方 node。

原因：

```text
保持 neupan_ros2 尽可能接近 upstream。
```

改用外部 adapter。

---

# 19. Phase 10：新增 `neupan_ackermann_adapter.py`

新增：

```text
src/ackermann_control/neupan_ackermann_adapter.py
```

职责必须非常单一：

```text
/neupan_cmd_vel_raw
Twist:
  linear.x  = v
  angular.z = steering ψ

        ↓

ω = v * tan(ψ) / L

        ↓

/neupan_cmd_vel
Twist:
  linear.x  = v
  angular.z = yaw rate ω
```

默认参数：

```text
input_topic  = /neupan_cmd_vel_raw
output_topic = /neupan_cmd_vel
wheelbase    = 0.593
```

要求：

- 使用 `rclpy`
- 输入输出都是 `geometry_msgs/msg/Twist`
- `wheelbase <= 0` 时启动失败
- NaN / inf 输入时发布零或拒绝该条并报错
- 不做速度优化
- 不做 rate limit
- 不做 DUNE 逻辑
- 不订阅 `/plan`
- 不订阅 `/scan`
- 不做 stale timeout
- 不做 `/stop`
- 不改变 `linear.x`

保持它只是数学适配器。

---

# 20. 更新 `ackermann_control/CMakeLists.txt`

当前该 package 通过：

```cmake
install(PROGRAMS ...)
```

安装 Python 可执行文件。

把：

```text
neupan_ackermann_adapter.py
```

加入现有 install list。

不要改 package 架构。

现有 `package.xml` 已经包含：

```text
rclpy
geometry_msgs
```

如果本地执行时仍然如此，则无需增加新依赖。

---

# 21. Phase 11：把 adapter 加入现有导航 launch

修改：

```text
src/ackermann_bringup/launch/navigation.launch.py
```

当前 launch 已经启动：

```text
localization
planning
pointcloud_to_laserscan
cmd_vel_mux
nav_status
rviz
```

新增：

```python
Node(
    package='ackermann_control',
    executable='neupan_ackermann_adapter.py',
    name='neupan_ackermann_adapter',
    output='screen',
    parameters=[{
        'use_sim_time': True,
        'wheelbase': 0.593,
        'input_topic': '/neupan_cmd_vel_raw',
        'output_topic': '/neupan_cmd_vel',
    }],
)
```

然后：

```text
adapter
→ existing cmd_vel_mux
```

不要修改当前 mux 的 topic：

```text
/neupan_cmd_vel
```

这样最终：

```text
官方 NeuPAN
  /neupan_cmd_vel_raw
        ↓
adapter
  /neupan_cmd_vel
        ↓
cmd_vel_mux
        ↓
controller
```

---

# 22. 本任务暂时不要修改 `cmd_vel_mux.py`

当前 mux 有一个已知问题：

```text
会周期性重发最后一条 NeuPAN command，
因此 planner 死亡时可能继续刷新旧命令时间戳。
```

这是应当后续修的安全问题。

但本任务目标是重新建立官方 NeuPAN baseline。

因此：

**先保持 mux 行为不变。**

在 migration report 中写入：

```text
Known follow-up:
add stale-command timeout to cmd_vel_mux
```

不要让本次变更范围继续膨胀。

---

# 23. Phase 12：保留当前 `third_party/NeuPAN` 运行来源

当前项目的：

```text
scripts/run_neupan.sh
```

会把：

```text
$PROJECT/third_party/NeuPAN
```

放到 `PYTHONPATH` 前面。

本任务保留这一行为。

原因：

```text
这次只测试 ROS2 wrapper reset。
不要同时替换 NeuPAN core。
```

允许修改 `run_neupan.sh` 的范围仅限：

1. 更新注释；
2. 增加 import 路径诊断；
3. 确认新的 `ackermann_robot/robot.yaml` 路径；
4. 不改变核心来源。

建议启动前打印：

```bash
python3 - <<'PY'
import neupan
import neupan_ros2

print("NeuPAN core:", neupan.__file__)
print("NeuPAN ROS2:", neupan_ros2.__file__)
PY
```

验收条件：

```text
NeuPAN core
```

应来自：

```text
/home/young/Project/AckermannRobot/third_party/NeuPAN/...
```

而：

```text
NeuPAN ROS2
```

应来自当前项目 build/install 的 `neupan_ros2`。

不得意外从：

```text
/home/young/Project/neupan_ros2
```

官方 clone 直接 import。

官方 clone 只是复制源。

---

# 24. Phase 13：PAN nested override 兼容性检查

官方 ROS2 wrapper 会用：

```python
pan = {'dune_checkpoint': self.dune_checkpoint}
neupan.init_from_yaml(..., pan=pan)
```

当前项目的 `third_party/NeuPAN` 在本计划编写时已经有 nested-dict merge 修复。

执行时必须检查：

```bash
grep -n -A30 "def init_from_yaml" \
  "$PROJECT/third_party/NeuPAN/neupan/neupan.py"
```

如果当前本地实现仍然会：

```text
merge 原 planner.yaml 的 pan 字典
+
覆盖 dune_checkpoint
```

则：

**不修改官方 `neupan_node.py`。**

如果本地实现已经变回浅层：

```python
config.update(kwargs)
```

导致整个 `pan:` 被覆盖：

- 不要擅自把旧大 patch 搬回；
- 在 migration report 中明确报告；
- 本任务允许做一个极小 nested-merge compatibility fix，
  但优先放在 `third_party/NeuPAN` 的单独 patch 中；
- 如果必须触碰 core，必须把它标记成“超出原计划但为配置正确性所必需”。

---

# 25. Phase 14：DUNE 权重策略

最终必须同时存在：

```text
src/neupan_ros2/config/robots/ackermann_robot/models/dune_model_5000.pth
```

这是第一阶段从官方 Ranger profile 复制来的 baseline model。

以及：

```text
src/neupan_ros2/config/dune_checkpoint/
```

这是用户明确要求保留的历史权重目录。

---

# 26. 第一阶段不要自动使用历史 `dune_checkpoint`

不要因为用户要求“保留”就自动配置成：

```text
scout_model_5000*.pth
```

除非存在明确训练记录证明：

```text
该模型对应当前 Ackermann geometry。
```

第一轮：

```yaml
dune_checkpoint_file: 'models/dune_model_5000.pth'
```

使用官方 Ranger baseline。

后续可以单变量 A/B：

```text
baseline Ranger checkpoint
vs
保留的历史 checkpoint
```

但这不是本次代码替换的验收前提。

---

# 27. Phase 15：可选新增 `ackermann_robot.launch.py`

这不是当前脚本运行链的硬要求，
因为项目使用 `scripts/run_neupan.sh` 独立启动 NeuPAN。

但为了自包含，可从：

```text
src/neupan_ros2/launch/ranger.launch.py
```

新建：

```text
src/neupan_ros2/launch/ackermann_robot.launch.py
```

只启动：

```text
neupan_node
optional RViz
```

不要复制 Ranger 专属：

```text
Hesai static TF
pointcloud_to_laserscan
Ranger-specific remapping
```

项目已经在 `ackermann_bringup` 中生成 `/scan`。

若新增 launch：

```text
robot_config_dir
```

必须指向：

```text
config/robots/ackermann_robot
```

---

# 28. Phase 16：重新构建所有受影响 package

在完成适配后：

```bash
cd "$PROJECT"

source /opt/ros/humble/setup.bash

rm -rf \
  build/neupan_ros2 \
  install/neupan_ros2 \
  build/ackermann_control \
  install/ackermann_control \
  build/ackermann_bringup \
  install/ackermann_bringup

colcon build \
  --symlink-install \
  --packages-select \
    neupan_ros2 \
    ackermann_control \
    ackermann_bringup \
    ackermann_smac_bridge
```

如果依赖图要求额外 package，可让 colcon 正常解析；
不要为了省事全删整个 `build/ install/`。

---

# 29. Python / YAML 静态检查

执行：

```bash
python3 -m py_compile \
  "$PROJECT/src/neupan_ros2/neupan_ros2/neupan_node.py" \
  "$PROJECT/src/ackermann_control/neupan_ackermann_adapter.py"

python3 - <<'PY'
import yaml

files = [
    "/home/young/Project/AckermannRobot/src/neupan_ros2/config/robots/ackermann_robot/robot.yaml",
    "/home/young/Project/AckermannRobot/src/neupan_ros2/config/robots/ackermann_robot/planner.yaml",
]

for p in files:
    with open(p, "r", encoding="utf-8") as f:
        yaml.safe_load(f)
    print("OK:", p)
PY
```

---

# 30. Phase 17：检查“旧 fork 污染”没有重新带回

在新的：

```text
src/neupan_ros2
```

中运行：

```bash
grep -RIn \
  -E 'avoidance_seed|scan_tf_max_age|scan_data_timeout|direct_goal_planning|command_rate_limit|executor_threads|min_speed' \
  "$PROJECT/src/neupan_ros2" \
  || true
```

第一阶段预期：

```text
min_speed              不应出现在 src/neupan_ros2
avoidance_seed_*       不应出现
scan_tf_max_age        不应出现
scan_data_timeout      不应出现
direct_goal_planning   不应出现
command_rate_limit     不应出现
executor_threads       不应出现
```

注意：

这些字符串可能仍然存在于：

```text
third_party/NeuPAN
```

这是允许的。

---

# 31. Phase 18：检查与 upstream 的最终差异

迁移完成后执行：

```bash
diff -qr \
  "$SOURCE" \
  "$TARGET" \
  --exclude=dune_checkpoint \
  > "$BACKUP/upstream_final_diff.txt" \
  || true

cat "$BACKUP/upstream_final_diff.txt"
```

期望差异应非常有限，主要是：

```text
config/robots/ackermann_robot/       # 新机器人 profile
launch/ackermann_robot.launch.py     # 如果新增
neupan_ros2/neupan_node.py           # 仅 reverse gear 最小 patch
```

以及：

```text
config/dune_checkpoint/              # 用户保留目录
```

不要出现几十个官方文件都被重新编辑的情况。

---

# 32. Phase 19：运行前 topic 拓扑检查

重新 source：

```bash
cd "$PROJECT"
source /opt/ros/humble/setup.bash
```

当前正式运行流程保持：

终端 1：

```bash
bash scripts/nav_liorf_neupan.sh maps/mini
```

终端 2：

```bash
bash scripts/run_neupan.sh
```

迁移后 topic 预期：

```text
/plan
  publisher: planner_server
  subscriber: neupan_node

/neupan_cmd_vel_raw
  publisher: neupan_node
  subscriber: neupan_ackermann_adapter

/neupan_cmd_vel
  publisher: neupan_ackermann_adapter
  subscriber: cmd_vel_mux

/ackermann_steering_controller/reference
  publisher: cmd_vel_mux
```

检查：

```bash
ros2 topic info /plan -v
ros2 topic info /neupan_cmd_vel_raw -v
ros2 topic info /neupan_cmd_vel -v
ros2 topic info /ackermann_steering_controller/reference -v
```

---

# 33. 必须确认 `/plan` 没有第二个 publisher

当前架构中：

```text
PlannerServer
```

应负责 `/plan`。

`ackermann_smac_bridge` 不应重新发布 `/plan`。

执行：

```bash
ros2 topic info /plan -v
```

如果出现多个非预期 publisher：

停止动态测试并定位。

不要让 NeuPAN 同时收到两条不同来源的全局路径。

---

# 34. Phase 20：TF 验证

运行：

```bash
ros2 run tf2_ros tf2_echo map rear_axle_link
ros2 run tf2_ros tf2_echo map laser_link
```

要求：

```text
map -> rear_axle_link
map -> laser_link
```

持续可用。

如果 TF 不稳定：

**不要开始调 NeuPAN 参数。**

本次先报告。

不要立刻重新移植旧 scan TF compensation。

---

# 35. Phase 21：传感器和控制频率检查

```bash
ros2 topic hz /scan
ros2 topic hz /plan
ros2 topic hz /neupan_cmd_vel_raw
ros2 topic hz /neupan_cmd_vel
```

记录：

```text
scan 实际 Hz
NeuPAN 实际输出 Hz
adapter 输出 Hz
```

官方：

```text
control_frequency=50 Hz
```

只是 timer 目标，不代表 CPU 上 PAN 一定能 50 Hz。

第一轮先测，不要提前降 horizon。

---

# 36. Phase 22：Ackermann adapter 数学验收

监听：

```bash
ros2 topic echo /neupan_cmd_vel_raw
ros2 topic echo /neupan_cmd_vel
```

取同一周期近似数据：

```text
raw:
  v
  ψ

adapted:
  v2
  ω
```

验收：

```text
v2 ≈ v
ω ≈ v * tan(ψ) / 0.593
```

允许浮点误差。

如果：

```text
ψ = 0
```

必须：

```text
ω ≈ 0
```

如果：

```text
v = 0
```

必须：

```text
ω = 0
```

---

# 37. Phase 23：第一组动态测试顺序

不要直接测试复杂动态避障。

严格顺序：

## Test A：无障碍直线

目标：

```text
v > 0
ψ ≈ 0
ω ≈ 0
```

机器人稳定直行。

---

## Test B：大半径左转

检查：

```text
raw ψ 符号
adapter ω 符号
车辆 yaw 符号
```

三者方向一致。

---

## Test C：大半径右转

同样检查。

---

## Test D：Smac 正向弯曲路径

检查：

```text
/plan orientation
NeuPAN ref path
opt path
车辆轨迹
```

---

## Test E：Reeds-Shepp 倒车

必须准备一个 Smac 确实产生 reverse segment 的场景。

预期：

```text
倒车 segment:
Path vehicle heading 与 segment travel direction 相反
        ↓
reverse gear patch 得到 gear=-1
        ↓
NeuPAN reference speed 为负
        ↓
/neupan_cmd_vel_raw.linear.x 可出现负值
```

如果 reverse 路径始终输出正速度：

优先检查：

```text
/plan orientation
path_callback gear
```

不要先调 `p_u`。

---

# 38. Phase 24：第一阶段避障测试

只有 A-E 都通过后，再放静态障碍。

第一阶段保持官方：

```text
q_s      = 0.1
p_u      = 0.5
eta      = 15.0
d_max    = 0.1
d_min    = 0.01
collision_threshold = 0.01
```

不要根据第一次避障表现立刻改五个参数。

先记录：

```text
DUNE points
NRMP points
opt trajectory
机器人是否停住
机器人是否绕行
实际 solve rate
```

---

# 39. 本任务完成后暂不解决的已知问题

以下项目必须写进 migration report，
但不是本次 baseline replacement 的必做修改。

## 39.1 `cmd_vel_mux` stale command timeout

当前 mux 会周期性重发上一次 command。

后续建议独立任务加入：

```text
0.2~0.5 s stale timeout
```

本任务暂不做。

---

## 39.2 NeuPAN core 当前仍为本地修改版

当前：

```text
third_party/NeuPAN
```

包含项目自定义内容。

后续如果要真正实现：

```text
官方 ROS2 + 官方 NeuPAN core
```

必须单独做第二阶段 core rebase。

不要在本任务偷偷完成。

---

## 39.3 DUNE geometry 与最终机器人 footprint

第一阶段使用 Ranger：

```text
length 0.720
width  0.500
checkpoint = Ranger baseline
```

这是为了建立可比较基线。

后续应单独评估：

```text
真实 Ackermann footprint
DUNE checkpoint training geometry
```

并决定是否重训。

---

## 39.4 非对称 `min_speed`

第一阶段不启用。

如果后续要求：

```text
forward max = 0.7
reverse max = 0.3
```

作为单独 core behavior patch / experiment。

---

# 40. 回滚方案

如果替换失败：

先停止所有 ROS 节点。

恢复旧 NeuPAN：

```bash
rm -rf "$PROJECT/src/neupan_ros2"

cp -a \
  "$BACKUP/neupan_ros2_old" \
  "$PROJECT/src/neupan_ros2"
```

外围文件不要用 `git reset --hard`。

根据 `$BACKUP/peripheral` 手工恢复本任务修改的文件。

例如：

```bash
cp -a \
  "$BACKUP/peripheral/CMakeLists.txt" \
  "$PROJECT/src/ackermann_control/CMakeLists.txt"

cp -a \
  "$BACKUP/peripheral/navigation.launch.py" \
  "$PROJECT/src/ackermann_bringup/launch/navigation.launch.py"

cp -a \
  "$BACKUP/peripheral/run_neupan.sh" \
  "$PROJECT/scripts/run_neupan.sh"
```

如果新建：

```text
neupan_ackermann_adapter.py
```

回滚时删除。

然后重新构建受影响 package。

---

# 41. Codex 完成后必须生成 `docs/neupan_ros2_upstream.md`

内容至少：

```markdown
# NeuPAN ROS2 Upstream Baseline

Source clone:
`/home/young/Project/neupan_ros2`

Upstream repository:
`KevinLADLee/neupan_ros2`

Imported commit:
`<实际 git rev-parse HEAD>`

Imported package:
`src/neupan_ros2`

Preserved local assets:
`src/neupan_ros2/config/dune_checkpoint`

Local modifications to vendored neupan_ros2:
1. Added `config/robots/ackermann_robot`
2. Reverse gear detection in `path_callback`
3. Added Ackermann launch file (if applicable)

Explicitly NOT ported:
- avoidance seed
- scan stale handling
- command rate limiter
- executor thread customization
- direct goal planning switch
- yaw-rate conversion inside NeuPAN node
```

---

# 42. Codex 完成后必须生成 `docs/neupan_ros2_migration_report.md`

报告至少包含：

```text
1. 执行日期
2. target project branch / SHA
3. upstream clone branch / SHA
4. 两个仓库执行前 dirty 状态
5. backup directory
6. dune_checkpoint 文件列表
7. dune_checkpoint SHA256 before/after
8. 替换后的 upstream diff
9. 实际修改文件列表
10. planner.yaml 最终参数
11. build 命令
12. build 是否成功
13. runtime tests 是否执行
14. topic topology 验证结果
15. TF 验证结果
16. reverse test 结果
17. 已知未解决问题
```

如果某项因为当前环境无法执行：

写：

```text
NOT RUN
```

不要伪造成功。

---

# 43. Codex 最终输出格式

Codex 完成修改后，在终端/回复中只需给一个清晰 summary：

```text
NeuPAN ROS2 upstream reset completed

Upstream SHA:
...

Target pre-migration SHA:
...

Preserved weights:
- ...
- ...

Vendored neupan_ros2 local diffs:
- ...
- ...

Peripheral integration diffs:
- ...

Build:
PASS / FAIL

Runtime smoke:
PASS / PARTIAL / NOT RUN

Known follow-ups:
- stale cmd timeout
- NeuPAN core rebase
- DUNE geometry retraining
```

并给出：

```bash
git -C /home/young/Project/AckermannRobot status --short
git -C /home/young/Project/AckermannRobot diff --stat
```

不要自动 commit。

---

# 44. 验收标准

本任务只有同时满足以下条件才算完成：

- [ ] 官方 clone 是 clean source，且记录了实际 SHA
- [ ] 原 `src/neupan_ros2` 有完整外部备份
- [ ] `config/dune_checkpoint` 所有文件均被保留
- [ ] 权重 SHA256 before/after 完全一致
- [ ] 新 `src/neupan_ros2` 主体来自官方 clone，而不是旧文件 merge
- [ ] 新增 `ackermann_robot` profile 来自官方 Ranger profile
- [ ] 第一轮 `q_s/p_u/eta/d_min/d_max/PAN` 主要参数保持官方 Ranger
- [ ] `wheelbase` 按当前机器人适配
- [ ] `ref_speed/max_speed/max steering` 按当前执行器/Smac 几何适配
- [ ] 不存在 `ind_range: 0`
- [ ] 不启用 `avoidance_seed_*`
- [ ] 不把旧 `min_speed` 配置带进 baseline
- [ ] NeuPAN 不直接订阅实际 `/goal_pose`
- [ ] `/plan` 仍来自 PlannerServer / Smac
- [ ] Smac reverse orientation 未被 bridge 改写
- [ ] `path_callback` 只有最小 reverse gear patch
- [ ] 官方 `generate_twist_msg` 没被改成项目专属 controller 逻辑
- [ ] steering→yaw-rate 位于 `ackermann_control` adapter
- [ ] adapter 已加入 navigation launch
- [ ] 当前 `third_party/NeuPAN` 未被本任务修改
- [ ] 新包和外围 package build 通过
- [ ] 生成 upstream notes
- [ ] 生成 migration report
- [ ] 没有自动 commit/push

---

# 45. 最重要的迁移原则

Codex 在执行过程中如果遇到一个新问题，不要采取：

```text
“旧代码里有一个实现，直接复制回来”
```

必须先判断：

```text
这是：
A. 官方 ROS2 wrapper 本身的行为？
B. 当前 Ackermann 硬件接口不兼容？
C. Smac / TF / scan 输入问题？
D. third_party NeuPAN core 行为？
E. 旧 fork 才引入的问题？
```

只有 **B 且当前系统无法工作** 的情况，
才优先在本任务增加最小适配。

其余问题：

```text
记录
报告
单独开后续 patch
```

这次迁移的成功标准不是：

> “把旧功能全部找回来”。

而是：

> **得到一个接近官方、能在当前 Ackermann + Smac 架构中解释清楚每一处差异的 NeuPAN ROS2 基线。**
