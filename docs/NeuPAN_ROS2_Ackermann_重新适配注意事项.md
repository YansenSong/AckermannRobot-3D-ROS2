# NeuPAN ROS2 从官方源码重新适配到 AckermannRobot-3D-ROS2 的注意事项

> 目标：假设当前项目中**完全没有 `src/neupan_ros2`**，重新以官方  
> `https://github.com/KevinLADLee/neupan_ros2` （本机已克隆到本地，路径：/home/young/Project/neupan_ros2）为唯一 ROS2 侧基线，把 NeuPAN 逐步接入 `AckermannRobot-3D-ROS2`（本地路径为：/home/young/Project/AckermannRobot）。
>
> 核心原则：**官方源码优先、最小侵入、参数尽量保持官方 Ackermann 示例、只有机器人接口/几何/执行器约束确实不兼容时才修改。**
>
> 审查基准日期：2026-09-10  
> 官方 NeuPAN ROS2 `main` 审查基准：`4ffb7ec...` 附近  
> NeuPAN 核心 `hanruihua/NeuPAN` 审查基准：`f5ae6d8...` 附近

---

## 1. 这次重做最重要的思路

不要把之前已经修改较多的 `src/neupan_ros2` 当作修补起点。

建议把迁移过程看成 4 层：

```text
Hybrid A* / 地图 / 定位
        │
        │ nav_msgs/Path  (/plan)
        ▼
官方 NeuPAN ROS2
        │
        │ NeuPAN 原生控制量
        │ Ackermann: [v, steering_angle]
        ▼
项目侧 Ackermann 适配层
        │
        │ geometry_msgs/Twist / TwistStamped
        ▼
ackermann_steering_controller
        │
        ▼
Gazebo Ackermann 机器人
```

这次应当尽量做到：

- **NeuPAN 算法本身不改**
- **NeuPAN ROS2 节点尽量不改**
- 必须修改的东西优先放到：
  - `config/robots/ackermann_robot/`
  - 新增的 `ackermann_robot.launch.py`
  - 项目已有的 `ackermann_control`
  - 项目已有的 `ackermann_bringup`
- 如果必须修改 `neupan_node.py`，每个修改只解决**一个明确问题**
- 一次只改一类问题，每改完一类都做验证

不要再同时修改：

- MPC horizon
- 车辆模型
- DUNE 参数
- 避障权重
- TF
- path direction
- cmd_vel 格式
- executor
- stale command
- Hybrid A* 参数

否则出现问题时几乎无法判断原因。

---

# 2. 先记录“官方基线”，以后所有修改都从这里比较

官方 ROS2 包主体：

```text
src/neupan_ros2/
├── config/
│   └── robots/
│       ├── limo/
│       ├── ranger/
│       ├── scout/
│       ├── simulation/
│       └── _template/
├── launch/
├── neupan_ros2/
│   ├── neupan_node.py
│   ├── visualization_manager.py
│   └── utils.py
├── package.xml
├── setup.py
├── setup.cfg
└── rviz/
```

官方已经有一个 Ackermann 平台：

```text
config/robots/ranger/
```

所以你的 AckermannRobot 应当首先以：

```text
ranger/
```

作为模板，而不是从 differential-drive 的 `simulation/` 或 `limo/` 开始魔改。

---

# 3. 建议的 Git 操作方式

## 3.1 第一个提交：只引入官方源码

建议第一步只做：

```text
copy official src/neupan_ros2
→ your_project/src/neupan_ros2
```

然后立即提交：

```bash
git add src/neupan_ros2
git commit -m "vendor: import upstream neupan_ros2 baseline"
```

这一提交中：

**不要修改任何代码。**

以后即使适配失败，也能永远回到这个干净基线。

---

## 3.2 第二个提交：只增加 AckermannRobot 配置

复制：

```text
src/neupan_ros2/config/robots/ranger
```

为：

```text
src/neupan_ros2/config/robots/ackermann_robot
```

然后提交：

```bash
git commit -m "config: add AckermannRobot NeuPAN profile"
```

这一步尽量**只改 YAML 和模型文件路径**。

---

## 3.3 第三个提交：只增加启动文件

复制：

```text
launch/ranger.launch.py
```

为：

```text
launch/ackermann_robot.launch.py
```

只删除 Ranger 专属部分并修改机器人配置路径。

提交：

```bash
git commit -m "launch: add AckermannRobot NeuPAN launcher"
```

---

## 3.4 后续每一个兼容性改动独立提交

例如：

```text
fix: preserve reverse gear from Hybrid A* path
fix: convert NeuPAN steering angle to Ackermann body yaw rate
fix: preserve PAN yaml when overriding DUNE checkpoint
safety: stop forwarding stale NeuPAN commands
```

不要一次塞进一个“大修 NeuPAN”提交。

---

# 4. 第一阶段不要接入整个导航栈

第一阶段只验证：

```text
Gazebo robot
+ TF
+ /scan
+ NeuPAN
```

先不要同时运行：

```text
Hybrid A*
navigation state machine
完整 bringup
stop mux
复杂 RViz
```

原因很简单：

如果 NeuPAN 节点连机器人状态、激光、模型都还没有单独验证，
此时把整个导航系统一起启动只会增加干扰源。

---

# 5. Python / NeuPAN 核心版本必须先固定

这是这次重做最容易忽略、但非常重要的事情。

官方 `neupan_ros2/setup.py` 的：

```python
install_requires=['setuptools']
```

**不会自动固定 NeuPAN 核心库版本。**

官方 README 是让用户另外执行：

```bash
pip3 install neupan
```

所以两台机器、两次安装，可能拿到不同的 `neupan` 核心实现。

在开始适配前务必记录：

```bash
python3 - <<'PY'
import inspect
import neupan

print("neupan module:", neupan.__file__)

from neupan import neupan as Neupan
print("neupan ctor:", inspect.signature(Neupan.__init__))

from neupan.robot import robot
print("robot ctor:", inspect.signature(robot.__init__))
PY
```

同时记录：

```bash
pip3 show neupan
pip3 freeze | grep -E "neupan|torch|numpy|cvxpy|cvxpylayers|ecos"
```

把结果保存到项目文档或 issue。

## 特别注意

官方 README 明确提示：

```text
numpy < 2.0
```

因此不要在 NeuPAN 还没跑通之前随意升级 NumPy。

---

# 6. 不要一开始修改 NeuPAN 核心 `site-packages`

第一阶段禁止直接编辑：

```text
/usr/lib/python...
~/.local/lib/python...
site-packages/neupan/...
```

原因：

1. 不进 Git
2. 很难知道当前机器到底跑的哪份代码
3. 重装 pip 后修改全部消失
4. 后面无法复现实验结果

如果将来确实需要修改 NeuPAN 核心：

- fork `hanruihua/NeuPAN`
- 固定 commit
- 从 Git 安装自己的 fork

而不是修改系统安装目录。

---

# 7. 官方 Ackermann Ranger 参数：先把它当“行为基线”

官方 Ranger `planner.yaml` 当前主要值：

```yaml
receding: 15
step_time: 0.2
ref_speed: 1.0
device: 'cpu'
time_print: False
collision_threshold: 0.01

robot:
  kinematics: 'acker'
  max_speed: [2.0, 0.656]
  max_acce: [1.0, 0.328]
  length: 0.720
  width: 0.500
  wheelbase: 0.494

ipath:
  interval: 0.03
  curve_style: 'dubins'
  min_radius: 0.81
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
  q_s: 0.1
  p_u: 0.5
  eta: 15.0
  d_max: 0.1
  d_min: 0.01
```

**新的适配第一原则：**

> 除非能够从你的机器人模型、控制器接口或 Hybrid A* 集成中证明“不改就不正确”，否则先保持这些值。

尤其第一轮不要再随意改：

```text
q_s
p_u
eta
d_max
d_min
iter_num
nrmp_max_num
```

先让官方行为在你的车上复现出来，再调。

---

# 8. 哪些官方 Ranger 参数必须适配你的机器人

你的仿真机器人当前可以明确得到：

```text
rear axle x relative to base_link ≈ -0.30449 m

front axle x relative to base_link ≈ +0.2893 m

wheelbase ≈
0.2893 - (-0.30449)
≈ 0.59379 m

controller wheelbase:
0.593 m

wheel radius:
0.093 m

front steering joint limit:
±0.52 rad
```

因此以下参数属于**机器人硬件/模型参数**，不能照抄 Ranger。

---

## 8.1 `wheelbase`：必须改

官方 Ranger：

```yaml
wheelbase: 0.494
```

你的机器人：

```yaml
wheelbase: 0.593
```

建议第一版使用和 ros2_control 完全一致的：

```yaml
wheelbase: 0.593
```

不要同时出现：

```text
Hybrid A*       0.594
NeuPAN          0.593
controller      0.59
URDF            0.60
```

尽量只保留一个标准值。

---

# 9. `base_frame` 建议使用 `rear_axle_link`

你的模型已经有：

```text
base_link
└── rear_axle_link
```

其中：

```text
rear_axle_link = base_link x -0.30449 m
```

NeuPAN Ackermann 使用自行车模型：

```text
[x, y, theta]
+
[v, steering_angle]
```

对这种模型最自然的状态参考点是**后轴中心**。

因此你的 NeuPAN ROS 参数建议：

```yaml
base_frame: 'rear_axle_link'
```

而不是照抄 Ranger：

```yaml
base_frame: 'base_link'
```

这是**必须适配机器人坐标定义**，不是算法调参。

---

# 10. LiDAR frame 和 TF 必须按你的机器人来

官方 Ranger：

```yaml
lidar_frame: 'base_laser'
```

你的项目应使用实际 TF 中存在的：

```yaml
lidar_frame: 'laser_link'
```

启动 NeuPAN 前验证：

```bash
ros2 run tf2_ros tf2_echo map rear_axle_link
ros2 run tf2_ros tf2_echo map laser_link
```

两条 TF 都应持续正常输出。

如果这一步不成立：

**不要调 NeuPAN 参数。**

先修 TF。

---

# 11. 官方 Ranger launch 中的专属节点不要搬过来

官方 `ranger.launch.py` 会启动：

- Ranger LiDAR static TF
- Hesai LiDAR static TF
- PointCloud2 → LaserScan
- NeuPAN
- RViz

你的 Gazebo 项目已经有自己的机器人 TF 和 `/scan`。

所以新的：

```text
ackermann_robot.launch.py
```

建议只保留：

```text
NeuPAN node
optional RViz
```

不要复制启动：

```text
hesai_lidar_transform_publisher
laser_to_hesai_tf
pointcloud_to_laserscan_node
```

否则很容易产生：

- 重复 TF
- 错误 LiDAR frame
- 两路 `/scan`
- frame tree 冲突

---

# 12. 推荐的新配置目录

最终建议保持官方结构：

```text
src/neupan_ros2/
└── config/
    └── robots/
        └── ackermann_robot/
            ├── robot.yaml
            ├── planner.yaml
            └── models/
                └── dune_model_5000.pth
```

不要再把：

```text
robot params
planner params
model path
Hybrid A* params
controller params
```

全部混在一个 YAML 中。

---

# 13. `robot.yaml` 第一版应当“官方 Ranger + 必要接口修改”

建议第一轮：

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

    # 你的激光最小有效距离需要按仿真传感器来
    scan_range_min: 0.15

    # 第一轮可以继续保留官方局部感知尺度
    scan_range_max: 5.0

    flip_angle: false
    refresh_initial_path: true

    # 如果使用 Hybrid A* Reeds-Shepp 倒车，后文需要配套小补丁
    include_initial_path_direction: true

    # 先保持官方
    control_frequency: 50.0

    cmd_vel_topic: '/neupan_cmd_vel_raw'
    scan_topic: '/scan'
    plan_input_topic: '/plan'

    # 不让官方 NeuPAN goal callback 抢 Hybrid A* 的 goal
    goal_topic: '/neupan_unused_goal'

    plan_output_topic: '/neupan_plan'
    ref_state_topic: '/neupan_ref_state'
    initial_path_topic: '/neupan_initial_path'
```

注意这里刻意使用：

```text
/neupan_cmd_vel_raw
```

而不是直接送控制器。

原因见下一节。

---

# 14. 非常重要：官方 Ackermann 输出不能直接喂你的 controller

官方 `neupan_node.py` 最终做：

```python
speed = vel[0]
steer = vel[1]

Twist.linear.x = speed
Twist.angular.z = steer
```

对于 differential drive：

```text
第二维 = yaw rate
```

这样没有问题。

但对于 NeuPAN Ackermann：

```text
第二维 = steering angle ψ
```

而你的：

```text
ackermann_steering_controller
```

接收的是车辆 body twist：

```text
linear.x = v
angular.z = yaw rate ω
```

所以不能简单把：

```text
steering_angle
```

塞到：

```text
Twist.angular.z
```

---

# 15. 最小侵入解决方案：不要先改官方 neupan_node.py

推荐新增一个**项目侧适配器**。

例如：

```text
src/ackermann_control/neupan_ackermann_adapter.py
```

输入：

```text
/neupan_cmd_vel_raw
```

其中：

```text
linear.x  = v
angular.z = steering angle ψ
```

输出：

```text
/ackermann_steering_controller/reference
```

转换公式：

```text
ω = v * tan(ψ) / L
```

其中：

```text
L = 0.593 m
```

这样：

```text
官方 NeuPAN ROS2
```

完全不需要因为你的 controller 接口而修改。

这比直接改 `neupan_node.py` 更容易未来和 upstream 比较。

---

# 16. 如果继续使用项目已有 `cmd_vel_mux.py`

你项目当前的 mux 会：

```text
/neupan_cmd_vel
→ TwistStamped
→ /ackermann_steering_controller/reference
```

但是它当前只是原样转发：

```text
linear.x
angular.z
```

所以如果直接使用官方 NeuPAN 输出，它会把：

```text
steering angle
```

错误地当：

```text
yaw rate
```

处理。

因此可以有两种方案：

### 方案 A：推荐

```text
NeuPAN
  ↓ /neupan_cmd_vel_raw
Ackermann adapter
  ↓ /neupan_cmd_vel
cmd_vel_mux
  ↓
controller
```

### 方案 B

把：

```text
steering_angle → yaw_rate
```

转换功能放进现有 `cmd_vel_mux.py`。

如果希望保持模块职责清楚，推荐 A。

---

# 17. 项目现有 mux 还应注意 stale command

当前 mux 每 0.05 秒：

```text
重新发布上一次收到的 NeuPAN command
```

只要 `/stop` 没有触发，就会一直重复最后一条命令。

这会产生一个隐患：

```text
NeuPAN 卡住 / 崩溃 / 不再发布
        ↓
mux 仍每 0.05 s 给旧命令重新打时间戳
        ↓
controller 看起来一直有“新命令”
        ↓
机器人继续执行旧速度
```

这是项目侧安全问题，不应该靠 NeuPAN 参数解决。

后续建议给 mux 增加：

```text
command timeout ≈ 0.2~0.5 s
```

如果超过这个时间没有新 NeuPAN command：

```text
强制发布 0
```

但建议这作为**单独提交**完成。

---

# 18. Hybrid A* 与官方 NeuPAN 的 `/goal_pose` 冲突必须隔离

官方 NeuPAN 节点同时订阅：

```text
/plan
```

和：

```text
/goal_pose
```

收到 `/plan`：

```text
set_initial_path(...)
```

收到 `/goal_pose`：

```text
update_initial_path_from_goal(...)
```

也就是说，如果 Hybrid A* 和 NeuPAN 同时收到同一个导航 goal：

```text
Hybrid A* 生成了一条全局路径
```

但 NeuPAN 也可能：

```text
根据 start/goal 自己重新生成 initial path
```

这会让系统中出现两个“谁负责全局路径”的来源。

你的架构已经明确：

```text
Hybrid A* = global planner
NeuPAN    = local planner/controller
```

所以第一版**不要修改官方代码**，只在 YAML 中：

```yaml
goal_topic: '/neupan_unused_goal'
```

保证没有节点往这个 topic 发目标。

这样 NeuPAN 就只吃：

```text
/plan
```

---

# 19. Hybrid A* 路径必须确认 frame

NeuPAN 默认假定 path point 与 robot state、obstacle points 都最终在：

```text
map
```

因此需要检查：

```bash
ros2 topic echo /plan --once
```

确认：

```yaml
header:
  frame_id: map
```

如果 Hybrid A* 发布的是：

```text
odom
base_link
rear_axle_link
```

则不要调 NeuPAN 参数，先统一坐标系。

---

# 20. 官方 `include_initial_path_direction` 并不真正支持倒车 gear

这是 Hybrid A* + Reeds-Shepp 接入时必须特别注意的地方。

官方节点：

```text
include_initial_path_direction = true
```

只会读取：

```text
Path.pose.orientation
```

作为：

```text
theta
```

但是后面仍然创建：

```text
gear = 1
```

即所有点都是：

```text
forward
```

所以：

> 官方 ROS2 这一版不能仅靠 `include_initial_path_direction=true`
> 就完整保留 Hybrid A* 的倒车段。

---

# 21. 如果 Hybrid A* 需要倒车，这是一个“允许的小补丁”

这属于少数值得修改 `neupan_node.py` 的地方。

逻辑应当非常简单：

对于路径点 `i`：

```text
vehicle heading =
Path.pose.orientation yaw
```

路径实际运动方向：

```text
segment heading =
atan2(y[i+1]-y[i], x[i+1]-x[i])
```

如果：

```text
cos(vehicle_heading - segment_heading) >= 0
```

则：

```text
gear = +1
```

否则：

```text
gear = -1
```

然后将 initial path 构造为：

```text
[x, y, theta, gear]
```

这就是一个**明确、局部、可测试**的补丁。

不要顺手在这里：

- 平滑路径
- 改 steering
- 改速度
- 改 horizon
- 改障碍逻辑

一个补丁只解决“gear”。

---

# 22. 倒车补丁必须单独验证

准备一条非常简单的路径：

```text
机器人车头朝 +X
路径点却向 -X 移动
```

预期：

```text
gear = -1
NeuPAN reference speed < 0
```

如果不能明确看到负参考速度：

不要继续测试复杂 Reeds-Shepp 路径。

---

# 23. `planner.yaml` 第一轮建议从官方 Ranger 原样开始

最推荐的做法不是立刻写一套“最佳 Ackermann 参数”，而是：

```text
复制 Ranger planner.yaml
```

然后只改**必须和你的执行器匹配**的值。

---

# 24. 参数分类：哪些先保留官方

第一阶段建议保留：

| 参数 | 官方 Ranger | 第一轮策略 |
|---|---:|---|
| `receding` | 15 | 保持 |
| `step_time` | 0.2 | 保持 |
| `device` | cpu | 保持 |
| `time_print` | False | 保持 |
| `collision_threshold` | 0.01 | 先保持 |
| `max_acce[0]` | 1.0 | 先保持 |
| `max_acce[1]` | 0.328 | 保持 |
| `ipath.interval` | 0.03 | 保持 |
| `ipath.curve_style` | dubins | 外部 `/plan` 模式暂时保持 |
| `ipath.arrive_threshold` | 0.5 | 保持 |
| `ipath.close_threshold` | 0.05 | 保持 |
| `ipath.arrive_index_threshold` | 3 | 保持 |
| `pan.iter_num` | 2 | 保持 |
| `pan.dune_max_num` | 200 | 保持配置值 |
| `pan.nrmp_max_num` | 10 | 保持 |
| `pan.iter_threshold` | 0.1 | 保持 |
| `q_s` | 0.1 | 保持 |
| `p_u` | 0.5 | 保持 |
| `eta` | 15.0 | 保持 |
| `d_max` | 0.1 | 保持 |
| `d_min` | 0.01 | 保持 |

第一轮**不要因为之前的经验直接改成另一整套权重。**

---

# 25. `ref_speed` 必须根据你的执行器适配

官方 Ranger：

```yaml
ref_speed: 1.0
```

但是你的后轮：

```text
max wheel velocity = 10 rad/s
wheel radius       = 0.093 m
```

直线理论轮缘速度约：

```text
10 × 0.093
= 0.93 m/s
```

而 Ackermann 转弯时外侧轮还需要更高线速度。

因此：

```text
ref_speed = 1.0 m/s
```

已经接近或超过你的仿真驱动器能力。

这是**必须针对机器人改**的参数，不属于随意调算法。

第一轮推荐：

```yaml
ref_speed: 0.5
```

这样先观察：

- 跟踪是否稳定
- 转弯是否饱和
- PAN 实际计算周期
- controller 是否能跟上

后面再逐步加。

---

# 26. `max_speed[0]` 同样不能照抄 2.0

官方：

```yaml
max_speed: [2.0, ...]
```

而你的轮子理论直线极限仅：

```text
≈ 0.93 m/s
```

所以 2 m/s 对你的仿真机器人不是“偏大”，而是：

> 优化器允许生成执行器无法实现的解。

这会导致：

```text
NeuPAN预测轨迹
≠
真实机器人轨迹
```

然后下一周期又基于错误预期重新优化。

第一轮建议：

```yaml
max_speed[0]: 0.7
```

如果希望更保守：

```yaml
0.6
```

---

# 27. `max_speed[1]` 不能直接用实体前轮 ±0.52

这是 Ackermann 适配最容易犯的错误之一。

你的 Xacro 中：

```text
left steering joint  ∈ [-0.52, +0.52]
right steering joint ∈ [-0.52, +0.52]
```

这个是**实体左右前轮转角**。

NeuPAN Ackermann 的第二控制量：

```text
ψ
```

是**等效自行车模型中心转角**。

两者不是同一个量。

考虑：

```text
wheelbase ≈ 0.593
front track ≈ 0.510
```

如果限制内侧实体轮不超过：

```text
0.52 rad
```

对应中心等效转角大约：

```text
0.43 rad
```

建议第一轮：

```yaml
max_speed: [0.7, 0.43]
```

如果希望留一点余量：

```yaml
max_speed: [0.7, 0.414]
```

---

# 28. 建议把 Hybrid A* 最小转弯半径同步到同一物理模型

如果 NeuPAN 的等效中心转角约：

```text
0.414 rad
```

则：

```text
Rmin = L / tan(ψ)
≈ 0.593 / tan(0.414)
≈ 1.35 m
```

因此后续应该确保：

```text
Hybrid A* Rmin
NeuPAN fallback min_radius
Ackermann controller physical geometry
```

是同一套运动学。

但注意：

> 如果 NeuPAN 当前只消费 Hybrid A* 的 `/plan`，
> `ipath.min_radius` 并不会重新约束外部 path。

真正决定全局路径曲率的仍然是 Hybrid A*。

---

# 29. 第一版 planner.yaml 建议

为了尽量接近官方，只做硬件必要变化：

```yaml
# MPC
receding: 15
step_time: 0.2

# 必须适配你的机器人速度能力
ref_speed: 0.5

device: 'cpu'
time_print: False
collision_threshold: 0.01

robot:
  kinematics: 'acker'

  # 必须适配执行器
  max_speed: [0.7, 0.43]

  # 第一轮保持官方 Ranger
  max_acce: [1.0, 0.328]

  # DUNE 几何问题见后文
  length: 0.720
  width: 0.500

  # 必须适配机器人
  wheelbase: 0.593

ipath:
  interval: 0.03
  curve_style: 'dubins'

  # 当前由 Hybrid A* 提供 /plan 时主要作为 fallback
  min_radius: 0.81

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
  q_s: 0.1
  p_u: 0.5
  eta: 15.0
  d_max: 0.1
  d_min: 0.01
```

这不是最终最佳配置。

它的目的只有一个：

> **最大限度保留官方 Ackermann 行为，只修改你的机器人确实无法照抄的运动学参数。**

---

# 30. 为什么第一轮 length / width 反而建议暂时保留 Ranger

这和 DUNE checkpoint 有关。

官方 Ranger 模型目录：

```text
config/robots/ranger/models/dune_model_5000.pth
```

DUNE 网络训练与机器人多边形几何有关。

如果：

```text
checkpoint 按 Ranger geometry 训练
```

但是运行时突然改：

```text
length
width
wheelbase reference geometry
```

那么 DUNE 网络与运行时几何之间会不完全一致。

因此适配最好分两阶段。

---

# 31. DUNE 模型适配建议分两阶段

## 阶段 A：官方行为复现

目的：

```text
确认 ROS 接口、TF、scan、path、control pipeline 能跑
```

使用：

```text
官方 Ranger checkpoint
官方 Ranger length/width
```

只修改必须用于 Ackermann 运动学的：

```text
wheelbase
speed/steering executable limits
```

此阶段只在：

```text
空旷场地
宽走廊
低速
```

进行验证。

**不要拿这个阶段做最终碰撞安全评价。**

---

## 阶段 B：机器人几何一致化

确认接口完全工作后，再根据你的实际 robot footprint：

```text
重新训练/准备适合你的 DUNE checkpoint
```

然后同时更新：

```text
length
width
wheelbase
checkpoint
```

不要：

```text
只改 geometry，不换 checkpoint
```

然后靠 `eta/d_min` 去“补”。

---

# 32. 你的机器人 footprint 不应只看 chassis box

你的 chassis collision box 是：

```text
0.7 × 0.3
```

但车轮也有 collision。

根据当前 Xacro 粗略计算，包含轮胎后整个碰撞包络约：

```text
长度 ≈ 0.78 m
宽度 ≈ 0.57 m
```

因此将来训练自己的 DUNE 时，
可以考虑使用接近：

```text
0.78 × 0.57
```

或略保守的：

```text
0.80 × 0.58
```

但这应该属于**DUNE 几何一致化阶段**，
不要作为第一天接入官方源码时就同时修改的变量。

---

# 33. 官方 `pan:` 参数存在版本兼容注意事项

官方 ROS2 节点为了注入模型路径会调用类似：

```python
pan = {'dune_checkpoint': self.dune_checkpoint}

neupan.init_from_yaml(
    self.planner_config_file,
    pan=pan
)
```

当前我审查到的 NeuPAN 核心 `init_from_yaml()` 使用的是：

```python
config.update(kwargs)
```

这是**浅层更新**。

如果你的实际安装版本也是这样，那么：

```text
planner.yaml 原 pan:
  iter_num
  dune_max_num
  nrmp_max_num
  iter_threshold
  dune_checkpoint
```

有可能被新的：

```python
{'dune_checkpoint': ...}
```

整个替换。

结果就是：

```text
YAML 中部分 PAN 参数看起来写了
实际上回落到 NeuPAN 核心默认
```

---

# 34. 不要看到这个问题就立刻改源码

第一步先确认你的实际版本。

可以运行：

```bash
python3 - <<'PY'
import inspect
from neupan import neupan

print(inspect.getsource(neupan.init_from_yaml))
PY
```

如果能看到：

```python
config.update(kwargs)
```

并且没有 nested merge，
再做一个**单独的兼容性补丁**。

这个补丁只负责：

```text
保留 planner.yaml pan 字典
+
只覆盖 dune_checkpoint
```

不要顺手修改其它算法行为。

---

# 35. 建议的 PAN merge 修复原则

逻辑应当是：

```text
读取 planner.yaml 中原始 pan
        +
覆盖其中 dune_checkpoint
        =
传给 NeuPAN
```

而不是：

```text
新建只有 dune_checkpoint 的 pan
        ↓
覆盖整个原 pan
```

这个修复属于：

```text
配置正确性
```

而不是：

```text
算法调参
```

所以如果版本确认存在问题，值得修。

---

# 36. `ipath.interval` 不要作为第一轮主要调参项

在当前 NeuPAN 核心里，当 ROS2 收到外部 `/plan` 并调用：

```text
set_initial_path(path)
```

核心会根据输入 path 重新计算平均点间距。

所以：

```yaml
ipath:
  interval: 0.03
```

主要影响 NeuPAN 自己生成路径时的行为。

对于：

```text
Hybrid A* → /plan → NeuPAN
```

真正决定路径点密度的更多是：

```text
Hybrid A* 输出
地图 resolution
路径后处理
```

因此第一轮直接保留官方：

```yaml
interval: 0.03
```

即可。

---

# 37. `curve_style: dubins` 第一轮也可以先保留

因为你的主路径由 Hybrid A* 提供。

如果我们已经把 NeuPAN 的：

```text
goal_topic
```

隔离，不让它自己通过 goal 生成路径，

那么：

```yaml
curve_style: dubins
min_radius: 0.81
```

在主要 `/plan` 工作流里基本只是 fallback 配置。

所以第一轮为了保持官方：

```yaml
curve_style: 'dubins'
min_radius: 0.81
```

可以先不动。

等以后真的需要：

```text
NeuPAN 自己根据 goal 生成 Ackermann 路径
```

再改成与你机器人一致的：

```text
reeds / dubins
Rmin
```

---

# 38. 不要再使用 `ind_range: 0`

官方 Ranger 配置本来就**没有显式写 `ind_range`**。

当前核心默认通常为：

```text
10
```

因此这次重新开始时：

> **直接跟官方 Ranger 一样，不写 `ind_range`。**

不要重新加入：

```yaml
ind_range: 0
```

因为在当前核心实现中这会造成空搜索区间。

只有以后确认确实需要扩大最近点搜索范围时，再例如：

```yaml
ind_range: 30
```

或：

```yaml
ind_range: 50
```

每次只做单变量实验。

---

# 39. 第一轮不要加入任何“自创参数”

例如之前类似：

```yaml
avoidance_seed_enabled:
avoidance_seed_distance:
avoidance_seed_clearance:
avoidance_seed_steer:
```

如果官方源码和 NeuPAN 核心里没有明确读取：

**不要放进 YAML。**

这条原则非常重要：

> 配置文件中每一个参数，都应该能找到对应源码读取位置。

否则 YAML 会逐渐变成：

```text
看起来调了很多
实际上不知道谁生效
```

---

# 40. 第一轮不要加入 `min_speed`

当前 upstream NeuPAN 核心的 Ackermann `robot` 实现主要使用：

```text
max_speed
```

并通过：

```text
abs(u) <= max_speed
```

形成对称约束。

如果当前安装版本没有原生 `min_speed`：

```yaml
min_speed:
```

就不要自己加。

这次重做的原则是：

> **官方核心有什么参数，就先用什么参数。**

如果将来确实需要：

```text
前进最大 0.7
倒车最大 0.3
```

再把“非对称速度约束”作为一个独立功能开发。

不要在基线阶段先做。

---

# 41. 因为暂时没有 min_speed，正反向速度先一起保守

如果使用：

```yaml
max_speed: [0.7, 0.43]
```

当前核心通常意味着：

```text
-0.7 <= v <= +0.7
```

因此 Hybrid A* 倒车测试第一阶段一定要低速。

如果担心倒车太快：

```yaml
max_speed: [0.5, 0.43]
ref_speed: 0.4
```

也比立刻修改 NeuPAN 核心速度约束简单得多。

---

# 42. `max_acce` 第一轮建议保持官方

官方 Ranger：

```yaml
max_acce: [1.0, 0.328]
```

你的仿真关节能力并没有明显证明这一值不可执行。

因此：

```yaml
max_acce: [1.0, 0.328]
```

第一轮先保持。

尤其不要再把：

```text
第二维
```

理解成：

```text
rad/s²
```

NeuPAN 的第二控制量本身是转向角，
其相邻控制量差值限制对应的是：

```text
steering angle rate
```

更接近：

```text
rad/s
```

---

# 43. 第一轮不要先改 `q_s / p_u / eta`

这是这次重做最需要克制的地方。

官方 Ranger：

```yaml
q_s: 0.1
p_u: 0.5
eta: 15.0
d_max: 0.1
d_min: 0.01
```

先保持。

只有在：

```text
TF 正确
scan 正确
path 正确
gear 正确
控制接口正确
机器人实际速度正确
DUNE geometry/checkpoint 基本一致
```

之后，才开始调这些权重。

否则：

```text
eta 调大
```

可能只是在掩盖 geometry 错误；

```text
q_s 调小
```

可能只是在掩盖 path orientation 错误。

---

# 44. `control_frequency: 50` 第一轮先不改

官方 Ranger 使用：

```text
50 Hz
```

第一轮保留是有意义的，因为我们想知道：

> 官方目标频率在你的机器上实际能跑多少。

运行时测：

```bash
ros2 topic hz /neupan_cmd_vel_raw
```

如果只得到：

```text
1~5 Hz
```

那说明 CPU PAN solve 才是真正限制因素。

这时再单独讨论：

```text
control_frequency
receding
PAN iterations
dune point number
visualization
```

不要提前改。

---

# 45. 可视化第一轮建议打开

虽然会有一些 CPU 开销，但第一轮调试非常有价值。

建议：

```yaml
enable_visualization: true
enable_dune_markers: true
enable_nrmp_markers: true
enable_robot_marker: true
```

先确认：

- DUNE points 在正确障碍位置
- robot footprint 没有偏移
- ref path 正确
- opt trajectory 正确

稳定之后再做性能测试时关闭：

```yaml
enable_visualization: false
```

---

# 46. 建议的启动顺序

## Step 1：Gazebo 机器人

确认：

```text
/clock
/joint_states
/odom
TF
/scan
```

正常。

---

## Step 2：只启动 NeuPAN

```bash
ros2 launch neupan_ros2 ackermann_robot.launch.py
```

此时不要给 goal。

观察：

```text
Node starts
planner config found
DUNE checkpoint found
robot geometry logged
waiting for path
```

---

## Step 3：检查 TF

```bash
ros2 run tf2_ros tf2_echo map rear_axle_link
ros2 run tf2_ros tf2_echo map laser_link
```

---

## Step 4：检查 scan

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

---

## Step 5：再启动 Hybrid A*

确认：

```bash
ros2 topic echo /plan --once
```

---

## Step 6：观察 NeuPAN，但先不接控制器

把输出保持在：

```text
/neupan_cmd_vel_raw
```

观察：

```bash
ros2 topic echo /neupan_cmd_vel_raw
ros2 topic hz /neupan_cmd_vel_raw
```

确认：

```text
v
steering angle
```

数值合理。

---

## Step 7：再接 Ackermann adapter

检查转换后的：

```text
yaw rate
```

是否符合：

```text
ω = v tan(ψ) / L
```

---

## Step 8：最后才让 controller 真正执行

这是避免机器人“一启动就乱跑”的最稳方式。

---

# 47. 第一组运动测试必须非常简单

不要第一天就做动态避障。

建议顺序：

### Test A：直线前进

```text
无障碍
直线路径
```

验证：

```text
steering ≈ 0
robot 跟踪稳定
```

---

### Test B：固定大半径左转

验证：

```text
steering 符号正确
yaw rate 符号正确
机器人朝预期方向转
```

---

### Test C：固定大半径右转

防止：

```text
角度符号反了
```

---

### Test D：低速窄弯

看：

```text
steering 是否长期顶到上限
```

如果长期饱和：

首先检查：

```text
Hybrid A* 曲率
```

而不是调 NeuPAN weight。

---

### Test E：静态障碍

再测试：

```text
DUNE
NRMP
collision stop
```

---

### Test F：倒车

最后再测试 Reeds-Shepp reverse。

---

# 48. Hybrid A* 和 NeuPAN 的职责一定要固定

建议明确：

```text
Hybrid A*
负责：
- 全局搜索
- 前进/倒车拓扑
- 全局路径曲率可行性
- goal

NeuPAN
负责：
- 跟踪全局 path
- 局部避障
- 输出局部 Ackermann 控制量
```

不要同时让：

```text
Hybrid A* 规划全局路径
```

又让：

```text
NeuPAN goal_callback 再生成一条路径
```

---

# 49. Hybrid A* 路径 orientation 不能随便重算

如果路径包含倒车：

```text
路径移动方向
```

与：

```text
车辆车头方向
```

可能相差约 π。

所以 Hybrid A* `/plan` 中：

```text
Pose.orientation
```

最好表示：

```text
车辆 heading
```

而不是简单：

```text
atan2(next_y-y, next_x-x)
```

否则后续无法可靠判断：

```text
forward / reverse
```

这一点比 NeuPAN `q_s`、`eta` 更优先。

---

# 50. DUNE checkpoint 不要随便复制旧项目版本

这次既然按“没有旧 `src/neupan_ros2`”处理，
就不要一开始把旧项目中已经使用过的自定义：

```text
dune_model_5000.pth
```

混回来。

第一阶段建议直接使用：

```text
官方 Ranger checkpoint
```

并明确标记：

```text
仅用于官方基线行为复现
```

之后如果决定训练自己 geometry 的模型，
再单独替换。

这样出了问题时能够回答：

```text
“这是官方 Ranger 模型的表现”
```

而不是：

```text
“我不知道这个 pth 是哪次训练出来的”
```

---

# 51. 每一个 DUNE 模型都应配套保存训练元数据

将来你训练自己的模型后，建议目录：

```text
models/
└── ackermann_080x058/
    ├── dune_model_5000.pth
    ├── train_config.yaml
    ├── README.md
    └── geometry.txt
```

至少记录：

```text
length
width
wheelbase
training data range
epoch
NeuPAN core commit
date
```

不要只保留一个孤零零的：

```text
model_5000.pth
```

---

# 52. 任何参数修改都做单变量记录

建议建立：

```text
docs/neupan_experiments.md
```

每次只记录：

```text
日期
Git commit
场景
修改参数
修改前
修改后
机器人表现
是否保留
```

例如：

```text
2026-09-xx

change:
ref_speed 0.5 -> 0.6

scenario:
empty + 90deg bend

result:
tracking OK
outer wheel no saturation
PAN 4.7 Hz

decision:
keep
```

这样几周后还能知道为什么某个参数是当前值。

---

# 53. 参数调试顺序

如果将来正式开始调参，建议严格按这个顺序：

```text
1. 机器人几何
2. TF/reference frame
3. command semantics
4. max speed / max steering
5. Hybrid A* curvature
6. DUNE model geometry
7. collision distance
8. reference speed
9. path tracking weights
10. avoidance weights
11. CPU/performance parameters
```

不要反过来。

---

# 54. 哪些症状优先怀疑什么

| 症状 | 第一怀疑项 | 不要第一时间改 |
|---|---|---|
| 车往反方向转 | steering/yaw-rate 转换符号 | `eta` |
| 路径转弯车根本跟不上 | Rmin / steering limit | `q_s` |
| 倒车段变成前进 | path orientation / gear | `p_u` |
| 机器人已经到了但不停 | path index / arrive logic | `d_min` |
| 障碍位置在 RViz 中飘 | TF / scan frame | DUNE weight |
| NeuPAN 规划轨迹正常但车不按轨迹走 | controller interface | PAN |
| 一避障就原地卡 | geometry / DUNE checkpoint | 随机改 eta |
| 输出突然停在旧速度 | command timeout | ref_speed |
| YAML 改 PAN 参数没变化 | pan merge / core version | 继续加大参数 |
| CPU 很高 | solve rate / DUNE points / visualization | 先改运动学 |

---

# 55. 推荐保留的“小修改”

从官方源码开始，我认为下面这些修改是合理的，而且都能明确解释。

## 可以接受

### A. 新增 `ackermann_robot` 配置

属于标准平台适配。

### B. 新增 `ackermann_robot.launch.py`

属于标准平台适配。

### C. Hybrid A* reverse gear 识别

如果确实需要 Reeds-Shepp 倒车，这是必要接口补丁。

### D. Ackermann steering angle → yaw rate adapter

你的 controller 接口和 NeuPAN 原生控制量不同，这是必要转换。

### E. DUNE checkpoint 的 PAN dictionary merge 修复

前提是你的实际核心版本确认存在 shallow-update 问题。

### F. stale command timeout

属于执行安全保护，建议放在项目控制层。

---

# 56. 第一阶段不建议重新引入的修改

暂时不要重新加入：

```text
scan_tf_max_age
scan_data_timeout
direct_goal_planning
command_rate_limit
executor_threads configurable
avoidance_seed_*
自定义 path smoother
自定义 steering seed
自动绕障初始曲线
非对称 min_speed
复杂 watchdog 状态机
大量参数动态更新
```

这些功能并不是永远不能加。

而是：

> **等官方基线跑通以后，一个一个加。**

每增加一个功能，都要回答：

```text
它解决了哪个已经复现的问题？
```

如果没有明确问题，就不要加。

---

# 57. 关于之前有效的改动，不要直接否定

之前的一些修改本身可能是合理的，比如：

```text
Ackermann steering → yaw rate
reverse gear detection
stale scan handling
command rate limit
stale command timeout
```

问题不一定在于“这些想法错了”。

更可能的问题是：

```text
太多改动同时存在
+
参数也同时改变
+
难以确定哪一层造成了效果下降
```

这次重做后，可以把这些功能一个一个重新应用，
每一个都通过 A/B 测试证明它确实改善了问题。

---

# 58. 推荐的逐步恢复顺序

建议按照下面的版本演进。

## V0 — Official baseline

```text
官方 neupan_ros2 原码
官方 Ranger config
官方 Ranger model
```

只验证：

```text
包能编译
节点能启动
```

---

## V1 — Robot interface adaptation

只修改：

```text
robot type
TF frames
scan topic
plan topic
use_sim_time
launch
```

不动车。

---

## V2 — Ackermann kinematic adaptation

修改：

```text
wheelbase
max linear speed
max bicycle steering
ref_speed
```

加入：

```text
steering → yaw-rate adapter
```

测试正向行驶。

---

## V3 — Hybrid A* integration

隔离：

```text
NeuPAN /goal_pose
```

只使用：

```text
/plan
```

验证纯前进 Hybrid A* 路径。

---

## V4 — Reverse support

只加入：

```text
path gear detection
```

验证倒车。

---

## V5 — DUNE geometry adaptation

重新处理：

```text
robot footprint
DUNE checkpoint
```

测试静态避障。

---

## V6 — Safety hardening

加入：

```text
stale command timeout
scan timeout
stop override
```

---

## V7 — Performance tuning

最后才碰：

```text
receding
control_frequency
iter_num
dune_max_num
nrmp_max_num
visualization
```

---

## V8 — Behavior tuning

最后最后才调：

```text
q_s
p_u
eta
d_min
d_max
collision_threshold
```

---

# 59. 每阶段验收条件

不要以：

```text
“感觉好像能跑”
```

作为验收。

至少要满足：

### V1

```text
TF 连续稳定
/scan 正常
NeuPAN 能加载 checkpoint
```

### V2

```text
直线稳定
左右转符号正确
steering 不异常饱和
```

### V3

```text
NeuPAN 使用的 path 与 Hybrid A* /plan 一致
goal 不会覆盖 path
```

### V4

```text
正向段 v > 0
倒车段 v < 0
gear 切换符合 Hybrid A*
```

### V5

```text
障碍位置正确
机器人 footprint 正确
避障轨迹方向合理
```

### V6

```text
NeuPAN 停止发布后机器人能自动归零
/stop 始终拥有最高优先级
```

### V7

```text
实际输出频率稳定
CPU 不持续打满
没有明显 scan backlog
```

---

# 60. 推荐的调试命令清单

## 节点

```bash
ros2 node list
ros2 node info /neupan_node
```

## TF

```bash
ros2 run tf2_ros tf2_echo map rear_axle_link
ros2 run tf2_ros tf2_echo map laser_link
```

## LaserScan

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

## Hybrid A*

```bash
ros2 topic hz /plan
ros2 topic echo /plan --once
```

## NeuPAN raw command

```bash
ros2 topic hz /neupan_cmd_vel_raw
ros2 topic echo /neupan_cmd_vel_raw
```

## Controller command

```bash
ros2 topic echo /ackermann_steering_controller/reference
```

## Odometry

```bash
ros2 topic hz /odom
ros2 topic echo /odom --once
```

---

# 61. 第一轮必须记录的运行数据

建议至少记录：

```text
NeuPAN实际输出频率
/scan频率
/odom频率
最大 v
最大 steering
controller实际 wheel speed
controller实际 steering joints
PAN是否频繁stop
DUNE min distance
CPU占用
```

否则所谓：

```text
“效果不好”
```

很难定位。

---

# 62. 不要把 controller 饱和误认为 NeuPAN 算法差

你的后轮 command limit：

```text
±10 rad/s
```

实体 steering joint：

```text
±0.52 rad
```

如果 NeuPAN 经常请求超过可执行运动范围，
controller 会饱和。

此时看到的是：

```text
规划轨迹很好
实际机器人越走越偏
```

这不是 `eta` 的问题。

而是：

```text
optimizer model ≠ actuator model
```

因此速度和 steering limit 属于第一批必须匹配的参数。

---

# 63. 暂时不要追求“倒车比前进慢”

这是很容易让系统再次复杂化的一点。

当前官方核心模型通常对：

```text
max_speed
```

做对称限制。

第一版先接受：

```text
|reverse speed| == |forward speed limit|
```

但把整体速度降到安全值。

等：

```text
正向
倒车
避障
```

都跑通后，

如果实际确实需要：

```text
reverse max 0.3
forward max 0.7
```

再单独实现非对称速度约束。

---

# 64. Hybrid A* 的参数也不要同时大改

NeuPAN 重建期间：

Hybrid A* 应尽量保持一个已经确定可以正常搜索的版本。

只要确保：

```text
minimum turning radius
reference frame
path orientation
```

和 Ackermann 实际运动学一致即可。

不要同时重新调：

```text
reverse penalty
gear-change penalty
analytic expansion
cost penalty
smoother
```

否则 NeuPAN 表现变化可能其实来自 global path 改了。

---

# 65. 测试环境建议固定

为了比较不同 commit：

准备 3 个固定世界：

```text
A. 空旷 + S弯
B. 窄走廊
C. 路径中央单个静态障碍
```

然后每个版本都在同一：

```text
start
goal
map
robot pose
```

下测试。

否则不能比较参数。

---

# 66. 建议建立 upstream 差异文档

新增：

```text
src/neupan_ros2/UPSTREAM_NOTES.md
```

只记录：

```text
upstream repository
upstream commit
local changes
为什么必须改
测试方法
```

例如：

```markdown
## Local patch 1: Ackermann reverse gear

Reason:
Hybrid A* outputs Reeds-Shepp path with reverse segments.
Upstream ROS2 wrapper sets all path gear values to +1.

Scope:
neupan_node.py::path_callback only.

No planner weight changes.
```

以后更新官方源码时非常有用。

---

# 67. 不要直接把旧 `src/neupan_ros2` 文件覆盖回去

如果之前某个功能需要恢复：

正确方式：

```text
阅读旧版本对应 diff
        ↓
理解它解决什么
        ↓
在新官方基线上重新实现最小版本
```

而不是：

```text
cp old/neupan_node.py new/neupan_node.py
```

否则等于重新把旧复杂度全部带回来。

---

# 68. 推荐的最终目录职责

```text
src/
├── neupan_ros2/
│   ├── upstream NeuPAN ROS wrapper
│   ├── ackermann_robot config
│   └── minimal compatibility patch only
│
├── hybrid_astar_planner/
│   └── global path
│
├── ackermann_control/
│   ├── controller config
│   ├── NeuPAN Ackermann command adapter
│   └── command mux / safety timeout
│
├── ackermann_simulation/
│   └── physical robot model
│
└── ackermann_bringup/
    └── decides which modules to launch
```

这个边界很重要。

---

# 69. NeuPAN ROS2 不应负责知道太多项目细节

尽量不要让 `neupan_node.py` 逐渐知道：

```text
你的 stop state machine
你的 Hybrid A* implementation
你的 Gazebo wheel joints
你的 controller topic internal details
你的 navigation status
```

NeuPAN 节点最好只知道：

```text
robot pose
scan
initial path
planner config
cmd output
```

项目细节放 adapter / bringup。

这样以后换 NeuPAN upstream 版本不会痛苦。

---

# 70. 本次迁移的“不要做”清单

第一阶段禁止：

- [ ] 直接复制旧 `src/neupan_ros2`
- [ ] 一次修改十几个 planner 参数
- [ ] 修改 NeuPAN pip package 源码
- [ ] 用实体前轮 `0.52 rad` 直接当 bicycle steering limit
- [ ] 保留 `max_speed=2 m/s`
- [ ] 同时让 Hybrid A* 和 NeuPAN 处理 `/goal_pose`
- [ ] 把 steering angle 直接当 `Twist.angular.z`
- [ ] 使用 `ind_range: 0`
- [ ] 加入 upstream 不认识的自创参数
- [ ] DUNE geometry 改了但 checkpoint 不管
- [ ] 一开始关闭所有可视化然后靠猜
- [ ] 一开始加入所有旧版 watchdog / limiter / seed 功能
- [ ] 出问题第一反应就调 `eta/q_s/p_u`

---

# 71. 本次迁移的“必须先做”清单

- [ ] 从官方仓库重新复制干净 `src/neupan_ros2`
- [ ] 记录 upstream commit
- [ ] 记录实际安装的 `neupan` 核心版本
- [ ] 建立 `ackermann_robot/` 配置
- [ ] 从 Ranger planner 参数开始
- [ ] `wheelbase → 0.593`
- [ ] `base_frame → rear_axle_link`
- [ ] `lidar_frame → laser_link`
- [ ] `scan_range_min → 你的 LiDAR 有效最小距离`
- [ ] 把 `ref_speed` 降到执行器真实可实现范围
- [ ] 把 `max_speed[0]` 限制到执行器真实范围
- [ ] bicycle steering limit 与 Ackermann 几何一致
- [ ] NeuPAN goal topic 与 Hybrid A* 隔离
- [ ] 使用 raw command topic
- [ ] 在项目控制层做 steering→yaw-rate 转换
- [ ] 正向路径跑通后再做 reverse gear 补丁
- [ ] 障碍测试前确认 DUNE geometry/checkpoint 策略

---

# 72. 建议第一轮最终只允许出现 4 类本地差异

与官方 `neupan_ros2` 相比，
在第一次可运行版本里最好只有：

```text
1. ackermann_robot 配置目录
2. ackermann_robot.launch.py
3. reverse gear 小补丁（如果需要倒车）
4. PAN config merge 小补丁（仅当版本确认有问题）
```

Ackermann 控制量转换和 watchdog 尽量放在：

```text
ackermann_control
```

而不是 `neupan_ros2`。

这样你的 NeuPAN ROS2 目录会保持非常接近 upstream。

---

# 73. 推荐的第一版参数摘要

第一版核心目标不是“最佳”，而是“官方基线 + 必要机器人限制”。

```yaml
receding: 15
step_time: 0.2
ref_speed: 0.5
device: 'cpu'
time_print: False
collision_threshold: 0.01

robot:
  kinematics: 'acker'
  max_speed: [0.7, 0.43]
  max_acce: [1.0, 0.328]
  length: 0.720
  width: 0.500
  wheelbase: 0.593

ipath:
  interval: 0.03
  curve_style: 'dubins'
  min_radius: 0.81
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
  q_s: 0.1
  p_u: 0.5
  eta: 15.0
  d_max: 0.1
  d_min: 0.01
```

其中真正主动偏离官方 Ranger 的只有：

```text
ref_speed
max linear speed
max steering
wheelbase
```

理由都是：

```text
你的机器人执行器 / Ackermann 几何不允许直接照抄
```

这符合本次重做原则。

---

# 74. 后续参数不要预先决定

第一版跑通以后再根据症状决定：

```text
是否降低 receding
是否降低 control_frequency
是否降低 dune_max_num
是否改变 collision threshold
是否改变 q_s
是否改变 eta
是否重训 DUNE
```

不要现在一次性把未来的优化全部塞进基线。

---

# 75. 最后一个原则：每一次修改都必须能回答三个问题

修改任何文件前先写：

```text
1. 当前具体问题是什么？
2. 为什么官方行为不适合我的机器人？
3. 我如何证明这次修改解决了问题？
```

如果三个问题答不出来：

**先不要改。**

这可能是这次从官方重新开始后，最重要的一条规则。

---

# 附录 A：建议关注的官方文件

NeuPAN ROS2：

```text
https://github.com/KevinLADLee/neupan_ros2
```

重点：

```text
src/neupan_ros2/neupan_ros2/neupan_node.py
src/neupan_ros2/config/robots/ranger/robot.yaml
src/neupan_ros2/config/robots/ranger/planner.yaml
src/neupan_ros2/launch/ranger.launch.py
src/neupan_ros2/setup.py
src/neupan_ros2/README_cn.md
```

NeuPAN 核心：

```text
https://github.com/hanruihua/NeuPAN
```

重点：

```text
neupan/neupan.py
neupan/robot/robot.py
neupan/blocks/initial_path.py
neupan/blocks/pan.py
neupan/blocks/nrmp.py
neupan/blocks/dune.py
```

你的项目：

```text
https://github.com/YansenSong/AckermannRobot-3D-ROS2
```

重点：

```text
src/ackermann_simulation/robot/xacro/chassis.xacro
src/ackermann_simulation/robot/xacro/control.xacro
src/ackermann_control/config/ackermann_controllers.yaml
src/ackermann_control/cmd_vel_mux.py
Hybrid A* planner config / Path publisher
ackermann_bringup launch files
```

---

# 附录 B：建议的实施里程碑

```text
M0  导入官方源码，零修改编译成功
M1  NeuPAN 能加载模型、TF、scan
M2  NeuPAN 能收到 Hybrid A* /plan
M3  raw Ackermann command 数值正确
M4  adapter 后车辆直线/左右转正确
M5  Hybrid A* 前进路径稳定跟踪
M6  Reeds-Shepp reverse 正确
M7  DUNE geometry/model 一致
M8  静态避障稳定
M9  stale command / stop safety 完成
M10 再开始正式算法调参
```

只要严格按这个顺序做，后面出现问题时定位成本会低很多。
