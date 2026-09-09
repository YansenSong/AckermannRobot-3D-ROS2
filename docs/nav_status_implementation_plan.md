# `nav_status` 导航状态机实施计划

> 项目：`YansenSong/AckermannRobot-3D-ROS2`  
> 目标目录：`src/nav_status`  
> 目标：增加一个**纯观察者（observer-only）**导航状态节点，通过 ROS 2 topic 对外发布导航状态，不负责发送目标、不负责规划、不负责控制、不改变现有 Hybrid A* + NeuPAN 导航链路。

---

## 1. 实施目标

在现有导航系统外增加一个独立的 `nav_status` ROS 2 package，汇总以下已有信号：

- `/goal_pose`：收到新导航目标；
- `/plan_path`：Hybrid A* 成功生成全局路径；
- `/global_path_remaining_distance`：当前沿全局路径到终点的剩余距离；
- `/odom_wheel`：车辆实际运动速度；

并发布统一导航状态：

```text
WAITING_FOR_GOAL
PLANNING
MOVING
ARRIVED
FAILED        # 预留，V1 暂不自动进入
```

核心要求：

1. `nav_status` **只能观察现有 topic**，不得调用 `/plan` 服务，不得向 Hybrid A*、NeuPAN 或底盘发命令；
2. 不改变 `/goal_pose -> Hybrid A* -> /plan -> NeuPAN -> cmd_vel_mux` 的现有数据流；
3. V1 不通过 timeout 猜测规划失败；`FAILED` 只在消息定义和内部状态枚举中预留；
4. 状态变化应立即发布，同时使用 transient-local QoS，使后启动的 UI/上位机能立刻获得当前状态；
5. 到达判定必须同时考虑**剩余路径距离 + 实际车速 + 持续时间**，避免瞬时误判。

---

## 2. 当前项目事实与设计依据

当前仓库的导航链路已经是：

```text
/goal_pose
    ↓
Hybrid A*
    ├── /plan
    ├── /plan_path
    └── /global_path_remaining_distance
            ↓
        NeuPAN
            ↓ /neupan_cmd_vel
        cmd_vel_mux
            ↓
/ackermann_steering_controller/reference
```

相关现有代码：

- `src/hybrid_astar_planner/standalone_planner/src/planner_node.cpp`
  - 订阅 `/goal_pose`；
  - 规划成功后发布 `/plan`、`/plan_path`；
  - 维护 `active_path_`；
  - 约 10 Hz 发布 `/global_path_remaining_distance`；
  - 当前规划失败时不会发布一个明确的 failure topic，也不会发布空 `/plan_path`。
- `docs/topic_info.md`
  - 已记录 `/goal_pose`、`/plan_path`、`/global_path_remaining_distance`、`/odom_wheel` 等接口。
- `src/ackermann_bringup/launch/navigation.launch.py`
  - 当前组合 localization、planning、scan、cmd_vel mux 和 RViz；
  - `nav_status` 应在这里被启动。
- `src/ackermann_bringup/launch/navigation_sim.launch.py`
  - 已经 include `navigation.launch.py`；
  - 因此不要再次单独启动 `nav_status`，否则会重复实例化节点。

本项目目前不是标准 Nav2 `NavigateToPose` action 驱动的导航链路，而是 Hybrid A* + NeuPAN 的 topic-based 链路，因此 V1 不依赖 Nav2 action status。

---

## 3. 状态定义

建议新增自定义消息：

`src/nav_status/msg/NavigationStatus.msg`

```text
uint8 WAITING_FOR_GOAL=0
uint8 PLANNING=1
uint8 MOVING=2
uint8 ARRIVED=3
uint8 FAILED=4

builtin_interfaces/Time stamp
uint8 state
string detail
```

说明：

- `state` 是程序逻辑使用的稳定枚举；
- `detail` 只用于调试/日志/UI 提示，不应用于其他节点的状态判断；
- `FAILED` 在 V1 保留，但暂时没有自动转换入口。

推荐发布 topic：

```text
/navigation/state
```

消息类型：

```text
nav_status/msg/NavigationStatus
```

---

## 4. 状态机定义

### 4.1 状态转移图

```text
                    system startup
                         │
                         ▼
                ┌──────────────────┐
                │ WAITING_FOR_GOAL │
                └────────┬─────────┘
                         │ new /goal_pose
                         ▼
                ┌──────────────────┐
                │     PLANNING     │
                └────────┬─────────┘
                         │ next fresh non-empty /plan_path
                         ▼
                ┌──────────────────┐
                │      MOVING      │
                └────────┬─────────┘
                         │ arrival condition held long enough
                         ▼
                ┌──────────────────┐
                │     ARRIVED      │
                └──────────────────┘

任意状态 + new /goal_pose  → PLANNING

FAILED：V1 仅预留，不主动进入。
```

### 4.2 状态语义

#### `WAITING_FOR_GOAL`

含义：

- 节点已经启动；
- 当前没有正在跟踪的新导航任务。

进入条件：

- 节点启动时直接进入。

V1 中 `ARRIVED` 不自动退回 `WAITING_FOR_GOAL`；`ARRIVED` 保持到下一个新目标到来，这样 UI 能稳定看到任务已经完成。

#### `PLANNING`

含义：

- 已观察到新的 `/goal_pose`；
- 正等待 Hybrid A* 对该目标生成新的有效路径。

进入条件：

- 在任意状态收到新的 `/goal_pose`。

退出条件：

- 收到新的、非空 `/plan_path`：进入 `MOVING`。

注意：

- 不要在 `PLANNING` 状态自行调用 planner；
- 不要通过固定 timeout 自动转 `FAILED`；
- 现有 Hybrid A* 每次成功目标只主动发布一次 `/plan_path`，因此状态节点可把“进入 PLANNING 后收到的下一条非空 `/plan_path`”视为当前目标的规划成功结果。

#### `MOVING`

含义：

- 当前目标已有有效全局路径；
- 导航任务处于执行阶段。

这里的 `MOVING` 表示“导航执行 active”，不要求车辆每一瞬间都具有非零速度。车辆可能因 NeuPAN 避障、局部停车等原因瞬时速度为 0，但仍应保持 `MOVING`。

进入条件：

- `PLANNING` 状态收到非空 `/plan_path`。

退出条件：

- 满足到达判定并持续足够时间：进入 `ARRIVED`；
- 收到新 `/goal_pose`：重新进入 `PLANNING`。

#### `ARRIVED`

含义：

- 车辆已经接近当前全局路径终点；
- 实际速度已经接近停止；
- 上述条件持续了一定时间，排除瞬时误判。

进入后保持该状态，直到收到新 `/goal_pose`。

#### `FAILED`

V1：只定义、不自动进入。

原因：当前 Hybrid A* 规划失败时没有提供一个可靠、明确的 failure topic。纯观察者若仅依靠 timeout 无法区分：

- planner 仍在计算；
- TF 暂时不可用；
- CPU 较慢；
- 真正规划失败。

因此禁止 V1 用猜测逻辑进入 `FAILED`。

未来启用方式见第 12 节。

---

## 5. 到达判定

不能只依赖：

```text
/global_path_remaining_distance < threshold
```

原因：当前 remaining distance 是“机器人投影到全局路径后，沿路径到终点的剩余弧长”。在终点附近或略微越过终点时，仅靠该值可能过早判定到达。

V1 使用三个条件：

```text
remaining_distance <= arrival_distance_threshold
AND
actual_speed <= stopped_speed_threshold
AND
条件连续成立 >= arrival_hold_time
```

建议默认值：

```yaml
arrival_distance_threshold: 0.20   # m
stopped_speed_threshold: 0.05      # m/s
arrival_hold_time: 0.80            # s
```

实际速度来源：

```text
/odom_wheel
nav_msgs/msg/Odometry
```

计算：

```cpp
double speed = std::hypot(
    odom.twist.twist.linear.x,
    odom.twist.twist.linear.y);
```

不要使用 `/neupan_cmd_vel` 做到达速度判断，因为它表示控制命令而非车辆实际运动状态。

### 5.1 输入 freshness

到达判定还应要求 remaining distance 和 odom 都是近期数据，避免使用失联前的旧值误判。

建议参数：

```yaml
input_timeout: 1.0   # s
```

逻辑：

- 任一输入从未收到：不允许 ARRIVED；
- 任一输入距当前时间超过 `input_timeout`：不允许 ARRIVED；
- freshness 不满足时清空 arrival hold timer。

### 5.2 debounce / hold

维护：

```cpp
std::optional<rclcpp::Time> arrival_candidate_since_;
```

当两个阈值首次同时满足：记录当前时刻。

只要任一条件重新不满足：立即 reset。

持续超过 `arrival_hold_time` 后：

```text
MOVING -> ARRIVED
```

---

## 6. ROS 接口

### 6.1 Subscribers

| Topic | Type | 用途 |
|---|---|---|
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 新目标事件，任意状态切换到 `PLANNING` |
| `/plan_path` | `nav_msgs/msg/Path` | 在 `PLANNING` 中观察规划成功，非空路径切换到 `MOVING` |
| `/global_path_remaining_distance` | `std_msgs/msg/Float64` | 到达距离条件 |
| `/odom_wheel` | `nav_msgs/msg/Odometry` | 到达速度条件 |

以上 topic 名都建议做成参数，默认值保持现有仓库接口。

### 6.2 Publisher

默认：

```text
/navigation/state
nav_status/msg/NavigationStatus
```

QoS：

```cpp
rclcpp::QoS qos(rclcpp::KeepLast(1));
qos.reliable();
qos.transient_local();
```

理由：导航状态属于 state，不是瞬时 telemetry。新启动的 UI/监控节点应该立即拿到最后一个已知状态。

### 6.3 发布策略

必须：

- 状态变化时立即 publish；
- 节点启动后立即发布 `WAITING_FOR_GOAL`。

建议额外增加低频 heartbeat：

```yaml
status_heartbeat_rate: 1.0   # Hz
```

heartbeat 重发当前状态，便于日志和系统健康监控。

---

## 7. 参数设计

建议配置文件：

`src/nav_status/config/nav_status.yaml`

```yaml
nav_status_node:
  ros__parameters:
    goal_topic: "/goal_pose"
    plan_topic: "/plan_path"
    remaining_distance_topic: "/global_path_remaining_distance"
    odom_topic: "/odom_wheel"
    status_topic: "/navigation/state"

    arrival_distance_threshold: 0.20
    stopped_speed_threshold: 0.05
    arrival_hold_time: 0.80
    input_timeout: 1.0

    evaluation_rate: 20.0
    status_heartbeat_rate: 1.0
```

不要在 package YAML 中强耦合 `use_sim_time`；由 bringup launch 根据当前系统传入。当前项目仿真启动时可继续传：

```python
{'use_sim_time': True}
```

---

## 8. 推荐 package 结构

新增：

```text
src/nav_status/
├── CMakeLists.txt
├── package.xml
├── msg/
│   └── NavigationStatus.msg
├── src/
│   └── nav_status_node.cpp
└── config/
    └── nav_status.yaml
```

V1 不需要额外 launch 文件；统一由 `ackermann_bringup/launch/navigation.launch.py` 启动。

---

## 9. C++ 节点实现建议

类名建议：

```cpp
class NavStatusNode : public rclcpp::Node
```

内部状态枚举可直接映射自定义消息常量：

```cpp
enum class State : uint8_t {
  WAITING_FOR_GOAL = NavigationStatus::WAITING_FOR_GOAL,
  PLANNING = NavigationStatus::PLANNING,
  MOVING = NavigationStatus::MOVING,
  ARRIVED = NavigationStatus::ARRIVED,
  FAILED = NavigationStatus::FAILED,
};
```

### 9.1 建议成员

```cpp
State state_;

std::optional<double> remaining_distance_;
std::optional<double> actual_speed_;

std::optional<rclcpp::Time> remaining_distance_stamp_;
std::optional<rclcpp::Time> odom_stamp_;
std::optional<rclcpp::Time> arrival_candidate_since_;

rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr remaining_distance_sub_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

rclcpp::Publisher<nav_status::msg::NavigationStatus>::SharedPtr status_pub_;
rclcpp::TimerBase::SharedPtr evaluation_timer_;
rclcpp::TimerBase::SharedPtr heartbeat_timer_;
```

如果节点使用默认 single-threaded executor，可不必为了 V1 引入额外 mutex；但代码应避免 callback 中做阻塞操作。

### 9.2 核心方法

```cpp
void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
void onPlan(const nav_msgs::msg::Path::SharedPtr msg);
void onRemainingDistance(const std_msgs::msg::Float64::SharedPtr msg);
void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

void evaluateArrival();
void transitionTo(State next, const std::string & detail);
void publishStatus(const std::string & detail = "");
```

### 9.3 `onGoal()`

伪代码：

```cpp
void onGoal(...)
{
  arrival_candidate_since_.reset();
  transitionTo(State::PLANNING, "new goal received; waiting for global plan");
}
```

要求：

- 无论当前是 `WAITING_FOR_GOAL`、`PLANNING`、`MOVING`、`ARRIVED`，新目标都进入/保持 `PLANNING`；
- 即使 state 已经是 `PLANNING`，新的 goal 也应该重新 publish 状态或至少更新 detail/log，表示这是一次新的导航任务；
- 不调用 planner。

### 9.4 `onPlan()`

伪代码：

```cpp
void onPlan(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (state_ != State::PLANNING) {
    return;
  }

  if (msg->poses.empty()) {
    return;
  }

  arrival_candidate_since_.reset();
  transitionTo(State::MOVING, "global plan available; navigation active");
}
```

不要因为在 `MOVING` 状态再次观察到 path 就重新触发状态。

### 9.5 `onRemainingDistance()`

```cpp
remaining_distance_ = msg->data;
remaining_distance_stamp_ = now();
```

若值为 NaN / Inf：忽略该样本，且不要用它做 ARRIVED 判断。

### 9.6 `onOdom()`

```cpp
actual_speed_ = std::hypot(
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y);
odom_stamp_ = now();
```

### 9.7 `evaluateArrival()`

只在 `MOVING` 时执行 arrival 判断。

伪代码：

```cpp
if (state_ != State::MOVING) {
  arrival_candidate_since_.reset();
  return;
}

if (!remaining_distance_ || !actual_speed_ ||
    !remaining_distance_stamp_ || !odom_stamp_) {
  arrival_candidate_since_.reset();
  return;
}

const auto t = now();

if ((t - *remaining_distance_stamp_).seconds() > input_timeout_ ||
    (t - *odom_stamp_).seconds() > input_timeout_) {
  arrival_candidate_since_.reset();
  return;
}

const bool close_enough =
    *remaining_distance_ <= arrival_distance_threshold_;

const bool stopped =
    *actual_speed_ <= stopped_speed_threshold_;

if (!close_enough || !stopped) {
  arrival_candidate_since_.reset();
  return;
}

if (!arrival_candidate_since_) {
  arrival_candidate_since_ = t;
  return;
}

if ((t - *arrival_candidate_since_).seconds() >= arrival_hold_time_) {
  transitionTo(State::ARRIVED, "goal reached and vehicle stopped");
  arrival_candidate_since_.reset();
}
```

---

## 10. 状态日志

只在状态变化时打印 INFO：

```text
Navigation state: WAITING_FOR_GOAL -> PLANNING
Navigation state: PLANNING -> MOVING
Navigation state: MOVING -> ARRIVED
```

输入超时、无效 remaining distance 等不要高频刷屏；若需要日志，使用 throttle WARN/DEBUG。

建议 `transitionTo()` 统一负责：

1. 修改 `state_`；
2. 打印 transition；
3. 更新最近 detail；
4. 立即 publish 状态。

避免在各 callback 中重复写状态发布逻辑。

---

## 11. 构建配置

### 11.1 `src/nav_status/package.xml`

至少依赖：

```text
ament_cmake
rclcpp
geometry_msgs
nav_msgs
std_msgs
builtin_interfaces
rosidl_default_generators
rosidl_default_runtime
```

并添加：

```xml
<member_of_group>rosidl_interface_packages</member_of_group>
```

### 11.2 `src/nav_status/CMakeLists.txt`

需要：

1. `find_package(...)` 上述依赖；
2. `rosidl_generate_interfaces()` 生成 `NavigationStatus.msg`；
3. 编译 `src/nav_status_node.cpp`；
4. 将本 package 生成的 C++ message typesupport 链接到该 executable；
5. 安装 executable 到 `lib/nav_status`；
6. 安装 `config/` 到 `share/nav_status`；
7. export `rosidl_default_runtime`。

Codex 应根据仓库 ROS 2 发行版实际可用的 rosidl CMake API 使用标准写法，不要额外创建第二个 messages package。

---

## 12. `FAILED` 的未来启用方式

V1 不修改 `hybrid_astar_planner`。

未来如需要真正启用：

```text
PLANNING -> FAILED
```

应先给 Hybrid A* 增加**明确可观察的规划结果事件**，例如：

```text
/planning/result
```

不要让 `nav_status` 自己调用 planner，也不要仅靠 timeout 猜失败。

未来 result 至少应能表达：

```text
SUCCESS
FAILED
```

届时 `nav_status` 仍然只是订阅结果：

```text
PLANNING + planning FAILED -> FAILED
FAILED + new /goal_pose -> PLANNING
```

如果未来要正式设计该接口，优先使用枚举型自定义消息而不是自由字符串。

---

## 13. Bringup 集成

修改：

```text
src/ackermann_bringup/package.xml
```

新增：

```xml
<exec_depend>nav_status</exec_depend>
```

修改：

```text
src/ackermann_bringup/launch/navigation.launch.py
```

增加 package share：

```python
nav_status_share = get_package_share_directory('nav_status')
```

增加节点：

```python
nav_status = Node(
    package='nav_status',
    executable='nav_status_node',
    name='nav_status_node',
    output='screen',
    parameters=[
        os.path.join(nav_status_share, 'config', 'nav_status.yaml'),
        {'use_sim_time': True},
    ],
)
```

把 `nav_status` 加入最终 `LaunchDescription`。

不要修改 `navigation_sim.launch.py` 来再次启动该节点，因为它已经 include `navigation.launch.py`。

---

## 14. 文档更新

修改：

```text
docs/topic_info.md
```

增加：

| 话题 | 类型 | 作用 |
|---|---|---|
| `/navigation/state` | `nav_status/msg/NavigationStatus` | 当前导航任务状态：等待目标、规划、移动、到达；FAILED 预留 |

并补充状态说明：

```text
WAITING_FOR_GOAL -> PLANNING -> MOVING -> ARRIVED
```

以及：

```bash
ros2 topic echo /navigation/state
```

---

## 15. V1 明确禁止的实现

Codex 实施时不要做以下事情：

- 不要把状态机写进 `hybrid_astar_planner`；
- 不要把状态机写进 `neupan_ros2`；
- 不要修改 NeuPAN 控制逻辑；
- 不要修改 `/goal_pose` 发布/订阅链路；
- 不要让 `nav_status` 调 `/plan` service；
- 不要让 `nav_status` 发布 `/plan`；
- 不要让 `nav_status` 发布速度命令；
- 不要自动 cancel goal；
- 不要为了 FAILED 增加随意 timeout；
- 不要把 `MOVING` 定义成 `speed > 0`；
- 不要只使用 remaining distance 判断 ARRIVED；
- 不要在 `navigation_sim.launch.py` 重复启动状态节点；
- 不要使用 `std_msgs/String` 作为正式状态接口。

---

## 16. 测试计划

### 16.1 构建

至少执行：

```bash
colcon build --packages-select nav_status ackermann_bringup
source install/setup.bash
```

如果接口生成依赖导致第一次选择性编译不完整，则按实际依赖补充 package 或执行完整 `colcon build`。

### 16.2 单节点 topic 注入测试

启动：

```bash
ros2 run nav_status nav_status_node --ros-args --params-file \
  src/nav_status/config/nav_status.yaml
```

观察：

```bash
ros2 topic echo /navigation/state
```

#### Case A：启动状态

期望：

```text
WAITING_FOR_GOAL
```

#### Case B：发布目标

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{...}"
```

期望：

```text
PLANNING
```

#### Case C：发布非空 path

向 `/plan_path` 发布包含至少一个 pose 的 `nav_msgs/msg/Path`。

期望：

```text
MOVING
```

#### Case D：未到达

发布：

```text
remaining_distance = 1.0
speed = 0.0
```

期望：仍为 `MOVING`。

再发布：

```text
remaining_distance = 0.10
speed = 0.20
```

期望：仍为 `MOVING`。

#### Case E：到达 debounce

持续发布：

```text
remaining_distance = 0.10
speed = 0.00
```

持续时间短于 `arrival_hold_time`：仍为 `MOVING`。

持续超过 `arrival_hold_time`：进入 `ARRIVED`。

#### Case F：到达后新目标

在 `ARRIVED` 时发布新 `/goal_pose`。

期望：

```text
ARRIVED -> PLANNING
```

#### Case G：移动过程中重新设目标

在 `MOVING` 时发布新 `/goal_pose`。

期望：

```text
MOVING -> PLANNING
```

随后收到新的非空 `/plan_path`：

```text
PLANNING -> MOVING
```

#### Case H：输入 freshness

进入 `MOVING` 后停止更新 `/odom_wheel` 或 `/global_path_remaining_distance` 超过 `input_timeout`。

即使最后缓存值满足到达阈值，也不能进入 `ARRIVED`。

#### Case I：transient-local

当系统已经处于 `MOVING` 或 `ARRIVED` 时，新开一个：

```bash
ros2 topic echo /navigation/state
```

确认新 subscriber 可以得到当前状态，而无需等待下一次状态变化。

### 16.3 全系统测试

运行现有导航 launch，确认：

```text
startup        -> WAITING_FOR_GOAL
RViz set goal  -> PLANNING
Hybrid A* path -> MOVING
vehicle arrive -> ARRIVED
new RViz goal  -> PLANNING
```

同时确认：

- Hybrid A* `/plan` 与 `/plan_path` 行为无变化；
- NeuPAN 输入输出无变化；
- `cmd_vel_mux` 行为无变化；
- 原有导航可在完全不订阅 `/navigation/state` 的情况下正常工作。

这一条是“观察者模式”是否被保持的关键验收项。

---

## 17. 验收标准（Definition of Done）

全部满足后才视为完成：

- [ ] 新增 `src/nav_status` package；
- [ ] 自定义 `NavigationStatus.msg` 成功生成；
- [ ] 状态常量仅为 `WAITING_FOR_GOAL / PLANNING / MOVING / ARRIVED / FAILED`；
- [ ] 启动立即发布 `WAITING_FOR_GOAL`；
- [ ] 新 `/goal_pose` 进入 `PLANNING`；
- [ ] `PLANNING` 收到非空 `/plan_path` 进入 `MOVING`；
- [ ] `MOVING` 使用 remaining distance + odom speed + hold time 判定 `ARRIVED`；
- [ ] 输入 freshness 生效；
- [ ] `ARRIVED` 收到新目标进入 `PLANNING`；
- [ ] `MOVING` 收到新目标进入 `PLANNING`；
- [ ] `FAILED` 已定义但 V1 没有 timeout 猜测式入口；
- [ ] `/navigation/state` 使用 reliable + transient-local QoS；
- [ ] `navigation.launch.py` 启动 `nav_status_node`；
- [ ] `navigation_sim.launch.py` 没有重复启动该节点；
- [ ] `ackermann_bringup/package.xml` 增加 `nav_status` runtime dependency；
- [ ] `docs/topic_info.md` 更新；
- [ ] 不修改 Hybrid A*、NeuPAN、cmd_vel mux 的核心导航/控制行为；
- [ ] `colcon build` 成功；
- [ ] 手工 topic 测试通过；
- [ ] 完整仿真导航状态转换通过。

---

## 18. 建议 Codex 的执行顺序

1. 阅读：
   - `src/hybrid_astar_planner/standalone_planner/src/planner_node.cpp`
   - `docs/topic_info.md`
   - `src/ackermann_bringup/launch/navigation.launch.py`
   - `src/ackermann_bringup/launch/navigation_sim.launch.py`
   - `src/ackermann_bringup/package.xml`
2. 创建 `src/nav_status` package 和 `NavigationStatus.msg`；
3. 实现 `nav_status_node.cpp`；
4. 添加 YAML 参数文件；
5. 编译 `nav_status`，先解决接口生成/链接问题；
6. 使用人工 topic 注入完成状态机单元级验证；
7. 修改 `ackermann_bringup` 进行 launch 集成；
8. 更新 `docs/topic_info.md`；
9. 完整 `colcon build`；
10. 运行现有仿真导航并验证真实状态流；
11. 最终检查 git diff，确保没有越过 observer-only 边界。

---

## 19. 最终目标架构

```text
                        ┌────────────────────────────┐
/goal_pose ────────────→│                            │
/plan_path ────────────→│       nav_status_node      │──→ /navigation/state
remaining_distance ────→│       (observer only)      │
/odom_wheel ───────────→│                            │
                        └────────────────────────────┘

/goal_pose
    │
    └────────→ Hybrid A* ──→ /plan ──→ NeuPAN ──→ cmd_vel_mux ──→ vehicle

nav_status_node 不位于控制闭环中。
即使 nav_status_node 崩溃，现有导航本身仍应继续工作。
```

这个“节点退出不影响导航”的性质，是本次设计最重要的架构约束之一。
