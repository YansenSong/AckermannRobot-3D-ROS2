# Ackermann Nav2 机器人参数适配与调参指南

本文用于后续把当前独立的 Ackermann Nav2 导航栈迁移到其他 Ackermann 底盘。目标是：给出新机器人的尺寸、转向、速度、制动、传感器和控制接口参数后，可以按本文推导并修改导航配置，再通过固定的验证顺序完成调参。

本文针对当前项目的链路：

```text
地图 / 定位
    -> Nav2 global costmap
    -> Smac Hybrid-A*（DUBIN，前进约束）
    -> MPPI Ackermann Controller
    -> Velocity Smoother
    -> Twist -> TwistStamped bridge
    -> 机器人底盘控制器
```

当前方案默认机器人只能前进，不使用倒车；局部避障效果还会受到 NeuPAN 或其他下游跟踪器影响。因此，本指南首先保证 Nav2 输出的全局/局部轨迹符合新底盘的几何和动力学约束，再单独处理下游控制器的跟踪能力。

## 1. 当前基线和配置入口

当前 Ackermann Nav2 栈的主要入口如下：

| 内容 | 文件 |
|---|---|
| Nav2 参数 | `src/ackermann_nav/config/nav2_params.yaml` |
| 点云转 LaserScan 参数 | `src/ackermann_nav/config/pcl_to_scan.yaml` |
| 导航启动与生命周期管理 | `src/ackermann_nav/launch/navigation.launch.py` |
| `Twist` 转 `TwistStamped` 桥接 | `src/ackermann_nav/scripts/cmd_bridge.py` |
| 一键启动 | `scripts/run_ackermann_nav.sh` |

当前仿真基线可作为起始参考，不应直接复制到新机器人：

| 参数 | 当前基线 |
|---|---:|
| 参考车体坐标系 | `rear_axle_link` |
| 轴距 `L` | `0.593 m` |
| 最小转弯半径 `Rmin` | `1.320 m` |
| 足迹 | `x=-0.10~0.74 m`，`y=-0.36~0.36 m` |
| 最大线速度 | `0.70 m/s` |
| 最小线速度 | `0.0 m/s`，不允许倒车 |
| 控制器频率 | `20 Hz` |
| MPPI 时间步 | `45` |
| MPPI 模型步长 | `0.05 s` |
| MPPI 预测时域 | `2.25 s` |
| MPPI batch size | `2000` |
| 局部代价地图 | `10 m x 10 m`，分辨率 `0.05 m` |
| 规划重规划频率 | `1 Hz` |

这些数值只说明当前仿真的工作点。迁移时优先重新测量和计算 `reference_frame`、`footprint`、`wheelbase`、`minimum_turning_radius`、速度/角速度上限及制动距离。

## 2. 迁移前必须提供的机器人参数

后续更换机器人时，建议先把下面模板填写完整，再开始改 YAML。缺少的数据要标记为“未知”，不要用旧机器人的数值代替。

```yaml
robot:
  name: "新机器人名称"
  reference_frame: "rear_axle_link"       # Nav2 规划和控制参考点
  base_frame: "base_link"                 # 机器人主体 TF

geometry:
  wheelbase_m: 0.000                       # 前后轴中心距 L
  track_width_m: 0.000                     # 左右轮距 W
  front_overhang_m: 0.000                  # 前轴到车体最前端
  rear_overhang_m: 0.000                   # 后轴到车体最后端
  body_width_m: 0.000
  body_height_m: 0.000
  footprint_source: "measured|urdf|collision"

steering:
  max_steering_angle_deg: 0.0              # 参考：前轮等效转角
  max_inner_wheel_angle_deg: 0.0           # 如果可获得
  max_outer_wheel_angle_deg: 0.0           # 如果可获得
  steering_rate_deg_s: 0.0
  steering_delay_s: 0.0
  measured_min_turning_radius_m: 0.0       # 实车/仿真原地画圆测得

dynamics:
  max_forward_speed_mps: 0.0
  normal_forward_speed_mps: 0.0
  max_accel_mps2: 0.0
  normal_accel_mps2: 0.0
  comfortable_decel_mps2: 0.0
  emergency_decel_mps2: 0.0
  max_yaw_rate_rps: 0.0                    # 若有限制
  controller_rate_hz: 0.0
  command_timeout_s: 0.0

sensors:
  lidar_frame: "laser_link"
  lidar_topic: "/scan"
  lidar_rate_hz: 0.0
  lidar_range_min_m: 0.0
  lidar_range_max_m: 0.0
  pointcloud_topic: ""
  localization_frame: "map"
  odom_frame: "odom"
  map_resolution_m: 0.0

control_interface:
  controller_type: "twist|twist_stamped|ackermann_drive_stamped"
  command_topic: ""
  command_frame: "base_link"
  wheelbase_used_by_controller_m: 0.0
  steering_angle_semantics: "front_wheel|equivalent|not_applicable"
```

必须区分以下概念：

- `wheelbase` 是前后轴距离，不是车体总长。
- `track_width` 是左右轮距，不是车体宽度。
- `max_steering_angle` 应明确是等效前轮转角、内侧轮转角还是外侧轮转角。
- `minimum_turning_radius` 应以机器人参考点为中心定义，不能只由车体外侧圆估算。
- `angular.z` 在当前 Nav2 控制链中表示期望偏航角速度，不是前轮转角。
- `rear_axle_link`、`base_link` 和激光雷达坐标系的相对位置必须通过 TF 明确给出。

## 3. 从机器人参数推导 Nav2 参数

### 3.1 最小转弯半径

简化自行车模型下：

```text
R_geometry = L / tan(|delta_max|)
```

其中 `L` 为轴距，`delta_max` 为等效前轮最大转角，单位必须使用弧度。

如果能够获得内外侧车轮转角，应考虑 Ackermann 几何和轮距 `W`：

```text
R_from_inner = L / tan(delta_inner_max) + W / 2
R_from_outer = L / tan(delta_outer_max) - W / 2
R_geometry = max(R_from_inner, R_from_outer)
```

最终给 Smac 和 MPPI 的半径应取保守值：

```text
Rmin = max(R_geometry, R_slip, R_control)
```

其中：

- `R_slip`：轮胎打滑、地面摩擦、载荷变化造成的实际半径；
- `R_control`：转向执行器响应、控制周期和转向死区造成的可跟踪半径。

最可靠的方法是在目标载荷和常用地面上，以最大安全转角低速画圆，测量参考点轨迹半径，然后增加约 `5%~15%` 的安全余量。若只是用几何公式，建议也留出同样余量。

当前基线的 `Rmin=1.320 m` 不能直接沿用到新机器人。

### 3.2 角速度上限

Ackermann 运动学近似为：

```text
|omega| <= v / Rmin
omega_at_vmax = vmax / Rmin
```

例如当前基线：

```text
0.70 / 1.320 = 0.53 rad/s
```

因此新机器人至少需要同步检查：

- MPPI 的 `vx_max`、`wz_max`；
- `Twist`/`TwistStamped` 桥接器中的角速度约束；
- 下游控制器根据轴距和转角换算出的角速度；
- 底盘实际允许的最大转向角速度和最大偏航角速度。

如果 `wz_max` 大于 `vmax/Rmin`，规划器可能输出底盘无法实现的急转弯；如果过小，则会使路径转弯过于保守。对前进约束机器人，低速时还应检查 `v≈0` 时不要单独产生较大的 `angular.z`。

### 3.3 足迹和参考点

足迹坐标应以 Nav2 使用的 `robot_base_frame` 或 `reference_frame` 为原点。若参考点是后轴中心：

- `x=0` 表示后轴中心；
- 后悬越过后轴的部分对应负 `x`；
- 前悬和车头对应正 `x`；
- 左右边界对应 `± width/2`，并加上必要的外壳/传感器余量。

推荐先使用矩形足迹，再根据实际外形改为多边形。足迹必须覆盖车体在所有允许转角下的实际占用区域；不能只填写轮胎包络或 `base_link` 附近的一小块区域。

足迹的安全关系可按下面顺序检查：

```text
实际车体包络
    <= footprint
    + footprint_padding
    <= costmap collision model
```

如果足迹过小，机器人可能擦碰障碍物；如果足迹过大，狭窄通道会被错误判定为不可行。改变车体尺寸时，优先改 `footprint`，再根据通道和避障需求复核 inflation 参数。

### 3.4 速度、制动距离和障碍物发现距离

从“感知到停止”的角度估计安全距离：

```text
D_trigger ≈ v * T_total + v^2 / (2 * a_decel)
             + body_front_distance + safety_margin
```

其中：

```text
T_total = T_sensor + T_costmap + T_controller + T_actuator
```

`D_trigger` 不是一个单独的 Nav2 参数，而是由以下链路共同决定：

- 激光雷达的有效量程和刷新频率；
- 点云转 LaserScan 的时间延迟；
- costmap 更新频率和 obstacle layer 是否真的收到数据；
- MPPI/底盘控制器的执行周期；
- 机器人从当前速度减速到零所需的时间和距离；
- 车体前端到参考点的距离；
- 期望安全余量。

如果机器人只有接近临时障碍物时才避障，应按以下顺序排查：

1. `scan` 是否持续发布并且时间戳正常；
2. `laser_link -> base/reference_frame` TF 是否稳定；
3. local costmap 的 obstacle layer 是否订阅了正确的 topic；
4. `obstacle_max_range` 是否小于雷达有效量程；
5. local costmap 窗口是否覆盖制动距离和车体前方区域；
6. 速度是否过高或减速度过小；
7. MPPI 是否在障碍物已经进入 costmap 后仍继续跟踪旧轨迹。

只有在上述链路正确后，才通过降低速度、增加减速能力或增加安全余量来调整行为。单纯把 inflation 半径调大，不能替代传感器和制动距离配置。

## 4. 机器人参数到配置项的对应关系

| 机器人输入 | 推导量 | 主要配置/代码入口 | 优先级 |
|---|---|---|---|
| 参考坐标系 | Nav2 规划原点 | `robot_base_frame`、Smac `base_frame`、MPPI `robot_frame` | P0 |
| 轴距 `L` | 曲率、转向换算 | Smac `minimum_turning_radius`、MPPI Ackermann 模型、桥接器 | P0 |
| 最大转角 | `R_geometry`、`wz_max` | Smac 半径、MPPI `wz_max`、底盘控制器 | P0 |
| 实测最小半径 | 可跟踪 `Rmin` | Smac `minimum_turning_radius`、MPPI 几何约束 | P0 |
| 车长/前后悬 | 参考点到车头距离 | `footprint`、局部 costmap、goal checker | P0 |
| 车宽/轮距 | 横向包络 | `footprint`、inflation、通道可行性 | P0 |
| 最大速度 | 速度边界 | MPPI `vx_max`、速度平滑器、桥接器 | P0 |
| 最大/舒适加速度 | 纵向动态边界 | MPPI `ax_max`、速度平滑器 | P1 |
| 制动减速度 | 停止距离 | 速度上限、局部 costmap 窗口、安全余量 | P1 |
| 转向速率/延迟 | 可跟踪曲率变化 | MPPI horizon、转向相关约束、桥接器 | P1 |
| 控制频率/延迟 | 预测时域 | MPPI `model_dt`、`time_steps`、controller frequency | P1 |
| 雷达量程/频率 | 障碍物可见距离 | scan 转换、obstacle layer、costmap range | P0 |
| 地图分辨率 | 轨迹/障碍物离散精度 | global/local costmap resolution | P1 |
| 控制器消息类型 | 命令语义 | bridge、controller topic、`use_stamped` | P0 |

P0 参数错误通常会导致 TF、碰撞检测、路径运动学或控制接口失效；P1 参数错误通常表现为路径过于保守、转向滞后、速度振荡或临时障碍物反应偏晚；P2 参数才适合用于轨迹平滑和性能优化。

## 5. 推荐的分阶段调参顺序

每次只修改一个参数组，并记录修改前后的 YAML、测试场景和现象。不要在几何、速度和 MPPI critic 权重同时变化时判断结果。

### 阶段 0：接口和 TF 基线

先确认新机器人已经具备：

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic info /scan -v
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link laser_link
```

还要确认：

- `map -> odom -> base/reference_frame` TF 连续；
- 激光雷达时间戳没有早于 TF 缓存；
- `/scan` 的 `frame_id` 与配置一致；
- 地图服务已经 active，并且 RViz 可以收到 `/map`；
- 控制器的命令 topic、消息类型和是否使用 stamped 消息一致。

### 阶段 1：只调几何和运动学

先只修改：

- `robot_base_frame` / `reference_frame`；
- `footprint`；
- `wheelbase`；
- `minimum_turning_radius`；
- MPPI 中与 Ackermann 运动学直接相关的轴距和角速度上限；
- bridge 中的最小转弯半径或等效运动学限制。

使用空旷场地依次测试：

1. 直行；
2. 左转；
3. 右转；
4. 大半径圆弧；
5. 低速接近目标；
6. 狭窄但有足够余量的通道。

此阶段不要先改 MPPI critic 权重。若路径本身违反转弯半径或足迹碰撞，继续调 critic 只会掩盖基础参数错误。

### 阶段 2：速度和动力学

从较保守的速度开始：

```text
normal_speed <= 0.5 * max_speed
```

确认机器人能够稳定停止后，再逐步提高速度。需要同步检查：

- MPPI `vx_min/vx_max`、`wz_max`；
- velocity smoother 的 `max_velocity`、`min_velocity`、加速度/减速度；
- bridge 的速度/角速度限制；
- 底盘控制器自己的速度和转向限制；
- 命令超时后是否停止；
- 速度变化是否出现锯齿、延迟或过冲。

观察重点：

- 直线速度是否稳定；
- 转弯时是否因速度过高而外甩；
- 到障碍物前能否留出完整制动距离；
- 新轨迹到来时机器人是否突然刹车或加速；
- 控制器 CPU 是否能在周期内完成计算。

### 阶段 3：MPPI 预测时域和计算量

MPPI 预测时域为：

```text
T_horizon = time_steps * model_dt
```

推荐先让 `model_dt` 接近控制器周期。例如控制器约 `20 Hz` 时，`model_dt` 可从 `0.05 s` 开始。然后根据机器人动态响应调整 `time_steps`：

- 机器人转向慢、速度高：需要更长的预测时域；
- 机器人小、速度低、计算资源有限：可以缩短时域；
- 时域过短：临近弯道才开始转向，容易切弯或急转；
- 时域过长：计算变慢，且可能使用过时的障碍物信息。

建议先固定 `time_steps` 和 `model_dt`，只逐步测试 batch size，例如：

```text
2000 -> 1500 -> 1000
```

每次只改一个值，确认控制周期仍能满足要求。不要为了追求轨迹更平滑而让控制器实际计算时间超过消息更新时间。

### 阶段 4：传感器、costmap 和临时障碍物

按下面的链路逐层验证：

```text
/points_raw 或 /scan
    -> 点云转 LaserScan
    -> obstacle layer marking
    -> obstacle layer clearing
    -> footprint collision
    -> local costmap
    -> MPPI obstacle critic
    -> cmd_vel
```

推荐顺序：

1. 在 RViz 单独显示 `/scan`，确认临时障碍物有回波；
2. 显示 local costmap，确认障碍物在正确位置出现；
3. 确认机器人 footprint 与障碍物的关系；
4. 检查 obstacle layer 的 topic、`obstacle_max_range`、`raytrace_max_range`；
5. 再观察 MPPI 是否减速、绕行或停止。

关键原则：

- `obstacle_max_range` 不能大于传感器可信有效量程；
- `raytrace_max_range` 要与清除射线的有效范围匹配；
- 局部 costmap 的窗口要覆盖机器人前方的制动距离；
- costmap 更新频率不能明显低于机器人运动和传感器变化速度；
- 代价地图的时间戳必须与 TF 缓存可匹配；
- 临时障碍物应同时测试 marking 和 clearing，避免障碍物消失后仍长期残留。

### 阶段 5：目标容差和进场姿态

只有几何、速度和障碍物链路稳定后，才调整目标进场行为：

- `xy_goal_tolerance`：目标位置允许误差；
- `yaw_goal_tolerance`：目标朝向允许误差；
- MPPI 的 `PathAlign`、`PathFollow`、`GoalAngle` 等 critic；
- `PreferForward`、`UsePathApproachOrientation`、`RemovePassedGoals` 等行为开关。

对于 Ackermann 机器人，最后一段轨迹通常需要给转向执行器足够的准备距离。`UsePathApproachOrientation.heading_baseline` 可先按约 `1.2~1.5` 倍轴距设置，再根据现象调整：

- 目标前最后一段仍然急转：增加该距离；
- 很早就开始转向、路径偏离明显：减小该距离；
- 每次改动建议为 `0.1~0.2 m`。

如果到达目标后仍频繁重新规划，先区分是目标容差过小、定位抖动，还是下游控制器没有真正停止；不要直接放大所有 tolerance。

## 6. 常见更换场景的最小修改集合

### 6.1 只改变车长或车宽

修改：

1. 重新生成 `footprint`；
2. 复核参考点到车头/车尾的距离；
3. 复核局部 costmap 窗口和 inflation；
4. 在狭窄通道和障碍物旁低速测试。

如果轴距和转角没有变化，不要仅因为车体变长就修改 `minimum_turning_radius`；但前悬变长会改变停止距离和障碍物安全余量。

### 6.2 只改变轴距

修改：

1. `wheelbase`；
2. 根据最大转角重新计算 `R_geometry`；
3. 重新测量或估计 `Rmin`；
4. 更新 MPPI 和 bridge 中的轴距；
5. 重新检查 `wz_max <= vmax/Rmin`。

轴距变长而最大转角不变时，转弯半径通常变大；不能只改一个 `wheelbase` 而继续使用旧的 Smac 半径。

### 6.3 只改变最大转角或转向响应

修改：

1. 重新计算并实测 `Rmin`；
2. 更新 Smac 的 `minimum_turning_radius`；
3. 更新 MPPI 的角速度边界和转向相关限制；
4. 根据转向延迟重新检查 `model_dt` 和预测时域；
5. 低速验证左转、右转以及转向回正。

转向执行器慢时，几何上可行的轨迹不一定可跟踪。此时优先增加预测余量或降低速度，不要用过大的角速度上限掩盖执行器延迟。

### 6.4 只改变速度或制动能力

修改：

1. MPPI 线速度上限；
2. velocity smoother 的速度/加速度/减速度；
3. bridge 和底盘控制器限制；
4. local costmap 窗口；
5. 障碍物有效量程和安全余量复核。

如果速度提高而雷达、costmap 或控制周期不变，避障距离通常会变差。速度、制动和感知范围必须作为一个整体重新验证。

### 6.5 更换激光雷达或点云来源

修改：

1. `laser_frame`、topic 和 QoS；
2. 点云转 LaserScan 的输入/输出 topic；
3. 高度、角度范围、最小/最大量程和过滤条件；
4. costmap obstacle layer 的 source 配置；
5. TF 时间戳和传感器安装位置；
6. 在 RViz 同时查看原始数据和 local costmap。

不要只改 topic 名称。雷达安装高度、盲区和俯仰角会直接改变机器人实际能看到的障碍物。

### 6.6 更换底盘控制器或消息接口

当前 bridge 面向 `Twist`/`TwistStamped` 的速度命令。如果新控制器需要 `AckermannDriveStamped`，就需要重新确认：

- Nav2 输出的是速度/偏航角速度，还是直接输出转角；
- 转角换算使用的轴距和参考点；
- 控制器要求的是前轮转角、等效转角还是转角速度；
- 消息是否 stamped，时间戳是否由 bridge 正确填写；
- 命令超时和零速度行为是否安全。

不能把当前 `angular.z` 直接当作 Ackermann 前轮转角发送。两者量纲和物理意义不同。

## 7. Smac Hybrid-A* 与 MPPI 的调参边界

### Smac Hybrid-A*

Smac 负责生成满足基本运动学约束的全局路径。迁移时优先检查：

- `motion_model` 是否仍为 `DUBIN`；
- `minimum_turning_radius` 是否是新车可实现且留有余量的半径；
- `angle_quantization_bins` 是否足以表达新机器人的转向变化；
- analytic expansion 的半径是否合理；
- 路径平滑器是否在平滑后破坏了原始转弯约束。

analytic expansion 的搜索半径可先按约 `4~6 * Rmin` 检查，当前机器人可先从 `5 * Rmin` 附近开始。调大该值会增加计算和连接机会，调小则可能使弯道连接失败；它不是用来弥补错误 `Rmin` 的。

### MPPI Ackermann

MPPI 负责在局部范围内对轨迹进行动力学可执行性、障碍物和跟踪代价评估。迁移时优先保证：

- 预测模型的轴距和参考坐标系一致；
- 速度/角速度上限符合 `Rmin` 和底盘限制；
- 控制周期与 `model_dt` 一致；
- 预测时域覆盖转向和制动响应；
- 机器人 footprint 与 local costmap 使用同一参考点。

只有在以上条件正确后，才按一次一个 critic 的方式调整 `PathAlign`、`PathFollow`、`GoalAngle`、`Obstacles`、`PreferForward` 等权重。critic 权重不能修复错误的轴距、半径、footprint 或消息语义。

## 8. 验收清单

### 几何与控制

- [ ] 直线运动方向、参考点和 TF 方向正确。
- [ ] 左右转弯半径与实测值一致，并且不小于配置的 `Rmin`。
- [ ] `wz_max` 不违反 `vmax/Rmin` 或底盘偏航上限。
- [ ] 前进约束生效，不会产生无法执行的倒车命令。
- [ ] 速度、角速度、加速度和转向变化没有明显过冲。
- [ ] 控制命令超时后机器人进入安全状态。

### Nav2、TF 和地图

- [ ] `/map_server` active，RViz 能收到 transient-local `/map`。
- [ ] `map -> odom -> base/reference_frame` 连续可用。
- [ ] costmap footprint 覆盖实际车体。
- [ ] global/local costmap 分辨率和窗口符合场景。
- [ ] 重启或重新发送第二个目标后规划链路仍能工作。

### 轨迹和障碍物

- [ ] 空旷场地直行、左转、右转都能生成并执行轨迹。
- [ ] 狭窄通道不会因 footprint 偏小发生擦碰。
- [ ] 临时障碍物进入有效感知范围后能及时出现在 local costmap。
- [ ] 机器人会在制动距离以内减速、绕行或停止。
- [ ] 障碍物移除后 clearing 正常，旧障碍不会永久残留。
- [ ] 目标附近不会持续振荡或反复重规划。

## 9. 故障现象到排查顺序

| 现象 | 优先检查 |
|---|---|
| 路径几何上转不过去 | 轴距、最大转角、`Rmin`、Smac motion model |
| 机器人切弯或擦碰 | footprint、参考点、平滑器、local costmap footprint |
| 很晚才发现临时障碍物 | scan/TF、obstacle layer、costmap 窗口、速度和制动 |
| 轨迹可行但机器人跟不上 | 转向延迟、控制频率、MPPI 时域、底盘限制 |
| 转弯时速度振荡 | `vx_max`、`wz_max`、加速度、critic 权重、命令延迟 |
| 到目标后反复规划 | 目标容差、定位抖动、下游停止状态、goal checker |
| RViz 显示 no map received | map_server lifecycle、`/map` publisher、QoS、地图路径 |
| 第二个目标不动 | 旧路径是否清除、控制器超时、bridge 状态、重新规划后的命令 |
| 规划成功但底盘不动 | 控制 topic、消息类型、stamp、桥接器和底盘控制器接口 |

固定的排查顺序应是：

```text
传感器数据 -> TF -> costmap -> Smac 路径 -> MPPI 命令 -> bridge -> 底盘执行
```

## 10. 给 Codex 的后续输入模板

更换机器人后，可以把下面内容连同新机器人 URDF、控制器配置和传感器 topic 一起提供：

```yaml
task: "将 Ackermann Nav2 适配到新 Ackermann 机器人"
robot:
  name: ""
  reference_frame: ""
  base_frame: ""
  wheelbase_m: 0.0
  track_width_m: 0.0
  body_length_m: 0.0
  body_width_m: 0.0
  front_overhang_m: 0.0
  rear_overhang_m: 0.0
  max_steering_angle_deg: 0.0
  measured_min_turning_radius_m: 0.0
  steering_rate_deg_s: 0.0
  steering_delay_s: 0.0
  max_speed_mps: 0.0
  normal_speed_mps: 0.0
  max_accel_mps2: 0.0
  comfortable_decel_mps2: 0.0
  emergency_decel_mps2: 0.0
  controller_rate_hz: 0.0
  command_timeout_s: 0.0
sensor:
  lidar_frame: ""
  lidar_topic: ""
  lidar_range_min_m: 0.0
  lidar_range_max_m: 0.0
  lidar_rate_hz: 0.0
  pointcloud_topic: ""
control:
  type: "twist|twist_stamped|ackermann_drive_stamped"
  topic: ""
  wheelbase_used_by_controller_m: 0.0
  steering_semantics: ""
validation:
  measured_min_turning_radius_m: 0.0
  observed_stop_distance_m: 0.0
  notes: ""
```

收到这些信息后，建议让 Codex 按以下顺序处理：

1. 生成新的几何摘要和 `Rmin` 推导；
2. 只修改 P0 参数并做 TF、直行、左右转验证；
3. 修改速度、制动和传感器相关参数；
4. 调整 MPPI 预测时域和计算量；
5. 最后才调整 critic 权重、目标容差和轨迹风格；
6. 输出修改清单、验证命令和未确认的假设。

## 11. 不建议的做法

- 不要把旧机器人整份参数文件复制后只修改机器人名字。
- 不要把 URDF 中某个关节的转角上限直接当作等效前轮转角，除非已确认其语义。
- 不要把 `angular.z` 直接当成前轮转角发送给 Ackermann 控制器。
- 不要先调 MPPI critic 权重来解决 footprint、轴距或最小转弯半径错误。
- 不要同时大幅修改速度、footprint、costmap 和 critic 权重后再判断效果。
- 不要为了“早点避障”只把 inflation 半径无限增大；先确认传感器、costmap 更新和制动距离。
- 不要把 NeuPAN 的跟踪能力问题误判为 Smac 全局路径问题；先分别观察全局路径、MPPI 输出和最终底盘执行。
- 不要把 V550 或其他机器人的动力学参数直接套用到当前机器人。
- 不要在没有记录基线的情况下覆盖当前已经验证有效的配置。

最终适配的判断标准不是“规划器能返回 success”，而是以下四部分同时一致：

```text
机器人几何尺寸
    + 运动学约束
    + 动力学/制动能力
    + 传感器和控制接口
    = Nav2 可规划、可跟踪、可避障的参数集
```
