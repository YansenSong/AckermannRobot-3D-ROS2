# NeuPAN 阿克曼机器人参数适配与调参指南

本文档用于后续把当前项目的 NeuPAN 适配到尺寸、轴距、转向能力、速度和传感器安装方式不同的阿克曼机器人。

目标是：以后只要提供新机器人的物理参数、坐标系和控制器约束，Codex 就能按照本文档计算并修改项目中的 NeuPAN 及其必要的全局规划、控制接口参数，同时知道哪些参数不能直接照抄或一起修改。

本文档只描述参数适配方法，不改变当前已经恢复的 NeuPAN 基线，也不把 arrived 或机器人是否完整行驶到终点作为全局轨迹质量的主要判据。

---

## 1. 当前系统的三层参数

当前导航链路：

~~~
地图 / 定位
    │
    ▼
Nav2 SmacPlannerHybrid
    │  /plan：位置、姿态和可能的前进/倒车方向
    ▼
NeuPAN
    │  /neupan_cmd_vel_raw = [线速度 v, 自行车模型转角 ψ]
    ▼
neupan_ackermann_adapter
    │  /neupan_cmd_vel = [线速度 v, 车体角速度 ω]
    ▼
ackermann_steering_controller / Gazebo
~~~

适配新机器人时，把参数分成三类：

| 层级 | 代表参数 | 作用 | 主要位置 |
|---|---|---|---|
| 物理/接口层 | 轴距、轮距、轮胎半径、转向关节限制、底盘尺寸、传感器位姿 | 描述真实机器人和执行器 | URDF/Xacro、controller YAML、TF、launch |
| 全局规划层 | 最小转弯半径、footprint、运动模型、地图分辨率 | 生成车辆几何上可行的全局轨迹 | src/ackermann_bringup/config/smac_planner.yaml |
| NeuPAN 局部优化层 | length、width、wheelbase、速度、转角、加速度、安全距离、时域 | 在 /plan 周围避障并输出控制量 | src/neupan_ros2/config/robots/ackermann_robot/ |

一个物理参数经常需要同时映射到多个位置。例如轴距至少影响：

1. NeuPAN 的阿克曼运动学模型；
2. Smac 的最小转弯半径；
3. adapter 中的 v × tan(ψ) / L 换算；
4. URDF 和实际 Ackermann controller。

只改其中一个位置，会产生全局轨迹正确但局部控制不对、或者仿真能动但转弯半径不一致等问题。

---

## 2. 当前项目基线

下面是当前已经恢复的可运行基线。它是项目当前配置，不代表所有新机器人都应继续使用这些数值。

### 2.1 NeuPAN planner

文件：src/neupan_ros2/config/robots/ackermann_robot/planner.yaml

~~~yaml
receding: 15
step_time: 0.2
ref_speed: 0.5
collision_threshold: 0.01

robot:
  kinematics: 'acker'
  max_speed: [0.7, 0.422]
  max_acce: [1.0, 0.328]
  length: 0.720
  width: 0.500
  wheelbase: 0.593

ipath:
  interval: 0.03
  curve_style: 'dubins'
  min_radius: 1.320

pan:
  iter_num: 2
  dune_max_num: 200
  nrmp_max_num: 10

adjust:
  q_s: 0.1
  p_u: 0.5
  eta: 15.0
  d_max: 0.1
  d_min: 0.01
~~~

重点：

- robot.length 和 robot.width 是 NeuPAN 当前矩形模型的尺寸，不应自动当成完整 URDF 碰撞包络。
- robot.max_speed[1] 是自行车模型的中心线/虚拟转角 ψ，单位是弧度，不是某一只前轮的物理关节角。
- collision_threshold 是 NeuPAN 的碰撞停止阈值，不是开始主动绕障的距离阈值。
- 当前未显式设置 min_speed；核心会默认允许速度和转角使用对称上下界，因此当前配置允许产生倒车速度。

### 2.2 全局 Smac 基线

文件：src/ackermann_bringup/config/smac_planner.yaml

~~~yaml
minimum_turning_radius: 1.320
motion_model_for_search: "REEDS_SHEPP"
angle_quantization_bins: 72
smooth_path: true
analytic_expansion_max_length: 6.60
global_frame: map
robot_base_frame: rear_axle_link
footprint: "[[-0.10, -0.36], [0.74, -0.36], [0.74, 0.36], [-0.10, 0.36]]"
footprint_padding: 0.01
~~~

### 2.3 NeuPAN ROS2 基线

文件：src/neupan_ros2/config/robots/ackermann_robot/robot.yaml

~~~yaml
map_frame: map
base_frame: rear_axle_link
lidar_frame: laser_link
scan_range_min: 0.15
scan_range_max: 5.0
scan_downsample: 1
control_frequency: 50.0
refresh_initial_path: true
include_initial_path_direction: true
~~~

map_frame、base_frame、Smac 的 robot_base_frame 和 bridge 的参考坐标系必须表示同一个运动学参考点。当前项目选择后轴中心 rear_axle_link。

### 2.4 当前已有的数值差异

历史车辆文档记录过：车身约 0.70 × 0.52 m、Xacro 轴距约 0.59379 m、物理转向关节限制约 0.52 rad；当前 NeuPAN 使用 0.720 × 0.500 m、0.593 m、0.422 rad，Smac footprint 使用更保守的 [-0.10, 0.74] × [-0.36, 0.36] 包络。

这些数值的含义不同：

- 0.593 是当前运行配置使用的轴距；0.59379 是 Xacro 根据轮轴坐标计算的高精度值。后续新机器人应选一个权威值并保持一致，或明确记录四舍五入。
- 0.52 rad 是物理转向关节限制；0.422 rad 是当前与 R=1.320 m 全局规划半径匹配的保守虚拟转角。
- NeuPAN 矩形、Smac footprint 和 URDF collision envelope 不是同一个概念。适配时必须重新审计，不能只复制旧文档的数字。

---

## 3. 给 Codex 的新机器人参数清单

后续适配最好一次提供以下信息。若某项不知道，可以提供 URDF/Xacro、controller YAML 和传感器 TF 文件路径，让 Codex 从源码审计，不要只根据视觉外形估计。

~~~yaml
robot:
  name: <robot_name>

  planning_frame: rear_axle_link
  base_link_frame: base_link
  wheelbase_m: 0.59379
  front_track_m: 0.510
  rear_track_m: 0.510

  front_overhang_m: 0.20
  rear_overhang_m: 0.10

  steering:
    # virtual_center / inner_outer_wheel / curvature
    limit_type: virtual_center
    max_left_rad: 0.52
    max_right_rad: 0.52
    max_inner_rad: null
    max_outer_rad: null
    max_curvature_inv_m: null

  speed:
    max_forward_mps: 0.7
    max_reverse_mps: 0.7
    desired_cruise_mps: 0.5
    max_accel_mps2: 1.0
    max_decel_mps2: 1.0
    max_steering_rate_radps: 0.328
    allow_reverse: true

  footprint:
    x_min_m: -0.10
    x_max_m: 0.74
    y_min_m: -0.36
    y_max_m: 0.36
    source: urdf/xacro collision geometry
  footprint_polygon: null

  lidar:
    frame: laser_link
    x_m: 0.0
    y_m: 0.0
    z_m: 0.0
    range_min_m: 0.15
    range_max_m: 5.0
    horizontal_fov_rad: 6.283185
    frequency_hz: 20.0
    timestamp_source: simulation_or_driver

  execution:
    controller_frequency_hz: 100.0
    odometry_frequency_hz: 50.0
    command_latency_s: 0.0
    planner_cpu: cpu

planner_policy:
  map_resolution_m: from_map_yaml
  desired_clearance_m: 0.10
  goal_tolerance_m: 0.25
  use_reeds_shepp: true
  checkpoint_available: true
  checkpoint_path: models/dune_model_5000.pth
~~~

必须额外说明转角定义：中心虚拟转角、内侧轮角、外侧轮角，还是 controller 直接接收的 steering command。左右转向不对称时，左、右限制要分别给出。

---

## 4. 尺寸和坐标参考点如何映射到 NeuPAN

### 4.1 当前 NeuPAN 矩形模型

当前核心根据 length、width、wheelbase 生成相对于规划参考点的矩形。参考点为后轴中心时，等价于：

~~~text
x_min = -(length - wheelbase) / 2
x_max =  (length + wheelbase) / 2
y_min = -width / 2
y_max =  width / 2
~~~

当前配置对应：

~~~text
length    = 0.720 m
width     = 0.500 m
wheelbase = 0.593 m

NeuPAN 矩形约为：
x = [-0.0635, 0.6565] m
y = [-0.2500, 0.2500] m
~~~

这只是 NeuPAN 优化中的机器人形状，不等于 Smac 的完整碰撞 footprint。

### 4.2 对称前后外伸

若后轴中心到车身前、后的外伸量近似相同，可以设置：

~~~text
length    = 车体总长
width     = 最大横向车体宽度
wheelbase = 后轴中心到前轮转向轴中心的距离
~~~

单位全部为米。

### 4.3 非对称前后外伸

当前 NeuPAN 的 length/width/wheelbase 组合不能准确表达任意的前后非对称矩形。不要把车体总长直接填入后忽略参考点偏移。

推荐：

1. 如果可以接受保守矩形，取：

   ~~~text
   overhang = max(front_overhang, rear_overhang)
   length_for_neupan = wheelbase + 2 * overhang
   ~~~

   这样 NeuPAN 矩形覆盖真实前后外伸，但可能偏大。

2. Smac/global costmap 使用基于 URDF collision 的精确或保守 polygon。

3. 如果保守矩形明显降低通道利用率，再单独评估给 NeuPAN 核心增加显式 vertices 或参考点偏移支持。这个属于代码接口适配，不能只靠修改 YAML 猜测解决。

### 4.4 width 的取值

width 至少应覆盖：

1. 车身 collision 宽度；
2. 转向轮、转向节或其他会碰撞结构在转向极限下的横向外扩；
3. 必要的几何安全余量。

不要为了让路径通过狭窄位置而缩小 width。额外安全距离应通过 footprint padding 或代价地图参数表达，并避免重复加余量。

---

## 5. 转角、轴距和最小转弯半径

### 5.1 中心线/自行车模型转角

如果给出的是中心虚拟转角 δ：

~~~text
R_phys = L / tan(|δ_max|)
κ_phys = 1 / R_phys
~~~

其中 L 是后轴中心到前轮转向轴中心的轴距。角度必须先转换为弧度，不要用车身总长代替轴距。

### 5.2 内外侧前轮角

设前轮距为 T，内侧轮角为 δ_inner，外侧轮角为 δ_outer：

~~~text
R_inner_bound = L / tan(|δ_inner|) + T / 2
R_outer_bound = L / tan(|δ_outer|) - T / 2
R_phys        = max(R_inner_bound, R_outer_bound)
~~~

左转和右转分别计算。若左右限制不同，最终使用所有转向方向中更大的安全半径。

不能把内侧物理轮角原样填入 planner.yaml 的 max_speed[1]；NeuPAN 需要中心线/自行车模型虚拟转角，除非 controller 明确规定接收的就是该定义。

### 5.3 最大曲率

若直接给出最大曲率：

~~~text
R_phys = 1 / |κ_max|
~~~

### 5.4 物理半径和规划半径

R_phys 是理论物理下限；Smac 和 NeuPAN 建议使用带余量的规划半径：

~~~text
R_config = R_phys * (1 + margin)
~~~

首次适配可以从 2%～5% 的保守余量开始，再通过路径离散曲率和实际控制验证决定是否调整。

当前项目活动基线：

~~~text
Smac minimum_turning_radius = 1.320 m
NeuPAN ipath.min_radius      = 1.320 m
NeuPAN max_speed[1]          = 0.422 rad
~~~

其中 0.422 约等于 atan(0.593 / 1.320)，它是与规划半径匹配的虚拟转角，不等价于 Xacro 的物理前轮关节上限 0.52 rad。

新机器人应保持：

~~~text
NeuPAN ipath.min_radius
    与 Smac minimum_turning_radius 使用同一 R_config

NeuPAN max_speed[1]
    约等于 atan(wheelbase / R_config)
~~~

如果决定使用更接近物理极限的半径，Smac、NeuPAN 的 max_speed[1] 和 controller 限制必须一起验证。

---

## 6. 速度、加速度、转向速率和 MPC 时域

### 6.1 max_speed

对于 acker 模型：

~~~yaml
robot:
  max_speed: [最大线速度幅值 m/s, 最大虚拟转角 rad]
~~~

当前核心没有 min_speed 时默认使用对称边界：

~~~text
v ∈ [-max_speed[0], +max_speed[0]]
ψ ∈ [-max_speed[1], +max_speed[1]]
~~~

如果新机器人只能前进，不能只把 Smac 改成前进模型，还要明确设置 NeuPAN 速度下界或在控制接口层禁止倒车。

### 6.2 ref_speed

ref_speed 是局部优化参考巡航速度：

~~~text
0 < ref_speed ≤ 稳定巡航速度 ≤ max_speed[0]
~~~

新机器人质量、附着、控制延迟或障碍物密度变化时，优先先降低 ref_speed 做验证，不要同时改变所有避障权重。

### 6.3 max_acce

核心使用：

~~~text
acce_bound = max_acce * step_time
Δv_max = max_acce[0] * step_time
Δψ_max = max_acce[1] * step_time
~~~

因此：

~~~yaml
robot:
  max_acce: [线速度变化率上限 m/s², 虚拟转角变化率上限 rad/s]
~~~

第二项虽然字段名叫 max_acce，本质是每秒允许的虚拟转角变化量，不是转向电机角加速度。需与真实 controller 的加速度、转向速率和命令延迟共同校验。

### 6.4 receding、step_time 和控制频率

NeuPAN 预测时域：

~~~text
T_horizon = receding * step_time
~~~

当前基线为 15 × 0.2 s = 3.0 s。

step_time 是 MPC 离散步长，robot.yaml 的 control_frequency 是 ROS 控制循环频率，两者不是同一个参数。适配新机器人时同时考虑：

- 速度提高后预测时域是否仍能看到障碍物和弯曲；
- 控制器延迟是否占用了有效预测时间；
- 每次求解是否能在控制周期内完成；
- dune_max_num、nrmp_max_num、iter_num 增大后 CPU 是否超时。

若求解不能稳定完成，先记录耗时，再选择降低点数、迭代次数、参考速度或时域；不要只提高 ROS 发布频率掩盖计算超时。

---

## 7. Smac 与 NeuPAN 必须保持一致

### 7.1 运动模型和初始路径方向

当前 Smac 使用 REEDS_SHEPP，允许倒车和换向。NeuPAN 当前设置 include_initial_path_direction: true，用于保留 Smac pose 中的方向信息。

按机器人能力选择：

| 能力 | Smac | NeuPAN |
|---|---|---|
| 只能前进 | DUBIN/前进模型 | dubins；速度下界非负 |
| 支持倒车 | REEDS_SHEPP | 保留路径方向；确认曲线插值支持倒车段 |

当前 curve_style: dubins 是现有基线的一部分。若新机器人确实需要倒车段，不能只改 Smac；应先确认当前 NeuPAN 版本对带方向路径的曲线插值支持，再决定是否改为其支持的 reeds 形式，并用包含倒车段的路径测试。

### 7.2 最小转弯半径相关项

重新计算 R_config 后，至少同步检查：

~~~yaml
# NeuPAN
ipath:
  min_radius: <R_config>

# Smac
GridBased:
  minimum_turning_radius: <R_config>
  analytic_expansion_max_length: <约 5 * R_config，需验证>
~~~

当前 analytic_expansion_max_length 为 6.60 m，即 5 × 1.320 m。该项是搜索解析扩展长度，不是机器人尺寸本身。

以下参数先保持当前基线，除非有可复现的规划行为证据：

~~~yaml
downsample_costmap: false
downsampling_factor: 1
angle_quantization_bins: 72
reverse_penalty: 2.0
change_penalty: 0.0
non_straight_penalty: 1.2
cost_penalty: 2.0
retrospective_penalty: 0.015
smooth_path: true
~~~

这些是路径偏好和搜索性能参数，不能用来补偿错误的轴距、转角或 footprint。

### 7.3 footprint

Smac/global costmap 应使用相对于同一规划参考点的碰撞包络。当前参考点为 rear_axle_link。

应从 URDF/Xacro 的 collision geometry 审计：

- 零转角；
- 左最大转角；
- 右最大转角；
- 转向节、连杆或其他运动部件的横向外扩。

真实外形不是矩形时，优先给出逆时针 polygon。不要只使用 robot_radius，也不要只看 visual mesh。

footprint_padding 是 costmap 额外安全余量，不应与已经写进 polygon 的余量重复计算。

---

## 8. 轴距必须同步到 command adapter

当前 NeuPAN 原始输出解释为：

~~~text
linear.x  = v
angular.z = ψ    # 这里是虚拟转角，不是车体角速度
~~~

adapter 再换算：

~~~text
ω = v * tan(ψ) / L
~~~

文件：src/ackermann_control/neupan_ackermann_adapter.py

新机器人轴距变化后必须同步修改：

1. planner.yaml 的 robot.wheelbase；
2. navigation.launch.py 传给 adapter 的 wheelbase；
3. adapter 默认值（如果 launch 没覆盖）；
4. URDF/Xacro 和实际 controller 的轴距；
5. Smac 转弯半径计算使用的轴距。

当前 launch 中仍有 wheelbase: 0.593，这不是可以遗忘的默认参数。新机器人不能只修改 planner.yaml。

---

## 9. 激光雷达和 TF

### 9.1 量程不是主动避障阈值

robot.yaml 的 scan_range_min/max 是进入 NeuPAN 前保留的 LaserScan 范围，不表示“在 scan_range_max 处一定开始绕障”。

主动避障还取决于：

- 障碍物是否出现在 /scan；
- laser_link 到 rear_axle_link 的 TF；
- 时间戳是否与 TF 缓存一致；
- DUNE 的学习距离；
- dune_max_num 和 nrmp_max_num；
- d_max、d_min、eta 和路径/控制代价；
- 当前速度和 MPC 预测时域。

因此不能把 collision_threshold 简单调大来替代输入和局部避障检查。

### 9.2 新机器人要核对

~~~text
map → planning_frame
planning_frame → lidar_frame
LaserScan 的 frame_id 和时间戳
scan_angle_min / scan_angle_max
scan_range_min / scan_range_max
是否需要下采样
传感器刷新频率和命令延迟
~~~

如果出现“消息早于 TF 缓存中的所有数据”等过滤日志，应先修正仿真时间、传感器时间戳或 TF 发布关系，再做避障调参。

### 9.3 scan_downsample

点数过多会增加 DUNE/NRMP 求解负担，但下采样过强会让临时障碍物变稀疏。新机器人先使用 1 做基线；确认点数和求解耗时后，才单独评估下采样。

---

## 10. NeuPAN 避障参数的正确含义

| 参数 | 含义 | 是否等于物理距离 |
|---|---|---|
| scan_range_min/max | LaserScan 输入过滤范围 | 是传感器输入范围，但不是主动避障起始距离 |
| dune_max_num | DUNE 处理的障碍点上限 | 否 |
| nrmp_max_num | NRMP 考虑的障碍点上限 | 否 |
| d_max | NRMP 独立距离变量上界 | 通常不是原始激光距离 |
| d_min | NRMP 独立距离变量下界 | 通常不是原始激光距离 |
| eta | 安全距离相关的松弛/正则权重 | 否 |
| collision_threshold | 根据 DUNE 当前最小距离触发 stop 的阈值 | 不是提前绕障距离 |

当前核心中，DUNE 对障碍点计算学习距离并记录 min_distance；NRMP 对独立距离变量施加 d_min ≤ distance ≤ d_max 约束。这个距离的标定依赖 DUNE 模型和机器人几何，不能直接当成“激光测距 0.1 m”。

当前基线：

~~~yaml
collision_threshold: 0.01

adjust:
  eta: 15.0
  d_max: 0.1
  d_min: 0.01
~~~

这些值作为当前基线记录，不是新机器人无条件复制的物理安全距离。

### 10.1 后续出现避障偏晚时的排查顺序

1. 确认 /scan 在预期距离已包含障碍物，frame_id、TF 和时间戳正确；
2. 确认机器人外形、rear_axle_link、lidar 外参和 footprint 没有缩小或错位；
3. 记录 DUNE/NRMP 点、min_distance、原始控制指令和输出控制指令；
4. 检查求解耗时是否落后于控制周期；
5. 以上都正确后，一次只修改一个避障参数。

推荐单变量顺序：

~~~text
几何/TF/传感器
    → d_max
    → eta
    → d_min
    → q_s / p_u
    → iter_num、dune_max_num、nrmp_max_num
~~~

这不是固定的“越大越好”规则。d_max 是 NeuPAN 内部距离约束的尺度，实际效果必须用固定场景验证；不要根据一次仿真同时改变 d_max、eta、d_min、q_s 和速度。

### 10.2 collision_threshold

collision_threshold 适合作为接近碰撞时的最后安全停止条件，不适合承担提前绕障的主要职责，因为：

- 检查的是 NeuPAN 的 min_distance，不是直接的 LaserScan 最近点；
- 阈值过大可能频繁急停；
- 阈值过小不能解决 DUNE 没有及时形成有效避障梯度的问题。

如果目标是提前绕障，应先解决输入、几何、DUNE 模型和 NRMP 链路，而不是直接把它改成很大的数。

### 10.3 DUNE checkpoint 与新尺寸

当前 checkpoint：

~~~text
src/neupan_ros2/config/robots/ackermann_robot/models/dune_model_5000.pth
~~~

它来自当前项目采用的官方 Ranger Ackermann 基线。DUNE 的距离学习和机器人几何有关，因此：

- 仅改变期望速度或控制频率时，可以先保留 checkpoint 做 A/B 测试；
- 改变长度、宽度、前后外伸、激光雷达安装方式或碰撞包络后，不能默认 checkpoint 仍然等价；
- 尺寸变化较大时，应使用匹配新机器人几何的模型重新训练/微调，或至少在固定障碍物场景比较旧模型和新模型的 min_distance、避障方向及碰撞结果；
- 不要只改 planner.yaml 的长度和宽度，就声称 DUNE 已适配新底盘。

是否必须重新训练没有一个仅由百分比决定的统一阈值，最终以固定场景的几何和避障验证为准。

---

## 11. 新机器人适配时修改哪些文件

| 文件 | 何时修改 | 修改内容 |
|---|---|---|
| URDF/Xacro | 实体尺寸、轮轴、转向、传感器变化 | collision、轮轴位置、TF、关节限制、传感器外参 |
| controller YAML | 控制器参数变化 | 轴距、轮距、轮半径、转向/轮速限制 |
| neupan planner.yaml | NeuPAN 模型和局部控制约束变化 | 尺寸、轴距、虚拟转角、速度、加速度、时域、避障参数 |
| neupan robot.yaml | ROS 接口和传感器变化 | frame、LaserScan 范围、频率、路径方向、topic |
| smac_planner.yaml | 全局规划几何变化 | minimum_turning_radius、footprint、解析扩展长度、运动模型 |
| navigation.launch.py | adapter/launch 参数变化 | adapter wheelbase、topic、use_sim_time |
| neupan_ackermann_adapter.py | 接口语义变化 | 普通换底盘一般不改算法 |
| DUNE checkpoint | 模型和几何不再匹配 | 切换匹配模型或重新训练/微调 |

普通尺寸变化不应修改：

~~~text
third_party/NeuPAN 的核心算法
nav2_smac_planner 官方算法源码
~~~

如果确实需要修改核心，应单独记录原因、输入输出变化和回归测试，不要把核心变化隐藏在一次 YAML 调参中。

---

## 12. 推荐的实际适配顺序

### 阶段 A：物理参数和 TF 审计

1. 从 URDF/Xacro 计算前、后轮轴位置，得到实际轴距。
2. 区分物理前轮关节角和自行车模型中心转角。
3. 审计所有 collision geometry，并转换到规划参考点坐标系。
4. 确认 map → planning_frame → lidar_frame TF 和时间戳。
5. 记录控制频率、指令延迟、速度、加速度和转向速率。

### 阶段 B：计算并更新全局几何

1. 由轴距和转角/曲率计算 R_phys。
2. 加入小的规划余量，得到 R_config。
3. 同步 Smac 的 minimum_turning_radius 和 NeuPAN 的 ipath.min_radius。
4. 用 ψ = atan(L / R_config) 更新 NeuPAN 虚拟转角上限。
5. 更新 Smac footprint，检查 footprint_padding 是否重复加余量。
6. 按 R_config 复核 analytic_expansion_max_length。

### 阶段 C：更新 NeuPAN 动力学和接口

1. 更新 planner.yaml 的 length/width/wheelbase。
2. 更新 max_speed、ref_speed、max_acce。
3. 根据是否允许倒车设置速度下界、Smac motion model 和路径方向处理。
4. 更新 launch 中 adapter 的 wheelbase。
5. 只有传感器接口确实变化时才更新 robot.yaml 的 scan/frame/topic。

### 阶段 D：分层验证

1. 无障碍、低速检查直线和转弯控制。
2. 静态小地图只验证 Smac：曲率、footprint、换向方向和碰撞检查。
3. 接入 NeuPAN，确认 /scan、/plan、TF 和原始控制指令正确。
4. 固定位置、固定尺寸障碍物验证避障，一次只改一个参数。
5. 最后做完整 Gazebo 行驶 smoke test。

全局轨迹应主要依据 /plan 的几何、方向和碰撞结果判断；NeuPAN 跟踪误差、机器人是否到达终点和 arrived 状态另行记录。

---

## 13. 单变量调参记录表

每次改变 NeuPAN 参数时至少记录：

~~~text
场景编号：
机器人版本：
地图和障碍物位置：
起点/目标点：

修改前参数：
修改参数及旧值：
修改参数及新值：

/scan 首次看到障碍物的时间和距离：
DUNE/NRMP 相关日志：
min_distance（如果有）：
NeuPAN 规划耗时：
/neupan_cmd_vel_raw：
/neupan_cmd_vel：
是否发生急停：
最小实际车体间距：
是否出现路径方向或 TF 异常：
结论：
~~~

必须区分：传感器首次检测、局部优化开始改变轨迹、急停阈值和真实车体间距。不能把“看起来避障更早”直接写成“collision_threshold 就是新的避障距离”。

---

## 14. 后续给 Codex 的最小请求模板

~~~text
请将当前项目 NeuPAN 适配到新机器人：<名称>

URDF/Xacro：<路径>
controller 配置：<路径>
规划参考坐标系：<frame>
base_link：<frame>
激光坐标系：<frame>

轴距 L：<m>
前轮距 / 后轮距：<m> / <m>
前外伸 / 后外伸：<m> / <m>
车体总长 / 最大宽度：<m> / <m>
collision footprint：<polygon 或要求从 URDF 审计>

转角定义：<中心虚拟转角 / 内外轮角 / 最大曲率>
左转限制：<值和单位>
右转限制：<值和单位>
最大前进速度：<m/s>
最大倒车速度：<m/s>
是否允许倒车：<是/否>
最大线加速度 / 减速度：<m/s²> / <m/s²>
最大虚拟转角变化率：<rad/s>

激光量程和频率：<min/max/frequency>
控制器频率：<Hz>
里程计频率：<Hz>
控制延迟：<s>
地图分辨率：<m 或 map.yaml 路径>

是否有匹配新机器人的 DUNE checkpoint：<路径/没有>

请先审计参数和 TF，再计算 R_phys、R_config、NeuPAN 虚拟转角和 footprint；
保持与当前基线无关的参数不变；只修改必要文件；完成构建和静态检查；
最后报告所有修改项、计算过程、未验证项和需要实车确认的项目。
~~~

---

## 15. 适配完成后的检查

~~~bash
git diff --check
colcon build --symlink-install --packages-select neupan_ros2

ros2 param get /neupan_node robot_type
ros2 param get /neupan_node base_frame
ros2 param get /neupan_node lidar_frame

ros2 topic echo /plan --once
ros2 topic echo /neupan_cmd_vel_raw --once
ros2 topic echo /neupan_cmd_vel --once
~~~

运行时确认：

~~~text
[ ] NeuPAN 启动日志中的 length / width / wheelbase 与新机器人一致
[ ] Smac minimum_turning_radius 与计算值一致
[ ] NeuPAN ipath.min_radius 与 Smac 一致
[ ] adapter wheelbase 与 planner 和 URDF 一致
[ ] global_costmap.robot_base_frame 与 NeuPAN base_frame 一致
[ ] footprint 覆盖 steering 姿态下的 collision envelope
[ ] /scan 的 frame_id、时间戳和 TF 正常
[ ] /plan 的姿态和前进/倒车方向没有被 bridge 改写
[ ] 原始转角经 ω = v × tan(ψ) / L 换算后仍在 controller 能力范围内
[ ] 求解耗时小于系统可接受的控制周期
[ ] DUNE checkpoint 是否匹配新机器人已经明确记录
[ ] 避障调参每次只改变一个主要变量
~~~

完成几何、TF、接口和模型检查后，才适合讨论 eta、d_max、q_s 或 p_u 等行为参数；否则继续调这些数值没有稳定意义。
