# Ackermann 底盘适配 Nav2 SmacPlannerHybrid 参数指南

本文是后续把本项目全局规划器迁移到其他尺寸、其他转向参数的阿克曼底盘时，提供给 Codex 的输入和执行规则。目标是保持官方 Humble nav2_smac_planner/SmacPlannerHybrid 算法不变，只根据新底盘的几何、坐标系和运动能力调整配置。

## 1. 先明确边界

全局规划器的验收对象是：

~~~
起点/终点坐标系正确
Smac 能生成可碰撞检查的 SE(2) 轨迹
曲率不超过底盘运动学限制
footprint 覆盖真实碰撞外形
Reeds-Shepp 倒车方向被保留
规划结果正确发布给 /plan 和 /plan_path
~~~

NeuPAN 是否能够跟踪轨迹、Gazebo 中车辆是否最终到达，以及实际控制器的跟踪误差属于下游控制和集成验收，不应单独作为“全局规划失败”的判据。

不得修改 src/nav2_smac_planner/** 来适配某个底盘。底盘差异应进入：

~~~
src/ackermann_bringup/config/smac_planner.yaml
src/ackermann_bringup/launch/planning.launch.py
ackermann_smac_bridge 的 robot_frame 参数
~~~

只有在新底盘的 ROS 坐标系或接口发生变化时，才另行处理 controller、xacro 或 NeuPAN 配置；不要把这些下游修改混入 Smac 算法适配。

## 2. 交给 Codex 的新机器人参数清单

最好一次提供下面这份信息。若某项未知，可以直接给出 URDF/Xacro 路径，让 Codex 从源文件和 mesh collision 中审计，不要用视觉外形猜测。

~~~yaml
robot:
  # Smac 和全局 costmap 使用的参考点，推荐为后轴中心
  planning_frame: rear_axle_link
  base_link_frame: base_link

  # 后轴中心到前轮转向轴中心，单位 m
  wheelbase_m: 0.59379

  # 单位 m；不是轮胎直径
  front_track_m: 0.509914
  rear_track_m: 0.509914

  steering:
    # virtual_center: 控制器/自行车模型的中心转角
    # inner_outer_wheel: 分别提供内外侧轮最大转角
    # curvature: 直接提供最大曲率
    limit_type: virtual_center
    max_left_rad: 0.52
    max_right_rad: 0.52
    max_inner_rad: null
    max_outer_rad: null
    max_curvature_inv_m: null

  # 相对于 planning_frame 的所有 collision 几何包络，单位 m
  footprint:
    x_min_m: -0.093
    x_max_m: 0.73248
    y_min_m: -0.35235
    y_max_m: 0.35235
    source: urdf/xacro collision geometry

  # 如果 footprint 不是矩形，优先提供逆时针 polygon 顶点
  footprint_polygon: null

map:
  yaml: maps/<name>/map.yaml
  resolution_m: from_map_yaml
  allow_unknown: true
  desired_clearance_m: 0.10

planner_behavior:
  allow_reverse: true
  goal_tolerance_m: 0.25
  strict_discrete_curvature_check: true
~~~

还应说明转向角到底是哪一种角度：虚拟自行车模型中心转角、内侧轮角、外侧轮角，还是 controller 直接接收的 steering command。左右两侧若不对称，也要分别给出限制。

以下参数通常不影响 Smac 全局几何，不必为了规划器重复提供：轮胎半径、车重、电机额定功率、最大线速度和加速度。这些主要属于 controller、里程计或 NeuPAN。

## 3. 从阿克曼几何计算最小转弯半径

坐标约定为 x 向前、y 向左，规划参考点位于后轴中心。令：

~~~
L = wheelbase
T = front_track
δ = 中心线/虚拟自行车模型转角
R = 后轴中心到瞬时转动中心的距离
~~~

若给的是中心线转角或 controller 的虚拟转角：

~~~
R_phys = L / tan(|δ_max|)
κ_phys = 1 / R_phys
~~~

若给的是内外侧车轮角，分别换算为后轴中心半径：

~~~
R_inner_bound = L / tan(|δ_inner_max|) + T / 2
R_outer_bound = L / tan(|δ_outer_max|) - T / 2
R_phys = max(R_inner_bound, R_outer_bound)
~~~

左右转向分别计算；如果左右不对称，取所有转向方向中更大的半径。这样 Smac 虽然只有一个 minimum_turning_radius，仍不会要求车辆超过任一方向的极限。

若直接给出最大曲率：

~~~
R_phys = 1 / |κ_max|
~~~

角度必须先转换为弧度。L 必须是后轴中心和前转向轴中心的距离，不能用车身总长代替。当前项目的例子是：

~~~
L = 0.28930 - (-0.30449) = 0.59379 m
δ_max = 0.52 rad
R_phys = 0.59379 / tan(0.52) = 1.29203 m
~~~

### 规划半径和物理半径的区别

R_phys 是底盘的物理下限；minimum_turning_radius 是 Smac 搜索使用的规划半径。首次可以设为 R_phys。如果打开官方 smoother 后，严格的离散曲率测试仍超出 1 / R_phys，应先把规划半径增加 2%～5% 重新验证，而不是修改官方 smoother 或直接改算法：

~~~
R_config = R_phys * (1 + conservative_margin)
~~~

当前项目采用：

~~~
R_phys   = 1.29203 m
R_config = 1.32000 m
~~~

这是约 2.16% 的保守余量，用来抵消官方 smoother 的离散曲率峰值；不是把真实车辆半径重新“推导”为 1.32 m。

## 4. footprint 的计算规则

global_costmap 必须使用 polygon footprint，不要改成 robot_radius。polygon 坐标必须相对于 Smac 的 robot_base_frame，推荐也就是后轴中心 frame。

footprint 至少应覆盖以下 collision 几何在所有相关姿态下的 XY 投影：

~~~
车身碰撞盒
前后轮碰撞体
转向节/转向连杆 mesh
传感器或其他会参与碰撞检查的结构
~~~

应从 URDF/Xacro 的 collision geometry 审计，至少采样左右最大转角和零转角；只看 visual mesh 或只看车身盒会漏掉转向轮、转向节的横向外扩。

矩形包络的初始构造为：

~~~
footprint = [
  [x_min - margin, y_min - margin],
  [x_max + margin, y_min - margin],
  [x_max + margin, y_max + margin],
  [x_min - margin, y_max + margin]
]
~~~

margin 是几何审计余量；footprint_padding 是 costmap 另外施加的 padding，二者不要重复计算。真实外形明显不是矩形时，提供保守的逆时针 polygon，不能为了让狭窄通道“规划成功”而缩小 footprint。

当前项目的审计结果为：

~~~
collision envelope: x=[-0.09300, 0.73248], y=[-0.35235, 0.35235]
configured polygon: x=[-0.10, 0.74], y=[-0.36, 0.36]
footprint_padding: 0.01 m
~~~

## 5. 参数如何映射到 Smac 配置

| 新底盘输入/需求 | 配置项 | 调整规则 |
|---|---|---|
| wheelbase、转角或最大曲率 | GridBased.minimum_turning_radius | 按第 3 节计算 R_config |
| 是否允许倒车 | GridBased.motion_model_for_search | 允许倒车用 REEDS_SHEPP；只允许前进才用 DUBIN |
| 几何包络 | global_costmap.global_costmap.footprint | 用 planning frame 下的 collision envelope 构造 polygon |
| 几何测量误差 | footprint_padding | 小余量；不能替代 footprint 审计 |
| 地图分辨率 | launch 重写 global_costmap...resolution | 从 map.yaml 读取，不要硬编码 0.05 |
| 未知区域策略 | allow_unknown、track_unknown_space | 两者保持一致 |
| 目标位置容差 | GridBased.tolerance | 按任务精度要求改；与车辆尺寸不是同一个量 |
| R_config | analytic_expansion_max_length | 第一版取 5 * R_config，并向上取两位小数 |
| 方向离散精度 | angle_quantization_bins | 第一版保留 72（5°）；只有路径离散误差明显时才提高 |

当前配置中的 analytic_expansion_ratio: 3.5 不随车身尺寸直接计算；当前 analytic_expansion_max_length: 6.60 是 5 * 1.320 的结果。

下面这些先保持官方 Humble 基线，除非有可重复的规划行为证据：

~~~yaml
downsample_costmap: false
downsampling_factor: 1
smooth_path: true
cache_obstacle_heuristic: false
reverse_penalty: 2.0
change_penalty: 0.0
non_straight_penalty: 1.2
cost_penalty: 2.0
retrospective_penalty: 0.015
~~~

它们的含义是：reverse_penalty 越大越不愿意倒车，change_penalty 越大越不愿意换挡，non_straight_penalty 越大越偏好直线，cost_penalty 越大越偏好远离高代价区域。这些是路径偏好或性能参数，不应用来“补偿”错误的 wheelbase、转角或 footprint。

官方 smoother 保持开启并使用 Humble 默认的 smoother 参数。若 smoother 造成可重复的换向方向错误或曲率超限，优先增加 R_config；只有形成独立 testcase 后，才评估是否改变 smoother 设置。

### costmap inflation

inflation_layer 不是 Smac 的车辆运动学参数。新底盘尺寸变化后，应重新检查 footprint 的 circumscribed radius：

~~~
circumscribed_radius = max(sqrt(x_i^2 + y_i^2))
~~~

如果新 footprint 或期望安全距离明显增大，应相应检查或设置 inflation_radius 和 cost_scaling_factor；它们影响障碍物代价梯度和路径偏好，不能代替 footprint collision checking。第一版仍保持 static_layer + inflation_layer，不加入动态障碍层，除非任务明确要求。

## 6. 坐标系和 launch 必须同步

新底盘最容易出现的错误不是公式，而是参考点不一致。至少保证：

~~~
global_costmap.global_frame = map
global_costmap.robot_base_frame = <后轴中心 frame>
ackermann_smac_bridge.robot_frame = <同一个后轴中心 frame>
ComputePathToPose 的 start 和 goal frame = map
~~~

如果新机器人只有 base_link，而 base_link 不在后轴中心，应优先新增或使用一个表示后轴中心的 fixed link，并让 costmap、bridge 和路径坐标都使用它。不要把 base_link 的名字直接套进公式，却仍把后轴偏移留在路径参考点之外。

/goal_pose 的语义应保持为上述规划参考点的目标姿态。bridge 负责将目标变换到 map、读取当前起点并调用 /compute_path_to_pose；它不应重新计算 /plan 的 yaw。

## 7. 给新机器人执行适配的步骤

Codex 收到参数后，按以下顺序工作：

1. 检查 git status，保留与本任务无关的用户修改。
2. 从 Xacro/URDF、controller 配置和 TF 关系复核 wheelbase、track、转角定义及 planning frame。
3. 计算 R_phys、必要的 R_config、analytic_expansion_max_length 和 footprint；把计算过程写入适配记录。
4. 只修改 smac_planner.yaml、必要的 launch frame 参数和文档；不修改官方 nav2_smac_planner 算法源码。
5. 构建并运行 bridge、bringup 和 vendored Smac 的测试。
6. 在静态 TF + 无障碍小地图上测试路径，不依赖 NeuPAN 是否驶达终点。
7. 读取 /unsmoothed_plan 与 /plan，检查曲率、heading 和换向 cusp；比较 /plan 与 /plan_path，确认 bridge 没有改写 pose orientation。
8. 若允许倒车，检查 cos(yaw - segment_direction) < 0 的段，并确认方向信息能原样到达 NeuPAN 的 /plan。
9. 最后把全流程作为集成 smoke test；NeuPAN 的跟踪误差或 ARRIVED 状态单独记录，不覆盖全局规划器的几何结论。

## 8. 最小验收清单

~~~
[ ] minimum_turning_radius 来源于新底盘几何
[ ] analytic_expansion_max_length 与 R_config 一致
[ ] footprint 覆盖所有 steering 姿态的 collision envelope
[ ] map / robot_base_frame / bridge robot_frame 一致
[ ] map_server 和 planner_server active
[ ] /compute_path_to_pose 可用
[ ] planner plugin = nav2_smac_planner/SmacPlannerHybrid
[ ] /plan 只有 PlannerServer 发布
[ ] /plan_path 只有 bridge 发布
[ ] Smac 的 pose orientation 未被 bridge 重写
[ ] 若允许倒车，Reeds-Shepp 倒车段可识别
[ ] |kappa_max| <= 1 / R_phys（跳过换向 cusp 的常规段）
[ ] footprint 和 costmap 碰撞检查通过
[ ] steering joint 不超过新 URDF/controller 的限制（单独记录）
~~~

全局规划测试至少应覆盖：直行目标、带姿态的转弯目标，以及允许倒车时的 Reeds-Shepp 目标。只要能稳定生成正确轨迹，就不需要为了完成本清单让 NeuPAN 实际行驶完整路线。

## 9. 适配结果报告模板

~~~
Robot / URDF:
Planning frame:
Wheelbase L:
Front/rear track:
Steering limit and definition:
R_phys:
R_config:
angle_quantization_bins:
motion_model_for_search:
analytic_expansion_max_length:
Footprint source and final polygon:
Footprint padding:
Map resolution / unknown policy:

Build and tests:
Lifecycle / action:
Forward path:
Reverse path:
Regular-segment max curvature:
/plan versus /plan_path orientation delta:
Steering joint check:
NeuPAN integration smoke test (optional):
Unverified items / NOT VERIFIED:
~~~

这个报告应同时写明 R_phys 和 R_config。如果两者不同，必须说明保守余量的原因；不能只给一个未经解释的 minimum_turning_radius 数字。

