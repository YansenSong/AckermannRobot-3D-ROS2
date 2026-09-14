# 实车 Ackermann 底盘适配 SmacPlannerHybrid 参数指南

本文只面向实车 `real-vehicle-integration` 分支，用于根据真实底盘的几何、转向能力、坐标系和地图参数配置 Nav2 `SmacPlannerHybrid`。

## 1. 需要提供的实车参数

```yaml
robot:
  planning_frame: rear_axle_link
  base_link_frame: base_link
  wheelbase_m: <后轴中心到前转向轴中心>
  front_track_m: <前轮距>
  rear_track_m: <后轮距>

  steering:
    limit_type: virtual_center   # 或 inner_outer_wheel / curvature
    max_left_rad: <左最大转角>
    max_right_rad: <右最大转角>
    max_inner_rad: null
    max_outer_rad: null
    max_curvature_inv_m: null

  footprint:
    x_min_m: <collision 包络>
    x_max_m: <collision 包络>
    y_min_m: <collision 包络>
    y_max_m: <collision 包络>

map:
  yaml: maps/<name>/map.yaml
  resolution_m: from_map_yaml
  allow_unknown: true
  desired_clearance_m: <期望安全余量>
```

不要凭外观估计这些值。优先使用实车 CAD、URDF、机械图纸、转向机构测量结果或实际标定数据。

## 2. 最小转弯半径

若控制接口中的转角是虚拟自行车模型前轮中心转角：

```text
R_phys = L / tan(|delta_max|)
kappa_max = 1 / R_phys
```

其中 `L` 是轴距，`delta_max` 为最大前轮转角。

若左右转角能力不对称，应分别计算并采用更保守的半径。若给的是内外侧轮实际转角，需要先根据 Ackermann 几何换算到后轴中心转弯半径。

`GridBased.minimum_turning_radius` 的初始值建议不小于真实物理极限。若离散规划和 smoother 在边界附近产生轻微曲率超限，可在物理半径基础上加入小幅保守余量，而不是修改规划器算法源码。

## 3. Footprint

`global_costmap` 应使用真实车辆在规划参考坐标系下的碰撞包络。至少考虑：

- 车身外壳
- 前后轮
- 最大转向时前轮/转向机构横向外扩
- 可能参与碰撞的传感器或机械结构

矩形包络可写成：

```text
[
  [x_min - margin, y_min - margin],
  [x_max + margin, y_min - margin],
  [x_max + margin, y_max + margin],
  [x_min - margin, y_max + margin]
]
```

若真实轮廓明显不是矩形，应使用保守 polygon。不要为了让窄通道规划成功而缩小真实 footprint。

## 4. 关键 Smac 参数映射

| 实车约束 | 配置项 | 规则 |
|---|---|---|
| 轴距 + 最大转角/曲率 | `GridBased.minimum_turning_radius` | 按实车几何计算 |
| 是否允许倒车 | `GridBased.motion_model_for_search` | 允许倒车用 `REEDS_SHEPP`；只允许前进用 `DUBIN` |
| 车辆碰撞外形 | `global_costmap.global_costmap.footprint` | 使用真实 collision 包络 |
| 地图分辨率 | costmap resolution | 与实际 `map.yaml` 保持一致 |
| 未知区域策略 | `allow_unknown` / `track_unknown_space` | 两者语义保持一致 |
| 目标位置容差 | `GridBased.tolerance` | 按任务精度设置 |
| 解析扩展长度 | `analytic_expansion_max_length` | 可先按约 `5 * minimum_turning_radius` 设置 |

路径偏好类参数，例如 `reverse_penalty`、`change_penalty`、`non_straight_penalty`、`cost_penalty`，不能用来补偿错误的轴距、转角或 footprint。

## 5. 坐标系

至少保证以下参考点一致：

```text
global_costmap.global_frame = map
global_costmap.robot_base_frame = <规划参考 frame>
ackermann_smac_bridge.robot_frame = <同一个规划参考 frame>
ComputePathToPose start/goal frame = map
```

对于 Ackermann 底盘，规划参考点推荐放在后轴中心。如果 `base_link` 不在后轴中心，应提供明确的固定 TF，而不是直接把 `base_link` 当作自行车模型参考点。

## 6. 实车适配顺序

1. 根据机械参数或实际测量确认轴距、轮距、转角定义和最大转角。
2. 确认规划参考坐标系以及 `base_link`、LiDAR、IMU 的 TF。
3. 计算真实最小转弯半径与最大曲率。
4. 从真实碰撞外形得到 footprint。
5. 修改 `src/ackermann_bringup/config/smac_planner.yaml` 和必要的 frame 参数。
6. 构建并确认 planner server、map server、bridge 能正常启动。
7. 在静态地图中验证直行、转弯、窄通道，以及允许倒车时的 Reeds-Shepp 路径。
8. 检查 `/plan` 的曲率不超过实车物理限制。
9. 最后再接 NeuPAN 和 `/ackermann_cmd -> motion_control -> STM32` 做整车集成验证。

## 7. 最小验收清单

```text
[ ] minimum_turning_radius 来源于实车几何/标定
[ ] footprint 覆盖真实 collision envelope
[ ] map / robot_base_frame / bridge robot_frame 一致
[ ] map_server 和 planner_server active
[ ] /compute_path_to_pose 可用
[ ] planner plugin = nav2_smac_planner/SmacPlannerHybrid
[ ] /plan 只有 PlannerServer 发布
[ ] /plan_path 只有 ackermann_smac_bridge 发布
[ ] 若允许倒车，Reeds-Shepp 倒车段方向信息保留
[ ] 常规路径段最大曲率不超过实车物理极限
[ ] 规划结果可以原样传入 NeuPAN
```

所有尚未实车测量或验证的数据都应明确标记为 `NOT VERIFIED`，不要继续沿用旧分支中的测试基线值。
