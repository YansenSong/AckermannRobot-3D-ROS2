# 阿克曼小车仿真参数

本文档记录当前 Gazebo 仿真模型、底盘控制器、Hybrid A* 和 NeuPAN 使用的车辆参数。

## 几何与运动学

| 参数 | 当前值 | 说明 |
|---|---:|---|
| 规划车长 | `0.70 m` | Hybrid A* / NeuPAN 车体长度 |
| 规划车宽 | `0.52 m` | Hybrid A* / NeuPAN 车体宽度 |
| 底盘碰撞盒 | `0.70 × 0.30 × 0.10 m` | URDF base_link 碰撞几何 |
| 配置轴距 | `0.593 m` | 控制器、Hybrid A*、NeuPAN |
| Xacro 实际轴距 | `0.59379 m` | 前后轮轴坐标差 |
| 最大转角 | `±0.52 rad`（约 `±29.8°`） | 前轮转向关节限制 |
| 配置最小转弯半径 | `1.05 m` | Hybrid A* / NeuPAN |
| 车轮半径 | `0.093 m` | 四个车轮相同 |
| 车轮宽度 | `0.05 m` | 碰撞圆柱长度 |
| 配置前轮距 | `0.510 m` | 控制器参数 |
| 配置后轮距 | `0.510 m` | 控制器参数 |
| 规划参考点 | `rear_axle_link` | 阿克曼后轴中心 |

来源：

- [chassis.xacro](../src/ackermann_simulation/robot/xacro/chassis.xacro)
- [ackermann_controllers.yaml](../src/ackermann_control/config/ackermann_controllers.yaml)
- [planner_params.yaml](../src/hybrid_astar_planner/standalone_planner/config/planner_params.yaml)
- [NeuPAN planner.yaml](../src/neupan_ros2/config/robots/ackermann_robot/planner.yaml)

### 关键坐标

相对于 `base_link`：

```text
rear_axle_link = (-0.30449, 0, 0.045) m
前轮轴约为     = ( 0.28930, 0, 0.045) m
```

因此 Xacro 计算出的实际轴距为 `0.59379 m`。

## 质量与惯量

底盘主体：

```text
mass = 5.46103 kg
ixx  = 0.03415 kg·m²
iyy  = 0.07738 kg·m²
izz  = 0.10971 kg·m²
```

当前 Xacro 中有效部件质量合计约为：

```text
9.13533 kg
```

该合计包括底盘、四个车轮、两个转向部件、虚拟转向部件、激光雷达、相机和 IMU。

## 底盘控制器

来源：[ackermann_controllers.yaml](../src/ackermann_control/config/ackermann_controllers.yaml)

```text
控制器更新频率：100 Hz
里程计发布频率：50 Hz
轮速限制：±10 rad/s
转向角限制：±0.52 rad
轮半径：0.093 m
轴距：0.593 m
odom → base_link：由 robot_localization EKF 发布
```

## NeuPAN 控制参数

来源：[NeuPAN planner.yaml](../src/neupan_ros2/config/robots/ackermann_robot/planner.yaml)

```text
最大前进速度：1.5 m/s
最大倒车速度：0.5 m/s
最大转角：0.52 rad
最大加速度：1.0 m/s²
最大转角速度：0.5 rad/s
参考速度：0.8 m/s
碰撞停止距离：0.05 m
```

## 3D 激光雷达

来源：[sensors.xacro](../src/ackermann_simulation/robot/xacro/sensors.xacro)

```text
坐标：laser_link = (-0.07777, 0.00081, 0.3842) m
水平采样：1800
垂直采样：16
水平视场：360°
垂直视场：±15°
更新频率：20 Hz
量程：0.5–100 m
高斯噪声标准差：0.03 m
```

原始点云话题为 `/points_raw`，经适配器补充 LIO-SAM 所需字段后发布到 `/points_lio`。

## 参数不一致项

当前文件之间存在以下轻微差异，后续精确调参时建议统一：

1. Xacro 后轮中心距为 `0.5187 m`，控制器配置为 `0.510 m`。
2. `0.593 / tan(0.52)` 计算得到的最小转弯半径约为 `1.036 m`，配置值为保守近似 `1.05 m`。
3. 规划宽度 `0.52 m` 与底盘碰撞盒宽度 `0.30 m` 不同；规划宽度是车辆运动学近似尺寸，碰撞盒是 Gazebo 物理碰撞尺寸。

精确调参时，建议以 Xacro 中的实际轮轴坐标和碰撞几何为基准，并同步更新控制器、Hybrid A* 与 NeuPAN 配置。

## Hybrid A* 当前导航基线

`planner_params.yaml` 当前按“前进优先、必要时允许倒车”的导航场景设置：目标容差为 `0.10 m`，静态地图未知区域不通行，倒车惩罚为 `5.0`，前进/倒车换挡惩罚为 `4.0`，分析扩展上限为 `5.0 m`。这些参数用于避免规划器提前结束、频繁换挡以及通过过长的 Reeds–Shepp 近道。

如果任务是泊车或明确需要倒车入库，再降低 `reverse_penalty` / `gear_change_penalty`；如果地图仍在在线建图阶段，则应重新评估 `allow_unknown`，不能直接套用静态地图配置。
