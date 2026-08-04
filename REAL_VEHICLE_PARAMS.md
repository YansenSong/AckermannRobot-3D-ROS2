# 实车部署 — 待确认参数清单

创建日期: 2026-08-04
说明: 本文件记录实车部署中尚未由你确认、由 Claude 自行填写的参数。
      每个参数标注了默认值、影响范围和建议的确认方式。

---

## 需要实车测量/确认的参数

### 1. LiDAR 安装位置 (`real_vehicle.launch.py` → static TF `base_link → hesai_lidar`)

| 参数 | 当前默认值 | 影响 |
|------|-----------|------|
| `--x` | `0.0` | LiDAR 在 base_link 下的 X 偏移 (m) |
| `--y` | `0.0` | LiDAR 在 base_link 下的 Y 偏移 (m) |
| `--z` | `0.4` | LiDAR 安装高度 (m)，相对 base_link 原点 |

**确认方式**: 测量 LiDAR 中心相对车体几何中心（base_link 原点）的三维位置。
**影响**: pointcloud_to_laserscan 的高度切片、NeuPAN 中 scan 点云到 map 的 TF 变换。

### 2. lidar_frame 策略 (`robot.yaml`)

| 参数 | 当前默认值 | 备选值 |
|------|-----------|--------|
| `lidar_frame` | `'base_link'` | `'hesai_lidar'` |

当前选择 `base_link` 的原因是 pcl_to_scan 的 `target_frame` 也设为 `base_link`，
这样 scan 直接输出在车体坐标系中，无需额外的 base_link→hesai_lidar TF 来变换 scan。

如果将来需要更精确的障碍物定位（考虑 LiDAR 安装偏移），应改为 `hesai_lidar`，
同时调整 pcl_to_scan 的 `target_frame`。

### 3. 后轴偏移 (`planner_params_real.yaml` → Hybrid A*)

| 参数 | 当前默认值 | 影响 |
|------|-----------|------|
| `rear_axle_offset_x` | `-0.25` | 后轴相对于车体几何中心的 X 偏移 (m) |

当前值从仿真抄来（仿真车长 0.70m，后轴在中心后方 0.25m）。
实车长 1.40m、轴距 0.97m，这个值大概率不对。

**确认方式**: 测量后轴中心到车体几何中心的距离。如果几何中心在车体正中间（0.70m 处），
后轴在车尾方向 0.70 - 0.97/2 的位置... 实际上需要实际测量。

---

## 需要跟训模型确认的参数

### 4. planner.yaml 中须与训练配置一致的几何参数

这些参数在 DUNE 模型训练时已固定，修改后需要重新训练：

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `robot.length` | `1.40` | ✅ 已确认 |
| `robot.width` | `0.93` | ✅ 已确认 |
| `robot.wheelbase` | `0.97` | ✅ 已确认 |
| `robot.kinematics` | `'acker'` | ✅ 已确认 |

---

## 根据其他参数计算得出的参数（数学推导，非猜测）

| 参数 | 值 | 公式 |
|------|-----|------|
| `planner.yaml → min_radius` | `1.68` | `wheelbase / tan(30°)` = 0.97 / 0.577 |
| `planner.yaml → max_speed[1]` | `0.524` | `30° → rad` |
| `planner_params_real.yaml → minimum_turning_radius` | `1.68` | 同上 |

---

## 从仿真沿用、需实车验证/调优的参数

这些参数在当前阶段没有数据支撑，在实车跑起来后需要按 `docs/neupan_tuning.md` 方法论调优：

### planner.yaml — MPC

| 参数 | 当前值 | 来源 |
|------|--------|------|
| `receding` | `15` | 仿真 ackermann_robot |
| `step_time` | `0.2` | 仿真 ackermann_robot |
| `ref_speed` | `1.0` | 仿真 ackermann_robot |

### planner.yaml — 动力学

| 参数 | 当前值 | 来源 |
|------|--------|------|
| `max_acce[0]` (linear) | `0.5` | 未确认，Claude 占位值（仿真用 1.0） |
| `max_acce[1]` (steering) | `0.3` | 未确认，Claude 占位值（仿真用 0.5） |
| `collision_threshold` | `0.10` | 未确认，Claude 占位值（仿真用 0.05） |

### planner.yaml — 控制调优 (adjust)

| 参数 | 当前值 | 来源 |
|------|--------|------|
| `q_s` | `0.5` | 仿真 ackermann_robot |
| `p_u` | `0.5` | 未确认，Claude 占位值（仿真用 0.3），意图让转向更平滑 |
| `eta` | `4.0` | 仿真 ackermann_robot |
| `d_max` | `1.0` | 未确认，Claude 占位值（仿真用 0.5），意图扩大安全距离 |
| `d_min` | `0.10` | 未确认，Claude 占位值（仿真用 0.05） |

### robot.yaml — 激光扫描

| 参数 | 当前值 | 来源 |
|------|--------|------|
| `scan_range_max` | `30.0` | 未确认，Claude 占位值。PandarXT-16 实际可达 ~100m |
| `scan_range_min` | `0.3` | 未确认，Claude 占位值，用于滤除车体自反射 |
| `scan_downsample` | `2` | 未确认，Claude 占位值 |

### pcl_to_scan.yaml

| 参数 | 当前值 | 来源 |
|------|--------|------|
| `min_height` | `-0.4` | 未确认，Claude 占位值，取决于 LiDAR 安装高度 |
| `max_height` | `1.0` | 未确认，Claude 占位值 |
| `range_max` | `50.0` | 未确认，Claude 占位值 |

---

## 状态图例

- ✅ 已确认
- ⚠️ 待确认（需要测量）
- 🔧 需实车调优
