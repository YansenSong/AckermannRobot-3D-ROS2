# 实车部署 — 待确认参数清单

创建日期: 2026-08-04
说明: 本文件记录实车部署中尚未由你确认、由 Claude 自行填写的参数。
      每个参数标注了默认值、影响范围和建议的确认方式。

---

## 需要实车测量/确认的参数

### 1. LiDAR 安装位置 (`vehicle_config/config/real_vehicle.yaml` → `lidar_mount`)

| 参数 | 当前值 | 影响 |
|------|--------|------|
| `x/y/z` | 查看统一配置 | LiDAR 在 base_link 下的位置偏移 (m) |
| `roll/pitch/yaw` | 查看统一配置 | LiDAR 相对 base_link 的姿态 (rad) |

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

### 3. 后轴偏移 (`vehicle_config/config/real_vehicle.yaml` → `rear_axle_offset_x`)

| 参数 | 当前值 | 影响 |
|------|--------|------|
| `rear_axle_offset_x` | 查看统一配置 | 后轴相对于车体几何中心的 X 偏移 (m) |

当前值是尚未测量的占位值，大概率不符合实车。

**确认方式**: 测量后轴中心到车体几何中心的有符号 X 向距离。

---

## 已确认的公共参数

### 4. 须与训练配置一致的几何参数

数值只保存在 `src/vehicle_config/config/real_vehicle.yaml`。几何参数修改后，
需确认 DUNE 模型是否必须重新训练。

| 参数 | 唯一来源 | 说明 |
|------|----------|------|
| 车长、车宽、轴距、轮子半径 | `vehicle` 段 | ✅ 已确认 |
| 运动学类型 | NeuPAN `planner.yaml` | ✅ `acker` |

---

## 根据其他参数计算得出的参数（数学推导，非猜测）

| 参数 | 值 | 公式 |
|------|-----|------|
| NeuPAN `min_radius` | 运行时计算 | `wheelbase / tan(max_steer_deg)` |
| NeuPAN 转角限制 | 运行时计算 | `max_steer_deg` 转换为 rad |
| Hybrid A* `minimum_turning_radius` | 运行时计算 | 同上 |

---

## 未经实车验证、需要调优的参数

这些参数在当前阶段没有数据支撑，在实车跑起来后需要按 `docs/neupan_tuning.md` 方法论调优：

### planner.yaml — MPC

| 参数 | 当前值 | 来源 |
|------|--------|------|
| `receding` | `15` | 初始值，待实车验证 |
| `step_time` | `0.2` | 初始值，待实车验证 |
| `ref_speed` | `1.0` | 初始值，待实车验证 |

### planner.yaml — 动力学

| 参数 | 当前值 | 来源 |
|------|--------|------|
| `max_acce[0]` (linear) | `0.5` | 保守初值，待测加速度 |
| `max_acce[1]` (steering) | `0.3` | 保守初值，待测转向响应 |
| `collision_threshold` | `0.10` | 安全初值，待实车验证 |

### planner.yaml — 控制调优 (adjust)

| 参数 | 当前值 | 来源 |
|------|--------|------|
| `q_s` | `0.5` | 初始值，待实车调优 |
| `p_u` | `0.5` | 初始值，待实车调优 |
| `eta` | `4.0` | 初始值，待实车调优 |
| `d_max` | `1.0` | 按车长比例给出的初值，待验证 |
| `d_min` | `0.10` | 按车长比例给出的初值，待验证 |

### robot.yaml — 激光扫描

| 参数 | 当前值 | 来源 |
|------|--------|------|
| `scan_range_max/min` | 查看统一配置 | 需根据环境和车体自反射调整 |
| `scan_downsample` | 查看统一配置 | 需在点云密度和计算量之间调整 |

### pcl_to_scan.yaml

| 参数 | 当前值 | 来源 |
|------|--------|------|
| `min_height/max_height` | 查看统一配置 | 取决于雷达安装高度和障碍物高度 |
| `range_min/range_max` | 查看统一配置 | 与 NeuPAN 共用同一扫描距离范围 |

---

## 状态图例

- ✅ 已确认
- ⚠️ 待确认（需要测量）
- 🔧 需实车调优
