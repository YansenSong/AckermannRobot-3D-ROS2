# IMU 安装位姿集成：静态 TF + LIO-SAM 外参注入

> 状态: 已集成（代码改动完成，待实车建图验证）
> 日期: 2026-08-08
> 目的: 让 `real_vehicle.yaml` 中 `sensors.imu.mount`（IMU 安装位姿）成为唯一数据源，
> 并让静态 TF 与 LIO-SAM 建图外参从同一处消费，消除手工转录造成的重复与不一致。

---

## 1. 背景

实车已安装 LPMS-IG1-RS485 IMU 并完成静止标定，安装位姿录入
`src/vehicle_config/config/real_vehicle.yaml → sensors.imu.mount`：

```yaml
mount:
  x: 0.13          # 相对 base_link 的 X 向偏移（m），卷尺测量
  y: 0.0
  z: 0.26
  roll: -0.006945  # 5min 静止加速度标定 ≈ -0.398°
  pitch: -0.006617 # ≈ -0.379°
  yaw: 0.0         # IMU X 轴指向车头
```

集成前该字段**没有消费方**：

- 无 `base_link → imu` 静态 TF（对比 `base_link → hesai_lidar` 早已发布）；
- LIO-SAM `params.yaml` 的 `extrinsic*` 是手工转录的第二份拷贝；
- LIO-SAM `robot.urdf.xacro` 的 `gyro_joint` 是第三份拷贝（且未被任何 launch 消费）。

## 2. 集成架构

```text
real_vehicle.yaml (唯一数据源)
  │  sensors.imu.mount  +  sensors.lidar.mount
  ├── vehicle_config.imu_transform()        → IMU launch 的 base_link→imu 静态 TF
  └── vehicle_config.lio_sam_extrinsics()  → LIO-SAM run.launch.py 参数 overlay
```

| 消费方 | 函数 | 注入方式 |
|---|---|---|
| `lpms_ig1` IMU 驱动 launch | `imu_transform()` | `static_transform_publisher` 节点 |
| LIO-SAM 4 个节点 | `lio_sam_extrinsics()` | launch `parameters` 列表后覆盖前 |

两处数据都来自 `real_vehicle.yaml`，改动 IMU 安装位姿只需改这一处。

## 3. 文件改动

| 文件 | 内容 |
|---|---|
| `src/vehicle_config/vehicle_config/loader.py` | 新增 `imu_transform()`、`lio_sam_extrinsics()` |
| `src/vehicle_config/vehicle_config/__init__.py` | 导出两个新函数 |
| `src/vehicle_config/test/test_vehicle_config.py` | 新增测试（结构、正交性、逐值校验） |
| `src/lpms_ig1/launch/real_vehicle_imu.launch.py` | 添加 `base_link→imu` 静态 TF 节点 |
| `src/LIO-SAM/launch/run.launch.py` | 计算外参 overlay 并注入 4 个 LIO-SAM 节点 |
| `src/LIO-SAM/config/params.yaml` | 删除硬编码 `extrinsicTrans/Rot/RPY` 块 |
| `src/LIO-SAM/config/robot.urdf.xacro` | 标注 `gyro_joint` 值来源 |
| `src/lpms_ig1/package.xml`、`src/LIO-SAM/package.xml` | 补充 `<exec_depend>vehicle_config</exec_depend>` |

## 4. 外参坐标约定（重要）

LIO-SAM `imuConverter`（`include/lio_sam/utility.hpp`）将 IMU 系向量转到车体/lidar 系：

```cpp
acc = extRot * acc;   gyr = extRot * gyr;          // v_imu → v_body
q_body_world = q_imu_world * extQRPY;              // extQRPY = quat(extrinsicRPY)
```

因此 `extrinsicRot` 和 `extrinsicRPY` 都必须等于 **`R_base_imu = Rz(yaw) @ Ry(pitch) @ Rx(roll)`**，
与 `static_transform_publisher` 的 rpy 约定一致；`extrinsicTrans` 为 lidar 原点在 IMU 系中的平移
（`R_base_imu^T @ (lidar − imu)`）。三者均由 `lio_sam_extrinsics()` 程序化计算。

### ⚠ 修正记录：旧硬编码矩阵是转置

本次核对发现旧 `params.yaml` 的 `extrinsicRot` 是正确矩阵的**转置**
（手工录入时 roll/pitch 符号取反）：

| 位置 | 正确值（注入后） | 旧硬编码值 |
|---|---|---|
| `rot[2]`（第 1 行第 3 列） | **−0.00661679** | +0.00661695 |
| `rot[5]`（第 2 行第 3 列） | **+0.00694494** | −0.00694479 |
| `rot[6]`（第 3 行第 1 列） | **+0.00661695** | −0.00661679 |
| `rot[7]`（第 3 行第 2 列） | **−0.00694479** | +0.00694494 |

差异是 ~0.4° 量级的符号级偏差，会影响重力补偿方向（roll/pitch 估计）。
注入后 LIO-SAM 的 IMU 坐标系朝向与静态 TF 一致。

## 5. 静态 TF

`real_vehicle_imu.launch.py` 在 IMU 驱动旁发布：

```
base_link → imu    xyz=(0.13, 0, 0.26)   rpy=(-0.006945, -0.006617, 0)
```

与 `navigation.launch.py` 的 `base_link → hesai_lidar` 同模式。IMU 驱动一启动，TF 树即有该帧。

## 6. 验证

### 自动化（已通过）

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon test --packages-select vehicle_config --event-handlers console_direct+
# 4 passed（含 imu_transform / lio_sam_extrinsics）

# 两个 launch 可构建 LaunchDescription（不启动节点）
python3 -c "
import importlib.util
spec = importlib.util.spec_from_file_location('m', 'src/LIO-SAM/launch/run.launch.py')
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
print(len(m.generate_launch_description().entities))  # 7 实体
"
```

### 实车

```bash
# 1. TF 生效
ros2 launch lpms_ig1 real_vehicle_imu.launch.py &
ros2 run tf2_ros tf2_echo base_link imu
# 期望 xyz (0.13, 0, 0.26)，rpy (-0.006945, -0.006617, 0)

# 2. LIO-SAM 外参注入生效
ros2 launch lio_sam run.launch.py &
ros2 param dump /lio_sam_mapOptimization | grep -A1 extrinsicRot
# 期望 9 元素矩阵 ≈ [0.999978, 0.000046, -0.006617, ...]，而非 identity 默认值
```

若实车建图发现 roll/pitch 补偿方向反了，先核对 IMU 铭牌轴向与 ROS 车体系约定
（X 前、Y 左、Z 上）；确认一致则矩阵公式正确，仅需修改 `real_vehicle.yaml` 一处。

## 7. 点云时间字段适配（2026-08-08 已修复）

**原问题**：Hesai 驱动发布的 PointCloud2 字段名为 `timestamp`（FLOAT64，绝对时间），而 LIO-SAM 的
`VelodynePointXYZIRT` 注册字段名为 `time`。PCL `moveFromROSMsg` 按字段名精确匹配，导致每点 `time`
未填充（为 0），去畸变实际是空操作 —— 不崩、不丢帧，但运动畸变未消除，**建图会漂**。`ring` 字段名匹配正常。

**修复**（驱动侧 `source_driver_ros2.hpp` 的 `ToRosMsg`）：字段改为 `time`（FLOAT32），
值 = `point.timestamp − frame_start_timestamp`（相对扫描起始的偏移秒）。header.stamp 取自
`frame_start_timestamp`，与 LIO-SAM `timeScanCur` 基准一致，去畸变恢复生效。

**⚠ 仅改字段名不够**：LIO-SAM `imageProjection.cpp:266` 用 `timeScanCur + points.back().time` 计算
`timeScanEnd`，若 `time` 仍为绝对时间（~1.7e9）会令 `timeScanEnd` 爆炸导致 IMU/odom 对时错乱甚至丢帧。
必须同时转相对时间。

## 8. 后续待办

- [ ] 实车建图验证：确认外参注入后建图精度较旧值有改善
- [x] 处理 `time`/`timestamp` 字段名不匹配，恢复去畸变（驱动已改发 `time` 相对时间，待实车建图确认）
- [ ] 雷达 PTP 时间同步（治本），随后 `use_timestamp_type` 改回 0
- [ ] 确认 `Horizon_SCAN`（当前 1800，雷达每帧 32000 点 = 16 线 × 2000 列，疑似应为 2000）
- [ ] 若 xacro 将来接入 `robot_state_publisher`，改为从 `real_vehicle.yaml` 动态生成

## 9. 传感器时间同步（2026-08-08 修复）

**问题**：LIO-SAM 的 `imageProjection` 一直 `Waiting for IMU data ...`，整个链路无输出。
根因是**雷达与 IMU 时间域不一致**：

| 传感器 | 时间戳来源 | 实测域 |
|---|---|---|
| Hesai 雷达 | `use_timestamp_type=0` → 雷达内部 UTC | **2019 年**（1564031551，PTP 从未同步，UTC 停留在出厂时间） |
| LPMS-IG1 | `this->now()` 主机时钟 | **2026 年**（1786...） |

LIO-SAM `imageProjection.cpp:328-334` 要求 IMU 数据覆盖雷达扫描窗口 `[timeScanCur, timeScanEnd]`；
两个时间域相差 7 年，`imuQueue.front().stamp > timeScanCur` 恒真 → 永远等待 → 无 cloud_info →
无 odometry/lidar → `base_link→odom` 不发布 → rviz 报 `No transform from [base_link] to [map]` 等。

**修复**（`real_vehicle.yaml` → `vehicle_config.materialize_lidar_driver_config`）：
`use_timestamp_type: 1`，雷达点云时间戳改用**主机接收时间**，与 IMU 的 `now()` 同域。
相对时间 `time` 字段逻辑不受影响（同域内相减）。LIO-SAM 随即开始消费 IMU 并恢复输出。

**治本方案**：给雷达配置 PTP 时间同步（RK3588 上跑 ptp4l + 雷达 PTC 使能 PTP），
同步成功后把 `use_timestamp_type` 改回 `0`（用真实雷达 UTC），IMU 侧也应考虑接入同一时钟源。
使用主机接收时间会有网络抖动，去畸变精度略低于真 fire-time，但对当前建图足够。
