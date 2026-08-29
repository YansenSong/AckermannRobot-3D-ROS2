# NeuPAN 调参与实车使用经验

本文记录 AckermannRobot-3D 中 NeuPAN 的实际调试结论，作为从仿真迁移到真实车辆的操作手册。参数示例以当前 0.70 m × 0.52 m 仿真车为基准；实车必须先替换为实测几何、传感器和底盘限制，不能直接照搬速度。

## 先记住的结论

1. NeuPAN 的 Ackermann 控制量是 `[v, ψ]`：`v` 为线速度，`ψ` 为前轮转角。ROS `Twist.angular.z` 需要的是车体角速度 `ω`，当前接口使用 `ω = v·tan(ψ)/L`。
2. NeuPAN 的状态原点是后轴中心 `rear_axle_link`，不是车体中心 `base_link`。车辆几何、Hybrid A* 和 NeuPAN 必须使用同一个参考点。
3. DUNE 权重与车辆几何绑定。只要实车的车长、车宽或轴距与训练模型差异明显，就应重新训练或至少先做距离误差验证。
4. `d_max` 不是“多远才接收 LaserScan”的开关，而是 NRMP 的避障作用距离。对 Ackermann 车，它必须给最小转弯半径和计算延迟留出空间。
5. 障碍物正好压在直线路径中央时，左右绕行具有对称局部最优。当前版本通过 `avoidance_seed_*` 给 PAN 一个带方向记忆的 S 形初值，打破这个对称性。
6. 调参顺序必须是：接口/几何/TF → 传感器 → 速度和底盘限制 → 求解延迟 → `d_max/d_min` → `eta` → `q_s/p_u` → 迭代次数。不要用代价权重掩盖坐标系或控制接口错误。

## 当前导航链和接口约定

```text
全局栅格地图 → Hybrid A* → /plan (nav_msgs/Path)
                                      ↓
/scan (LaserScan) → NeuPAN → /neupan_cmd_vel (Twist)
                                      ↓
                              cmd_vel_mux
                                      ↓
                         Ackermann steering controller
```

当前 Ackermann 状态参考链为：

```text
map → odom → base_link → rear_axle_link
                           └→ laser_link（经 base_link）
```

NeuPAN 使用 `map → odom → rear_axle_link` 计算状态；扫描点使用扫描时间戳的 TF，必要时通过 `map→odom + odom→laser_link` 补偿定位链的延迟。

当前 ROS wrapper 的控制转换为：

```text
NeuPAN 原生输出：[v, ψ]
ROS Twist.linear.x = v
ROS Twist.angular.z = v · tan(ψ) / wheelbase
```

如果实车底盘桥接程序直接需要“前轮转角”，就必须在桥接层明确使用 `ψ`，不能把已经转换过的 `angular.z` 再当作转角使用。

## 当前仿真基线

配置文件：

```text
src/neupan_ros2/config/robots/ackermann_robot/planner.yaml
src/neupan_ros2/config/robots/ackermann_robot/robot.yaml
```

当前仿真车经过验证的起始值如下：

| 项目 | 当前值 | 说明 |
|---|---:|---|
| 车长 | 0.70 m | DUNE 训练几何 |
| 车宽 | 0.52 m | DUNE 训练几何 |
| 轴距 | 0.593 m | 后轴中心到前轴中心 |
| 最大转角 | 0.52 rad | 约 29.8° |
| 最小转弯半径 | 约 1.05 m | `L/tan(ψmax)` |
| `receding` | 15 | 预测步数 |
| `step_time` | 0.20 s | 预测离散时间 |
| 预测时域 | 3.0 s | `15×0.20` |
| `ref_speed` | 0.8 m/s | 仿真起始巡航速度 |
| `max_speed` | `[1.5, 0.52]` | `[线速度, 转角]` |
| `min_speed` | `[-0.5, -0.52]` | 当前允许倒车 |
| `max_acce` | `[1.0, 0.5]` | `[速度变化率, 转角变化率]` |
| `iter_num` | 2 | PAN 迭代次数 |
| `dune_max_num` | 200 | DUNE 点数上限 |
| `nrmp_max_num` | 10 | NRMP 点数上限 |
| `q_s` | 0.8 | 路径/状态跟踪权重 |
| `p_u` | 1.0 | 速度跟踪权重 |
| `eta` | 0.5 | 避障距离收益权重 |
| `d_max` | 1.2 m | 避障作用距离 |
| `d_min` | 0.15 m | NRMP 期望的最小距离 |
| `collision_threshold` | 0.05 m | 当前距离的紧急停车阈值 |
| `avoidance_seed_steer` | 0.25 rad | 绕障初值的峰值转角 |

这组值是小型仿真车的基线，不是实车最终值。特别是 `ref_speed`、`max_speed`、`min_speed` 和 `max_acce`，必须根据实车底盘能力、制动距离和通信延迟重新确认。

## 参数含义和调节方向

### 1. 车辆和速度约束

`robot.length/width/wheelbase` 决定 DUNE 几何和 Ackermann 运动学，优先级最高。修改几何后，必须重新检查训练模型是否匹配。

`max_speed: [v_max, ψ_max]` 同时约束线速度和转角上限；`min_speed` 是真正的非对称下界。当前源码已经支持非对称速度约束，不能再假设速度范围总是 `[-v_max, v_max]`。

实车第一次测试建议：

```yaml
ref_speed: 0.2        # 或 0.3，确认控制方向后再提高
robot:
  max_speed: [0.4, <实测最大转角>]
  min_speed: [0.0, <负的最大转角>]
```

如果全局路径确实包含倒车段，不能简单把第一项固定为 0；应单独验证“前进段不自动倒车”和“倒车段可控”后，再恢复负速度下界。

`max_acce` 是每秒变化率：

- 第一项限制速度变化，过小会让小车在障碍前来不及减速；过大则会出现速度突变。
- 第二项限制转角变化，过小会转不过弯，过大容易左右摆动。
- ROS wrapper 还会对实际发送的第一条 `[v, ψ]` 做同样的变化率限制，因为 MPC 预测序列内部的约束不能约束“上一条已经发出的命令”。

### 2. MPC 时间参数

`receding × step_time` 是预测时域。`step_time` 必须和“控制命令实际保持时间”大致一致。

当前运行中 CPU 求解可能比 20 Hz 控制周期慢很多，日志里出现过约 1 秒才产生一条新规划命令的情况。此时模型按 0.2 秒预测、底盘却可能执行约 1 秒，轨迹会滞后并在障碍前振荡。

实车调试时先测每次 NeuPAN 求解耗时：

```yaml
time_print: true
```

确认性能后再决定：

- 降低 `ref_speed`，给规划器和底盘更多反应时间；
- 减少 `dune_max_num` 或 `receding`，降低 CPU 负担；
- 让实际重规划频率与 `step_time` 相近；
- 不要只提高 `control_frequency`，它不会让一次 CPU 求解变快。

### 3. 路径跟踪权重 `q_s`

`q_s` 越大，越坚持跟随 `/plan`；越小，越容易偏离全局路径绕障。它同时影响位置和航向状态，转角本身不由 `q_s` 单独限制。

现象对应关系：

| 现象 | 调整方向 |
|---|---|
| 无障碍时偏离全局路径 | 增大 `q_s` |
| 障碍物前只沿原路顶住/停车 | 先检查 `d_max` 和绕障初值，再考虑减小 `q_s` |
| 绕障幅度过大、回不来 | 减小 `eta` 或 `avoidance_seed_steer`，不要一次大幅修改 `q_s` |
| 轨迹对定位噪声敏感 | 减小 `q_s`、降低速度、改善 TF |

不要为了修复“转向角被当角速度”而盲目调 `q_s`。

### 4. 速度权重 `p_u`

`p_u` 主要惩罚线速度偏离参考速度，**不直接惩罚转角**。

- 太小：停车或倒车可能比前进绕障更便宜；
- 太大：遇到窄路或真实障碍时不愿减速；
- 它不是转向平滑参数，转向平滑主要由 `max_acce[1]`、底盘控制器和求解时延决定。

本仿真中将 `p_u` 从 0.3 提高到 1.0 后，居中障碍不再优先选择倒车。实车应从 0.5 或 1.0 开始，并配合低速测试。

### 5. 避障参数 `eta/d_max/d_min`

`d_max` 决定避障距离变量的上界和开始产生明显避障收益的范围，`eta` 决定避障收益的权重，`d_min` 是优化层期望的安全距离下界。

本车实测：LiDAR 前方约 0.925 m 的障碍换算到车体前缘后，净空约 0.506 m，正好等于旧配置 `d_max=0.5 m`。此时才开始避障，对 `R_min≈1.05m` 的 Ackermann 车已经太晚。因此仿真基线改为：

```yaml
eta: 0.5
d_max: 1.2
d_min: 0.15
```

实车起始建议：

1. 先固定 `eta`，逐步增大 `d_max`，直到车辆在障碍前能有完整转弯空间；
2. 再调 `eta`：不绕行就增加，绕得过宽或对远处障碍过敏就减小；
3. 根据车宽、定位误差、点云噪声和制动距离设置 `d_min`；
4. `collision_threshold` 是紧急停车阈值，不要为了让车辆“继续走”而把它调到不安全的值。

一个实用估计是：

```text
避障启动距离 ≈ 车辆需要的横向绕行空间
             + v × 单次求解/通信延迟
             + 定位和传感器余量
```

该距离最终要用实车低速实验确认，不应只按车长机械套公式。

### 6. PAN、DUNE 和 NRMP 点数

- `dune_max_num`：DUNE 每次最多接收的障碍点数。看到 `down sample the obs points from 722 to 200` 是正常日志，不是丢失全部障碍物。
- `nrmp_max_num`：进入凸优化层的有效障碍点数量，过小可能只看到一小部分障碍几何。
- `iter_num`：SCP/PAN 迭代次数。增加有机会提高避障质量，但会增加耗时；实车先用 2，确认实时性后再测试 3。
- 障碍点数量不是越大越好。CPU 求解超过命令保持周期时，增加点数反而会使避障滞后。

## 临时障碍绕行的特殊机制

仅靠从完全直的 Ackermann 名义轨迹开始优化，居中障碍会出现左右对称局部最优：第一次线性化中，转角对横向位置的影响可能还没有显现，优化器就会选择减速、停车或倒车。

当前配置中的：

```yaml
avoidance_seed_enabled: true
avoidance_seed_distance: 1.2
avoidance_seed_clearance: 0.15
avoidance_seed_steer: 0.25
```

含义是：检测到障碍物位于全局路径前方且阻断车体通道时，生成一个小幅、动态可行的 S 形控制初值；根据左右附近点数选择较空的一侧，并短暂保持方向，避免扫描噪声导致左右翻转。之后仍由 DUNE/NRMP 优化，不是直接硬编码绕行轨迹。

正常触发日志：

```text
Path-blocking obstacle detected; seeding left bypass
```

或：

```text
Path-blocking obstacle detected; seeding right bypass
```

如果实车不触发，按以下顺序检查：

1. `/scan` 是否包含障碍物，坐标是否正确；
2. 当前进程是否加载了最新配置，启动日志中的 DUNE 点数是否为 200；
3. 车辆状态是否使用 `rear_axle_link`；
4. 障碍物是否离车太近，已经没有足够转弯空间；
5. `avoidance_seed_steer` 是否过小；
6. 现场是否是窄通道，左右确实都没有可行空间。

## 实车调试流程

### 阶段 A：只验证接口，不放障碍物

实车启动 NeuPAN 时不要使用仿真时钟：

```bash
cd ~/AckermannRobot-3D
source /opt/ros/humble/setup.bash
source install/setup.bash
NEUPAN_USE_SIM_TIME=false bash scripts/run_neupan.sh
```

脚本默认仍使用仿真时钟，所以实车必须显式设置 `NEUPAN_USE_SIM_TIME=false`。实车的 `robot.yaml` 也应在最终部署分支中改成 `use_sim_time: false`。

先检查：

```bash
ros2 node list
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo map rear_axle_link
ros2 run tf2_ros tf2_echo rear_axle_link laser_link
ros2 topic echo /neupan_cmd_vel
```

确认：

- `/scan` 频率和时间戳稳定；
- `map→rear_axle_link` 连续且方向正确；
- `rear_axle_link→laser_link` 的平移与实车安装位置一致；
- 车辆静止时命令为 0；
- 低速前进时 `linear.x` 符号正确；
- 当前 `angular.z` 是角速度，不是前轮角度。

### 阶段 B：低速直线和大半径转弯

先把 `ref_speed` 和 `max_speed[0]` 限制在实车安全低速。无障碍测试要求：

- 小车沿全局路径稳定前进；
- 无左右摆动；
- 转弯方向与 RViz 中路径方向一致；
- 停车后没有持续输出非零命令。

如果直线都不稳定，停止调避障参数，优先检查 TF、轮速里程计、IMU、转角符号和底盘控制接口。

### 阶段 C：单个临时障碍物

在宽阔、可人工急停的区域，把障碍物放在全局路径中央，车辆前方留出至少 `d_max + v×延迟` 的车体净空。建议从低速和 2 m 左右距离开始。

预期过程：

```text
检测到障碍 → 选择左/右绕行侧 → 速度适当下降
          → 轨迹横向偏移 → 通过障碍 → 回到全局路径
```

一次只改一个参数。推荐顺序：

```text
d_max → eta → d_min → avoidance_seed_steer → q_s → p_u
```

不要在障碍已经贴近车体时反复试验；此时任何局部规划器都可能只能急停。

### 阶段 D：窄通道和动态障碍物

先验证障碍物左右都有足够间隙，再测试窄通道。动态障碍物不能依赖静态绕行初值，应优先保证减速和停车：

- 传感器丢失或 TF 连续失效时必须停车；
- `collision_threshold` 必须大于系统误差和底盘制动需求；
- 动态障碍物场景中宁可降低速度，也不要用增大 `eta` 代替制动策略。

## 日志和故障判断

| 日志/现象 | 含义 | 处理 |
|---|---|---|
| `down sample ... to 200` | 障碍点数量达到 DUNE 上限 | 正常；若漏检再检查点数和点云质量 |
| `Timestamped map-to-scan TF unavailable; using map-to-odom + odom-to-scan` | 定位链存在延迟，已用 odom 补偿 | 偶尔出现可接受；持续出现要检查时间戳 |
| `No recent usable LaserScan` | 超过 `scan_data_timeout` 没有有效扫描/TF | 必须停车；检查 `/scan` QoS、频率和 TF |
| `LaserScan TF unavailable; retaining last valid scan` | 单帧 TF 失败，暂时保留上一帧 map 坐标障碍点 | 偶发可接受；频繁出现要降低回调阻塞、查 TF |
| `Collision risk detected` | DUNE 当前最小距离低于紧急阈值 | 当作真实危险处理，不要先降低阈值 |
| `Path-blocking obstacle detected; seeding ... bypass` | 已识别全局路径被临时障碍阻断，并选定绕行侧 | 正常；观察 `/neupan_plan` 是否横向偏移 |
| 车辆只停不绕 | 作用距离太短、障碍太近、左右无空间或绕障初值未触发 | 先查 seed 日志，再增大 `d_max`，最后调权重 |
| 转角在左右最大值之间翻转 | 对称局部最优、点云抖动或求解过慢 | 固定/检查 seed、降低速度、限制转角变化率 |
| RViz 路径方向正确但车辆转向相反 | 航向、激光角度或底盘转角符号错误 | 查 TF、`flip_angle` 和控制器约定 |
| 无障碍也大幅绕行 | 点云包含地面/车体/自反射，或 `eta/d_max` 过大 | 先清理点云，再减小作用范围/权重 |
| 规划器约 1 秒才更新一次命令 | CPU 求解耗时超过控制周期 | 降低点数/时域/速度，或更换计算平台 |

## 实车参数记录表

每次实车试验建议复制下面表格，记录一项变化和结果，不要只凭 RViz 主观观察：

| 日期/版本 | 场景 | `ref_speed` | `d_max` | `d_min` | `eta` | `q_s` | `p_u` | 求解耗时 | 最小净空 | 结果 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
|  | 直线/转弯/障碍/窄道 |  |  |  |  |  |  |  |  |  |

建议同时保存：

```text
/scan
/plan
/neupan_initial_path
/neupan_ref_state
/neupan_plan
/neupan_cmd_vel
/tf
/tf_static
/odometry/filtered
```

## 相关文件

| 文件 | 用途 |
|---|---|
| `src/neupan_ros2/config/robots/ackermann_robot/planner.yaml` | NeuPAN MPC、机器人几何、PAN 和避障参数 |
| `src/neupan_ros2/config/robots/ackermann_robot/robot.yaml` | ROS 参数、TF、LaserScan、控制频率和安全超时 |
| `src/neupan_ros2/neupan_ros2/neupan_node.py` | ROS 输入输出、TF、扫描处理、命令限幅和安全停车 |
| `src/NeuPAN/neupan/neupan.py` | NeuPAN 主流程和临时障碍绕行初值 |
| `src/NeuPAN/neupan/blocks/pan.py` | PAN 交替优化、DUNE/NRMP 调用 |
| `src/NeuPAN/neupan/blocks/nrmp.py` | CVXPY/凸优化代价与约束 |
| `src/NeuPAN/neupan/robot/robot.py` | Ackermann 几何、速度边界和线性化模型 |
| `src/NeuPAN/example/dune_train/model/ackermann_robot_sim_070x052/model_5000.pth` | 当前仿真车训练权重 |
| `src/neupan_ros2/config/robots/ackermann_robot/models/dune_model_5000.pth` | ROS 运行时加载的 DUNE 权重 |
| `scripts/run_neupan.sh` | Conda 环境和 NeuPAN 节点启动脚本 |

## 重新编译和启动

修改 ROS wrapper 或子模块源码后：

```bash
cd ~/AckermannRobot-3D
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select neupan_ros2
```

修改 YAML 后，重新启动 NeuPAN 即可；当前脚本直接读取源码配置目录。启动日志应核对：

```text
Robot dimensions - Length: ...
Robot wheelbase: ...
PAN config - ... DUNE points: 200, NRMP points: 10
Ackermann command rate limiter enabled ...
NeuPAN node started - ... Threads: 4
```

仿真启动：

```bash
bash scripts/run_neupan.sh
```

实车启动：

```bash
NEUPAN_USE_SIM_TIME=false bash scripts/run_neupan.sh
```
