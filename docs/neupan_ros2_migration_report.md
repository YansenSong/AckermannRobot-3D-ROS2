# NeuPAN ROS2 基线替换报告

## 1. 执行信息

- 执行日期：2026-09-10
- 项目目录：`/home/young/Project/AckermannRobot`
- 项目分支：`main`
- 项目执行前 SHA：`5d6fb60666bf2ff9043c705d37993b7dd75a53d7`
- upstream clone：`/home/young/Project/neupan_ros2`
- upstream 分支：`main`
- upstream SHA：`4ffb7ec2dc45ff7ee9024f64083813237906af98`

执行前 dirty 状态：

- 项目仓库：仅有用户提供的计划文件
  `docs/CODEX_NeuPAN_ROS2_官方基线替换与Ackermann最小适配计划.md`
- upstream clone：clean

## 2. 备份与替换

完整备份目录：

`/home/young/Project/AckermannRobot_neupan_backup_20260910_195107`

备份内容包括：

- 旧 `src/neupan_ros2`
- 允许修改的外围文件：`ackermann_control/CMakeLists.txt`、
  `ackermann_control/cmd_vel_mux.py`、`ackermann_bringup/launch/navigation.launch.py`、
  `scripts/run_neupan.sh`
- 旧包内所有 `.pth` 文件清单
- `config/dune_checkpoint` 独立副本及 before/after SHA256

替换方式为旧包整体移入备份目录，再整体复制 clean upstream package；没有
对新旧包做 rsync 混合合并。旧目录额外保留在
`neupan_ros2_old_removed`，因此可以恢复。

## 3. 权重保留结果

目标目录：`src/neupan_ros2/config/dune_checkpoint`

| 文件 | SHA256（before/after 相同） |
|---|---|
| `scout_model_5000.pth` | `0f8e51f22240e8fb87735e1fa864bd94cb0db6d4001d4dc2bf05f5f1f13dc972` |
| `scout_model_5000_v2.pth` | `66c55fedae14bc4684f0dee9c37fe5d0e5a86abfce74ddf99af2663e85edb8de` |

SHA256 对比为空差异，验收通过。

## 4. 实际修改文件

### Vendored NeuPAN ROS2

- `src/neupan_ros2/neupan_ros2/neupan_node.py`
  - 仅增加 Smac Reeds-Shepp 路径的最小 reverse gear 识别。
  - 官方 `generate_twist_msg()` 未改。
- `src/neupan_ros2/config/robots/ackermann_robot/robot.yaml`
- `src/neupan_ros2/config/robots/ackermann_robot/planner.yaml`
- `src/neupan_ros2/config/robots/ackermann_robot/models/dune_model_5000.pth`

`ackermann_robot` profile 来自官方 Ranger profile；历史
`config/dune_checkpoint` 没有被用作第一轮 checkpoint。第一轮仍使用
官方 Ranger baseline model。

### Ackermann 外围适配

- 新增 `src/ackermann_control/neupan_ackermann_adapter.py`
- `src/ackermann_control/CMakeLists.txt`
  - 安装 adapter executable
- `src/ackermann_bringup/launch/navigation.launch.py`
  - 在 NeuPAN 与现有 mux 之间加入 adapter
- `scripts/run_neupan.sh`
  - 固定新的 Ackermann profile 路径
  - 启动前打印 NeuPAN core / ROS2 wrapper import 来源

没有修改 `cmd_vel_mux.py`，也没有修改 Smac、LIORF、导航状态机或
`third_party/NeuPAN`。

## 5. 最终 NeuPAN 配置

### ROS2 profile

- `robot_type: ackermann_robot`
- `map_frame: map`
- `base_frame: rear_axle_link`
- `lidar_frame: laser_link`
- `cmd_vel_topic: /neupan_cmd_vel_raw`
- `scan_topic: /scan`
- `plan_input_topic: /plan`
- `goal_topic: /neupan_unused_goal`
- `include_initial_path_direction: true`
- `control_frequency: 50.0`
- `scan_range_min/max: 0.15 / 5.0 m`

### `planner.yaml`

```yaml
receding: 15
step_time: 0.2
ref_speed: 0.5
collision_threshold: 0.01

robot:
  kinematics: acker
  max_speed: [0.7, 0.422]
  max_acce: [1.0, 0.328]
  length: 0.720
  width: 0.500
  wheelbase: 0.593

ipath:
  interval: 0.03
  curve_style: dubins
  min_radius: 1.320
  loop: false
  arrive_threshold: 0.5
  close_threshold: 0.05
  arrive_index_threshold: 3

pan:
  iter_num: 2
  dune_max_num: 200
  nrmp_max_num: 10
  iter_threshold: 0.1

adjust:
  q_s: 0.1
  p_u: 0.5
  eta: 15.0
  d_max: 0.1
  d_min: 0.01
```

当前 `ψ_max=0.422 rad` 来自：

`atan(wheelbase / Smac_minimum_turning_radius) = atan(0.593 / 1.320)`。

baseline 中没有 `ind_range: 0`、`min_speed` 或 `avoidance_seed_*`。

### Adapter

adapter 只执行：

`ω = v * tan(ψ) / 0.593`

不改变 `linear.x`，不订阅 `/plan` 或 `/scan`，不做限速、超时、DUNE
或避障逻辑；非有限输入发布零命令，`wheelbase <= 0` 启动失败。

## 6. 构建与静态检查

纯官方包先单独构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select neupan_ros2
```

结果：PASS。

完成适配后的完整工作区构建：

```bash
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH bash -lc \
  'source /opt/ros/humble/setup.bash && \
   cd /home/young/Project/AckermannRobot && \
   colcon build --symlink-install --allow-overriding nav2_smac_planner'
```

结果：10 packages PASS。`pcd2gridmap` 的“没有 install target”是已有构建
警告；`ackermann_control` 只有 CMake 最低版本弃用提示。

通过的静态/接口检查：

- `python3 -m py_compile`：NeuPAN node、adapter PASS
- Ackermann 两份 YAML 解析与参数断言 PASS
- launch Python AST 解析 PASS
- `bash -n scripts/run_neupan.sh` PASS
- `ros2 launch ackermann_bringup navigation.launch.py --show-args` PASS
- `ros2 pkg executables` 可见 `neupan_ackermann_adapter.py` PASS
- `git diff --check` PASS
- 新 `src/neupan_ros2` 中未发现旧 fork 的
  `avoidance_seed_*`、`scan_tf_max_age`、`scan_data_timeout`、
  `direct_goal_planning`、`command_rate_limit`、`executor_threads`、
  `min_speed`、`odom_frame` 字段

## 7. 运行时验证

运行流程：

```bash
bash scripts/nav_liorf_neupan.sh maps/mini
bash scripts/run_neupan.sh
```

结果：

- LIORF 成功加载 `maps/mini/GlobalMap.pcd`，32135 points
- `/initialpose` 后 ICP alignment succeeded
- Smac 使用 Reeds-Shepp，普通测试生成 32 poses
- 反向目标测试生成 46 poses；根据 pose heading 与 segment travel
  direction 判定 45 个 reverse segments
- `map -> rear_axle_link` TF 可用；LIORF 同时解析 `laser_link -> base_link`
  静态变换
- NeuPAN import 来源：
  `/home/young/Project/AckermannRobot/third_party/NeuPAN/neupan/...`
- NeuPAN ROS2 import 来源：当前项目的
  `/home/young/Project/AckermannRobot/build/neupan_ros2/neupan_ros2/...`
  （`--symlink-install` 的正常路径）

topic ownership：

| Topic | Publisher | Subscriber |
|---|---|---|
| `/plan` | `planner_server`（1 个） | `neupan_node`、RViz |
| `/neupan_cmd_vel_raw` | `neupan_node`（1 个） | `neupan_ackermann_adapter` |
| `/neupan_cmd_vel` | `neupan_ackermann_adapter`（1 个） | `cmd_vel_mux` |
| `/ackermann_steering_controller/reference` | `cmd_vel_mux`（1 个） | Ackermann controller |

实测频率约为：

- `/scan`: 20.2 Hz
- `/neupan_cmd_vel_raw`: 8.9 Hz
- `/neupan_cmd_vel`: 8.9 Hz
- `/ackermann_steering_controller/reference`: 8.7 Hz

adapter 数学测试：

- 输入 `v=0.5, ψ=0.2`
- 输出 `v=0.5, ω=0.170919085589`
- 理论值 `0.5*tan(0.2)/0.593=0.170919085589`
- wheelbase `0` 启动失败

运行时倒车命令测试：

- NeuPAN raw：`linear.x=-0.3797737062`、`angular.z=+0.0035552301`
- adapter：`linear.x=-0.3797737062`、`angular.z=-0.0022768779`
- 线速度保持不变，yaw-rate 符号符合 `v*tan(ψ)/L`

本次不把 NeuPAN 的 `arrived` 或机器人实际行驶路线作为验收条件；用户已
明确这部分主要受 NeuPAN core 影响。本次验收关注全局 `/plan`、接口拓扑、
TF、倒车方向和控制量数学适配。

## 8. 已知未解决问题 / 后续任务

- `cmd_vel_mux.py` 仍可能重发最后一条命令；后续独立加入 stale-command
  timeout（建议 0.2--0.5 s）。
- `third_party/NeuPAN` 仍是项目本地修改版；若需要官方 core，需要单独做
  core rebase，本次没有偷偷替换。
- 当前第一轮沿用 Ranger DUNE 几何与 checkpoint；后续应根据真实 Ackermann
  footprint 评估重训或 A/B。
- 第一轮未启用非对称 `min_speed`；前进/倒车不同限速应单独做 core 行为实验。
- 为保持官方 baseline，本次没有把旧 fork 的 path-update `arrive/stop`
  reset 逻辑带回；如后续需要连续多目标任务，应单独评估官方 core 的状态
  重置行为。
- Gazebo `mini.world` 的 non-unique link、wheel-slip normal-force 和
  Gazebo Classic EOL 警告属于仿真环境已有问题，本次未修改。

## 9. Git 状态

没有自动 commit 或 push。最终状态请用：

```bash
git -C /home/young/Project/AckermannRobot status --short
git -C /home/young/Project/AckermannRobot diff --stat
```
