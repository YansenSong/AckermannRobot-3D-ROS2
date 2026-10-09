# Alpha

3D LiDAR 阿克曼底盘的 Gazebo 仿真、建图与导航项目。

## 项目结构

`src/` 按功能分层，ROS 包名保持不变：

```text
src/
├── mapping/lio-sam/                         # 建图
├── localization/liorf_localization/         # 定位
├── planning/
│   ├── nav2_smac_planner/                    # 全局规划
│   └── smac_neupan_bridge/                # 阿克曼规划接口
├── control/
│   ├── neupan_ros2/                          # 局部避障
│   └── vehicle_control/                    # 运动控制
├── bringup/                             # 系统启动编排（robot_bringup）
├── simulation/          # 仿真、模型和传感器
└── extension/
    ├── nav_status/                         # 导航状态监控
    ├── mission_manager/                    # 机器人侧任务管理
    └── area_rules/                         # 区域规则

third_party/RobotPilot/                         # 机器人网页与后端
third_party/pcd2pgm/                            # PCD 转 2D 地图工具
```

## 1. 编译

在项目根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

S0 平台接口、机器人事件补传与安全门禁的当前状态见 [S0 实施记录](docs/s0/S0_IMPLEMENTATION_REPORT.md)、[架构边界](docs/s0/architecture.md) 和 [协议目录](docs/s0/contracts/api-v1.md)。可用 `bash scripts/check_s0.sh` 执行本地构建与单测；需要先在 `third_party/RobotPilot/web` 运行 `npm ci`。脚本会明确跳过需要 Gazebo 运行态与实机设备的验收。

每个新终端均需加载环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 2. 建图

启动 mini.world、LIO-SAM 和键盘控制：

```bash
bash scripts/mapping_mini.sh
```

当前终端直接控制车辆：方向键行驶和转向，`B` 切换前进/倒车，空格急停，`Q` 或 `Ctrl+C` 退出并关闭建图仿真。

完成环境覆盖后，在另一个已加载 ROS 环境的终端保存地图。先创建地图目录，例如 `mini`：

```bash
mkdir -p maps/mini
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
  "{resolution: 0.2, destination: $PWD/maps/mini/}"
```

保存结果位于 `maps/mini/`，其中导航至少需要 `GlobalMap.pcd`。

## 3. 生成 2D 地图

首次使用时编译 PCD 转图工具：

```bash
cmake -S third_party/pcd2pgm -B third_party/pcd2pgm/build
cmake --build third_party/pcd2pgm/build -j
```

转换指定地图目录：

```bash
bash scripts/pcd_to_map.sh maps/mini
```

会在同一目录生成 `map.pgm` 和 `map.yaml`。

## 4. 自主导航

本项目使用 LIORF、Smac Hybrid-A* 和 NeuPAN 完成自主导航。

### 4.1 LIORF + Smac + NeuPAN

确认 `maps/mini/` 内已有：

```text
GlobalMap.pcd
map.pgm
map.yaml
```

终端 1 启动 Gazebo、liorf 先验地图定位和 Smac Hybrid A*：

```bash
bash scripts/nav_liorf_neupan.sh maps/mini
```

终端 2 启动 NeuPAN：

```bash
bash scripts/run_neupan.sh
```

在 RViz 中依次使用 **2D Pose Estimate** 设置 liorf 初始位姿，等待定位稳定后使用 **2D Goal Pose** 设置目标点。

NeuPAN 导航栈的主要链路为：

```text
/goal_pose
    -> smac_neupan_bridge
    -> Nav2 planner_server + SmacPlannerHybrid
    -> /plan_path
    -> NeuPAN
    -> 阿克曼底盘控制器
```

其中 `src/planning/smac_neupan_bridge` 负责接收目标点、调用
`/compute_path_to_pose`、发布 `/plan_path` 和全局路径剩余距离；它不是
Smac 规划器本身，也不负责局部控制。

## 5. RobotPilot 网页

首次使用先安装前端依赖，并按第 1 节编译 ROS 工作区：

```bash
cd third_party/RobotPilot/web
npm ci
cd ../../..
bash scripts/run_robotpilot.sh
```

打开 `http://localhost:3000/`。脚本同时启动前端和 ROS 后端（`127.0.0.1:5050`），按 `Ctrl+C` 一起停止。它默认面向本机仿真；建图和导航仍需分别启动。更多说明见 [RobotPilot README](third_party/RobotPilot/README.md)。

## 脚本速查

| 脚本 | 用途 |
|---|---|
| `mapping_mini.sh` | 启动 mini.world、LIO-SAM 建图和键盘控制 |
| `pcd_to_map.sh maps/<地图名>` | 将 `GlobalMap.pcd` 转为 `map.pgm`、`map.yaml` |
| `nav_liorf_neupan.sh maps/<地图名>` | 启动仿真导航主体：LIORF 定位和 Smac 规划 |
| `run_neupan.sh` | 在另一个终端启动 NeuPAN 局部规划 |
| `run_robotpilot.sh` | 同时启动 RobotPilot 前端和后端 |

以上脚本均从项目根目录以 `bash scripts/<脚本名>` 运行。

## 单独启动仿真

仅启动 Gazebo：

```bash
ros2 launch simulation gazebo.launch.py
```

仅预览机器人模型：

```bash
ros2 launch simulation display.launch.py
```
