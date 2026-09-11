# AckermannRobot-3D-ROS2

3D LiDAR 阿克曼底盘的 Gazebo 仿真、建图与导航项目。

## 1. 编译

在项目根目录执行：

```bash
source /opt/ros/humble/setup.bash
git submodule update --init --recursive
colcon build --symlink-install
source install/setup.bash
```

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

本项目同时保留两套导航系统，分别用于不同的测试和对比场景，不删除任何一套。

### 4.1 NeuPAN 导航栈：LIORF + Smac + NeuPAN

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
    -> ackermann_smac_bridge
    -> Nav2 planner_server + SmacPlannerHybrid
    -> /plan_path
    -> NeuPAN
    -> 阿克曼底盘控制器
```

其中 `src/ackermann_smac_bridge` 负责接收目标点、调用
`/compute_path_to_pose`、发布 `/plan_path` 和全局路径剩余距离；它不是
Smac 规划器本身，也不负责局部控制。

### 4.2 Nav2 导航栈：Ackermann Nav2

Nav2 导航栈位于 `src/ackermann_nav`，不启动 NeuPAN，使用 Nav2 自带的
Smac Hybrid-A*、MPPI 和速度平滑器：

```bash
bash scripts/run_ackermann_nav.sh maps/mini
```

主要链路为：

```text
BT Navigator
    -> Smac Hybrid-A*（DUBIN，前进约束）
    -> MPPI Ackermann Controller
    -> Velocity Smoother
    -> cmd_bridge.py
    -> 阿克曼底盘控制器
```

两套系统的定位链和仿真环境可以复用，但导航控制链不同：

| 导航系统 | 全局规划 | 局部控制 | 主要入口 |
|---|---|---|---|
| NeuPAN 导航栈 | Smac Hybrid-A*（由 `ackermann_smac_bridge` 调用） | NeuPAN | `scripts/nav_liorf_neupan.sh` + `scripts/run_neupan.sh` |
| Nav2 导航栈 | Smac Hybrid-A*（Nav2 BT Navigator 直接调用） | MPPI Ackermann | `scripts/run_ackermann_nav.sh` |

默认不要同时运行两套导航。两套系统可能同时使用 `/map`、`/tf`、
`/goal_pose` 以及同一个 ros2_control 控制命令接口，容易造成重复规划或
多个节点同时向底盘发送命令。需要进行对比时，建议先完整停止当前导航栈，
再启动另一套；如果确实需要同时运行，应使用不同的 ROS Domain、命名空间
和控制话题进行隔离。

NeuPAN 导航栈相关包和入口（包括 `ackermann_smac_bridge`）会继续保留，不作为
Nav2 导航栈 `ackermann_nav` 的替代品删除。

## 单独启动仿真

仅启动 Gazebo：

```bash
ros2 launch ackermann_simulation gazebo.launch.py
```

仅预览机器人模型：

```bash
ros2 launch ackermann_simulation display.launch.py
```
