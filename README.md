# AckermannRobot-3D-ROS2 — Real Vehicle Integration

本分支只保留实车部署链路，不包含 Gazebo、ros2_control 仿真控制器或仿真地图/启动脚本。

## 1. 主要组件

- `config/vehicle.yaml`：项目级车辆几何、硬限制、规划限制的唯一配置源（不是 ROS package）
- `src/lidar`：Hesai LiDAR 驱动（ROS package 名仍为 `lidar_driver`）
- `src/imu`：LPMS-IG1 IMU 驱动（ROS package 名仍为 `lpms_ig1`）
- `src/motion_control`：`/ackermann_cmd` 到 STM32 UDP 控制协议的实车后端
- `src/ackermann_control`：NeuPAN 控制命令安全门，统一输出 `/ackermann_cmd`
- `src/ackermann_bringup`：实车定位、规划、导航和硬件 bringup
- `src/lio-sam` / `src/liorf_localization`：建图与先验地图定位
- `src/ackermann_smac_bridge` / `src/nav2_smac_planner`：Smac 全局规划链路
- `src/neupan_ros2`：NeuPAN ROS2 接口

## 2. 编译

```bash
source /opt/ros/humble/setup.bash
git submodule update --init --recursive
colcon build --symlink-install
source install/setup.bash
```

## 3. 项目级车辆参数

车辆物理/运动相关参数统一放在仓库根目录：

```text
config/vehicle.yaml
```

它不是 ROS package，也不会通过 `get_package_share_directory()` 查找。当前由实车启动脚本将绝对路径传给 `ackermann_bringup`，再分别映射到 motion_control、Smac 和 NeuPAN。

配置分为两类限制：

- `control_limits`：底盘执行侧硬限制，例如最大前进/倒车速度和最大前轮转角
- `planning`：Smac/NeuPAN 使用的保守规划限制，例如规划速度、最小转弯半径和碰撞 footprint

当前几何和规划包络仍标记为 `NOT VERIFIED`，在正式实车导航前需要按实测值更新。LiDAR IP、STM32 IP、串口等设备/部署参数不放入 `vehicle.yaml`，继续留在各自驱动配置中。

## 4. 实车硬件入口

推荐使用：

```bash
./scripts/start_vehicle.sh lidar
./scripts/start_vehicle.sh imu
./scripts/start_vehicle.sh bridge
./scripts/start_vehicle.sh all
./scripts/start_vehicle.sh nav maps/<map_name>
```

也可以直接调用 launch，但启用控制或导航时需要显式提供根配置：

```bash
ros2 launch ackermann_bringup real_vehicle.launch.py \
  vehicle_config:=$(pwd)/config/vehicle.yaml \
  enable_lidar:=true \
  lidar_config:=/path/to/hesai.yaml \
  enable_imu:=true \
  imu_port:=/dev/ttyUSB0 \
  enable_control:=true
```

只测试某一个硬件模块时，只打开对应开关即可。底盘控制不会默认启动。

## 5. 控制接口

统一实车控制话题：

```text
/neupan_cmd_vel_raw
        ↓
   cmd_vel_mux
        ↓
 /ackermann_cmd
        ↓
 motion_control
        ↓
   UDP → STM32
```

`/ackermann_cmd` 使用 `geometry_msgs/msg/Twist`：

- `linear.x`：纵向速度，单位 m/s
- `angular.z`：前轮转角，单位 rad；不是车体 yaw rate

`/stop` 为集中停车覆盖话题，`std_msgs/msg/Bool(data=true)` 会强制输出零指令。

## 6. 导航

实车导航入口通过 `real_vehicle.launch.py enable_navigation:=true` 挂接 `navigation.launch.py`。Smac 的最小转弯半径、footprint 和 base frame 在启动时从 `config/vehicle.yaml` 注入。

NeuPAN 单独启动：

```bash
bash scripts/run_neupan.sh
```

`run_neupan.sh` 会从 `config/vehicle.yaml` 生成临时 NeuPAN planner 配置，因此 `src/neupan_ros2/config/robots/ackermann_robot/planner.yaml` 只保留算法调参，不再保存车辆几何/速度参数。默认使用系统时间（`use_sim_time=false`）。

定位、LiDAR/IMU 外参等实车标定仍需要继续整理；当前不会把旧仿真外参自动写进根车辆配置。

## 7. 地图

`maps/` 只用于存放实车采集/生成的地图。仿真 `mini.world` 对应地图不在本分支维护。
