# AckermannRobot-3D-ROS2 — Real Vehicle Integration

本分支只保留实车部署链路，不包含 Gazebo、ros2_control 仿真控制器或仿真地图/启动脚本。

## 1. 主要组件

- `config/vehicle.yaml`：项目级车辆几何、硬限制、规划限制、LiDAR/IMU 外参的唯一配置源（不是 ROS package）
- `src/sensors`：传感器源码分类目录（不是 ROS package）
- `src/sensors/lidar`：Hesai LiDAR 驱动（ROS package 名仍为 `lidar_driver`）
- `src/sensors/imu`：LPMS-IG1 IMU 驱动（ROS package 名仍为 `lpms_ig1`）
- `src/nav_neupan`：Smac Hybrid-A* + NeuPAN 导航栈源码分类目录（不是 ROS package）
- `src/nav_neupan/ackermann_smac_bridge`：项目侧 Smac 目标/路径适配桥（ROS package 名仍为 `ackermann_smac_bridge`）
- `src/nav_neupan/nav2_smac_planner`：Nav2 Smac 规划插件（ROS package 名仍为 `nav2_smac_planner`）
- `src/nav_neupan/neupan_ros2`：NeuPAN ROS2 接口（ROS package 名仍为 `neupan_ros2`）
- `src/motion_interface`：命令安全门 + `/ackermann_cmd` 到 STM32 UDP 协议的实车接口
- `src/ackermann_bringup`：实车硬件、定位以及 Smac + NeuPAN 导航栈 bringup
- `src/ackermann_nav`：第二套独立 Nav2 导航栈（Smac Hybrid-A* + MPPI + BT Navigator）
- `src/lio-sam` / `src/liorf_localization`：建图与先验地图定位

## 2. 编译

```bash
source /opt/ros/humble/setup.bash
git submodule sync --recursive
git submodule update --init --recursive
colcon build --symlink-install
source install/setup.bash
```

## 3. 项目级车辆参数

车辆物理、运动和传感器安装参数统一放在仓库根目录：

```text
config/vehicle.yaml
```

它不是 ROS package。实车启动链路把它的绝对路径传给各模块，再分别映射到 `motion_interface`、两套导航栈、NeuPAN、LIORF 和 LIO-SAM。

主要职责：

- `geometry`：车长、车宽、轴距、轮距、轮胎尺寸、前后悬
- `control_limits`：STM32 执行侧硬限制
- `planning`：Smac、NeuPAN、MPPI 共用的规划/运动约束和 footprint
- `sensor_extrinsics.lidar_to_imu`：LIORF 与 LIO-SAM 共用的 LiDAR/IMU 外参

当前车辆几何和 footprint 已标记为 `VERIFIED`；底盘速度/转角硬限制和 LiDAR/IMU 外参仍需要实车确认。设备 IP、端口、串口等部署参数继续留在各自驱动/接口配置中。

LiDAR/IMU 外参直接对应 LIO-SAM/LIORF：

```yaml
vehicle:
  sensor_extrinsics:
    lidar_to_imu:
      status: "VERIFIED"
      translation_xyz: [x, y, z]
      rotation_matrix: [r00, r01, r02, r10, r11, r12, r20, r21, r22]
      orientation_matrix: [r00, r01, r02, r10, r11, r12, r20, r21, r22]
```

外参未标记为 `VERIFIED` 时，LIORF 和 LIO-SAM 都会拒绝启动。

## 4. 实车启动入口

```bash
./scripts/start_vehicle.sh lidar
./scripts/start_vehicle.sh imu
./scripts/start_vehicle.sh bridge
./scripts/start_vehicle.sh all
```

两套导航栈分别启动：

```bash
# 栈 A：Smac Hybrid-A* + NeuPAN
./scripts/start_vehicle.sh nav maps/<map_name>
# 另开终端：bash scripts/run_neupan.sh

# 栈 B：完整 Nav2（Smac Hybrid-A* + MPPI + BT Navigator）
./scripts/start_vehicle.sh nav2 maps/<map_name>
```

**不要同时启动 `nav` 和 `nav2`**。两套栈都会使用标准 Nav2 节点名，并最终输出统一的 `/ackermann_cmd`。

## 5. 统一实车控制接口

最终 STM32 接口始终使用：

```text
/ackermann_cmd
  linear.x  = longitudinal speed [m/s]
  angular.z = front-wheel steering angle [rad]
```

### 栈 A：NeuPAN

```text
Smac Hybrid-A*
  -> /plan
  -> NeuPAN
  -> /neupan_cmd_vel_raw        (speed + steering angle)
  -> motion_interface/command_gate
  -> /ackermann_cmd
  -> motion_interface/stm32_bridge
  -> STM32
```

### 栈 B：ackermann_nav

```text
Smac Hybrid-A*
  -> MPPI
  -> velocity_smoother
  -> /ackermann_nav/cmd_vel_smoothed   (speed + yaw rate)
  -> nav2_cmd_adapter.py
  -> /ackermann_nav/ackermann_cmd_raw  (speed + steering angle)
  -> motion_interface/command_gate
  -> /ackermann_cmd
  -> motion_interface/stm32_bridge
  -> STM32
```

`ackermann_nav` 的 Nav2 输出中 `angular.z` 是 yaw rate，因此必须经过适配器按自行车模型换算为前轮等效转角；不能直接送给 STM32 bridge。

## 6. 两套导航栈的车辆参数

两套栈都不在包内维护独立的实车几何副本。

Smac + NeuPAN 从根 `vehicle.yaml` 获取最小转弯半径、车体几何和规划速度。`ackermann_nav` 启动时也会从同一个文件注入：

- wheelbase
- planning max speed / acceleration
- minimum turning radius
- local/global costmap footprint
- robot base frame
- MPPI Ackermann minimum turning radius
- Nav2-to-Ackermann 转角换算参数

这样修改实车几何时不会出现两套导航栈各用一份旧数据。

## 7. 定位与建图

两套导航栈当前都使用 LIORF 先验地图定位，并要求根配置中的 LiDAR/IMU 外参为 `VERIFIED`。

LIO-SAM 建图：

```bash
ros2 launch lio_sam run.launch.py \
  vehicle_config:=$(pwd)/config/vehicle.yaml \
  use_sim_time:=false
```

## 8. 地图

`maps/` 只用于存放实车地图。导航目录应包含：

```text
map.yaml
map.pgm
GlobalMap.pcd
```
