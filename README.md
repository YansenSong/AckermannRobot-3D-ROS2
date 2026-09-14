# AckermannRobot-3D-ROS2 — Real Vehicle Integration

本分支只保留实车部署链路，不包含 Gazebo、ros2_control 仿真控制器或仿真地图/启动脚本。

## 1. 主要组件

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

## 3. 实车硬件入口

所有硬件默认关闭，需要显式启用：

```bash
ros2 launch ackermann_bringup real_vehicle.launch.py \
  enable_lidar:=true \
  lidar_config:=/path/to/hesai.yaml \
  enable_imu:=true \
  imu_port:=/dev/ttyUSB0 \
  enable_control:=true
```

只测试某一个硬件模块时，只打开对应开关即可。底盘控制不会默认启动。

## 4. 控制接口

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

## 5. 导航

实车导航入口通过 `real_vehicle.launch.py enable_navigation:=true` 挂接 `navigation.launch.py`。当前定位、车辆几何、传感器外参等实车参数仍需要按实际车辆继续整理和标定；本分支不再保留仿真参数作为运行入口。

NeuPAN 可单独启动：

```bash
bash scripts/run_neupan.sh
```

默认使用系统时间（`use_sim_time=false`）。

## 6. 地图

`maps/` 只用于存放实车采集/生成的地图。仿真 `mini.world` 对应地图不在本分支维护。
