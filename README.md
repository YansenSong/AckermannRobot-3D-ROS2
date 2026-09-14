# AckermannRobot-3D-ROS2 — Real Vehicle Integration

本分支只保留实车部署链路，不包含 Gazebo、ros2_control 仿真控制器或仿真地图/启动脚本。

## 1. 主要组件

- `config/vehicle.yaml`：项目级车辆几何、硬限制、规划限制、LiDAR/IMU 外参的唯一配置源（不是 ROS package）
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

车辆物理、运动和传感器安装参数统一放在仓库根目录：

```text
config/vehicle.yaml
```

它不是 ROS package，也不会通过 `get_package_share_directory()` 查找。实车启动链路把它的绝对路径传给各模块，再分别映射到 motion_control、Smac、NeuPAN、LIORF 和 LIO-SAM。

配置中的主要职责：

- `geometry`：实测车长、车宽、轴距、轮距、轮胎尺寸、前后悬
- `control_limits`：底盘执行侧硬限制，例如最大前进/倒车速度和最大前轮转角
- `planning`：Smac/NeuPAN 使用的规划限制，例如规划速度、最小转弯半径和碰撞 footprint
- `sensor_extrinsics.lidar_to_imu`：LIORF 与 LIO-SAM 共用的 LiDAR/IMU 外参

当前车辆几何和 footprint 已按实测尺寸标记为 `VERIFIED`；底盘速度/转角硬限制和 LiDAR/IMU 外参仍需要实车确认。LiDAR IP、STM32 IP、串口等设备/部署参数不放入 `vehicle.yaml`，继续留在各自驱动配置中。

LiDAR/IMU 外参直接对应 LIO-SAM/LIORF 的参数语义：

```yaml
vehicle:
  sensor_extrinsics:
    lidar_to_imu:
      status: "VERIFIED"
      translation_xyz: [x, y, z]       # extrinsicTrans
      rotation_matrix: [r00, r01, r02, r10, r11, r12, r20, r21, r22]       # extrinsicRot
      orientation_matrix: [r00, r01, r02, r10, r11, r12, r20, r21, r22]    # extrinsicRPY
```

`translation_xyz` 单位为米；两个矩阵均为 row-major 3x3。为了避免轴向约定被错误转换，启动层不会从 Euler RPY 自动猜这两个矩阵。外参未标记为 `VERIFIED` 时，LIORF 和 LIO-SAM 都会拒绝启动。

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

## 6. 导航与定位

实车导航入口通过 `real_vehicle.launch.py enable_navigation:=true` 挂接 `navigation.launch.py`。Smac 的最小转弯半径、footprint 和 base frame 在启动时从 `config/vehicle.yaml` 注入；LIORF 的 `extrinsicTrans`、`extrinsicRot`、`extrinsicRPY` 也从同一个根配置注入。

因此，在 LiDAR/IMU 外参仍为 `NOT VERIFIED` 或为空时，实车定位/导航会 fail-fast，而不会继续使用旧仿真外参。

NeuPAN 单独启动：

```bash
bash scripts/run_neupan.sh
```

`run_neupan.sh` 会从 `config/vehicle.yaml` 生成临时 NeuPAN planner 配置，因此 `src/neupan_ros2/config/robots/ackermann_robot/planner.yaml` 只保留算法调参，不再保存车辆几何/速度参数。默认使用系统时间（`use_sim_time=false`）。

## 7. LIO-SAM 建图

LIO-SAM 同样要求使用根车辆配置中的 VERIFIED LiDAR/IMU 外参：

```bash
ros2 launch lio_sam run.launch.py \
  vehicle_config:=$(pwd)/config/vehicle.yaml \
  use_sim_time:=false
```

`src/lio-sam/config/params.yaml` 只保留 LIO-SAM 自身的传感器模型、噪声和算法调参；物理安装外参不再在该文件中重复保存。

## 8. 地图

`maps/` 只用于存放实车采集/生成的地图。仿真 `mini.world` 对应地图不在本分支维护。
