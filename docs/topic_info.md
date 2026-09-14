# 实车导航与控制话题

`real-vehicle-integration` 分支只维护实车链路。

## 控制链

```text
/goal_pose
    -> ackermann_smac_bridge
    -> Nav2 SmacPlannerHybrid
    -> /plan
    -> NeuPAN
    -> /neupan_cmd_vel_raw
    -> motion_interface/command_gate
    -> /ackermann_cmd
    -> motion_interface/stm32_bridge
    -> UDP -> STM32
```

## 关键话题

| 话题 | 消息类型 | 作用 |
|---|---|---|
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 导航目标 |
| `/plan` | `nav_msgs/msg/Path` | Smac 全局路径，供 NeuPAN 使用 |
| `/plan_path` | `nav_msgs/msg/Path` | 全局路径兼容输出，供状态机/RViz 使用 |
| `/lidar_points` | `sensor_msgs/msg/PointCloud2` | Hesai 实车点云默认输出 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 点云转换后的二维激光数据，供 NeuPAN/代价地图使用 |
| `/imu/data` | `sensor_msgs/msg/Imu` | LPMS-IG1 IMU 数据 |
| `/neupan_cmd_vel_raw` | `geometry_msgs/msg/Twist` | NeuPAN 输出；`linear.x`=速度，`angular.z`=前轮转角 |
| `/ackermann_cmd` | `geometry_msgs/msg/Twist` | 统一实车控制指令；`linear.x`=速度，`angular.z`=前轮转角 |
| `/stop` | `std_msgs/msg/Bool` | 集中停车覆盖；`true` 强制零指令 |
| `/navigation/state` | `nav_status/msg/NavigationStatus` | 导航状态 |

## `motion_interface`

`motion_interface` 包含两个独立节点：

```text
command_gate
  /neupan_cmd_vel_raw + /stop
  -> /ackermann_cmd

stm32_bridge
  /ackermann_cmd
  -> hard limits
  -> STM32 UDP protocol
```

两层都保留命令超时保护：上游规划器停止更新时 `command_gate` 输出零指令；如果 gate 本身停止工作，`stm32_bridge` 仍会因 `/ackermann_cmd` 超时向 STM32 发送停车帧。

## `/ackermann_cmd` 接口约定

```text
linear.x  : longitudinal speed [m/s]
angular.z : front-wheel steering angle [rad]
```

`angular.z` 不是 yaw rate。`stm32_bridge` 直接把前轮转角转换为 STM32 EPS 协议字段。

## 时间源

实车默认全部使用系统时间：

```text
use_sim_time = false
```
