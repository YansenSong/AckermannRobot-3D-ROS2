# 实车导航与控制话题

`real-vehicle-integration` 同时保留两套可选导航栈。不要同时启动两套栈。

## 栈 A：Smac Hybrid-A* + NeuPAN

```text
/goal_pose
    -> ackermann_smac_bridge
    -> Nav2 SmacPlannerHybrid
    -> /plan
    -> NeuPAN
    -> /neupan_cmd_vel_raw
       linear.x = speed
       angular.z = front-wheel steering angle
    -> motion_interface/command_gate
    -> /ackermann_cmd
    -> motion_interface/stm32_bridge
    -> UDP -> STM32
```

## 栈 B：ackermann_nav / Nav2 + MPPI

```text
Nav2 goal
    -> BT Navigator
    -> SmacPlannerHybrid
    -> MPPI Controller
    -> /ackermann_nav/cmd_vel_raw
    -> velocity_smoother
    -> /ackermann_nav/cmd_vel_smoothed
       linear.x = speed
       angular.z = yaw rate
    -> nav2_cmd_adapter
    -> /ackermann_nav/ackermann_cmd_raw
       linear.x = speed
       angular.z = front-wheel steering angle
    -> motion_interface/command_gate
    -> /ackermann_cmd
    -> motion_interface/stm32_bridge
    -> UDP -> STM32
```

## 关键话题

| 话题 | 消息类型 | 作用 |
|---|---|---|
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 导航目标 |
| `/plan` | `nav_msgs/msg/Path` | NeuPAN 栈的 Smac 全局路径 |
| `/lidar_points` | `sensor_msgs/msg/PointCloud2` | Hesai 实车点云默认输出 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 二维障碍物输入 |
| `/imu/data` | `sensor_msgs/msg/Imu` | LPMS-IG1 IMU 数据 |
| `/neupan_cmd_vel_raw` | `geometry_msgs/msg/Twist` | NeuPAN 原始 Ackermann 命令 |
| `/ackermann_nav/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | Nav2 平滑后的 body twist；`angular.z` 是 yaw rate |
| `/ackermann_nav/ackermann_cmd_raw` | `geometry_msgs/msg/Twist` | Nav2 适配后的 Ackermann 命令；`angular.z` 是前轮转角 |
| `/ackermann_cmd` | `geometry_msgs/msg/Twist` | 统一实车命令；`angular.z` 是前轮转角 |
| `/stop` | `std_msgs/msg/Bool` | 集中停车覆盖；`true` 强制零指令 |
| `/navigation/state` | `nav_status/msg/NavigationStatus` | NeuPAN 栈导航状态 |

## `motion_interface`

`motion_interface` 包含两个独立节点：

```text
command_gate
  stack-specific Ackermann command + /stop
  -> /ackermann_cmd

stm32_bridge
  /ackermann_cmd
  -> hard limits
  -> STM32 UDP protocol
```

两层都保留命令超时保护。

## `/ackermann_cmd` 接口约定

```text
linear.x  : longitudinal speed [m/s]
angular.z : front-wheel steering angle [rad]
```

`/ackermann_cmd.angular.z` 永远不是 yaw rate。Nav2/MPPI 的 yaw-rate 输出必须先经过 `nav2_cmd_adapter.py`。

## 时间源

实车默认全部使用系统时间：

```text
use_sim_time = false
```
