# 实车导航与控制话题

`real-vehicle-integration` 使用 Smac Hybrid-A* + NeuPAN 导航栈。

## Smac Hybrid-A* + NeuPAN

```text
/goal_pose
    -> smac_neupan_bridge
    -> Nav2 SmacPlannerHybrid
    -> /plan
    -> NeuPAN
    -> /neupan_cmd_vel_raw
       linear.x = speed
       angular.z = front-wheel steering angle
    -> motion_interface/command_gate
    -> /ackermann_cmd
    -> motion_interface/stm32_bridge
    -> UDP 5000 -> STM32 (192.168.5.50)
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
| `/ackermann_cmd` | `geometry_msgs/msg/Twist` | 统一实车命令；`angular.z` 是前轮转角 |
| `/stop` | `std_msgs/msg/Bool` | 集中停车覆盖；`true` 强制零指令 |
| `/navigation/state` | `nav_status/msg/NavigationStatus` | NeuPAN 栈导航状态 |

## `motion_interface`

`motion_interface` 包含三个独立节点：

```text
command_gate
  stack-specific Ackermann command + /stop
  -> /ackermann_cmd

stm32_bridge
  /ackermann_cmd
  -> hard limits
  -> STM32 UDP protocol (cmd__, 26 字节, -> 192.168.5.50:5000)

stm32_status
  <- sta__ 状态反馈 (56 字节, 192.168.5.50:5001 -> 192.168.5.11:5001)
  只校验、只解码、只统计，不发布任何话题、不发任何控制帧
```

两层都保留命令超时保护。

`stm32_status` 是纯被动节点，因此可以单独启动做台架验收：

```bash
./scripts/start_vehicle.sh status    # 只收状态，不下发任何控制
```

字段偏移、单位、有效位与枚举见 `docs/上位机接口说明_STA56.md`（权威，cmd__ 发送方向
与 sta__ 回传方向合并为一份），台架流程见同文档第 6 节。`stm32_status` 按该表做完整解码。

**帧长是 56 字节，不是旧的 48 字节**。两版 `Version` 都是 1，所以**长度是唯一能区分
二者的字段**——节点按 56 校验，收到 48 字节会以 `length` 拒收并提示这是被取代的修订。

两个容易误读的地方，读回传时务必注意：

- `EnableState = 0` 表示**未知**，不是"未使能"。必须结合 `FaultFlags` 位 13/14/15 区分。
  协议不提供 RT49/EPS 的真实使能反馈，所以 13/14 默认为 1。
- `FaultFlags` 位 11/19/20 是**口径说明**（线速度换算未标定、用了旧版 3000 RPM/(m/s)
  系数、用了直接 EPS 角度口径），**健康板子上就置位**，不能当作故障汇总读。

轻量网络栈无 ICMP，**不能用 ping 判断该链路通断**。

## `/ackermann_cmd` 接口约定

```text
linear.x  : longitudinal speed [m/s]
angular.z : front-wheel steering angle [rad]
```

`/ackermann_cmd.angular.z` 是前轮转角，不是 yaw rate。NeuPAN 的转角命令经 `command_gate` 发送给 STM32。

### 转角符号约定（已实车验证，勿改）

```text
angular.z > 0  =>  前轮左转（逆时针）
```

2026-09-17 实车验证：发布 `angular.z = +5°`，前轮**向左**转，转角幅度目测接近 5°。

整条链路**没有也不得有取反**：

```text
NeuPAN steering angle > 0
  -> command_gate           透传
  -> stm32_bridge           透传
  -> EPS 请求 = 转角(deg) × 100000
```

仍未确认：EPS 轴角与**车轮**转角的换算关系（上文的"目测接近 5°"只是视觉估计，
不是标定依据）；固件侧 EPS 的物理单位。这两项需与下位机工程师书面确认。

## 时间源

实车默认全部使用系统时间：

```text
use_sim_time = false
```
