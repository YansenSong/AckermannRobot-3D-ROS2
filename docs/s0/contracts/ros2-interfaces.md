# S0 ROS2 接口（静态源码清单）

此表来自当前源码，**尚未用运行中的 Gazebo ROS 图核对**；现有 UI 进程的局部图见[采样记录](../ROS_GRAPH_2026-10-09.md)。普通订阅/发布均为 `reliable`、`volatile`、深度 10，除非另列。新鲜度使用单调时间，ROS `/clock` 不用于网络超时。

| 名称 | 类型 | 发布者 → 订阅者 | QoS / 时限 |
|---|---|---|---|
| `/mission/command` | `std_msgs/String` JSON | Platform RobotBridge → mission_manager | 默认；ACK 等待 3 秒 |
| `/mission/ack` | `std_msgs/String` JSON | mission_manager → RobotBridge | 默认；`request_id` 关联 |
| `/mission/state` | `std_msgs/String` JSON | mission_manager → RobotBridge / Web | reliable、transient local、深度 1；平台 6 秒 stale |
| `/robot/heartbeat` | `std_msgs/String` JSON | mission_manager → RobotBridge | 2 秒；6 秒 stale；含墙上 UTC、`heartbeat_seq`、来源 |
| `/robot/events/outbox` | `std_msgs/String` JSON | mission_manager → RobotBridge | 5 秒重发，至多 100 条 |
| `/robot/events/ack` | `std_msgs/String` JSON | RobotBridge → mission_manager | 平台事务提交后确认 |
| `/battery/state` | `std_msgs/String` JSON | battery_monitor → RobotBridge / mission_manager / Web | 平台 10 秒、端侧运行许可 5 秒 stale；仿真为 sim，硬件为 serial/disabled |
| `/safety/software_stop/state` | `std_msgs/String` JSON | cmd_vel_mux → RobotBridge / mission_manager | 3 秒 stale；`durable` 且 `confirmed` 才许可 |
| `/area_rules/control` | `std_msgs/String` JSON | area_rules → cmd_vel_mux / mission_manager | `ready`、`stop`；收到后 3 秒 stale |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | ROS 诊断生产者 → RobotBridge / mission_manager | 3 秒去抖；严重级别阻止任务启动/恢复 |
| `/liorf_localization/mapping/odometry` | `nav_msgs/Odometry` | LIORF → RobotBridge / mission_manager | `map` 位姿；3 秒 stale |
| `/ackermann/routes/catalog` | `std_msgs/String` JSON | route_store → RobotBridge / mission_manager | 2D 地图版本；30 秒 stale |
| `/navigation/state` | `nav_status/NavigationStatus` | 导航栈 → RobotBridge / mission_manager | 平台 3 秒 stale |
| `/goal_pose` | `geometry_msgs/PoseStamped` | mission_manager → 导航栈 | frame 必须是 `map`；任务超时另计 |
| `/mission/hold` | `std_msgs/Bool` | mission_manager → cmd_vel_mux | 机器人侧停止仲裁 |

完整系统还包含 Nav2 Action、`/tf`、`/tf_static`、`/clock`、LiDAR/IMU、地图与路线管理 topic/service；需在 Gazebo 启动后用 `ros2 topic list -t`、`ros2 service list -t`、`ros2 action list -t`、`ros2 topic info -v` 和 `tf2_tools` 记录实测图，当前不能声称接口清单已验收完成。
