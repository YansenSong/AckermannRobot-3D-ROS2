# S0 当前架构与部署边界

```text
React (localhost:3000)
  ├─ 同源 HTTP/SSE ──> Flask Platform API (127.0.0.1:5050)
  │                    ├─ PlatformStore SQLite：命令、故障、审计、事件投影、点位
  │                    └─ RobotBridge：ROS2 订阅、受控任务命令、事件 ACK
  └─ 开发/遗留 ROSBridge (127.0.0.1:9090) ──> ROS2 读写 topic

ROS2: mission_manager ──> MissionStore SQLite：任务、执行、命令去重、事件 outbox
      area_rules ──> 本地规则 JSON ──> cmd_vel_mux
      battery_monitor ──> /battery/state
      LIORF / Nav2 / NeuPAN / Gazebo ──> 定位、导航、底盘命令
```

浏览器关闭时任务在 `mission_manager` 继续；Flask 停止时机器人 outbox 保留事件，Flask 恢复后重新发送并在事务提交后确认。平台 SSE 的重连只覆盖已到达平台的事件。命令受理、机器人确认、任务终态分别查询，不能互相代替。

当前可信边界：`AUTH_MODE=open` 仅允许 loopback；`AUTH_MODE=local` 需要 TLS、身份会话、角色和 CSRF。ROSBridge 9090 与 DDS 必须留在可信本机/内网；旧浏览器业务直写仍待迁移，**当前不能按已完成远端安全部署使用**。手动速度最终由 `cmd_vel_mux` 的停车、hold、区域和超时机制仲裁；软件停车不是物理急停。

机器人心跳 `/robot/heartbeat` 由 `mission_manager` 每 2 秒发布，平台 6 秒后判 stale；定位 3 秒、电池端侧 5 秒、区域控制 3 秒、2D 地图版本 30 秒。心跳是机器人 ROS 节点可达性，不代表浏览器 WebSocket、Wi-Fi 或 5G 网络质量；未验证的网络类型为 `unknown`。运行许可由机器人端作最终判断。

仿真配置 `ROBOT_MODE=simulation`、`BATTERY_SOURCE=sim`；`ROBOT_MODE=hardware` 时，机器人端与平台提示层均拒绝用 `simulated` 电池状态授权自主运动。实机须替换 Gazebo、LiDAR/IMU、里程计、BMS 和底盘设备适配器，保持上层协议，缺失值报 unavailable。3D 点云与 2D 栅格的共同版本门禁尚未完成，不能因名字相同宣称成对地图已同步。
