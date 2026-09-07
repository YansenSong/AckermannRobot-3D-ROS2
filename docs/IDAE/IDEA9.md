6. RAI Whoami：我觉得这个想法很“聪明”

这个模块非常有意思。

rai_whoami 会读取：

机器人文档
图片
URDF

然后生成机器人自身的：

Rules
Behaviors
Capabilities
Images
Vector DB

最后合成为 Agent 的 Embodiment System Prompt。

也就是说 Agent 不只是知道：

“你是机器人助手。”

它真正知道：

“我是怎样一台机器人？”

这非常适合 Chat2Drive。

你可以搞一个：

DriveWhoAmI

或者更严肃一点叫：

VehicleProfile

例如：

vehicle:
  platform: autoware_shuttle

dimensions:
  length: ...
  width: ...

capabilities:
  - named_navigation
  - multipoint_navigation
  - speed_limit
  - autonomous_stop
  - resume_navigation

operational_limits:
  max_agent_speed_mps: 5.0
  allowed_area: campus

sensors:
  - lidar
  - imu
  - gnss
  - camera

safety_rules:
  - emergency_stop_has_highest_priority
  - agent_cannot_disable_safety_modules

navigation:
  frame: map
  known_locations:
    - office
    - gate

以后不是把所有东西硬写进：

AGENTS.md

而是：

VehicleProfile
     ↓
Prompt generator
     ↓
Agent

不常用的大量车辆文档则进入 vector database：

用户：
“这辆车最高支持什么速度？”

Agent：
query vehicle docs

这个比把五十页 Autoware/车辆说明全塞 System Prompt 合理多了。