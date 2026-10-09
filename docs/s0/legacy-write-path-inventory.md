# 浏览器旧写入路径盘点（W0）

基线：`main@cbed9b8`。`AUTH_MODE=open` 仅供 loopback 开发；`AUTH_MODE=local` 的业务写入目标是受鉴权、校验、幂等、审计的 Flask API。下表中的“迁移”是待实施状态，不代表已经安全收口。

| 来源 | 当前写入/请求 | 类别 | 当前反馈 | `local` 目标 |
|---|---|---|---|---|
| `components/MissionClient.jsx`、`shared/missions/*` | **已迁移**：任务写走 `/missions`、`/tasks`、`/commands`；状态仍订阅 `/mission/state` | 任务写 | HTTP 命令结果与 ROS 状态 | `local` 网关现拒绝 `/mission/command` 写发布，仅允许遗留 `query`；仍需 Gazebo 整链确认 |
| `pages/MapPage.jsx` | `/initialpose` | 定位重置 | 没有可关联完成 ACK | 增加受控定位命令及确认，或正式模式禁用，不能把 publish 当完成 |
| `pages/MapPage.jsx`、`components/StatusBar.jsx` | Nav2 cancel service | 安全方向取消 | service response | 由任务命令 `cancel` 承接并查询端侧 ACK；保留急停/取消有效路径 |
| `pages/MapPage.jsx`、`pages/MapsPage.jsx` | `/ui_operation` 地图/路线操作 | 地图写、路线写 | 文本 `/ui_message`、目录刷新；非结构化 ACK | 白名单 REST 操作，保留 `folders_handler`/`route_store` 执行；结构化 `request_id` ACK |
| `pages/RoutePage.jsx` | `/ui_operation`、`/ackermann/routes/plan_request` | 路线编辑/规划写 | 路线目录/点位 topic；部分无完成 ACK | 白名单 REST 路线命令，返回关联状态 |
| `shared/hooks/useKeepoutZones.js` | `/area_rules/command` | 区域安全规则写 | `/area_rules/ack` 带 request_id | REST API → area_rules，保留机器人端版本校验和 ACK |
| `shared/hooks/useSoftwareStop.js` | `/safety/software_stop/request` | 软件停车及释放 | `/safety/software_stop/state` 带 request_id | 停车/释放分别授权，走受控 API；端侧持久锁存和释放保护不变 |
| `components/Joystick.jsx` | `/cmd_vel`（100 ms 循环） | 手动驾驶 | 无逐命令 ACK；依赖 mux 超时 | 正式远端模式禁用原始速度写；本机仿真保留并受 mux 仲裁 |
| `components/DockingControl.jsx` | `/goal_pose`、`/dock_trigger`、`/undock_robot` | 导航/对接写 | 状态 topic，无统一命令 ACK | 任务/对接 API；端侧状态与命令关联 |
| `components/ControlSwitcher.jsx` | 动态 `topicName` | 外设/配置写 | 无统一 ACK | 正式模式禁止任意 topic；显式白名单 API |
| `components/LifecycleStatus.jsx` | 节点 `/change_state` service | 生命周期写 | service response | 限工程角色的白名单管理 API；页面目前未挂载，不作为默认控制入口 |
| `components/Map.jsx`、`pages/MapsPage.jsx`、`pages/RoutePage.jsx`、`shared/hooks/useSavedWaypoints.js` | `/map`、目录、点位请求 topic | 只读请求 | 对应响应 topic | 可暂保留只读请求，网关限制 topic 白名单 |
| `pages/MapPage.jsx` | 单点/路线任务经 `taskApi.js` | 已受控任务写 | HTTP command + ROS ACK/任务状态 | 保留，补机器人端去重、状态与事件追踪 |
| `shared/hooks/useSavedWaypoints.js` | `/waypoints` CRUD | 已受控点位写 | HTTP/`If-Match` | 保留版本校验与地图绑定 |
| `shared/schedules/schedules.js` | `/schedules` CRUD | 已受控计划写 | HTTP | 保留机器人任务管理器权威与审计 |
| `features/recordings/recordingsApi.js` | `/api/recordings` | 录制管理写 | HTTP | 检查身份/文件路径和录制生命周期；页面目前未挂载 |

待迁移时顺序是：后端安全 API 与真实 ACK → 前端切换 → `local` 网关封禁旧写入 → 回归 `open` 本机仿真。所有写操作须区分“受理”“机器人确认”“任务终态”。
