# S0 对象与身份契约（v1）

时间字段使用带时区的 UTC RFC3339。`null` 表示没有可信观测值；`simulation: true` 或 `simulated: true` 表示仿真来源。ROS `/clock` 的时间只用于传感器/TF，不作为平台事件的墙上时间。

| 身份 | 含义与当前权威 | 关系 |
|---|---|---|
| `robot_id` | 单机器人部署 ID，平台配置与机器人 `ROBOT_ID` 一致 | 所有平台请求与机器人事件的作用域 |
| `map_id`、`map_version_id` | `route_store` 当前 2D 地图标识与版本 | 任务、点位绑定同一版本；3D 地图配对尚待实现 |
| `waypoint_id` | 平台 SQLite 点位 ID | `robot_id`、`map_id`、`map_version_id`、`revision` |
| `mission_id` | `MissionStore.missions.id`，任务定义 | 一个定义可产生多个执行 |
| `task_id` | `MissionStore.runs.task_id`，单次执行 | 关联任务事件与 `mission_id` |
| `command_id` | 平台 `commands` 表 ID | 作为发往机器人的 ROS `request_id` |
| `request_id` | HTTP 请求及机器人事件追踪 ID | 平台命令保留 `origin_request_id` |
| `event_id` | UUID，机器人侧事件永久标识 | 与 `robot_id`、`source_seq` 双重去重 |
| `fault_id`、`fault_code` | 平台故障 ID / 机器人诊断来源哈希 | 故障 raised/updated/resolved 均保留历史 |

`asset_id`、`inspection_result_id`、`evidence_id` 已在平台库 v3 预留最小元数据表 `assets`、`inspection_results`、`event_evidence`；另有 `device_snapshots`。证据字段为 `media_type`、`uri_or_path`、`checksum`、`available`，其中空路径/校验和表示尚无可信文件；不得生成虚构媒体。当前**没有写入生产者、增删改 API 或实际巡检结果**，也尚未建立全部跨表外键，因此该项未达到 S0-04/S0-10 的关闭标准。
