# 机器人事件补传协议（v1）

`mission_manager` 将任务状态变化和诊断状态变化写入其 SQLite `robot_event_outbox`。任务状态和 outbox 在同一事务中提交。`source_seq` 是单机器人库内自增序号，不使用 ROS 仿真时间；`event_id` 是全局 UUID。

事件 JSON 字段：`schema_version=1`、`event_id`、`robot_id`、`source`、`source_seq`、`type`、`occurred_at`、`recorded_at`、`simulation`、`correlation`、`severity`、`payload`。`correlation` 可包含 `task_id`、`mission_id`、`request_id`、`map_id`、`map_version_id`、`fault_code`。当前事件类型为 `task.status_changed`、`task.event`、`fault.raised`、`fault.updated`、`fault.resolved`。

1. 机器人每 5 秒从最早未确认项取至多 100 条，向 `/robot/events/outbox`（`std_msgs/String`）发布 `{schema_version:1, robot_id, events:[...]}`。Flask 下线时队列保留在机器人 SQLite；重启后重复发布。
2. `RobotBridge` 验证版本、机器人 ID、批内连续序号、时间、JSON 与单事件 64 KiB 限制；平台 `robot_events` 对 `event_id` 和 `(robot_id,source,source_seq)` 建唯一约束。提交平台 SQLite 后才发布 `/robot/events/ack` 的 `{schema_version:1, robot_id, through_seq}`。
3. 机器人收到匹配 ID 的确认，设置 `acked_at`；确认丢失时再次发送，平台幂等跳过并重新确认。历史查询 `GET /api/v1/robots/{robot_id}/events/history` 按平台游标倒序分页。

`/mission/state` 与平台 `/status.event_sync` 暴露待补传条数、最早待确认序号、最新序号、最大重试次数；待补传达到 10,000 条时 `capacity_warning=true`，机器人拒绝新启动/恢复，但继续保留并写入已有任务终态和故障事件，不静默丢弃。

限制：当前重试间隔固定 5 秒；没有指数退避、磁盘剩余空间检查或全链路 Gazebo 断网实测。已确认记录暂不清理，可用于排查平台库重建后的历史恢复。已用双 SQLite 实例重启单测覆盖离线写入与 ACK 丢失重投；S0-11 关闭前仍须补充真实 Flask 进程停机与 ROS2 传输试验。
