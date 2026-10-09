# RobotPilot SQLite 升级与回滚

当前机器人任务库 `PRAGMA user_version=3`，平台库 `PRAGMA user_version=3`。两者在启动时按版本迁移，拒绝读取比代码更新的数据库。机器人 v3 增加 `command_results`、`robot_event_outbox`、`diagnostic_fault_states`；平台 v2 增加 `robot_events`，v3 预留 `assets`、`inspection_results`、`event_evidence`、`device_snapshots`。旧表与行不主动删除。

升级前停止 Flask 和 `mission_manager`，分别复制 SQLite 主文件及存在的 `-wal`、`-shm` 文件，或使用 SQLite `.backup` 生成一致快照；同时保留地图目录和区域规则 JSON。将备份放在只供管理员读写的位置并记录代码提交与 `PRAGMA user_version`。停机后升级代码并启动服务，核对 `GET /api/v1/robots/{robot_id}/build` 中的平台版本、任务状态和机器人事件同步情况。

回滚旧代码时不能直接打开已升到 v3 的数据库。停止服务，先备份当前升级后数据，再恢复升级前的成套快照，随后运行旧代码。恢复快照会丢失升级期间的新事件/命令，需从保留的机器人 outbox 和升级后平台库单独导出审计，不能默默覆盖。尤其平台库重建后若机器人已确认旧 outbox，当前没有自动全量回放机制。

本次单测覆盖旧库迁移、未来版本拒绝、DDL 失败回滚和 outbox 重启；**未做生产数据量、断电或跨版本实机回滚演练**。
