# 平台 API v1（当前已接入部分）

前缀：`/api/v1/robots/{robot_id}`。`AUTH_MODE=open` 只允许本机 loopback；`AUTH_MODE=local` 使用会话、角色和写请求 CSRF。机器人 ID 必须与部署配置匹配。平台受理指令与机器人 ACK、任务终态是三个不同阶段。

| API | 权限 | 当前返回与语义 |
|---|---|---|
| `GET /status` | Viewer | 实时快照；`robot_connection` 独立标出心跳及 `stale`，位姿/电池/导航各自有 stale；`map_bundle` 提供 2D/3D 配对的 `ready`、`stale`、版本及失败原因；`autonomy_ready` 为平台提示，端侧最终判定 |
| `GET /health` | Viewer | 系统健康与存储观察值 |
| `GET /events/history?limit=&before=&type=&severity=&task_id=&request_id=&from=&to=` | Viewer | 机器人出站事件平台投影；`events` 与 `next_before`，按平台 cursor 倒序 |
| `GET /events` | Viewer | 原平台 SSE，包含 `robot_event` 通知；`Last-Event-ID` 只恢复平台已入库通知 |
| `GET /audit?limit=&before=&actor=&request_id=&method=&from=&to=` | Engineer | 平台审计条目；按机器人隔离、倒序分页，受配置的保留天数限制 |
| `POST /tasks`、`POST /tasks/{task_id}/commands` | Operator | `Idempotency-Key` 去重；返回 `command_id`、`status`、`request_id`，202 仅代表受理/待确认 |
| `GET /commands/{command_id}` | Viewer | 查询已记录的命令状态；不能把 `accepted` 当成任务成功 |
| `GET/POST/PATCH/DELETE /waypoints` | Viewer / Engineer | 地图版本绑定；更新使用 `If-Match` 保护 revision |
| `GET /maps/catalog`、`GET /maps/catalog/{group}/{map}` | Viewer | 从磁盘地图目录生成清单与版本信息 |

兼容期：浏览器任务编辑器已走任务 API，`local` 网关拒绝旧任务写发布、直接导航/对接目标、定位重置与任意外设操作；这些旧按钮在受保护模式暂不可用，直至受控 API 接入。地图、路线、区域等仍有 ROSBridge 旧写入，见 [迁移盘点](../legacy-write-path-inventory.md)。因此统一安全写入口尚未完成。原始 9090/DDS 只能部署在可信本机网络边界内。
