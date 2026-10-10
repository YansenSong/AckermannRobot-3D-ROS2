# S1 API 与数据模型（当前实现）

所有业务 API 使用原有同源前缀 /api/v1/robots/{robot_id}，沿用 Platform API 会话鉴权、机器人隔离、角色权限、CSRF、请求 ID 和审计。Qt 客户端可复用这些 HTTP 路径与现有事件/SSE 接入；不应直接发布 ROS 控制 topic。分页目前使用 limit/offset，不承诺游标稳定性。

## 接口

| 方法/路径 | 角色 | 当前行为 |
|---|---|---|
| GET /assets?map_id=... | Viewer | 返回最多 200 个台账项、metadata、revision、关联 waypoint_ids。 |
| POST /assets | Engineer | 在可信当前 map_id 上创建资产。 |
| PATCH /assets/{asset_id} | Engineer | 通过 If-Match revision 更新名称/metadata/map 或停用/启用；以软停用代替删除。 |
| GET /assets/{asset_id}/waypoints | Viewer | 查询关联点位。 |
| PUT /assets/{asset_id}/waypoints/{waypoint_id} | Engineer | 创建关联；资产和点位 map_id/map_version_id 必须相同。重复 PUT 幂等。 |
| DELETE /assets/{asset_id}/waypoints/{waypoint_id} | Engineer | 解除关系，不删除资产或点位。 |
| GET /waypoints/{waypoint_id}/action-plan | Viewer | 查询草稿计划与修订号。 |
| PUT /waypoints/{waypoint_id}/action-plan | Engineer | If-Match 保存最多 20 个白名单动作；首次修订使用版本 0。响应始终 executable=false。 |
| GET /inspection/capabilities | Viewer | 返回 MissionManager 收到的 provider capability/heartbeat 摘要；未连接、过期时明确 online=false/stale=true。 |
| GET /inspection/tasks | Viewer | 从 MissionManager 当前运行和最近携带的巡检历史按 ID/名称、状态、since/until 筛选；机器人状态目前最多带 10 条历史。 |
| POST /inspection/tasks | Operator | body 提供 waypoint_ids 与可选 name；默认要求当前 map/bundle/readiness/provider source/capability 可用，读取当前点位、关联资产和动作草稿，编译确定性 mission steps 并保存/启动。`compile_only=true` 时只绑定当前 map/version 并保存不可变 MissionManager mission，不启动且允许 Provider 离线；机器人开始实际执行时重新校验 bundle、readiness 和 capability。必须有 Idempotency-Key。 |
| POST /schedules | Operator | 复用既有 MissionManager schedule，将已编译巡检 mission_id 接入日/周周期任务；调度页在 inspection profile 可用。 |
| GET /inspection/results?limit=&offset=&task_id=&waypoint_id=&asset_id=&outcome=&source= | Viewer | 查询并过滤持久结果；asset_id 匹配任一关联资产且每结果只返回一行；limit 1–200、offset 0–100000。 |
| GET /inspection/results/{result_id} | Viewer | 结果详情与安全证据 metadata。 |
| GET /inspection/evidence/{evidence_id} | Viewer | 只返回 metadata 和媒体状态；当前 media_url 为 null，不提供文件读取。 |
| GET /inspection/alerts?map_id=&map_version_id=&limit=&offset= | Viewer | 先按成对 map_id/map_version_id 过滤，再按 last_seen 分页；默认 limit=100、最大 200，响应含 next_offset。有可信 `map` frame 位置时才带 position，地图版本字段独立存储。 |
| GET /inspection/alerts/{alert_id} | Viewer | 返回单条告警和关联证据元数据；绝不返回媒体宿主路径。 |
| PATCH /inspection/alerts/{alert_id} | Operator | 使用 If-Match 推进受限告警状态机并记录操作人/备注事件。 |
| GET /maps/catalog/{group}/{map}/quality-reviews | Viewer | 查询绑定当前文件 checksum/map version 的质量审核记录。 |
| POST /maps/catalog/{group}/{map}/quality-reviews | Engineer | 提交规定 checklist 与缺陷备注。 |
| PATCH /maps/quality-reviews/{review_id} | Admin | 审核批准/拒绝并记录 reviewer。 |

所有写入继续由服务端校验角色和输入；地图、动作计划和资产更新使用版本检查。错误包括 400（格式/枚举错误）、403（角色不足）、404（资源不存在）、409（地图版本/状态冲突）、412（If-Match 过期）、503（机器人可信地图或依赖不可用）。实际平台错误包络以现有 API 为准。

## schema 8 数据

原表 assets、inspection_results、event_evidence、device_snapshots 保留。增量新增：

- assets.revision：对旧表非破坏性补列；metadata_json 用于有限扩展字段，启用状态以 metadata.active 保存。
- asset_waypoints：robot/asset/waypoint 关联与操作者、时间。
- waypoint_action_plans：robot/waypoint、revision、白名单动作 JSON；仅草稿，不代表 provider 已支持。
- inspection_alerts：业务告警生命周期、fingerprint、最近/首次出现、计数、备注和修订号；不复用 Fault ACK/clear。
- inspection_result_assets：结果与全部资产的多对多索引；旧 `inspection_results.asset_id` 保留并在升级时回填关联。
- inspection_alerts 新列：map_id、map_version_id、waypoint_id、fingerprint、episode；schema 7 的旧告警 ID 和处理状态保留。旧告警过去若已错误合并不同点位，无法从单行中自动拆开历史计数。
- map_quality_reviews：地图组/名称、当前 checksum/version、checklist、缺陷、状态及审核人。

inspection.result 先由机器人 MissionStore 写入耐久 Outbox，再由既有平台事件摄入事务校验关联并落结果。重复 event_id/result/action 关联不会生成重复业务行。结果保留 status 技术执行态和 outcome 业务结果；INCONCLUSIVE 不映射为 NORMAL。异常类别来自 provider 输入，平台不做识别推断。

业务告警按类别、地图版本和目标对象归并：有资产时使用排序去重后的整组 asset_ids；无资产时使用 waypoint_id；两者均缺失时以 result_id 单独成案。RESOLVED 或 CLOSED 后再次发生会创建新 episode 和 OPEN 告警，旧 episode 保留供追溯。多资产当前按资产组生成一条告警；业务方仍需确认是否改为逐资产告警。

证据只接受有限 metadata：evidence ID、MIME、checksum、大小等白名单字段；未知字段、绝对路径、媒体 URL 和过大对象拒绝。当前数据库行的 available 不代表文件可读；由于没有实际受控媒体存储，本 API 始终不给客户端媒体 URL，页面显示“媒体未接入”。

## ROS2 任务/Provider 边界

mission_manager 是唯一任务执行和步骤推进者，提供 /inspection/action/request、/status、/result、/control；inspection_adapter 代理 Provider ROS topics 并验证消息。Provider 的 capability 与 heartbeat 被 MissionManager 校验并发布在任务状态中；平台通过 GET /inspection/capabilities 读取。POST /inspection/tasks 在下发前重新校验 source mode、capability、机器人 readiness、地图 bundle、点位/资产版本，并将当前计划编译为任务快照。动作 ID 由 robot/task/step/attempt 稳定派生，迟到回执只记 durable late event，不推进已取消/终止任务。fixture 显示 source_mode=fixture。当前支持指定一个或多个点位立即执行；常规模板页面和周期巡检模板联动仍缺。

Qt/其他客户端通过同源登录会话调用 REST；任务命令仍经原有受审计平台 API。断开 /events SSE 后重新查询任务、结果和告警历史，不把 SSE 当最终数据存储。
