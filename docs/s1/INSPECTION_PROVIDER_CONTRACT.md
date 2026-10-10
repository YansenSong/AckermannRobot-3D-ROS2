# InspectionProvider v1（平台接入契约；真实 Provider 尚待确认）

此契约规定适配器边界，不表示外部识别算法、相机 SDK 或真实 provider 已交付。识别结论、类别和置信度只能来自 provider。平台/fixture 不运行模型、不推断 NORMAL，也不产生真实图像。

## ROS2 通道

ROS 消息当前使用 std_msgs/String，载荷为 JSON。robot → adapter 的请求/控制与 adapter → mission 的状态/结果均在本地 ROS graph；provider adapter side 通道如下：

| Topic | 方向 | 用途 |
|---|---|---|
| /inspection/provider/capabilities | provider → adapter | JSON v1 能力清单；fixture 模式由 adapter 自己发布 fixture 标记能力。 |
| /inspection/provider/heartbeat | provider → adapter | JSON v1 新鲜心跳，robot_id/provider_id 必须与能力清单一致。 |
| /inspection/action/request | MissionManager → adapter | 唯一动作请求。 |
| /inspection/action/status | adapter → MissionManager | 接受、执行、暂停/取消阶段状态。 |
| /inspection/action/result | adapter → MissionManager | 技术状态和业务结果分开的最终回执。 |
| /inspection/action/control | MissionManager → adapter | cancel/pause/resume。 |
| /inspection/provider/action/request | adapter → provider | 外部动作请求。 |
| /inspection/provider/action/status | provider → adapter | 外部状态。 |
| /inspection/provider/action/result | provider → adapter | 外部最终结果。 |
| /inspection/provider/action/control | adapter → provider | 经能力确认后转发的控制。 |

## schema 字段和语义

- 通用：schema_version=1、robot_id、带时区 RFC3339 UTC 时间。
- capability：provider_id、software_version、source_mode、online、actions[]；每个 action 报 kind、supported、可选 detectors、can_pause、can_cancel。capability Topic 使用可靠、TRANSIENT_LOCAL QoS，订阅方可在重启后取到最近一次能力；心跳须持续更新，超过 5 秒不派给 provider。
- request：action_run_id、mission_id、task_id、step_id、attempt、map_id/map_version_id、waypoint_id、asset_ids、kind(capture/detect/broadcast)、detector_types、parameters、requested_at、timeout_ms、source_mode。
- result：result_id、同一组请求关联 ID、status(SUCCEEDED/FAILED/UNAVAILABLE/TIMEOUT)、outcome(NORMAL/ABNORMAL/INCONCLUSIVE/NOT_APPLICABLE)、observed_at、source_mode、detector_type、model_version、confidence(有限 [0,1] 或 null)、evidence metadata。
- 只有技术 status=SUCCEEDED 才允许报告业务 outcome；失败、不可用、超时使用 NOT_APPLICABLE，不能制造正常结论。INCONCLUSIVE 是已执行但业务不可确认。
- action_run_id 对同一请求重投必须一致，包含 requested_at 在内的原始请求内容也必须完全不变。MissionManager 与适配器分别在 SQLite 缓存请求/result；相同 ID 不同请求内容拒绝。暂停期间的有效回执先持久化，恢复时最多推进一次；取消后回执和旧 attempt 回执记为 late 事件，不推进任务。
- evidence 只允许 evidence_id、media_type、checksum、size_bytes 等小型 metadata；禁止路径、任意 URL、图像字节和未知字段。当前平台没有受控媒体存储，Web 应显示媒体不可用。
- source_mode 使用 fixture/simulation/hardware。fixture 场景可测试 normal、abnormal、inconclusive、rejected、offline、timeout、late_result、duplicate；默认运行模式 external，fixture 必须显式设置 INSPECTION_PROVIDER_MODE=fixture。fixture 结论的 source_mode 为 fixture，confidence 为 null，证据为空。

## 安全与执行约束

MissionManager 只在导航 ARRIVED、定位与 filtered odom 新鲜、速度连续 0.5 秒不超过 0.05 m/s，且 map identity/bundle 条件成立后发 inspection action。任务开始时和到点派发时都会检查已锁存的 capability、实时心跳、source_mode、动作及 detector 支持；浏览器不推进动作。waypoint action plan API 返回 executable=false；Operator 通过 POST /inspection/tasks 才能将选定点位、资产、地图版本和动作草稿编译为任务快照；已编译 mission 可复用现有周期调度 API。

fixture 为可控测试替身，不是识别模型或机器人拍摄。provider 不在线/能力不符时适配器发 UNAVAILABLE（当 request source mode 已知），未知 source mode 时不伪造结果。状态超时由 adapter 返回 TIMEOUT。暂停/取消只有 capability 声明支持时才转发；不支持时 mission 控制只约束之后的步骤。

适配器重启后从 SQLite 恢复未结束动作的关联 ID 与原始期限，等待原 Provider 回执；同 ID 重投不会再次向 Provider 发起可能有副作用的动作。若 Provider 失联且没有可核实回执，到期返回 TIMEOUT，不标记成功。外部 Provider 对自身重启后的动作恢复、取消确认和最终回执重放语义仍待联调。

## 待外部团队确认并提供

需给出最终 schema/类别枚举、动作能力、software/model/policy 版本字段、心跳周期、result deadline、失败码、重试和取消语义、相机/云台标识、位置/坐标来源、证据上传或对象存储读取协议、安全权限、样例及仿真/实机启动说明。负责人和交付日期未获确认；详见 INTEGRATION_PENDING.md。
