# S1 联调待办与责任边界

此列表只记录尚未交付/验证的接口和功能，不代外部团队填写负责人、承诺日期或检测结论。模型与识别算法由业务感知团队负责；本仓库不实现或模拟算法。

## 业务感知/相机 Provider 团队待提供

1. 最终 ROS2 或其他内部传输协议及字段 schema；确认 capability、heartbeat、动作 request/status/result/control 的版本与升级策略。
2. 动作和 detector 枚举、真实 supported 能力、software/model/policy 版本字段；区分拍摄成功与业务结论。明确 NORMAL、ABNORMAL、INCONCLUSIVE、NOT_APPLICABLE 产生条件及由谁赋值。
3. provider_id/robot_id/source_mode 标识方式，能力和 heartbeat 频率、过期阈值、在线/离线判定。
4. 各动作的截止时间、失败码、重试边界、pause/resume/cancel 语义、重复请求与晚到回执处理。
5. confidence 是否提供及其语义；观测时间、坐标 frame/position 来源；asset 与 detector 分类标识。
6. 图像/热图/标注证据安全交付方式：证据 ID、MIME、大小、checksum、对象存储或 robot→platform 同步、访问授权、保留策略。当前平台不保存/提供媒体文件。
7. normal/abnormal/inconclusive、provider offline、timeout、rejected、duplicate、late 的脱敏样例，以及 provider 启动、仿真调用、实机联调说明。

负责人、接口交付时间和实测指标均未由外部团队确认。上述输入到位后，需用同一契约补 provider integration tests，不能把 fixture 记录升级/覆盖成真实数据。

## 本仓库后续工作

- 为已编译 MissionManager 巡检快照补模板 revision 编辑/管理和调度运行历史视图；当前任务检索仅覆盖 MissionManager 状态携带的最近 10 条运行历史，完整归档分页/导出待扩展。日/周 schedule 的现有接口和 Scheduler 页面已可选择编译后的巡检 mission。
- 通过安全控制面实现 operating/allowed area；迁移受影响的 ROSBridge 地图/区域业务写操作并验证端侧 ACK。遵循 S0 map bundle 冷切换约束。
- 基准照片与结果证据的受控媒体上传、持久存储、授权读取、大小/MIME/hash 检查、审计和 missing/corrupt 处置；不暴露宿主路径或任意 URL。
- 为 Cockpit 告警摘要和按有效 map_id/map_version/frame 过滤的地图 pin 补浏览器 DOM/地图视觉测试。
- 为编译器增加多点选择/模板编辑页面、动作必选/失败策略完整语义和周期任务工作流；当前已支持指定的一次性点位任务并在服务端重验 capability。
- 逐设备 observed_at/source/stale health 汇总及 BMS/传感器断流测试。

## 仿真和实机阻断

- 当前已通过独立 ROS_DOMAIN_ID 的 Manager + fixture Adapter + PlatformStore 单测级集成；这不是 Gazebo 行为验证，不含真实导航、相机或模型。
- 本轮没有 Gazebo 启动日志、完整前后端多进程演示录像/截图、真实 provider 回执或实机证据。
- map bundle/地图身份、software stop、battery/area control、位姿新鲜度、稳定停车均由现有/扩展机器人安全门禁处理；没有因展示目的放宽。
- 实机验证需要机器人、外部 provider、相机/BMS/充电硬件、现场地图和明确的操作窗口；条件未提供时保持 EXTERNAL_PENDING/BLOCKED。
