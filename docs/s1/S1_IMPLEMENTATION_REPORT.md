# S1 实施报告（W0～W9）

## 总体结论

本地基线为 `fd998a40100fe51ab348dbaf7216dcc2e2a51dc2`，与交接文档记录的公开 main 一致。S0 保持原样并复用；平台业务表由 SQLite schema 3 增量迁移至 8。W1、W3、W5、W6 的业务接入层有实际代码，inspection action 有 ROS2 fixture 闭环测试；但本轮**没有完成 S1-01～S1-57 的全部验收**。没有运行 Gazebo 或实机，也没有外部检测 provider，因此不报告 `SIM_VERIFIED` 或 `REAL_VERIFIED`。

目前可交付的边界是：资产/点位关系、Provider capability/heartbeat 状态、单次及多点巡检 mission 编译/下发、编译后接入既有周期调度、结果和业务告警入库/查询/处置、告警证据元数据安全展示、带来源标记的 Provider ROS2 接入与 fixture、MissionManager 到点稳定停车后派发动作、当前地图版本告警 pin，以及运行页状态卡片。待完成的主要部分是模板版本编辑/管理和调度运行历史呈现、地图运行区和旧写入口收口、真实媒体存储/授权读取、地图 pin 浏览器视觉验收、以及真实仿真和实机联调。

## W0～W9 阶段状态

| 阶段 | 实际实现与证据 | 当前边界 |
|---|---|---|
| W0 基线与差异 | 保留用户原始交接文件；记录本地 HEAD、基线差异和测试记录于 `S1_BASELINE.md`。 | 当前分支开始时只有用户提供的交接文件未跟踪；本轮修改均保留。 |
| W1 Provider 契约/替身 | 新增 `inspection_adapter`，含 capability/heartbeat/request/result/control ROS topic、schema 验证、显式 fixture 模式和 8 个场景；导航启动链现有显式 opt-in Adapter 参数，退出写错误日志并自动重启，Manager 心跳过期标记 stale；契约见 `INSPECTION_PROVIDER_CONTRACT.md`。点位动作页显示实时 Provider 来源、在线/过期和动作能力。 | fixture 默认关闭；外部团队协议及真实 provider 尚未交付；没有独立的 Provider 管理页。 |
| W2 地图/区域 | 复用现有地图清单、地图 bundle 校验与 `area_rules`；新增 map quality review 的提交、Admin 审核、版本/checksum 绑定。 | 未实现建图会话和运行区白名单安全执行；旧地图/区域 ROSBridge 写入口未整体迁移；二维/三维切换仍遵循 S0 冷切换约束。 |
| W3 点位/资产 | 新增资产创建、编辑/停用、地图版本关联、点位关联/解除关联 API 与 Web；动作计划可校验、版本化保存为不可执行草稿。 | 未实现基准照片上传/存储。 |
| W4 任务编排 | 指定任务 API/UI 支持单点立即执行；多点序列可编译为不可变 MissionManager mission 快照，保存时 Provider 可离线；inspection profile 复用现有 SchedulerPage 和 `/schedules` 建立周期调度。MissionManager 在实际开始和到点派发时重验地图、安全状态和 Provider 能力；任务页可按 ID/名称、状态和时间筛选当前及最近巡检历史。 | 模板版本化编辑/管理和调度运行历史呈现未补齐；完整历史归档查询待扩展；周期触发只验证 API 接入，未运行真实调度周期；真实 provider 能力联调待外部提供。 |
| W5 运行页 | MapPage 在巡检 profile 显示在线/离线或未知、状态过期、电量来源、当前地图/模式、任务摘要、当前 map/version 告警数；告警位置经当前 map/version 过滤后显示为可点击地图 pin；明确物理急停未知。 | 当前页是地图操作页聚合，不是独立大屏验收；无 Gazebo 联动截图或地图 pin 的浏览器视觉验收。 |
| W6 结果/告警 | Outbox 事件事务写入结果；异常按资产组或巡检点形成业务告警，关闭后新异常另建 episode；结果按所有关联资产筛选、告警先按地图版本过滤再分页；告警详情/证据元数据、状态机及操作审计均有 API；Web 显示 Fixture/Simulation 来源及媒体不可用态，地图 pin 可跳转对应告警详情。 | 无图像文件上传、持久化、服务端授权读取或下载；处理页面目前提供确认按钮，其他处理状态主要通过 API；没有独立 SSE 告警流。 |
| W7 设备/BMS | 复用现有 Info/BMS/Health、`/battery/state`、`/config` 和 Fault 机制；状态缺失显示 unavailable/unknown。 | 本轮未扩展设备粒度健康快照；未完成传感器断流集成测试或硬件联调。 |
| W8 联动与回归 | Python 平台、MissionManager、Adapter、AreaRules、bringup 启动配置、前端 DOM/map pin 单测和 Vite build 已执行；一个隔离 ROS_DOMAIN_ID fixture 集成测试实际贯通到 PlatformStore。 | 完整 `check_s0.sh` 本轮结果另见测试表；Gazebo、ROS graph 多进程演示、截图和真实 provider 均未执行。 |
| W9 文档/交接 | 本报告、API/数据模型、测试矩阵、Provider 契约和外部待办均更新；S0 文档未修改。 | 由于上述 BLOCKED/EXTERNAL_PENDING 项，不能作为全部 57 项验收完成声明。 |

## S1-01～S1-57 逐项状态

状态仅使用交接文档定义的 `CODE_READY`、`FIXTURE_VERIFIED`、`EXTERNAL_PENDING`、`BLOCKED`。同一项可同时标明已有代码和未完成的验收边界。`CODE_READY` 只说明对应代码/测试存在，不代表系统级或实机验收。无条目达到 `SIM_VERIFIED` 或 `REAL_VERIFIED`。

| ID / 功能 | 阶段 | 状态 | 真实实现、测试证据及缺口 |
|---|---|---|---|
| S1-01 机器人在线卡片 | W5 | CODE_READY | `web/src/pages/MapPage.jsx` 使用 `useRobotStatus` 显示在线/未知及 stale；后续 DOM 测试覆盖在线和 stale 提示，真实断线浏览器联测未做。 |
| S1-02 实时二维地图 | W5 | CODE_READY | 复用 `web/src/components/Map.jsx` 与现有地图订阅；仍受当前地图/bundle 状态约束。 |
| S1-03 实时位置/朝向 | W5 | CODE_READY | 复用 Map 的 ROS pose 展示和状态 stale；Gazebo 位姿联动未测。 |
| S1-04 轨迹/规划路线 | W5 | CODE_READY | 复用 Map 已有轨迹和路线绘制；未增加持久轨迹历史。 |
| S1-05 任务进度总览 | W5 | CODE_READY | `MapPage.jsx` 使用 `useMissionRun` 显示当前运行摘要；未完成逐点巡检结果总览页。 |
| S1-06 电量/充电 | W5 | CODE_READY | 使用 `/status` 电池状态和 source；无电量时显示未知；大屏充电状态细分仍有限。 |
| S1-07 告警概览/跳转 | W5 | CODE_READY | MapPage 显示当前 map/version 的未关闭业务告警数和最近分类，可跳结果页；仅按当前有效地图过滤。 |
| S1-08 故障/安全状态 | W5 | CODE_READY | 页面明确状态过期及物理急停未确认；传感器级安全详情仍在现有 Health 页面。 |
| S1-09 大屏布局/异常提示 | W5 | CODE_READY / BLOCKED | 巡检 profile 增加状态卡和提示；独立 cockpit 布局与 1440×900/1280×720 视觉验收未做。 |
| S1-10 建图任务管理 | W2 | BLOCKED | 未增加 Web 建图会话；现有 `mapping_mini.sh`/保存接口不代表受控会话工作流。 |
| S1-11 3D/2D 地图成果 | W2 | CODE_READY / BLOCKED | 复用 `map.bundle.json` 校验；本轮未建成果检索/浏览流程。 |
| S1-12 地图保存/加载/切换 | W2 | CODE_READY / BLOCKED | 复用 MapsPage 与 S0 bundle/冷切换约束；未做 Gazebo 冷切换验收。 |
| S1-13 运行区域 | W2 | BLOCKED | 没有新增 operating/allowed area 端侧规则；`area_rules` 的既有禁行/限速能力不等于允许区。 |
| S1-14 禁行区/虚拟墙 | W2 | CODE_READY / BLOCKED | 复用 `AreaRulesEditor`/`area_rules`；旧 Web 写路径迁 API 和发布 ACK 验收未完成。 |
| S1-15 限速区 | W2 | CODE_READY / BLOCKED | 复用既有 AreaRules 速度规则；本轮未测端侧边界与恢复联动。 |
| S1-16 充电区/目标点 | W2 | CODE_READY / BLOCKED | 复用 charge 点类型和既有 dock 任务；无充电区管理/真实 dock 联调。 |
| S1-17 区域配置发布校验 | W2 | BLOCKED | 旧地图/区域写入口尚未整体改为受保护 API 并核实 robot ACK。 |
| S1-18 地图版本/生效态 | W2 | CODE_READY / BLOCKED | 复用地图清单、checksum、bundle 检查；S1 全量版本生效界面及冷切换联测未完成。 |
| S1-19 地图质量记录 | W2 | CODE_READY | `MapQualityReviewPanel.jsx` + `/maps/catalog/.../quality-reviews`；API 测试版本/checksum 绑定与 Admin 审核。几何/可达性检查未自动化。 |
| S1-20 多楼宇/楼层 | W2 | CODE_READY / BLOCKED | 复用 map metadata 的 campus/building/floor；本轮未补层级检索/组织 UI。 |
| S1-21 地图点选建点 | W3 | CODE_READY | 复用 MapPage、`useSavedWaypoints` 和点位 API；已有前端/S0 回归。 |
| S1-22 点位查询/编辑/删除 | W3 | CODE_READY | 复用点位 CRUD、If-Match 和任务引用删除保护；业务批量迁移/影响提示未扩展。 |
| S1-23 点位动作配置 | W3 | CODE_READY / EXTERNAL_PENDING | WaypointActionsPage.jsx + 版本化草稿 API；单次任务编译时验证当前 provider 的动作/类别能力。真实类别与 provider 仍待外部联调。 |
| S1-24 到点确认 | W3/W4 | FIXTURE_VERIFIED | `mission_manager/node.py` 要求 ARRIVED、新鲜 pose/odom、速度≤0.05 m/s 稳定 0.5 秒后才派发；`test_manager_fixture_flow.py` 覆盖到点后动作闭环。 |
| S1-25 地图/版本绑定 | W3/W4 | CODE_READY / BLOCKED | 点位/资产及编译任务绑定 map_id/map_version_id；任务下发前校验当前 bundle。已执行 mission 快照尚未记录 bundle checksum，地图切换后的历史快照完整性仍待补齐。 |
| S1-26 资产台账 | W3 | CODE_READY | `AssetsPage.jsx`、资产 GET/POST/PATCH、revision/停用；API/页面源已构建。资产删除采用停用。 |
| S1-27 资产关联点位 | W3 | CODE_READY | `asset_waypoints`、列表/PUT/DELETE、Web 建立与解除关系；API 测试覆盖地图版本与关系持久化。 |
| S1-28 基准照片/参考状态 | W3 | BLOCKED / EXTERNAL_PENDING | 资产 metadata 可记录非媒体字段；未实现基准照片安全上传、受控媒体存储或外部基准状态接口。 |
| S1-29 常规巡检模板 | W4 | CODE_READY / BLOCKED | WaypointActionsPage 可按顺序选择多个点位并将点位、资产、动作计划编译为可复用 MissionManager mission 快照；Provider 可离线保存，实际执行端重新校验。模板独立版本编辑/管理及页面 DOM 验收未做。 |
| S1-30 周期巡检 | W4 | CODE_READY / BLOCKED | inspection profile 开放既有 SchedulerPage；已编译巡检 mission 可用 `/schedules` 加入机器人日/周调度。API 接入测试通过，但没有等待一次实际周期触发或运行 schedule+inspection fixture 闭环。 |
| S1-31 指定巡检任务 | W4 | CODE_READY | POST /inspection/tasks + WaypointActionsPage.jsx 从当前选中点位动作草稿编译并下发；API 测试覆盖快照/idempotency/capability gate。编译 API 尚未与 fixture 节点做同一条端到端测试，真实 provider 未接入。 |
| S1-32 到点动作串联 | W4 | FIXTURE_VERIFIED / EXTERNAL_PENDING | MissionManager ↔ Adapter ROS topic 与 fixture 闭环已跑；后续补丁验证 waypoint→wait→detect、waypoint→capture→detect、waypoint→wait→detect→wait→capture 同点串联及孤立 wait 后拒绝检测；外部 provider、能力拒绝发布及现场动作待联调。 |
| S1-33 任务下发/命令确认 | W4 | CODE_READY | 复用 Platform API、Idempotency-Key、ACK；本轮任务状态由现有 MissionManager 持有。 |
| S1-34 任务全过程状态 | W4 | CODE_READY / FIXTURE_VERIFIED | MissionManager 发布 active action 状态、Outbox 记结果；fixture 集成测试验证；Web 任务时间线未新增。 |
| S1-35 任务暂停 | W4 | CODE_READY / EXTERNAL_PENDING | 机器人命令和 adapter pause 能力按声明转发；后续单测验证暂停期间回执持久化、恢复后才推进；真实 provider pause 能力及交互测试未验证。 |
| S1-36 任务恢复 | W4 | CODE_READY / EXTERNAL_PENDING | MissionManager 将首发请求及回执持久化，恢复使用原 action_run_id 和完全相同 payload；单测覆盖管理器重启前后恢复。真实 provider 恢复语义未联调。 |
| S1-37 取消/安全退出 | W4 | CODE_READY / FIXTURE_VERIFIED | MissionManager cancel/control 与晚到结果隔离；后续单测覆盖取消后成功回执仅记 late、不推进；真实 provider cancel 确认待联调。 |
| S1-38 失败重试 | W4 | CODE_READY | 失败后 retry 增加 attempt 并得到新 action_run_id；后续单测验证。外部动作是否可安全重试仍待 provider 确认。 |
| S1-39 任务异常分支 | W4 | CODE_READY / FIXTURE_VERIFIED | adapter 有 rejected/offline/timeout/late_result fixture 场景；后续单测验证 adapter 重启恢复 pending、不重复派发，Provider 未返回时保持 TIMEOUT。全场景多进程集成尚未跑。 |
| S1-40 任务地图/点位关联 | W4 | CODE_READY | 巡检任务编译检查 waypoint、asset map_id/map_version_id 与当前 map bundle，并把 pose/asset/action 参数快照写入 mission steps；后续补丁保留 wait/action 的 waypoint_id，机器人执行请求从已完成到点上下文取点位，运行记录将最近成功到点索引持久化。 |
| S1-41 任务执行历史 | W4 | CODE_READY / FIXTURE_VERIFIED | 复用 robot task history 与 durable Outbox；fixture 结果摄入 PlatformStore；业务历史筛选 UI 不完整。 |
| S1-42 任务检索 | W4 | CODE_READY / BLOCKED | InspectionPage 支持按任务 ID/名称、状态、开始/结束时间过滤 MissionManager 当前运行及携带的最近 10 条巡检历史；完整机器人任务归档分页、导出和超过该窗口的任务检索未实现。 |
| S1-43 统一结果入库 | W6 | FIXTURE_VERIFIED / EXTERNAL_PENDING | `inspection.result` 经 MissionStore Outbox → PlatformStore 去重存储；集成测试重复摄入后仅一条。真实 Provider 待联调。 |
| S1-44 异常事件接收 | W6 | FIXTURE_VERIFIED / EXTERNAL_PENDING | Adapter fixture result topic→MissionManager→Outbox 已通过；真实业务异常来源未接入。 |
| S1-45 告警分类/等级 | W6 | CODE_READY / EXTERNAL_PENDING | 仅依据外部 detector/category 和结果建立业务告警；无模型判定；外部类别/严重度策略需确认。 |
| S1-46 告警详情/证据 | W6 | CODE_READY / BLOCKED / EXTERNAL_PENDING | GET 告警详情返回关联证据 metadata；`InspectionPage.jsx` 展示媒体不可用态，拒绝文件路径/URL；无实际媒体读写/图像展示。 |
| S1-47 地图告警定位 | W5/W6 | CODE_READY / BLOCKED | 告警 API 从关联结果解析 map frame 坐标，map_id/map_version_id 独立索引并在 SQL 中先过滤再分页；MapPage 仅渲染当前身份的可点击 pin 并跳到告警详情列表。新增 DOM 测试核对当前地图版本和 stale 清 pin，nav2d 测试核对 map frame 坐标/点击/清除；Gazebo 视觉验收未做。 |
| S1-48 无法确认状态 | W6 | FIXTURE_VERIFIED / EXTERNAL_PENDING | `INCONCLUSIVE` 与技术状态分离且无默认正常；集成测试验证 INCONCLUSIVE 入库，以及 fixture 事件 `simulation=true`、`is_test_data=true`。 |
| S1-49 告警去重/持续 | W6 | CODE_READY | 同分类/地图版本按资产组优先、无资产时按巡检点归并；RESOLVED/CLOSED 后再触发新 OPEN episode，旧案留存。多资产按组还是逐资产告警、持续窗口仍需业务方确认。 |
| S1-50 告警查询/留存 | W6 | CODE_READY | 结果带 limit/offset 与 task/point/任一关联 asset/outcome/source 过滤；告警 SQL 先按 map/version 过滤再分页并返回 next_offset；无导出和稳定游标接口。 |
| S1-51 告警处理状态 | W6 | CODE_READY | Operator If-Match 状态机与平台事件审计；Web 当前仅显示 OPEN→ACKNOWLEDGED 操作。 |
| S1-52 设备详情 | W7 | CODE_READY / BLOCKED | 复用 `InfoPage.jsx`、`/build`、`/status`；本轮未加逐传感器设备快照视图。 |
| S1-53 实时 SOC | W7 | CODE_READY | 复用 `/battery/state` 与 `BmsPage.jsx`；无 BMS 时沿用 unavailable。 |
| S1-54 BMS 关键状态 | W7 | CODE_READY / BLOCKED | 复用既有电池状态 source/simulation 字段；硬件字段缺失和断流专项联测未做。 |
| S1-55 低电阈值 | W7 | CODE_READY | 复用平台配置版本与低电阈值；未新增策略。 |
| S1-56 低电/电池故障 | W7 | CODE_READY | 复用 PlatformStore 的低电 Fault 观察和既有故障页；充电故障硬件验证缺失。 |
| S1-57 设备健康总览 | W7 | CODE_READY / BLOCKED | 复用 `HealthPage.jsx` 的诊断/新鲜度；逐设备 source/observed_at/stale 数据工作流未实现。 |

## 主要源码和数据迁移

- 平台 API/存储：`third_party/RobotPilot/ros2/src/robotpilot_ui_package/robotpilot_ui_package/platform_api.py`；schema 3→8 增量迁移，新增 `inspection_alerts`、`inspection_result_assets`、`waypoint_action_plans`、`asset_waypoints`、`map_quality_reviews`，并给旧 `assets` 表补 `revision`。不删除旧业务行。生产升级前备份 SQLite；旧二进制不支持向下打开 schema 8，回滚需恢复备份。
- 机器人编排：`src/extension/mission_manager/mission_manager/{node.py,store.py}`；action 请求/控制/回执，停车门禁、任务关联与 Outbox 事件。
- 外部接入：新增 `src/extension/inspection_adapter/` ROS2 包；fixture 需显式设置 `INSPECTION_PROVIDER_MODE=fixture`，默认 external。
- Web：`third_party/RobotPilot/web/src/pages/{InspectionPage,AssetsPage,WaypointActionsPage,MapPage,MapsPage,registry}.jsx` 与 `components/MapQualityReviewPanel.jsx`。
- 测试：平台 API、MissionManager/store、Adapter contract + ROS fixture flow、registry 回归；未新增新页面的 DOM 交互测试。

## 测试与验收限制

本轮实测命令及准确结果记录在 [S1_TEST_MATRIX.md](S1_TEST_MATRIX.md)。本报告不把单测通过等同 Gazebo/系统验收。未得到真实 Provider 的 schema、能力/心跳、证据存储方式、异常分类策略或模型版本；相应责任与所需接口列于 [INTEGRATION_PENDING.md](INTEGRATION_PENDING.md)。

## 2026-10-10 后续审计修复

### Patch A：F01、F04 动作编排和点位归属

- 失败复现：新增 `test_inspection_steps_keep_their_waypoint_assignment` 在修复前得到 `['wp-1', None, None, None]`，证明 wait/检测的点位字段在 mission 验证时丢失。
- 修复：compiler 给等待步骤写入所属巡检点；MissionManager 验证并保留业务步骤的 `waypoint_id`，按最近连续业务段中的成功到点索引判断启动资格。运行库 schema 3→4 增加 `arrival_step_index`，旧行默认 `-1`，未确认到点时保持拒绝；派发前仍检查新鲜位姿、停稳、地图和 Provider。
- 回归：MissionManager/store 与平台 API 共 `71 passed`（补入孤立等待拒绝用例后单项再跑 `1 passed`）。覆盖三种合法排列、错误点位绑定、旧库迁移及未到点拒绝。未运行 Gazebo，不提升为 `SIM_VERIFIED`。

### Patch B：F02、F08 动作生命周期与重启

- 失败复现：`test_paused_inspection_result_is_reconciled_and_request_is_immutable` 在修复前恢复后仍停留 RUNNING；`test_external_pending_request_survives_adapter_restart_without_redispatch` 在修复前找不到持久 pending。
- 修复：MissionStore schema 4→5 新增请求/回执持久表。同一 `action_run_id` 的 `requested_at` 和整份请求保持不可变；暂停期间有效回执先落库，恢复后对账并生成确定性去重的 Outbox 事件。适配器从 SQLite 恢复 pending 和原始超时，不重新调用 Provider。Manager 的能力订阅改为可靠 TRANSIENT_LOCAL QoS，在线判定使用锁存能力及新鲜心跳。
- 回归：MissionManager 与 Adapter 共 `44 passed`；连同平台 API 的 Python 回归为 `137 passed、3 skipped`，React `40 passed`，Vite 构建通过，MissionManager 与 Adapter 两包分别构建通过。覆盖 pause 前后回执、管理器重启、适配器重启、重复请求、取消后晚到、retry attempt 变化及 Provider 无回执超时。未运行真实 Provider 或多进程中断测试；外部 Provider 重启后的回执重放协议仍需联调，不提升为 `SIM_VERIFIED`。

### Patch C：F03、F06、F07 告警与结果查询正确性

- 失败复现：新增的不同点同类、第二资产筛选和其他地图超过 100 条后查询当前地图三个用例在修复前均失败。
- 修复：平台 SQLite schema 7→8；`inspection_result_assets` 存全部资产并回填旧结果；告警增加结构化地图/点位、对象 fingerprint 和 episode。当前规则为资产组优先、无资产按巡检点；RESOLVED/CLOSED 后新异常建新 OPEN episode。告警列表在 SQL 中先过滤地图，再按 limit/offset 分页；摄入事务用 `BEGIN IMMEDIATE` 串行化同对象并发写入。
- 回归：平台旧库升级、关闭后再触发、不同点、并发、资产筛选和跨地图超过 100 条均通过；Python 全量 `158 passed、3 skipped`，React `40 passed`，Vite 构建通过。旧 schema 7 已错误合并的历史计数无法自动拆分，仍保留原告警以免丢失处置记录；多资产告警粒度待产品确认。

### Patch D：F05、F09 启动与 fixture 审计标签

- 失败复现：启动配置测试在修复前找不到 Adapter 节点及 fixture 仿真门禁；fixture Outbox 测试在修复前得到 `simulation=false`。
- 修复：`navigation.launch.py` 增加默认关闭的 Adapter 开关和 external/fixture 参数；仿真入口显式传递 profile，fixture 同时要求 `ROBOT_MODE=simulation`；脚本增加 `--inspection-adapter=...` 包检查，bringup 声明 ROS 包运行依赖。Adapter 退出记录错误并自动重启，Provider 心跳过期由 Manager 标记 stale。fixture Outbox 顶层为 `simulation=true`、`is_test_data=true`。
- 回归：Python `161 passed、3 skipped`，React `43 passed`（含运行页 DOM、地图 pin），Vite 构建通过；`robot_bringup`、`mission_manager`、`inspection_adapter` 分别构建通过，`ros2 launch ... --show-args` 显示默认关闭及 external 默认模式。曾尝试以 fixture 启动 `navigation_sim.launch.py`：Manager 与 Adapter 进程启动，但 Gazebo `gzserver` 因 `getifaddres: Operation not permitted` 退出，随后导航 TF 不完整；20 秒超时终止余下进程。此环境下 Gazebo+Web 多进程系统验收未完成，R12 继续 BLOCKED，不提升为 `SIM_VERIFIED`。
