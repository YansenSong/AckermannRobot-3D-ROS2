# S0 实施记录（2026-10-09）

## 环境与版本

- 本地 `main`，开始时 HEAD `cbed9b81757d81940504140b62eab76ec24cb5c3`；两份用户输入文档原本未跟踪，保持原样。
- Ubuntu / ROS Humble、Python 3.10、Node 24、npm 11；目标 ROS 包使用 `/tmp/alpha-s0-colcon-*` 独立目录构建，避开本地根 `build/` 的旧目录冲突。
- 未推送；未运行 Gazebo 整链或实机验收。

## 本次实际完成

| 阶段 | 实施与证据 | 状态 |
|---|---|
| W0 | [本地基线](S0_BASELINE.md)、[旧写入盘点](legacy-write-path-inventory.md) | 已完成静态盘点 |
| W1 | [契约目录](contracts/api-v1.md)、事件/身份/ROS/错误文档；机器人库 v3、平台库 v3 迁移及资产/证据/设备快照元数据表 | 部分完成：元数据生产者、外键与全 ROS 图未完成 |
| W2 | `MissionStore.command_results` 持久 request_id 声明、重复 ACK 回放、变更内容冲突拒绝；任务编辑器改走平台 API，`local` 网关拒绝旧任务直写 | 部分完成：地图/路线/区域等浏览器旧写路径仍在 |
| W3 | 机器人端低电、充电、停车、定位、地图、区域、严重诊断门禁；2 秒机器人心跳、平台 stale 字段与状态页连接提示 | 部分完成：定位质量、仿真时间/TF 验证与其他页面连接展示未全覆盖 |
| W4 | 机器人 SQLite outbox，任务状态与事件同事务；诊断 raise/update/resolve；平台幂等入库后 ACK；双 SQLite 重启/ACK 丢失单测；10,000 条待发送水位告警与启动门禁 | 部分完成：真实 Flask/ROS2 离线、指数退避与磁盘满运行验收未做 |
| W5 | 平台机器人事件历史筛选/分页 API、Engineer 审计查询与 EventsPage 历史 + SSE 刷新 | 部分完成：平台本地事件与机器人历史未合并 |
| W6 | 保持模拟来源字段，端侧低电/充电测试 | 未完成：2D/3D bundle、冷切换与 Gazebo 整链 |
| W7 | [架构边界](architecture.md)、[启动配置](../deployment/run-profiles.md)、[SQLite 升级回滚](../web-storage-migration.md)、[本地检查脚本](../../scripts/check_s0.sh)、README 入口 | 部分完成：local 远端部署与运行态安全复测未做 |

## S0-01～S0-15 关闭状态

| 条目 | 证据 | 判定与剩余工作 |
|---|---|---|
| S0-01 系统分层 | [architecture.md](architecture.md) | 部分：未按运行中进程核对端口/断网 |
| S0-02 ROS 接口 | [ros2-interfaces.md](contracts/ros2-interfaces.md)、[当前 ROS 图采样](ROS_GRAPH_2026-10-09.md) | 部分：UI 图已采样，Gazebo/任务节点运行图待核 |
| S0-03 桥接服务 | `platform_api.py` 的 RobotBridge、[旧写入盘点](legacy-write-path-inventory.md) | 未关闭：旧 ROSBridge 业务写入未收口 |
| S0-04 唯一 ID | `MissionStore`、`PlatformStore.robot_events/assets/inspection_results/event_evidence`、[objects.md](contracts/objects.md) | 部分：最小 ID 表已建，跨表引用约束/写入 API 未完成 |
| S0-05 统一消息 | [event-sync.md](contracts/event-sync.md)、[errors.md](contracts/errors.md) | 部分：事件 v1 已接入，其他 API 错误码尚不统一 |
| S0-06 ACK/幂等 | `store.py` 的 `reserve_command`、`node.py` 的 `on_command`，单测 | 部分：任务指令已加端侧去重；其他写命令未统一 |
| S0-07 心跳/新鲜度 | `node.py` 的 `publish_heartbeat`、`platform_api.py` 的 `robot_connection` | 部分：Web/Flask/ROS 分层断连试验未做 |
| S0-08 状态约束 | `node.py` 的 `motion_guard_block_reason`、门禁测试 | 部分：低电/充电/停车等端侧检查；3D 图和定位质量未实现 |
| S0-09 坐标/时间 | 地图版本检查及 UTC 事件 | 未关闭：Gazebo `/clock`、TF 和 2D/3D 对图试验未做 |
| S0-10 持久化 | 机器人库 v3、平台库 v3 迁移/单测、资产/结果/证据/设备快照表 | 部分：实际采样/写入 API 与生产备份回滚未做 |
| S0-11 离线补传 | `robot_event_outbox`、`ingest_robot_events`、双库重启与 ACK 丢失去重单测 | 未关闭：缺真实 Flask 停机期间 N 条事件的 ROS 整链验收 |
| S0-12 鉴权安全 | 沿用现有 auth/gateway | 未关闭：远端 local + TLS/Origin/9090 暴露复测未做 |
| S0-13 日志追溯 | `GET /events/history`、`GET /audit`、EventsPage | 部分：统一错误码和平台本地事件并表未做 |
| S0-14 仿真联调 | `simulation` 事件字段、模拟电池门禁测试 | 未关闭：Gazebo 场景矩阵未运行 |
| S0-15 可复现部署 | [check_s0.sh](../../scripts/check_s0.sh)、本报告 | 部分：目标包/单测可复跑，完整干净环境重建未做 |

## 已执行验证

| 检查 | 本机结果 |
|---|---|
| `npm test`（web） | PASS，14 文件、40 项 |
| `npm run build`（web） | PASS，Vite 生产构建 |
| 目标包 `colcon build --packages-up-to mission_manager robotpilot_ui_package` | PASS，4 包；setuptools 产生非致命旧选项警告 |
| Python `pytest`：platform API、mission_manager、area_rules、cmd_vel_mux | PASS，127 项；3 项按测试自身条件跳过 |
| `git diff --check` | PASS |
| Gazebo/SIM-01～SIM-10、完整 ROS 图、端到端断网 | BLOCKED：当前仅有旧 UI 进程，[ROS 图采样](ROS_GRAPH_2026-10-09.md)缺任务/Gazebo 节点；不能据单测声称通过 |

复跑：先 `cd third_party/RobotPilot/web && npm ci`，回仓库根目录执行 `bash scripts/check_s0.sh`。脚本只覆盖目标包、现有单测和构建，最后明确打印运行态 SKIPPED。

## 当前部署风险与下一批顺序

1. 优先完成旧 ROSBridge 地图/路线/区域/任务写入口的 Flask API、端侧 ACK、前端迁移与网关封禁；在此之前仅按 loopback 开发环境运行。
2. 补 2D/3D 地图 bundle 及定位版本确认，门禁覆盖切图；缺配对版本时拒绝自主任务。
3. 在运行中的 Gazebo 做 Flask 关闭/恢复、ACK 丢失、SSE 重连、低电/停车/地图错配、`/clock`/TF 试验；记录事件数、命令 ID 与 ROS 图。
4. 增加资产/巡检结果/证据元数据、事件保留和 outbox 容量/磁盘满策略；再做完整 local TLS 安全复测。

本次变更集中在 `src/extension/mission_manager`、RobotPilot 的 `platform_api.py`/`rosbridge_gateway.py`、Web 任务客户端与事件/状态页面、相关测试、`docs/s0/`、`docs/deployment/`、`scripts/check_s0.sh` 与两份 README。原有用户输入文档未编辑。
