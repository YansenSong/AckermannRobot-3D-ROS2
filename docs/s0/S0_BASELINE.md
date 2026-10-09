# S0 本地基线（2026-10-09）

## 工作区与环境

- 仓库：`/home/young/Project/AckermannRobot`；分支：`main`；开始时 HEAD：`cbed9b81757d81940504140b62eab76ec24cb5c3`。
- 开始时 `git status --short` 只有两份未跟踪输入文档：`docs/交接文档.md`、`docs/s0完善.md`；`git diff --stat` 为空。不得覆盖这两份文档。
- 本机 `ROS_DISTRO=humble`，`ros2` 位于 `/opt/ros/humble/bin/ros2`；`colcon` 位于 `/usr/bin/colcon`；`gazebo`、`gz` 可用。Node `v24.18.0`、npm `11.16.0`、Python `3.10.12`。
- 根 README 使用 Humble；`third_party/RobotPilot/README.md` 的独立构建示例仍写 Jazzy，需按本机 Humble 验证后统一。`CONTEXT.md` 与 `docs/远端权限与平台接口设计.md` 在本次本地检出中不存在；不能按交接文档的旧路径直接编辑。

## 回归基线

| 命令 | 结果 | 解释 |
|---|---|---|
| `cd third_party/RobotPilot/web && npm test`（依赖安装前） | BLOCKED | `vitest: not found` |
| `cd third_party/RobotPilot/web && npm ci && npm test` | PASS | 14 个测试文件、39 个测试通过 |
| `python3 -m pytest -q third_party/RobotPilot/ros2/src/robotpilot_ui_package/test src/extension/mission_manager/test src/extension/area_rules/test src/control/vehicle_control/test/test_cmd_vel_mux_hold.py` | BLOCKED | 收集阶段缺少已构建 ROS 消息包 `robotpilot_ui_msgs`、`nav_status`；需构建并 source 工作区后重跑 |

以上没有证明 Gazebo 整链、物理底盘或长期稳定性通过。

## 本地差异与任务矩阵

| 范围 | 已有实现 | S0 缺口 | 优先阶段 |
|---|---|---|---|
| 任务 | `mission_manager`、任务 SQLite、`/mission/ack`、HTTP 幂等命令 | ROS 端命令重放去重、统一运行许可、离线事件补传 | W2～W4 |
| 地图/点位/区域 | `folders_handler`、`route_store`、区域规则 JSON、`/maps/catalog`、`/waypoints` | 旧 ROSBridge 写入、2D/3D 配对证明与真实完成 ACK | W2、W6 |
| 身份与控制 | `open` 仅本机、`local` 会话/CSRF/TLS、网关角色 | 正式模式仍允许部分浏览器直接发业务命令 | W2、W7 |
| 状态与电池 | RobotBridge 新鲜度、`BATTERY_SOURCE=sim/serial/disabled`、页面 stale 提示 | 机器级心跳、定位质量/充电/低电端侧许可 | W3 |
| 平台数据 | platform SQLite schema v1、commands/faults/audit/platform_events/waypoints/map catalog | 资产/证据元数据、可靠事件历史与 outbox 投影、版本化契约 | W1、W4、W5 |
| 前端事件 | `EventsPage` 使用浏览器 `localStorage`；后端有 SSE | 跨浏览器历史与补传事件统一展示 | W5 |
| 可复现性 | npm lock、ROS 包、启动脚本 | Humble/Jazzy 文档冲突、部署文档死链、仿真验收记录 | W7 |

## 数据/运行边界

- 平台库：`PLATFORM_SCHEMA_VERSION=1`，表包含 `commands`、`faults`、`config_versions`、`audit_entries`、`platform_events`、`waypoints`、`map_metadata`、`map_catalog_versions`。
- 机器人任务库：`MISSION_SCHEMA_VERSION=2`，表包含 `missions`、`runs`、`run_events`、`schedules`、`schedule_runs`。任务状态和执行历史的权威在机器人侧，平台不应另建执行器。
- `platform_events` 只保存平台观察到的状态变化；SSE 的 `Last-Event-ID` 只能补发平台已入库事件，不能覆盖 Flask 停机期间的机器人事件。
- 写入路径逐项见 [legacy-write-path-inventory.md](legacy-write-path-inventory.md)。本机有 Gazebo 可执行文件，但尚未运行 S0 仿真验收用例。
