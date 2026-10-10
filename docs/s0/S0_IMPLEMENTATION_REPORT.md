# S0 实施记录（2026-10-09）

## 环境与版本

- 第一批实施开始时为 `main@cbed9b81757d81940504140b62eab76ec24cb5c3`。本轮收尾以已提交的 `main@34817f1dd22194680c6a363c53b3c76383eda577` 为基线；没有覆盖已有任务管理器或平台数据模型。
- Ubuntu / ROS Humble、Python 3.10、Node 24、npm 11；目标 ROS 包使用 `/tmp/alpha-s0-colcon-*` 独立目录构建，避开本地根 `build/` 的旧目录冲突。
- 第一批记录时未推送；本轮未执行推送。2026-10-10 已完成一条 Gazebo 近距离导航任务的运行态验证，完整场景矩阵与实机验收仍未完成。

## 本次实际完成

| 阶段 | 实施与证据 | 状态 |
|---|---|
| W0 | [本地基线](S0_BASELINE.md)、[旧写入盘点](legacy-write-path-inventory.md) | 已完成静态盘点 |
| W1 | [契约目录](contracts/api-v1.md)、事件/身份/ROS/错误文档；机器人库 v3、平台库 v3 迁移及资产/证据/设备快照元数据表 | 部分完成：元数据生产者、外键与全 ROS 图未完成 |
| W2 | `MissionStore.command_results` 持久 request_id 声明、重复 ACK 回放、变更内容冲突拒绝；任务编辑器改走平台 API，`local` 网关拒绝旧任务及本轮发现的直接运动/定位/外设写入 | 部分完成：地图/路线/区域等浏览器旧写路径仍在；被封的旧按钮尚需受控 API 替代 |
| W3 | 机器人端低电、充电、停车、定位、地图、区域、严重诊断门禁；2 秒机器人心跳、平台 stale 字段与状态页连接提示 | 部分完成：定位质量、仿真时间/TF 验证与其他页面连接展示未全覆盖 |
| W4 | 机器人 SQLite outbox，任务状态与事件同事务；诊断 raise/update/resolve；平台幂等入库后 ACK；双 SQLite 重启/ACK 丢失单测；10,000 条待发送水位告警与启动门禁 | 部分完成：真实 Flask/ROS2 离线、指数退避与磁盘满运行验收未做 |
| W5 | 平台机器人事件历史筛选/分页 API、Engineer 审计查询与 EventsPage 历史 + SSE 刷新 | 部分完成：平台本地事件与机器人历史未合并 |
| W6 | 保持模拟来源字段、端侧低电/充电测试；增加 2D/3D 清单、运行监测、任务启动/恢复的端侧 fail-closed 门禁及[冷切换步骤](map-bundle.md) | 部分完成：2026-10-10 本机 PCD 配对与 1 m Gazebo 导航通过；完整几何对图、切图失败恢复仍待验证 |
| W7 | [架构边界](architecture.md)、[启动配置](../deployment/run-profiles.md)、[SQLite 升级回滚](../web-storage-migration.md)、[本地检查脚本](../../scripts/check_s0.sh)、README 入口 | 部分完成：local 远端部署与运行态安全复测未做 |

## S0-01～S0-15 关闭状态

| 条目 | 证据 | 判定与剩余工作 |
|---|---|---|
| S0-01 系统分层 | [architecture.md](architecture.md) | 部分：未按运行中进程核对端口/断网 |
| S0-02 ROS 接口 | [ros2-interfaces.md](contracts/ros2-interfaces.md)、[当前 ROS 图采样](ROS_GRAPH_2026-10-09.md)、下文 2026-10-10 Gazebo 实测 | 部分：关键节点/话题已实测，全图及 QoS/TF 抽样待补 |
| S0-03 桥接服务 | `platform_api.py` 的 RobotBridge、`rosbridge_gateway.py`、[旧写入盘点](legacy-write-path-inventory.md) | 未关闭：直接运动等入口已在 `local` 网关封禁，地图/路线/区域旧业务写入尚未收口 |
| S0-04 唯一 ID | `MissionStore`、`PlatformStore.robot_events/assets/inspection_results/event_evidence`、[objects.md](contracts/objects.md) | 部分：最小 ID 表已建，跨表引用约束/写入 API 未完成 |
| S0-05 统一消息 | [event-sync.md](contracts/event-sync.md)、[errors.md](contracts/errors.md) | 部分：事件 v1 已接入，其他 API 错误码尚不统一 |
| S0-06 ACK/幂等 | `store.py` 的 `reserve_command`、`node.py` 的 `on_command`，单测 | 部分：任务指令已加端侧去重；其他写命令未统一 |
| S0-07 心跳/新鲜度 | `node.py` 的 `publish_heartbeat`、`platform_api.py` 的 `robot_connection` | 部分：Web/Flask/ROS 分层断连试验未做 |
| S0-08 状态约束 | `node.py` 的 `motion_guard_block_reason`、`map_binding_block_reason`、门禁测试 | 部分：低电/充电/停车与 2D/3D 清单端侧检查；定位质量阈值未实现 |
| S0-09 坐标/时间 | 地图版本与文件配对检查、UTC 事件；Gazebo `/clock` 与 1 m 导航的位姿变化 | 未关闭：完整 TF/2D/3D 几何对图与多点位试验未做 |
| S0-10 持久化 | 机器人库 v3、平台库 v3 迁移/单测、资产/结果/证据/设备快照表 | 部分：实际采样/写入 API 与生产备份回滚未做 |
| S0-11 离线补传 | `robot_event_outbox`、`ingest_robot_events`、双库重启与 ACK 丢失去重单测 | 未关闭：缺真实 Flask 停机期间 N 条事件的 ROS 整链验收 |
| S0-12 鉴权安全 | 沿用现有 auth/gateway | 未关闭：远端 local + TLS/Origin/9090 暴露复测未做 |
| S0-13 日志追溯 | `GET /events/history`、`GET /audit`、EventsPage | 部分：统一错误码和平台本地事件并表未做 |
| S0-14 仿真联调 | `simulation` 事件字段、模拟电池门禁；隔离 HTTP/ROS/SQLite 与 2026-10-10 Gazebo/LIORF/Nav2/NeuPAN/API 实测 | 部分：近距离导航及任务生命周期通过，完整 SIM-01～SIM-10 矩阵未完成 |
| S0-15 可复现部署 | [check_s0.sh](../../scripts/check_s0.sh)、本报告 | 部分：目标包、bringup/单测可复跑，完整干净环境重建未做 |

## 第一批已执行验证（续做后的最新计数见下文）

| 检查 | 本机结果 |
|---|---|
| `npm test`（web） | PASS，14 文件、40 项 |
| `npm run build`（web） | PASS，Vite 生产构建 |
| 目标包 `colcon build --packages-up-to mission_manager robotpilot_ui_package` | PASS，4 包；setuptools 产生非致命旧选项警告 |
| Python `pytest`：platform API、mission_manager、area_rules、cmd_vel_mux | PASS，127 项；3 项按测试自身条件跳过 |
| `git diff --check` | PASS |
| Gazebo/SIM-01～SIM-10、完整 ROS 图、端到端断网 | 第一批时 BLOCKED：当时仅有旧 UI 进程；2026-10-10 新进展见下文 |

复跑：先 `cd third_party/RobotPilot/web && npm ci`，回仓库根目录执行 `bash scripts/check_s0.sh`。脚本只覆盖目标包、现有单测和构建，最后明确打印运行态 SKIPPED。

## 第一批结束时的部署风险与后续顺序

1. 优先完成旧 ROSBridge 地图/路线/区域/任务写入口的 Flask API、端侧 ACK、前端迁移与网关封禁；在此之前仅按 loopback 开发环境运行。
2. 补 2D/3D 地图 bundle 及定位版本确认，门禁覆盖切图；缺配对版本时拒绝自主任务。
3. 在运行中的 Gazebo 做 Flask 关闭/恢复、ACK 丢失、SSE 重连、低电/停车/地图错配、`/clock`/TF 试验；记录事件数、命令 ID 与 ROS 图。
4. 增加资产/巡检结果/证据元数据、事件保留和 outbox 容量/磁盘满策略；再做完整 local TLS 安全复测。

本次变更集中在 `src/extension/mission_manager`、RobotPilot 的 `platform_api.py`/`rosbridge_gateway.py`、Web 任务客户端与事件/状态页面、相关测试、`docs/s0/`、`docs/deployment/`、`scripts/check_s0.sh` 与两份 README。原有用户输入文档未编辑。

## 2026-10-09 S0 收尾续做：实际范围与结果

### 代码与构建

| 功能 | 源码 | 已验证范围 |
|---|---|---|
| 2D/3D 文件清单与运行态监测 | `src/extension/mission_manager/mission_manager/map_bundle.py`、`setup.py`、`package.xml` | YAML/图像/PCD 校验和、实际 `/map` ID、LIORF 全局点云话题、文件变更；单测、隔离缺图状态及 2026-10-10 Gazebo 配对通过 |
| 机器人侧任务地图门禁 | `src/extension/mission_manager/mission_manager/node.py` | 新任务及恢复要求新鲜、就绪、与路线目录一致的配对状态；运行中地图/安全门禁失效则停车并暂停；HTTP→ROS→ACK 的缺图拒绝及暂停单测 |
| 平台状态展示 | `third_party/RobotPilot/ros2/src/robotpilot_ui_package/robotpilot_ui_package/platform_api.py` | `/status` 输出 `map_bundle` 与自主运行阻断原因 |
| 导航启动与冷切换 | `src/bringup/launch/navigation.launch.py`、`scripts/nav_liorf_neupan.sh`、[map-bundle.md](map-bundle.md) | bringup 包构建通过；本机 PCD 补齐后 Gazebo 冷启动成功。地图间冷切换与失败恢复未验证 |
| 可复现检查 | `scripts/check_s0.sh` | 目标 ROS 包及 bringup 构建、Python/前端测试、前端生产构建、diff 检查 |
| 受保护模式运动入口收紧 | `third_party/RobotPilot/ros2/src/robotpilot_ui_package/robotpilot_ui_package/rosbridge_gateway.py` | `local` 网关拒绝浏览器直发 `/goal_pose`、对接触发、`/initialpose` 与 `/periphery_operation`；角色判定单测。现有这些旧按钮须后续迁移到受控 API 才能在 `local` 使用 |

本轮最终 `bash scripts/check_s0.sh`：目标 ROS 包 4 个与 `robot_bringup` 构建通过；Python **130 项通过、3 项按条件跳过**；前端 **40 项通过**；Vite 生产构建与 `git diff --check` 通过。新增运行中地图失效自动暂停单测针对性通过。ROS 构建有 setuptools 旧选项和 underlay 提示，不影响本次结果。上述自动化结果不代表 Gazebo 验收。

### 隔离运行态证据

为避免覆盖用户正在运行的 5050/3000 旧 UI，使用 `ROS_DOMAIN_ID=177`、`ROS_HOME=/tmp/alpha-s0-runtime/ros`、独立 SQLite 路径，运行新构建 overlay 的 `mission_manager`、`map_bundle_monitor` 与 Flask `portApp:=5051`。这是**真实进程间 ROS2 和 HTTP 通信试验**，没有 Gazebo、LIORF、map_server 或浏览器操作。

1. `ros2 node list` 看到 `/mission_manager`、`/map_bundle_monitor`、`/flask`、`/ui_platform_bridge`；`GET http://127.0.0.1:5051/api/v1/robots/robot-001/status` 返回在线机器人心跳、`map_bundle.ready=false`、原因是 `maps/mini/GlobalMap.pcd` 缺失，`autonomy_ready=false`。
2. `POST /missions` 保存一个 2 秒等待任务，机器人 ACK 后 HTTP 202；同一 `Idempotency-Key`、同一请求重发为 HTTP 200，`command_id` 仍是 `42643d3d-0255-4bde-a0ef-e1003d8f886e`。首次 `POST /tasks` 因停车状态输入缺失被端侧拒绝；相同键重发复用了原拒绝结果，任务历史仍为空。
3. 隔离域内注入已确认停车状态、正常模拟电池、位姿、地图目录后，使用新键 `s0-runtime-map-gate-001` 再发 `POST /tasks`；机器人 ACK 以 HTTP 409 拒绝，错误为 `2D/3D map bundle is unavailable or unverified`。没有启动任务。
4. 停止 Flask 后，在隔离域持续发布测试诊断 `s0_runtime_fault`，机器人 SQLite `robot_event_outbox` 出现 1 条 `fault.raised` 且未 ACK。重启 Flask 后，平台 `robot_events` 收到该条，机器人待传数降为 0，`GET /events/history?type=fault.raised` 可查询相同 `event_id`、`source_seq=1`。此试验验证了一次 Flask 停机/恢复的持久补传；没有测试网络断线、磁盘满或大量积压。
5. **当时** `bash scripts/nav_liorf_neupan.sh maps/mini` 返回 1，明确指出缺少 `GlobalMap.pcd`。用户于 2026-10-10 补入文件；此条只保留为缺图失败用例记录。
6. 将三个更新包构建到项目默认 `install/` 后，重启 5050/3000 的 `run_robotpilot.sh`；`GET /status` 返回 `map_bundle.ready=false` 与对应阻断原因，Vite 首页 HTTP 200。该常用入口当前只有 UI 节点，地图和任务节点未随它启动，因此不能把它视作全栈导航联调。隔离域进程已停止，5050/3000 常用 UI 继续运行。

复现**当时**的隔离运行：先执行 `bash scripts/check_s0.sh`；各终端依次 `source /opt/ros/humble/setup.bash; source install/setup.bash; source /tmp/alpha-s0-check/install/local_setup.bash; export ROS_DOMAIN_ID=177 ROS_HOME=/tmp/alpha-s0-runtime/ros ROBOTPILOT_MISSION_DB=/tmp/alpha-s0-runtime/mission.sqlite3 ROBOTPILOT_DATA_DIR=/tmp/alpha-s0-runtime/data ROBOT_MODE=simulation BATTERY_SOURCE=sim AUTH_MODE=open`，然后分别运行 `ros2 run mission_manager mission_manager`、`ros2 run mission_manager map_bundle_monitor --ros-args -p map_yaml:=$PWD/maps/mini/map.yaml -p globalmap_pcd:=$PWD/maps/mini/GlobalMap.pcd`、`ros2 run robotpilot_ui_package flask --ros-args -p portApp:=5051`。只在隔离的临时数据库使用上述测试命令。

### 尚未关闭的关键事项

- Gazebo + LIORF + Nav2 + 新版 UI 的 HTTP API 创建、执行、暂停、恢复、取消、状态同步与 1 m 导航已于 2026-10-10 实测；浏览器实际点击链、多个目标点、地图间冷切换及失败恢复仍未验证。
- 2D/3D 几何配对需要真实同源地图与已知点、TF 核对；清单只绑定文件校验和与实际 2D ID，不能自动证明两个坐标系物理一致。切图失败恢复、任务对 bundle ID 的持久绑定仍待补齐。
- 地图管理、路线编辑、区域规则、手动驾驶等旧 ROSBridge 写入口仍在；`local` 网关的直接导航/对接/定位/外设写入已被封禁，但对应前端按钮尚无替代 API。[旧写入盘点](legacy-write-path-inventory.md)列出了逐项迁移目标，不能认定统一 API/权限/审计已经收口。`open` 开发模式仍只能在 loopback 信任边界使用。
- 浏览器刷新/关闭时的活跃任务恢复、重复 ROS 命令冲突、超时/重连、网络物理断开、异常状态门禁以及事件大量积压仍需 Gazebo 或更完整的隔离集成试验；现有单测只覆盖部分失败分支。

## 2026-10-10：补入真实点云后的 Gazebo 运行态验证

用户在本机补入 `maps/mini/GlobalMap.pcd`（609,339 点，文件 9,749,614 字节；受仓库 `*.pcd` 忽略规则影响，不在 Git 跟踪中）。本轮先在独立 ROS 域 178 启动 `planning.launch.py`，由实际 map_server `/map` 和 route_store `/ackermann/routes/catalog` 取得 `map_id=map_version_id=0c254af3b862`；使用 `map_bundle_manifest` 生成本地 `maps/mini/map.bundle.json`，bundle ID 为 `531478c17e738b11c91c7381b861f0703b86bd9801798b8530d8f0390bd081ac`。点云 609,107 / 609,339 点落在 2D 地图边界内，这只是范围检查，不能证明完整几何配准。

随后以 `ROS_DOMAIN_ID=178`、`ROS_HOME=/tmp/alpha-s0-gazebo/ros`、`ROBOTPILOT_MISSION_DB=/tmp/alpha-s0-gazebo/mission.sqlite3`、`ROBOTPILOT_DATA_DIR=/tmp/alpha-s0-gazebo/data`、`ROBOT_MODE=simulation`、`BATTERY_SOURCE=sim` 启动；关键节点/话题见[运行图抽样](ROS_GRAPH_2026-10-10.md)：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup navigation_sim.launch.py \
  map:=$PWD/maps/mini/map.yaml map_pgm:=$PWD/maps/mini/map.pgm \
  globalmap_pcd:=$PWD/maps/mini/GlobalMap.pcd gazebo_gui:=false
# 另两个终端，继承相同的上述环境变量：
bash scripts/run_robotpilot.sh
bash scripts/run_neupan.sh
```

Gazebo 初始机器人位姿为 (0, 0, 0)。向 `/initialpose` 发布 `map` 坐标系下 (0, 0, yaw 0) 后，LIORF 日志记录 `GlobalMap.pcd` 下采样后 32,135 点及 `Initial pose ICP alignment succeeded`。`GET /status` 显示 `map_bundle.ready=true`、正确的 map/bundle/PCD 哈希、定位在线、区域控制就绪、心跳在线与 `autonomy_ready=true`；`/clock` 有仿真时间，`tf2_echo map rear_axle_link` 可取得变换。此为一次启动和近距离对图证据。

| 试验 | 实际结果 | 证据边界 |
|---|---|---|
| API 创建/保存地图绑定的 30 秒等待任务，启动→暂停→恢复→取消 | HTTP 202 均收到端侧 ACK；任务状态依次为 `running`、`paused`、`running`、`cancelled`；task ID `b796c1d1-cd10-4f25-a420-2a636f0b9a42` | 从 HTTP API 下发，尚未实际点击浏览器按钮 |
| 2 秒等待任务 | task ID `2c65048d-64ee-470d-9f22-496270a17752` 最终 `succeeded`，含 4 条任务事件 | 验证状态闭环；不代表导航运动 |
| 1 m 自主导航 | task ID `321e4ae9-5c16-4990-9cd4-741bdb417508`：Nav 状态 `MOVING`→`ARRIVED`，位姿约 x=0→0.743→1.002 m，任务 `succeeded`，记录 RUNNING/step completed/SUCCEEDED 事件 | 仅一条无障碍近距离目标；未覆盖多点路线、避障与重规划 |
| HTTP 幂等与冲突 | 重发相同 `Idempotency-Key` 返回 HTTP 200、同一 `command_id`/`task_id`；机器人 SQLite 对该导航 mission 的 run 数仍为 1。相同键改 mission 返回 HTTP 409 | 未注入 ROS ACK 丢失或网络分区 |
| 低电中断 | 30 秒等待任务运行后，通过仿真电池 API 设置 `low_battery`；机器人自动停车并 `paused`，原因为 `safety guard: battery is low...`。`reset` 后恢复命令获 ACK，随后取消 | 验证机器人侧运行中门禁；电池来源为模拟 |
| mission_manager 重启 | 30 秒等待任务 ID `626be00f-d993-45ac-8d6a-01b4d9ac8f18` 运行中重启节点；ROS launch 自动拉起，SQLite 中任务变为 `paused`、`hold_active=1`、保留约 27.6 秒，原因为 `mission manager restarted; resume required`；平台 API 恢复后返回 `running`，再取消为 `cancelled` | 验证节点重启时安全暂停与任务持久化；未验证浏览器实际刷新或 Flask 重启时的活跃导航 |

首次任务启动因 `ekf_filter_node: odometry/filtered topic status` 报 `No events recorded.` 被拒绝；同一时刻 `/odometry/filtered` 实测约 10 Hz。`mission_manager/node.py` 现仅在**该精确诊断**与新鲜的实际 `/odometry/filtered` 流相矛盾时按正常处理；流中断后仍保留故障门禁，并增加回归测试。该修正后上述任务通过。EKF 诊断自身为何持续误报仍需在 ROS 组件层进一步排查。

本轮 `bash scripts/check_s0.sh`：目标 ROS 包与 bringup 构建通过，Python **131 项通过、3 项按条件跳过**，前端 **40 项通过**、生产构建与 `git diff --check` 通过。仍不能关闭 S0：地图/路线/区域旧写入口未迁移完；浏览器实际点击链、Flask/ROSBridge 断线期间的活跃导航、地图间冷切换与失败恢复、多点与障碍场景、完整 SIM 矩阵尚未验收。`GlobalMap.pcd` 未进入 Git，另一台机器复现前需单独提供同一文件。
