# S1 本地基线与差异记录

## 基线冻结

- 操作目录：仓库根目录。
- 开始时 HEAD：fd998a40100fe51ab348dbaf7216dcc2e2a51dc2；与交接文档记载的公开 main 基线相同。
- 开始时工作区已有的用户文件：docs/S1交接文档.md（未跟踪）。原文件保留，复制到 docs/s1/CODEX_HANDOFF.md。
- S0 按用户要求冻结。复用任务权威、Platform API/权限、MissionStore Outbox、事件补传和 map bundle 校验；不改写 S0 实施报告或 map bundle 文档。

## 本地源码核查

开始前核实：platform_api.py 已有 assets、inspection_results、event_evidence、device_snapshots 预留表以及点位/任务/告警/配置 API；MissionManager 已是机器人任务执行权威并有 Outbox；Web 已有 React/Vite 页面、地图/点位/BMS/Health/Events、受保护任务 API。交接清单记录的是公开 main 缺口，不能直接当本地差异结论。

本轮新增/扩展路径：

- 后端：third_party/RobotPilot/ros2/src/robotpilot_ui_package/robotpilot_ui_package/platform_api.py 与 test/test_platform_api.py。
- MissionManager：src/extension/mission_manager/mission_manager/node.py、store.py 和对应 test。
- 新 ROS2 适配包：src/extension/inspection_adapter/。
- Web：third_party/RobotPilot/web/src/pages/{InspectionPage,AssetsPage,WaypointActionsPage,MapPage,MapsPage,registry}.jsx、components/MapQualityReviewPanel.jsx。
- S1 文档：docs/s1/ 下契约、数据模型、测试矩阵、实施报告、待联调清单。

详见 CODEX_HANDOFF.md 第 7 节所列的 S1-01～57 原始核对方向，以及 S1_IMPLEMENTATION_REPORT.md 中按当前本地代码重新评定的状态。

## 回归情况

初次直接运行 scripts/check_s0.sh 时，137 passed、3 skipped、1 failed；唯一失败是 smoke test 因默认 ROS_LOG_DIR 指向只读的 ~/.ros/log。设置 ROS_LOG_DIR=/tmp/alpha-s1-ros-log 后，wrapper 的构建阶段完成，但其重复 Python 阶段被中断；最终独立 Python、前端和 ROS 包测试结果记录于 S1_TEST_MATRIX.md。该日志目录权限问题不是源码失败；Gazebo 和实机步骤没有运行。
