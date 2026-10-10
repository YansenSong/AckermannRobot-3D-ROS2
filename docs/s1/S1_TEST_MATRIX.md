# S1 测试矩阵与本轮实测

## 自动化检查

| 检查 | 复现命令/条件 | 本轮结果 | 边界 |
|---|---|---|---|
| Platform API + S0 Python 回归 | source /opt/ros/humble/setup.bash && source /tmp/alpha-s1-check-final/install/local_setup.bash；ROS_LOG_DIR=/tmp/alpha-s1-ros-log python3 -m pytest -q third_party/RobotPilot/ros2/src/robotpilot_ui_package/test src/extension/mission_manager/test src/extension/area_rules/test src/control/vehicle_control/test/test_cmd_vel_mux_hold.py | 143 passed、3 skipped | 最终全量源码回归包含本轮告警地图位置、任务筛选和 compile-only 模板调度接入测试。3 项 skip 分别是已有 docstring cleanup、style cleanup、generated source copyright 标记测试。 |
| 新增平台检索/地图位置回归 | source /opt/ros/humble/setup.bash && source /tmp/alpha-s1-check-final/install/local_setup.bash；ROS_LOG_DIR=/tmp/alpha-s1-ros-log python3 -m pytest -q third_party/RobotPilot/ros2/src/robotpilot_ui_package/test/test_platform_api.py | 43 passed | 覆盖最近任务筛选、告警位置关联/map-version 过滤/安全详情，以及 Provider 离线时编译多点快照并接入现有 schedule API；同样测试已包含于上方全量结果。 |
| React/Vite 回归 | cd third_party/RobotPilot/web && npm test -- --run && npm run build | 14 files / 40 tests passed；Vite production build passed | registry/page 模块测试，不包含新页面完整 DOM/E2E 操作测试。 |
| Inspection adapter + MissionManager 行为 | source /opt/ros/humble/setup.bash && source install/setup.bash；ROS_LOG_DIR=/tmp/alpha-s1-ros-log、PYTHONPATH 加 src/extension/inspection_adapter 后执行 pytest：src/extension/mission_manager/test src/extension/inspection_adapter/test src/extension/area_rules/test src/control/vehicle_control/test/test_cmd_vel_mux_hold.py | 54 passed | 含 Provider capability freshness gate、契约拒绝用例、fixture 节点及 test_manager_fixture_flow.py。 |
| Fixture → Outbox → PlatformStore | 随 adapter suite 执行 test_manager_fixture_flow.py；测试内使用 ROS_DOMAIN_ID=73 和隔离 SQLite | 1 项通过；明确 source_mode=fixture、outcome=INCONCLUSIVE；同一 batch 重放后结果仍仅一条 | 验证 ROS 节点测试级联通，不代表 Gazebo 导航、真实相机或识别。 |
| ROS2 扩展包构建 | source /opt/ros/humble/setup.bash && source install/setup.bash；colcon --log-base /tmp/alpha-s1-inspection-final/log build --base-paths src/extension/mission_manager src/extension/inspection_adapter --packages-select mission_manager inspection_adapter --symlink-install --build-base /tmp/alpha-s1-inspection-final/build --install-base /tmp/alpha-s1-inspection-final/install | 2 packages finished | 输出目录隔离，避免覆盖 workspace install。 |
| Flask 生产 UI bundle | Vite build 后执行 bash third_party/RobotPilot/scripts/sync_frontend_to_ros.sh；随后 colcon --log-base /tmp/alpha-s1-web-install/log build --packages-select robotpilot_ui_package --symlink-install --build-base /tmp/alpha-s1-web-install/build --install-base /tmp/alpha-s1-web-install/install | static/app 同步完成；robotpilot_ui_package build 完成 | Flask 从 ROS package static/app 提供页面；本次同步已替换原有哈希 bundle。 |
| S0 构建阶段 | ROS_LOG_DIR=/tmp/alpha-s1-ros-log S0_CHECK_ROOT=/tmp/alpha-s1-check-final bash scripts/check_s0.sh | nav_status、消息包、UI package、mission_manager 四包 build 及 robot_bringup build 完成 | 此次 wrapper 在进入其重复 Python 阶段后被中断；最终 Python、Vitest/Vite 已分开完整执行并通过，不把 wrapper 记为整体 PASS。 |
| 语法与差异 | python3 -m py_compile（平台 API、MissionManager、Adapter）；node --check third_party/RobotPilot/web/public/ros/nav2d.js；git diff --check | 通过 | 文档和静态 bundle 纳入工作区差异。 |

## 未执行的系统验收

| 场景 | 状态 | 原因/预期下一步 |
|---|---|---|
| Gazebo 全系统启动、Web 实时运行截图 | BLOCKED | 本轮没有启动 Gazebo/导航与完整 Flask/Web 进程组合；fixture 单测不能替代系统运行证据。 |
| map bundle 不一致、冷切换、地图/区域 ROS ACK | BLOCKED | 复用 S0 fail-closed 校验；没有本轮仿真环境验证地图工作流。 |
| provider 在线/离线实测和外部动作 | EXTERNAL_PENDING | 真实 provider 的 schema、capability/heartbeat 和服务未交付；adapter external 模式未连接 provider。 |
| 相机图像/基准照片/告警媒体 | EXTERNAL_PENDING | 没有受控媒体存储或访问 API；UI 如实显示媒体不可用，不显示假图。 |
| 硬件 BMS、充电、实机安全验证 | BLOCKED | 没有实机设备与现场测试窗口。 |

## 重跑命令

在仓库根目录：

    ROS_LOG_DIR=/tmp/alpha-s1-ros-log S0_CHECK_ROOT=/tmp/alpha-s1-check bash scripts/check_s0.sh

最终 Python 回归：

    source /opt/ros/humble/setup.bash
    source /tmp/alpha-s1-check-final/install/local_setup.bash
    ROS_LOG_DIR=/tmp/alpha-s1-ros-log python3 -m pytest -q third_party/RobotPilot/ros2/src/robotpilot_ui_package/test src/extension/mission_manager/test src/extension/area_rules/test src/control/vehicle_control/test/test_cmd_vel_mux_hold.py

Inspection adapter 和 MissionManager：

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    PYTHONPATH=src/extension/inspection_adapter:$PYTHONPATH ROS_LOG_DIR=/tmp/alpha-s1-ros-log python3 -m pytest -q src/extension/mission_manager/test src/extension/inspection_adapter/test src/extension/area_rules/test src/control/vehicle_control/test/test_cmd_vel_mux_hold.py

adapter package build 的临时输出在 /tmp/alpha-s1-inspection-final；UI package static bundle build 在 /tmp/alpha-s1-web-install。构建命令见上表。
