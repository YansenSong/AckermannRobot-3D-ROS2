# S1 启动、fixture 与迁移说明

## 构建

仓库根目录：

    source /opt/ros/humble/setup.bash
    colcon build --packages-up-to mission_manager inspection_adapter --symlink-install
    source install/setup.bash

Web 源码在 third_party/RobotPilot/web：

    npm test
    npm run build

按项目已有方式启动 Flask、rosbridge 与 React/Vite。巡检 profile 路由名为 /inspection、/assets 和 /waypoint-actions；首页复用 MapPage 运行状态组件。

## 只验证 fixture 的 ROS 闭环

fixture 默认关闭。可复现的节点级集成用例会在临时目录、隔离 ROS_DOMAIN_ID 和 SQLite 下自动启动 MissionManager 与 Adapter，构造安全状态/地图 bundle、模拟导航到点并验证 INCONCLUSIVE 结果经 Outbox 写入 PlatformStore：

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    PYTHONPATH=src/extension/inspection_adapter:$PYTHONPATH ROS_LOG_DIR=/tmp/alpha-s1-ros-log python3 -m pytest -q src/extension/inspection_adapter/test/test_manager_fixture_flow.py

独立进程试运行必须显式设置：

    ROS_DOMAIN_ID=<隔离域>
    ROBOT_MODE=simulation
    ROBOT_ID=<测试机器人>
    INSPECTION_PROVIDER_MODE=fixture
    INSPECTION_FIXTURE_SCENARIO=inconclusive
    INSPECTION_ADAPTER_DB=/tmp/<隔离目录>/inspection-adapter.sqlite3
    ROBOTPILOT_MISSION_DB=/tmp/<隔离目录>/mission-manager.sqlite3

MissionManager 还要求及时的定位、filtered odom、ARRIVED、software stop、电池、area control（如果已启用）以及 map identity/bundle；这些条件缺失时必须保持任务阻断。fixture 结果是测试数据，不应进入真实验收数据集。严禁将其 source_mode 改为 simulation 或 hardware。

## 数据迁移与回退

PlatformStore 初始化时会将 schema 3 数据库增量升级至 schema 7，不删除已有业务行。升级前应备份实际 SQLite 文件；schema 7 不能交给旧版本程序直接使用。需要回退时先停服务并恢复升级前备份，不手工下调 PRAGMA user_version。

WaypointActions 页面保存的点位动作计划本身仍是不可执行草稿。Operator 可在 Provider 在线且动作能力匹配时创建单次巡检；服务端会重新检查 capability/heartbeat、source mode、安全 readiness、map bundle、点位/资产版本后经现有 MissionManager API 下发。也可按顺序选择多个点位，将配置编译成不可变 MissionManager mission 快照（`compile_only=true`），再从巡检 profile 的 Scheduler 页面接入既有日/周调度。保存模板时 Provider 可离线；机器人开始任务时仍会重新验证 capability、地图 bundle 和安全 readiness。当前未提供模板版本编辑/管理和 schedule 运行历史的巡检专用视图。不要通过直接发布 ROS topic 绕过 Platform API 或 MissionManager。
