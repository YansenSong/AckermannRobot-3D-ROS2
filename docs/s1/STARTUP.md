# S1 启动、fixture 与迁移说明

## 构建

仓库根目录：

    source /opt/ros/humble/setup.bash
    colcon build --packages-up-to robot_bringup robotpilot_ui_package inspection_adapter --symlink-install
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

## Gazebo + Web fixture 启动入口（待现场执行验收）

下述命令提供可复现的启动链；本轮尚未完成 Gazebo 全流程验收。所有终端使用同一 ROS_DOMAIN_ID。构建完成后，终端 1：

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    export ROS_DOMAIN_ID=73
    export ROBOT_MODE=simulation
    export INSPECTION_FIXTURE_SCENARIO=inconclusive
    export INSPECTION_ADAPTER_DB=/tmp/alpha-s1-fixture/adapter.sqlite3
    export ROBOTPILOT_MISSION_DB=/tmp/alpha-s1-fixture/missions.sqlite3
    bash scripts/nav_liorf_neupan.sh maps/mini --inspection-adapter=fixture

终端 2 在同一域启动 `bash scripts/run_neupan.sh`；终端 3 设置同一 ROS_DOMAIN_ID、`ROBOT_MODE=simulation`、隔离的 `ROBOTPILOT_DATA_DIR=/tmp/alpha-s1-fixture/platform` 后运行 `bash scripts/run_robotpilot.sh`。Web 运行页和巡检任务页使用现有认证方式登录。真实导航 ARRIVED、定位/速度、地图 bundle、安全状态和 Provider heartbeat 必须均满足门禁，才会派发 fixture 动作。可通过 `/inspection/provider/capabilities`、`/inspection/provider/heartbeat` ROS topic 和 `GET /api/v1/robots/{robot_id}/inspection/capabilities` 核对在线状态；Adapter 进程退出时 launch 记录错误并重启，心跳过期后 Manager 显示 Provider stale。

默认导航入口不启动 Adapter。生产接入须显式使用 `--inspection-adapter=external`，并由外部团队提供真实 Provider；通用 `navigation.launch.py` 也有 `enable_inspection_adapter`、`inspection_provider_mode` 参数。fixture 仅允许通过仿真 profile 且 `ROBOT_MODE=simulation` 启动，页面及事件保留 `source_mode=fixture`/`is_test_data=true` 标识。运行完整流程后须保存 Gazebo、ROS topic、Web 告警和地图 pin 的日志/截图，才可将相关项记为 `SIM_VERIFIED`。

MissionManager 还要求及时的定位、filtered odom、ARRIVED、software stop、电池、area control（如果已启用）以及 map identity/bundle；这些条件缺失时必须保持任务阻断。fixture 结果是测试数据，不应进入真实验收数据集。严禁将其 source_mode 改为 simulation 或 hardware。

## 数据迁移与回退

PlatformStore 初始化时会将 schema 3 数据库增量升级至 schema 8，MissionStore 迁移至 schema 5，不删除已有业务行。升级前应备份实际 SQLite 文件；新 schema 不能交给旧版本程序直接使用。需要回退时先停服务并恢复升级前备份，不手工下调 PRAGMA user_version。

WaypointActions 页面保存的点位动作计划本身仍是不可执行草稿。Operator 可在 Provider 在线且动作能力匹配时创建单次巡检；服务端会重新检查 capability/heartbeat、source mode、安全 readiness、map bundle、点位/资产版本后经现有 MissionManager API 下发。也可按顺序选择多个点位，将配置编译成不可变 MissionManager mission 快照（`compile_only=true`），再从巡检 profile 的 Scheduler 页面接入既有日/周调度。保存模板时 Provider 可离线；机器人开始任务时仍会重新验证 capability、地图 bundle 和安全 readiness。当前未提供模板版本编辑/管理和 schedule 运行历史的巡检专用视图。不要通过直接发布 ROS topic 绕过 Platform API 或 MissionManager。
