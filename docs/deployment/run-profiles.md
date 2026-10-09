# RobotPilot 启动配置与信任边界

## 本机仿真开发

ROS 2 Humble 已验证；在仓库根目录先运行 `cd third_party/RobotPilot/web && npm ci`，再运行 `bash scripts/run_robotpilot.sh`。该脚本的默认 `AUTH_MODE=open` 仅绑定本机 loopback，Vite 为 `127.0.0.1:3000`，Flask 为 `127.0.0.1:5050`，原生 ROSBridge 仅供本机开发。`ROBOT_MODE=simulation`、`BATTERY_SOURCE=sim` 会把仿真来源写入状态与机器人事件。仿真导航需另启 `scripts/nav_liorf_neupan.sh` 和 `scripts/run_neupan.sh`，并具备相应地图文件。

`bash scripts/check_s0.sh` 使用 `/tmp/alpha-s0-check` 隔离构建目标 ROS 包、执行 Python 与前端测试；末尾会明确打印未运行 Gazebo、ROS 图和设备验收。

## local 身份模式

`AUTH_MODE=local` 使用账号会话、角色、CSRF 和角色感知 ROSBridge 网关；仓库的 Vite 开发脚本会拒绝此模式。局域网部署需单独提供 HTTPS 终止、同源反向代理、`ALLOWED_ORIGINS`、证书、Cookie 安全设置及防火墙。不得将原始 9090 或 DDS 暴露给不可信网络。机器人 ID、`ROS_DOMAIN_ID`、地图与数据库路径应按单机器人部署固定配置，前后端身份一致。

当前地图/路线/区域等浏览器旧写入仍有受网关角色允许的 ROSBridge 路径，因此 local 模式尚不能视作完成的远端安全部署。任务编辑器现走平台 API，网关只允许旧 `/mission/command` 的只读 query。部署前应完成[旧写入迁移](../s0/legacy-write-path-inventory.md)与整链权限回归。

## 数据与断线

任务、命令去重、机器人事件 outbox 在 `ROBOTPILOT_MISSION_DB`（或 `OPENAMR_MISSION_DB`）指定的 SQLite；默认 `ROS_HOME/ackermann_missions.sqlite3`。平台库由后端配置存放，保存命令、故障、审计和机器人事件投影。Flask 离线时机器人 outbox 留在本地，恢复后重发；浏览器离线不停止机器人任务。平台 SSE 的断线续传只针对已入平台库的事件。

实机适配时改用 `ROBOT_MODE=hardware` 与真实 BMS、定位、底盘数据源；数据缺失时返回 unavailable/stale，不能回退到模拟值。软件停车仍需由本地控制器和物理急停之外的安全链共同保护。
