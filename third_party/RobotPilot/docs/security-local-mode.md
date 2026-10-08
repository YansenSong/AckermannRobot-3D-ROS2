# 远端访问：local 模式

当前可用模式是 `open`（仅本机开发）与 `local`（本地用户和角色）。`external` 尚未实现，请求该模式会拒绝启动。`local` 的首次用户通过机器人本机命令创建，不提供匿名初始化接口。

## 启动

```bash
source ros2/install/setup.bash
ros2 run robotpilot_ui_package ui_admin admin --role Admin --robot-id robot-001
AUTH_MODE=local ROBOTPILOT_ROBOT_ID=robot-001 \
ROBOTPILOT_ALLOWED_ORIGINS=https://robot.example \
ROBOTPILOT_UI_TRUST_PROXY=1 bash scripts/run_ui_backend.sh
```

`ui_admin` 会交互式读取至少 12 字符的密码。数据库默认位于 `~/.robotpilot_ui/auth.sqlite3`；平台索引、故障和审计默认位于 `~/.robotpilot_ui/platform.sqlite3`。若已有旧版 `~/.openamr_ui` 且新目录尚不存在，服务会继续使用旧目录中的数据。两者应由运行用户独占并纳入备份。命令可通过 `ROBOTPILOT_AUTH_DB`、`ROBOTPILOT_PLATFORM_DB` 修改路径。前端通过浏览器访问 HTTPS 反向代理地址，登录后使用服务端会话。

应用、原始 rosbridge、受权限约束的 WebSocket 网关、摄像头服务分别绑定本机 `5050`、`9090`、`9091`、`8080`。外部仅开放 HTTPS 反向代理入口，不对外转发 `9090`、`8080` 或 ROS DDS 端口。若部署在不可信 ROS 网络，还需对 DDS 网络做隔离或使用 ROS 2 安全机制。

`AUTH_MODE=local` 时，未登录访问会进入 React 登录页（`/login`）；登录后显示当前用户名和角色，可从侧栏或移动端导航退出。`AUTH_MODE=open` 继续用于本机开发，不显示登录页。开发前端 `localhost:3000` 可连接本机 `5050`，仅允许这两个 loopback Origin 做带凭据的开发跨域请求。

反向代理需将 `/` 转到 `127.0.0.1:5050`，将 WebSocket `/rosbridge` 转到 `127.0.0.1:9091`，并原样传递浏览器 `Origin` 和 `Cookie`。`ROBOTPILOT_ALLOWED_ORIGINS` 必须是用户实际访问的 HTTPS Origin。代理不得将任意来源的 Origin 改写为可信地址。示例 Nginx 配置：

```nginx
server {
    listen 443 ssl;
    server_name robot.example;
    ssl_certificate /etc/ssl/robot/fullchain.pem;
    ssl_certificate_key /etc/ssl/robot/privkey.pem;

    location /rosbridge {
        proxy_pass http://127.0.0.1:9091/;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        proxy_set_header Origin $http_origin;
        proxy_set_header Cookie $http_cookie;
        proxy_read_timeout 3600s;
    }

    location / {
        proxy_pass http://127.0.0.1:5050;
        proxy_set_header Host $host;
        proxy_set_header X-Forwarded-Proto $scheme;
    }
}
```

## 权限边界

- `Viewer` 可订阅状态、查看平台数据；仅允许 `/nav_data_req`、`/ackermann/routes/request`、`/WP_req` 这几个读取请求主题，其他 ROS 发布和服务调用默认拒绝。
- `Operator` 可下发任务、目标点、停止和回充指令；不能写参数或地图配置。
- `Engineer` 可操作地图/定位/参数接口。远端 `/cmd_vel` 暂不开放；手动驾驶须在控制租约和死手超时完成后单独启用。
- `Admin` 可使用以上能力；用户管理目前通过机器人本机 `ui_admin` 命令进行。

网关以明确允许的 ROS 话题和服务列表判断写权限，未知操作默认拒绝。网页已有的某些旧按钮在保护模式下会收到拒绝响应；服务端会保持拒绝，不以隐藏按钮代替授权。

登录会话有效期为 12 小时；网关每 30 秒复查会话，因此登出、禁用账号或会话过期后，已打开的 WebSocket 最迟在下一次复查时关闭。HTTP 写请求使用 CSRF 令牌。HTTP API 请求结果以及 ROS 发布/服务调用的放行和拒绝会写入 SQLite 审计表。SSE 状态变化持久化并支持 `Last-Event-ID` 续传，每台机器人最多保留最近 10,000 条事件。

## 平台 API

`/api/v1/auth/me` 返回当前用户与角色；`/api/v1/auth/csrf` 返回写请求使用的 CSRF 令牌。机器人接口统一位于 `/api/v1/robots/{robot_id}`，包括 `status`、`health`、`missions`、`tasks`、`commands`、`faults`、`config`、`logs`。任务写请求必须提供 `Idempotency-Key`。平台配置接口当前只支持 `low_battery_threshold`，响应标明 `scope=platform`；不会冒充 ROS 参数修改。

`POST /tasks` 等命令返回 `command_id`，可通过 `GET /commands/{command_id}` 查询确认。HTTP `202` 表示受理，不代表任务完成。机器人侧 mission manager 仍保存任务定义、执行和事件。详情见[远端权限与平台接口设计](../../../docs/远端权限与平台接口设计.md)。
