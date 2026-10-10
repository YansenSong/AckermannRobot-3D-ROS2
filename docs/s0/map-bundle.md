# 2D/3D 地图配对与冷切换

导航使用同一目录中的 `map.yaml`、其 `image` 图像和 `GlobalMap.pcd`。`map.bundle.json` 将三份文件的 SHA-256 与 `route_store` 从实际 `/map` 发布得到的 12 位 `map_id` 绑定。清单由操作者在核对地图来源后生成；**仅凭同名文件不能证明 2D 与 3D 的物理坐标一致**。

1. LIO-SAM 保存 `GlobalMap.pcd`，从该点云生成 2D 图与 YAML。确认三份文件来自同一次建图，坐标原点和朝向一致。
2. 在无自主任务的情况下加载该 `map.yaml` 到 map_server，读取 `/ackermann/routes/catalog` 中 `active_files.map_id`。不要从文件名猜测 ID。
3. 执行 `ros2 run mission_manager map_bundle_manifest --map-yaml /绝对路径/map.yaml --globalmap-pcd /绝对路径/GlobalMap.pcd --map-id <12位ID>`，写入 `map.bundle.json`。留存原始文件，不覆盖点云。
4. 停止导航栈，使用 `bash scripts/nav_liorf_neupan.sh maps/<目录>` 冷启动。`map_bundle_monitor` 校验清单、文件校验和、运行中 `/map` 的哈希以及 LIORF 成功加载后发布的 `/liorf_localization/localization/global_map`。只有 `/localization/map_bundle.ready=true`、地图 ID 一致、定位新鲜且其他安全门禁通过时，`mission_manager` 才允许新任务或恢复。
5. 若在网页仅切换 2D 地图，运行中 `/map` 哈希会与已固定清单不符，`ready=false`，任务启动/恢复被拒绝。切换成对地图须先停任务与导航，重新按上述步骤启动；失败时保持停车，不自动恢复任务。

`map_bundle_monitor` 只证明已加载的 3D 点云与**经操作者确认后登记的** 2D 地图清单一致，并用运行时 LIORF 点云输出防止加载失败被误报为可用。它不能自动证明现实场景里点云与栅格的几何重合，首次建图和切图仍需用 Gazebo/现场已知点与 TF 核对。

2026-10-10：本机 `maps/mini/GlobalMap.pcd` 已补入（609,339 点，SHA-256 `70357ca4ce6c52b8f90ab29961a094780d9b4986a23b11e797915e18eb3a3181`）。从实际 `/map`/`route_store` 取得 ID `0c254af3b862`，生成 `maps/mini/map.bundle.json`；Gazebo 运行中清单为 `ready=true`，LIORF 位姿在线，1 m 导航任务到达目标。该 PCD 被仓库的 `*.pcd` 规则忽略，部署到另一台机器时必须单独提供同一文件并重新核对校验和。一次近距离试验尚不能替代全面的地图几何配准验收。
