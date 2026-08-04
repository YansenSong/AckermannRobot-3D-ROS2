# neupan_ros2（实车）

该包负责把 NeuPAN 接入真实阿克曼车辆，并编排 LiDAR、点云转二维扫描、Hybrid A*
和运动控制桥。

实车配置目录为 `config/robots/real_vehicle/`。车辆尺寸、运动学限制、LiDAR 网络、
话题、扫描处理参数及安装位姿不在此包重复维护，而是在
`vehicle_config/config/real_vehicle.yaml` 中统一配置并在启动时注入。

```bash
ros2 launch neupan_ros2 navigation.launch.py \
  map_pgm:=/absolute/path/map.pgm map_yaml:=/absolute/path/map.yaml

bash scripts/run_neupan.sh
```

`real_vehicle.launch.py` 可在 conda 环境已经激活时直接启动 NeuPAN。其静态
`map → base_link` 仅用于定位系统接入前的联调，实车定位可用后应移除该临时变换。
