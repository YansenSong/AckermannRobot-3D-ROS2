# liorf_localization 集成来源与差异

当前 vendored 源码来自：

```text
repository: YJZLuckyBoy/liorf_localization
branch:     liorf_localization-ros2
commit:     ad592cebc397209273245b4e669327399a591c9d
```

该版本保留在 `src/liorf_localization`，上游 `LICENSE` 和作者信息未删除。项目只做了 ROS 2 Humble 与 AckermannRobot 接口所需的集成修补：

- 增加 `globalmap_pcd` 参数；非空时按绝对文件路径加载 `GlobalMap.pcd`，加载失败或点数不足会明确报错并等待；
- `liorf_localization/localization/global_map` 使用 Reliable + Transient Local QoS，并在地图加载成功后立即发布一次；
- 移除 mapOptimization 的额外 `odom -> lidar_link` TF，避免与 `robot_state_publisher` 和 TransformFusion 形成多父节点；
- TransformFusion 使用 IMU odometry 原生 ROS 时间戳发布 `odom -> base_link`，不再通过 `uint32_t` 重建纳秒时间；
- `laser_link -> base_link` 的静态 TF 改为首次融合时查询，查询失败会重试并缓存成功结果；
- `mapOptimization` 使用非 intra-process 模式，以兼容 Transient Local 地图发布；
- 保持上游 scan-to-map、ICP 初始化、IMU deskew 和 GTSAM 预积分算法结构不变。

当前项目专用参数在 [`ackermann_bringup/config/liorf_localization.yaml`](../src/ackermann_bringup/config/liorf_localization.yaml)，统一启动编排在 [`localization.launch.py`](../src/ackermann_bringup/launch/localization.launch.py)。
