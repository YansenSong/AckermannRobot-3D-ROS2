# NeuPAN ROS 2 实车接入

本分支只保留真实车辆配置。详细启动方法见仓库根目录 `README.md`，参数调优方法见
`docs/neupan_tuning.md`。

所有实车公共参数统一来自：

```text
src/vehicle_config/config/real_vehicle.yaml
```

请勿在 `neupan_ros2` 内再次写死车长、车宽、轴距、转角/速度限制或 LiDAR 参数。
