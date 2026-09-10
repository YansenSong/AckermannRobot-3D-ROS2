# 规划导航运动相关话题

本文档记录当前 Ackermann 3D 激光雷达仿真项目中，规划、定位、传感器和底盘运动控制相关的 ROS 2 话题。

当前导航已经移除 DWB，使用 Hybrid A* + NeuPAN 完成全局规划、局部规划和运动控制。

## 1. 当前导航链路

```text
/initialpose
    ↓
liorf prior-map localization

/goal_pose
    ↓
Hybrid A*
    ├── /plan
    ├── /plan_path
    └── /global_path_remaining_distance
            ↓
        NeuPAN
            ↓ /neupan_cmd_vel
        cmd_vel_mux
            ↓
/ackermann_steering_controller/reference
            ↓
        Gazebo 阿克曼底盘
```

## 2. 核心导航话题

| 话题 | 类型 | 作用 |
|---|---|---|
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz 设置 liorf 先验地图定位的初始位姿 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz 发布导航目标，Hybrid A* 接收 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Hybrid A* 发布的栅格地图，供 RViz 显示 |
| `/plan` | `nav_msgs/msg/Path` | Hybrid A* 发布、NeuPAN 接收的全局路径 |
| `/plan_path` | `nav_msgs/msg/Path` | 全局路径的 RViz 显示话题 |
| `/global_path_remaining_distance` | `std_msgs/msg/Float64` | 当前沿全局路径到目标点的剩余距离，单位为米，约 10 Hz 更新 |
| `/navigation/state` | `nav_status/msg/NavigationStatus` | 当前导航状态：等待目标、规划、移动、到达；`FAILED` 仅预留 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 点云转换后的二维激光，供 NeuPAN 进行障碍物检测 |
| `/neupan_cmd_vel` | `geometry_msgs/msg/Twist` | NeuPAN 输出的速度指令 |
| `/stop` | `std_msgs/msg/Bool` | `data: true` 强制停车，`data: false` 解除停车覆盖 |
| `/ackermann_steering_controller/reference` | `geometry_msgs/msg/TwistStamped` | 最终发送给阿克曼底盘控制器的速度指令 |

### 剩余路径距离

`/global_path_remaining_distance` 不是当前位置到目标点的直线距离，而是：

1. 获取当前 `map` 坐标系下的机器人位姿；
2. 将当前位置投影到当前全局路径上；
3. 计算投影点沿路径到终点的弧长。

该距离基于 Hybrid A* 的规划参考点 `rear_axle_link` 计算。机器人接近目标点时，数值会趋近于 `0`。

查看剩余距离：

```bash
ros2 topic echo /global_path_remaining_distance
```

## 2.1 导航状态

`nav_status_node` 是一个只读观察者，订阅目标、全局路径、剩余路径距离和轮速里程计，发布统一状态：

```text
WAITING_FOR_GOAL -> PLANNING -> MOVING -> ARRIVED
```

到达判定同时要求剩余路径距离、实际车速满足阈值，并持续一段时间；`FAILED` 在当前版本只定义消息常量，不通过超时猜测规划失败。

查看当前状态：

```bash
ros2 topic echo /navigation/state
```

## 3. 传感器与定位支持话题

| 话题 | 类型 | 作用 |
|---|---|---|
| `/points_raw` | `sensor_msgs/msg/PointCloud2` | Gazebo 3D 激光雷达原始点云 |
| `/points_lio` | `sensor_msgs/msg/PointCloud2` | 添加 LIO-SAM/liorf 所需 `ring/time` 字段后的点云 |
| `/imu/data` | `sensor_msgs/msg/Imu` | Gazebo IMU 数据 |
| `/liorf_localization/localization/global_map` | `sensor_msgs/msg/PointCloud2` | liorf 发布的先验全局点云地图（Transient Local） |
| `/odom` | `nav_msgs/msg/Odometry` | liorf TransformFusion 输出的 `odom -> base_link` 融合里程计 |
| `/odom_wheel` | `nav_msgs/msg/Odometry` | 阿克曼控制器输出的轮速里程计 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | robot_localization EKF 融合后的里程计 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 仿真关节状态 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 动态坐标变换 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 静态坐标变换 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo 仿真时间 |

关键传感器链路：

```text
/points_raw → pointcloud_to_laserscan → /scan → NeuPAN
/points_raw → gazebo_lidar_adapter → /points_lio → liorf ImageProjection
/points_lio → liorf ImageProjection → liorf mapOptimization
/imu/data → liorf ImageProjection + IMUPreintegration / EKF / LIO-SAM
/odom_wheel + /imu/data → EKF → /odometry/filtered
```

## 4. NeuPAN 可视化话题

| 话题 | 类型 | 作用 |
|---|---|---|
| `/neupan_plan` | `nav_msgs/msg/Path` | NeuPAN 优化后的局部轨迹 |
| `/neupan_ref_state` | `nav_msgs/msg/Path` | NeuPAN 参考状态 |
| `/neupan_initial_path` | `nav_msgs/msg/Path` | NeuPAN 使用的初始路径 |

当前 Ackermann 配置中调试 Marker 可视化默认关闭，因此以下话题通常不会发布：

```text
/dune_point_markers
/nrmp_point_markers
/robot_marker
```

## 5. 未启用或已移除的话题

- `/cmd_vel`：DWB 相关代码已删除，当前导航不使用该话题。
- `/neupan_goal_pose`：当前 `direct_goal_planning: false`，目标由 Smac Hybrid A* 通过 `/goal_pose` 接收。

## 6. 注意事项

Hybrid A* 同时存在一个 `/plan` 话题和一个 `/plan` 服务：

- 话题类型：`nav_msgs/msg/Path`
- 服务类型：`nav_msgs/srv/GetPlan`

查看话题和服务时需要区分：

```bash
ros2 topic echo /plan
ros2 service call /plan nav_msgs/srv/GetPlan ...
```

导航启动后，通常只需要在 RViz 设置初始位姿，再发布 `/goal_pose`；全局规划、NeuPAN 控制和底盘速度转发会自动执行。
