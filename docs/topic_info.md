# 两套导航栈话题信息

本项目保留两套导航栈：

- **NeuPAN 导航栈**：Smac 全局规划 + NeuPAN 局部规划与控制；
- **Nav2 导航栈**：Smac Hybrid-A* 全局规划 + MPPI Ackermann 局部控制。

两套导航栈共用定位、地图、传感器和底盘控制器，但不要同时向同一个底盘
控制话题发布命令。

## 1. NeuPAN 导航栈

### 1.1 数据流

```text
/initialpose -> LIORF 定位

/goal_pose -> ackermann_smac_bridge -> Smac planner
                                      ├── /plan -> NeuPAN
                                      └── /plan_path -> nav_status_node

/scan + /plan -> NeuPAN
NeuPAN -> /neupan_cmd_vel_raw -> neupan_ackermann_adapter
        -> /neupan_cmd_vel -> cmd_vel_mux
        -> /ackermann_steering_controller/reference
```

### 1.2 话题列表

| 话题 | 消息类型 | 方向 | 关键格式/作用 |
|---|---|---|---|
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 输入 | `header.frame_id=map`；`pose.pose` 为初始位姿，`pose.covariance` 为协方差 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 输入 | `header.frame_id=map`；`pose.position` 为目标位置，`pose.orientation` 为目标朝向 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 输出 | 2D 栅格地图；`info.resolution` 为分辨率，`data[]` 为栅格占用值 |
| `/plan` | `nav_msgs/msg/Path` | 输出/输入 | Smac 全局路径；`header.frame_id=map`，`poses[]` 为路径点序列 |
| `/plan_path` | `nav_msgs/msg/Path` | 输出 | `ackermann_smac_bridge` 发布的兼容路径，供 `nav_status_node` 和 RViz 使用 |
| `/global_path_remaining_distance` | `std_msgs/msg/Float64` | 输出 | `data` 为沿全局路径到目标点的剩余距离，单位为 m |
| `/scan` | `sensor_msgs/msg/LaserScan` | 输入 | `header.frame_id=laser_link`；`ranges[]` 为激光距离，单位为 m |
| `/neupan_plan` | `nav_msgs/msg/Path` | 输出 | NeuPAN 优化后的局部轨迹 |
| `/neupan_ref_state` | `nav_msgs/msg/Path` | 输出 | NeuPAN 参考状态 |
| `/neupan_initial_path` | `nav_msgs/msg/Path` | 输出 | NeuPAN 接收和处理的初始路径 |
| `/neupan_cmd_vel_raw` | `geometry_msgs/msg/Twist` | 输出 | `linear.x=v`；`angular.z` 表示 NeuPAN 输出的虚拟前轮转角 |
| `/neupan_cmd_vel` | `geometry_msgs/msg/Twist` | 输出 | `linear.x=v`；`angular.z=ω`，已由轴距和转角换算为车体偏航角速度 |
| `/stop` | `std_msgs/msg/Bool` | 输入 | `data=true` 覆盖速度命令并停车；`false` 解除覆盖 |
| `/navigation/state` | `nav_status/msg/NavigationStatus` | 输出 | `stamp`、`state`、`detail`；状态值见下表 |
| `/ackermann_steering_controller/reference` | `geometry_msgs/msg/TwistStamped` | 输出 | `header.stamp`、`header.frame_id=base_link`；`twist.linear.x=v`，`twist.angular.z=ω` |

`NavigationStatus.msg` 格式：

```text
builtin_interfaces/Time stamp
uint8 state
string detail
```

状态值：

```text
WAITING_FOR_GOAL = 0
PLANNING         = 1
MOVING           = 2
ARRIVED          = 3
FAILED           = 4
```

## 2. Nav2 导航栈

### 2.1 数据流

```text
/initialpose -> LIORF 定位

/goal_pose -> bt_navigator -> planner_server + Smac Hybrid-A*
                           └── /plan

/scan -> local_costmap / global_costmap -> MPPI obstacle critic

/plan + /odom -> controller_server + MPPI Ackermann
              -> /ackermann_nav/cmd_vel_raw
              -> velocity_smoother
              -> /ackermann_nav/cmd_vel_smoothed
              -> cmd_bridge.py
              -> /ackermann_steering_controller/reference
```

### 2.2 话题列表

| 话题 | 消息类型 | 方向 | 关键格式/作用 |
|---|---|---|---|
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 输入 | `header.frame_id=map`；LIORF 初始定位位姿 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 输入 | `header.frame_id=map`；Nav2 BT Navigator 的目标位姿 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 输入 | `map_server` 发布的静态 2D 栅格地图 |
| `/plan` | `nav_msgs/msg/Path` | 输出 | Smac Hybrid-A* 全局路径；`poses[]` 为带朝向的路径点 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 输入 | local/global costmap 的障碍物 marking 和 clearing |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 输出 | 全局代价地图可视化数据 |
| `/global_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | 输出 | 全局代价地图原始数据 |
| `/global_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 输出 | 全局代价地图使用的机器人 footprint |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 输出 | 局部代价地图可视化数据 |
| `/local_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | 输出 | MPPI/行为服务器使用的局部代价地图 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 输出 | 局部代价地图使用的机器人 footprint |
| `/ackermann_nav/cmd_vel_raw` | `geometry_msgs/msg/Twist` | 输出 | MPPI 输出；`linear.x=v`，`angular.z=ω` |
| `/ackermann_nav/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | 输出 | 速度平滑器输出；`linear.x=v`，`angular.z=ω` |
| `/ackermann_steering_controller/reference` | `geometry_msgs/msg/TwistStamped` | 输出 | `header.frame_id=base_link`；最终底盘速度命令 |

Nav2 导航栈不发布 NeuPAN 专用话题：

```text
/neupan_plan
/neupan_ref_state
/neupan_initial_path
/neupan_cmd_vel_raw
/neupan_cmd_vel
/plan_path
/global_path_remaining_distance
/navigation/state
```

## 3. 两套导航栈共用的话题

| 话题 | 消息类型 | 关键格式/作用 |
|---|---|---|
| `/points_raw` | `sensor_msgs/msg/PointCloud2` | Gazebo 3D 激光雷达原始点云 |
| `/points_lio` | `sensor_msgs/msg/PointCloud2` | LIORF 使用的点云，包含 `ring/time` 等字段 |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU 线加速度和角速度 |
| `/odom` | `nav_msgs/msg/Odometry` | LIORF 输出的定位里程计，Nav2 使用 |
| `/odom_wheel` | `nav_msgs/msg/Odometry` | Ackermann 控制器输出的轮速里程计，NeuPAN 栈的 `nav_status_node` 使用 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 关节位置、速度和力矩状态 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 动态坐标变换，如 `map -> odom -> base_link` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 静态坐标变换，如 `base_link -> laser_link` |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo 仿真时间 |

## 4. 控制命令格式区别

两套导航栈最终都向 `/ackermann_steering_controller/reference` 发布
`TwistStamped`，但上游命令语义不同：

| 话题 | `linear.x` | `angular.z` |
|---|---|---|
| `/neupan_cmd_vel_raw` | 线速度 `v` | 虚拟前轮转角 `δ` |
| `/neupan_cmd_vel` | 线速度 `v` | 车体偏航角速度 `ω` |
| `/ackermann_nav/cmd_vel_raw` | 线速度 `v` | 偏航角速度 `ω` |
| `/ackermann_nav/cmd_vel_smoothed` | 线速度 `v` | 偏航角速度 `ω` |
| `/ackermann_steering_controller/reference` | 线速度 `v` | 偏航角速度 `ω` |

因此不能把 NeuPAN 的 `/neupan_cmd_vel_raw.angular.z` 直接当作 Nav2 的
`angular.z`，也不能把任一 `angular.z` 直接当成前轮转角。

