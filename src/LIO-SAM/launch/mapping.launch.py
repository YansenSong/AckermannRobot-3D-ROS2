#!/usr/bin/env python3
"""LIO-SAM 实车建图一键启动脚本（独立于导航/运控链路）。

组合建图所需的完整传感器链路：

    LPMS-IG1-RS485 IMU 驱动   → /imu/data + base_link→imu TF     (lpms_ig1)
    Hesai LiDAR 驱动          → /lidar_points                    (lidar_driver)
    静态 TF base_link→hesai_lidar                                  (vehicle_config)
    LIO-SAM 4 节点 + map→odom TF + RViz（含 IMU/LiDAR 外参注入）   (lio_sam)

不启动运动控制桥 / NeuPAN / hdl_localization。所有安装位姿与话题的唯一数据源是
src/vehicle_config/config/real_vehicle.yaml。

说明：
  * base_link→hesai_lidar 静态 TF 单独发布：LIO-SAM imuPreintegration 按
    lidarFrame→baselinkFrame 查询该 TF，缺失时退化为恒等变换，里程计坐标系错乱。
    该 TF 原本只在 neupan_ros2/real_vehicle.launch.py 里发布（会连带启动 NeuPAN），
    不适合建图场景，故在此单独补上。
  * lidar_driver 自带 rviz 通过 with_rviz:=false 关闭，只保留 LIO-SAM 的 rviz（显示地图）。

用法:
  ros2 launch lio_sam mapping.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from vehicle_config import lidar_transform, load_real_vehicle_config


def _launch_file(package, name):
    """Resolve <package>/launch/<name> to a PythonLaunchDescriptionSource."""
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), 'launch', name))


def generate_launch_description():
    vehicle = load_real_vehicle_config()
    lidar_mount = lidar_transform(vehicle)
    lidar_frames = vehicle['sensors']['lidar']['frames']

    # ---- 静态 TF: base_link → hesai_lidar ----
    # 与 neupan_ros2/real_vehicle.launch.py 同一数据源（sensors.lidar.mount）。
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=[
            '--x', str(lidar_mount['x']),
            '--y', str(lidar_mount['y']),
            '--z', str(lidar_mount['z']),
            '--yaw', str(lidar_mount['yaw']),
            '--pitch', str(lidar_mount['pitch']),
            '--roll', str(lidar_mount['roll']),
            '--frame-id', lidar_frames['scan_target'],
            '--child-frame-id', lidar_frames['sensor'],
        ],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        # IMU 驱动 + base_link→imu 静态 TF
        IncludeLaunchDescription(
            _launch_file('lpms_ig1', 'real_vehicle_imu.launch.py')),
        # LiDAR 驱动（关闭自带 rviz，避免与 LIO-SAM 的 rviz 重复）
        IncludeLaunchDescription(
            _launch_file('lidar_driver', 'start.py'),
            launch_arguments={'with_rviz': 'false'}.items(),
        ),
        static_tf_base_to_lidar,
        # LIO-SAM 本体（4 节点 + map→odom 静态 TF + rviz2）
        IncludeLaunchDescription(
            _launch_file('lio_sam', 'run.launch.py')),
    ])
