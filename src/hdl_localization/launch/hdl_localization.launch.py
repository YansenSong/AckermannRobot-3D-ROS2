"""
hdl_localization — NDT-based LiDAR localization

Usage:
  ros2 launch hdl_localization hdl_localization.launch.py

Requires:
  - Pre-built 3D point cloud map (GlobalMap.pcd) in the map directory
  - LiDAR point cloud on /points_raw
  - Optional: IMU on /imu_raw

Architecture:
  globalmap_server → /globalmap (latched PointCloud2)
  hdl_localization → map→odom TF + /odom (NDT matching result)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('hdl_localization').find('hdl_localization')

    # Default paths
    default_pcd = os.path.join(pkg_share, '..', '..', '..', '..', 'src', 'maps', 'GlobalMap.pcd')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')

    # ---- Arguments ----
    pcd_arg = DeclareLaunchArgument(
        'globalmap_pcd',
        default_value=default_pcd,
        description='Path to the pre-built 3D point cloud map (PCD file)')
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the hdl_localization parameter YAML file')

    # ---- globalmap_server ----
    globalmap_server = Node(
        package='hdl_localization',
        executable='globalmap_server_node',
        name='globalmap_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'globalmap_pcd': LaunchConfiguration('globalmap_pcd'),
        }],
    )

    # ---- hdl_localization ----
    hdl_localization = Node(
        package='hdl_localization',
        executable='hdl_localization_node',
        name='hdl_localization',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('/imu_raw', '/imu/data'),
        ],
    )

    return LaunchDescription([
        pcd_arg,
        params_arg,
        globalmap_server,
        hdl_localization,
    ])
