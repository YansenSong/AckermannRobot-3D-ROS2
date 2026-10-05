"""Start liorf prior-map localization and the map-to-odom identity TF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('ackermann_bringup')
    default_params = os.path.join(
        bringup_share, 'config', 'liorf_localization.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    globalmap_pcd = LaunchConfiguration('globalmap_pcd')
    liorf_parameters = [
        params_file,
        {
            'use_sim_time': use_sim_time,
            'globalmap_pcd': globalmap_pcd,
        },
    ]

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    image_projection = Node(
        package='liorf_localization',
        executable='liorf_localization_imageProjection',
        name='liorf_localization_imageProjection',
        parameters=liorf_parameters,
        output='screen',
    )
    imu_preintegration = Node(
        package='liorf_localization',
        executable='liorf_localization_imuPreintegration',
        name='liorf_localization_imuPreintegration',
        parameters=liorf_parameters,
        output='screen',
    )
    map_optimization = Node(
        package='liorf_localization',
        executable='liorf_localization_mapOptmization',
        name='liorf_localization_mapOptmization',
        parameters=liorf_parameters,
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'globalmap_pcd', default_value='',
            description='Absolute path to GlobalMap.pcd'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='liorf parameter file'),
        map_to_odom,
        image_projection,
        imu_preintegration,
        map_optimization,
    ])
