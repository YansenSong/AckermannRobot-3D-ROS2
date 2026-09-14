import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from vehicle_config import (
    load_real_vehicle_config,
    materialize_lidar_driver_config,
)


def generate_launch_description():
    package_dir = get_package_share_directory('lidar_driver')
    rviz_config = os.path.join(package_dir, 'rviz', 'rviz2.rviz')
    vehicle = load_real_vehicle_config()
    yaml_config = materialize_lidar_driver_config(
        vehicle, os.path.join(package_dir, 'config', 'config.yaml'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'with_rviz',
            default_value='true',
            description='Launch RViz2 for raw point cloud visualization; '
                        'LIO-SAM mapping.launch.py passes false to avoid a '
                        'duplicate window.',
        ),
        Node(
            namespace='lidar_driver',
            package='lidar_driver',
            executable='lidar_driver_node',
            output='screen',
            parameters=[{'config_path': yaml_config}],
        ),
        Node(
            namespace='rviz2',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('with_rviz')),
        ),
    ])
