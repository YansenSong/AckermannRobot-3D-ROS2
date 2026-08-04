import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
        ),
    ])
