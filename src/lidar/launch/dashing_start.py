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
            package='lidar_driver',
            node_namespace='lidar_driver',
            node_name='lidar_driver_node',
            node_executable='lidar_driver_node',
            output='screen',
            parameters=[{'config_path': yaml_config}],
        ),
        Node(
            package='rviz2',
            node_namespace='rviz2',
            node_name='rviz2',
            node_executable='rviz2',
            arguments=['-d', rviz_config]
        )
    ])
