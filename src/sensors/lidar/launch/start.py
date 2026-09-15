import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('lidar_driver')
    default_config = os.path.join(package_dir, 'config', 'config.yaml')
    rviz_config = os.path.join(package_dir, 'rviz', 'rviz2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config,
            description='Path to Hesai lidar_driver YAML configuration.',
        ),
        DeclareLaunchArgument(
            'with_rviz',
            default_value='true',
            description='Launch RViz2 for raw point cloud visualization.',
        ),
        Node(
            namespace='lidar_driver',
            package='lidar_driver',
            executable='lidar_driver_node',
            output='screen',
            parameters=[{'config_path': LaunchConfiguration('config_path')}],
        ),
        # Standalone raw-cloud viewing needs a TF tree root: the driver only
        # stamps ros_frame_id on the cloud and publishes no TF, so RViz would
        # report "Fixed Frame [laser_link] does not exist" and render nothing.
        # Gated on with_rviz so the navigation stack (which owns the real TF
        # tree via liorf) is never given a second parent for laser_link.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_link_static_tf',
            arguments=['0', '0', '0', '0', '0', '0',
                       'rear_axle_link', 'laser_link'],
            condition=IfCondition(LaunchConfiguration('with_rviz')),
        ),
        Node(
            namespace='rviz2',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('with_rviz')),
        ),
    ])
