"""Start the complete Gazebo navigation stack under one launch service."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    simulation_share = get_package_share_directory('ackermann_simulation')
    bringup_share = get_package_share_directory('ackermann_bringup')

    arguments = [
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('map_pgm'),
        DeclareLaunchArgument('globalmap_pcd'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('ackermann_bringup'),
                'config', 'liorf_localization.yaml')),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'publish_ekf_tf': 'false',
            'use_rviz': 'false',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'map_pgm': LaunchConfiguration('map_pgm'),
            'globalmap_pcd': LaunchConfiguration('globalmap_pcd'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    # Let Gazebo spawn the robot and controllers before localization starts.
    delayed_navigation = TimerAction(period=5.0, actions=[navigation])

    return LaunchDescription(arguments + [gazebo, delayed_navigation])
