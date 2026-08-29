"""Start the Gazebo mapping stack with LIO-SAM as the odometry TF owner."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ackermann_share = get_package_share_directory('ackermann_robot')
    lio_share = get_package_share_directory('lio_sam')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ackermann_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'publish_ekf_tf': 'false',
            'use_rviz': 'false',
        }.items(),
    )

    lio_sam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lio_share, 'launch', 'run.launch.py')),
        launch_arguments={
            'params_file': os.path.join(lio_share, 'config', 'params.yaml'),
        }.items(),
    )

    return LaunchDescription([gazebo, lio_sam])
