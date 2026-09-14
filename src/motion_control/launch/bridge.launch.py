#!/usr/bin/env python3
"""Launch the STM32 real-vehicle motion-control bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('motion_control')
    yaml_path = os.path.join(pkg_dir, 'config', 'bridge_params.yaml')

    arguments = [
        DeclareLaunchArgument('max_speed', default_value='2.0'),
        DeclareLaunchArgument('max_reverse_speed', default_value='-0.5'),
        DeclareLaunchArgument('max_steer_deg', default_value='30.0'),
    ]

    bridge = Node(
        package='motion_control',
        executable='bridge_node',
        name='vehicle_bridge_node',
        output='screen',
        parameters=[
            yaml_path,
            {
                'max_speed': ParameterValue(
                    LaunchConfiguration('max_speed'), value_type=float
                ),
                'max_reverse_speed': ParameterValue(
                    LaunchConfiguration('max_reverse_speed'), value_type=float
                ),
                'max_steer_deg': ParameterValue(
                    LaunchConfiguration('max_steer_deg'), value_type=float
                ),
            },
        ],
        emulate_tty=True,
    )

    return LaunchDescription(arguments + [bridge])
