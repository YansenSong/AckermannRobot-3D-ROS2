#!/usr/bin/env python3
"""
Launch file for the STM32 vehicle bridge node.

Protocol, network, and timing parameters are loaded from
config/bridge_params.yaml. Vehicle geometry/limits are intentionally not
coupled to a shared vehicle_config package in this integration branch; until a
new parameter strategy is introduced, unspecified values fall back to the
node defaults and can be overridden with ROS parameters.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('motion_control')
    yaml_path = os.path.join(pkg_dir, 'config', 'bridge_params.yaml')

    return LaunchDescription([
        Node(
            package='motion_control',
            executable='bridge_node',
            name='vehicle_bridge_node',
            output='screen',
            parameters=[yaml_path],
            emulate_tty=True,
        ),
    ])
