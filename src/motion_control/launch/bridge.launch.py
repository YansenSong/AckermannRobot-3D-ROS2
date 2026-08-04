#!/usr/bin/env python3
"""
Launch file for the UART Vehicle Bridge node.

Protocol/timing parameters are loaded from config/bridge_params.yaml.
Vehicle parameters are injected from vehicle_config/config/real_vehicle.yaml.
To override other parameters, edit bridge_params.yaml or use --ros-args:
    ros2 launch motion_control bridge.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from vehicle_config import bridge_parameters, load_real_vehicle_config


def generate_launch_description():
    pkg_dir = get_package_share_directory('motion_control')
    yaml_path = os.path.join(pkg_dir, 'config', 'bridge_params.yaml')
    vehicle = load_real_vehicle_config()

    return LaunchDescription([
        Node(
            package='motion_control',
            executable='bridge_node',
            name='vehicle_bridge_node',
            output='screen',
            parameters=[yaml_path, bridge_parameters(vehicle)],
            emulate_tty=True,
        ),
    ])
