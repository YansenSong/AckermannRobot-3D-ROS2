"""
hybrid_astar_planner.launch.py

启动 Hybrid A* 全局路径规划器节点。
用法:
  ros2 launch hybrid_astar_planner hybrid_astar_planner.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('hybrid_astar_planner')
    config_path = os.path.join(pkg_dir, 'config', 'planner_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_path,
            description='Path to YAML parameter file'
        ),

        Node(
            package='hybrid_astar_planner',
            executable='hybrid_astar_planner_node',
            name='hybrid_astar_planner',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
