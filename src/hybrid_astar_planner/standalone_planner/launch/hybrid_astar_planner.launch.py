"""
hybrid_astar_planner.launch.py

启动 Hybrid A* 全局路径规划器节点。
用法:
  ros2 launch hybrid_astar_planner hybrid_astar_planner.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from vehicle_config import hybrid_astar_parameters, load_real_vehicle_config


def launch_setup(context):
    config_file = LaunchConfiguration('config_file').perform(context)
    vehicle = load_real_vehicle_config()

    return [
        Node(
            package='hybrid_astar_planner',
            executable='hybrid_astar_planner_node',
            name='hybrid_astar_planner',
            output='screen',
            parameters=[
                config_file,
                hybrid_astar_parameters(vehicle),
                {'use_sim_time': False},
            ],
        ),
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('hybrid_astar_planner')
    config_path = os.path.join(pkg_dir, 'config', 'planner_params_real.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_path,
            description='Path to YAML parameter file'
        ),

        OpaqueFunction(function=launch_setup),
    ])
