"""Start the Humble Nav2 Smac planner and its Ackermann bridge."""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterFile
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _planner(context):
    map_yaml = LaunchConfiguration('map').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    if not map_yaml:
        raise RuntimeError("planning.launch.py requires a non-empty 'map' YAML path")

    with open(map_yaml, encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream) or {}
    resolution = float(metadata.get('resolution', 0.05))
    if resolution <= 0.0:
        raise RuntimeError(f"Map resolution must be positive, got {resolution}")

    params = os.path.join(
        get_package_share_directory('ackermann_bringup'),
        'config', 'smac_planner.yaml')
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
                'global_costmap.global_costmap.resolution': str(resolution),
            },
            convert_types=True),
        allow_substs=True)

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[configured_params])
    planner_server = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen', parameters=[configured_params])
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_planning', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'planner_server'],
        }])
    bridge = Node(
        package='ackermann_smac_bridge', executable='ackermann_smac_bridge',
        name='ackermann_smac_bridge', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'global_frame': 'map',
            'robot_frame': 'rear_axle_link',
            'planner_action': '/compute_path_to_pose',
            'planner_id': 'GridBased',
        }])
    return [map_server, planner_server, lifecycle_manager, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('map_pgm', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_planner),
    ])
