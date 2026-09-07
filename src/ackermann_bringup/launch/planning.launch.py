"""Start the Hybrid A* global planner only."""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _planner(context):
    map_yaml = LaunchConfiguration('map').perform(context)
    map_pgm = LaunchConfiguration('map_pgm').perform(context)
    resolution, origin_x, origin_y = 0.05, 0.0, 0.0
    if map_yaml:
        with open(map_yaml, encoding='utf-8') as stream:
            metadata = yaml.safe_load(stream)
        resolution = float(metadata.get('resolution', resolution))
        origin_x, origin_y = (float(value) for value in metadata.get('origin', [0.0, 0.0])[:2])
        if not map_pgm:
            map_pgm = os.path.join(os.path.dirname(map_yaml), metadata.get('image', 'map.pgm'))
    params = os.path.join(get_package_share_directory('hybrid_astar_planner'), 'config', 'planner_params.yaml')
    return [Node(package='hybrid_astar_planner', executable='hybrid_astar_planner_node',
                 name='hybrid_astar_planner', output='screen', parameters=[params, {
                     'use_sim_time': True, 'map_path': map_pgm, 'resolution': resolution,
                     'origin_x': origin_x, 'origin_y': origin_y,
                 }])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('map_pgm', default_value=''),
        OpaqueFunction(function=_planner),
    ])
