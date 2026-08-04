#!/usr/bin/env python3
"""
Real-vehicle navigation infrastructure launch.

Starts the full perception + planning + control bridge chain WITHOUT NeuPAN
(NeuPAN requires conda env and must be launched separately via run_neupan.sh).

Components:
  - lidar_driver           → /lidar_points (PointCloud2)
  - pointcloud_to_laserscan → /scan (LaserScan, from /lidar_points)
  - hybrid_astar_planner   → /plan (Path) + /map (OccupancyGrid)
  - motion_control bridge  → /cmd_vel → UDP → STM32

Usage:
  ros2 launch neupan_ros2 navigation.launch.py map_pgm:=/path/to/map.pgm

  # Optionally specify map.yaml to auto-resolve resolution/origin:
  ros2 launch neupan_ros2 navigation.launch.py \
      map_pgm:=/path/to/map.pgm map_yaml:=/path/to/map.yaml

Then in another terminal (with conda neupan env):
  bash scripts/run_neupan.sh
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context):
    map_pgm = LaunchConfiguration('map_pgm').perform(context)
    map_yaml_path = LaunchConfiguration('map_yaml').perform(context)

    # Resolve resolution and origin from map.yaml if provided
    resolution = 0.05
    origin_x = 0.0
    origin_y = 0.0
    if map_yaml_path and os.path.isfile(map_yaml_path):
        try:
            with open(map_yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            resolution = float(data.get('resolution', resolution))
            origin = data.get('origin', [0.0, 0.0, 0.0])
            origin_x = float(origin[0])
            origin_y = float(origin[1])
            if not map_pgm:
                map_dir = os.path.dirname(map_yaml_path)
                image_rel = data.get('image', 'map.pgm')
                map_pgm = os.path.join(map_dir, image_rel)
        except Exception as e:
            print(f"[WARN] Failed to parse {map_yaml_path}: {e}, using defaults")

    pkg_neupan = get_package_share_directory('neupan_ros2')
    pkg_hybrid = get_package_share_directory('hybrid_astar_planner')

    # ---- 1. Hesai LiDAR driver (node only, no rviz2) ----
    lidar_node = Node(
        package='lidar_driver',
        executable='lidar_driver_node',
        name='lidar_driver',
        output='screen',
    )

    # ---- 2. PointCloud2 → LaserScan ----
    pcl_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[os.path.join(
            pkg_neupan, 'config', 'robots', 'real_vehicle', 'pcl_to_scan.yaml'
        )],
        remappings=[
            ('cloud_in', '/lidar_points'),
            ('scan', '/scan'),
        ],
    )

    # ---- 3. Hybrid A* global planner (real vehicle kinematics) ----
    hybrid_planner = Node(
        package='hybrid_astar_planner',
        executable='hybrid_astar_planner_node',
        name='hybrid_astar_planner',
        output='screen',
        parameters=[os.path.join(
            pkg_hybrid, 'config', 'planner_params_real.yaml'
        ), {
            'use_sim_time': False,
            'map_path': map_pgm,
            'resolution': resolution,
            'origin_x': origin_x,
            'origin_y': origin_y,
        }],
    )

    # ---- 4. Motion control bridge ----
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('motion_control'),
            '/launch/bridge.launch.py'
        ])
    )

    return [
        lidar_node,
        pcl_to_scan,
        hybrid_planner,
        bridge_launch,
    ]


def generate_launch_description():
    map_pgm_arg = DeclareLaunchArgument(
        'map_pgm',
        default_value='',
        description='Absolute path to PGM map file for hybrid_astar_planner'
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='',
        description='Path to map.yaml (for resolution/origin). Overrides defaults.'
    )

    return LaunchDescription([
        map_pgm_arg,
        map_yaml_arg,
        OpaqueFunction(function=launch_setup),
    ])
