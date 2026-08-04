#!/usr/bin/env python3
"""
Launch NeuPAN with real vehicle configuration.

Launches NeuPAN node + static TF publishers for the real vehicle.
NeuPAN requires the conda 'neupan' environment — use scripts/run_neupan.sh
if you cannot activate conda before launching.

The static TF publishers are bootstrap transforms needed until real
localization (hdl_localization + EKF) is ready:
  - map → base_link (identity): assumed initial pose at map origin
  - base_link → hesai_lidar: LiDAR mounting position on vehicle roof

Usage:
  # With conda env active:
  ros2 launch neupan_ros2 real_vehicle.launch.py

  # Or via the shell script (recommended):
  bash scripts/run_neupan.sh
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )

    pkg_share = get_package_share_directory('neupan_ros2')
    robot_config_dir = os.path.join(pkg_share, 'config', 'robots', 'real_vehicle')
    robot_config = os.path.join(robot_config_dir, 'robot.yaml')

    # ---- Static TF: map → base_link (bootstrap until localization is ready) ----
    # Without this, NeuPAN's _get_robot_transform() fails silently and
    # the node never publishes cmd_vel. Remove this when hdl_localization
    # + EKF provide real map→base_link transforms.
    static_tf_map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_base',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'base_link'
        ],
        parameters=[{'use_sim_time': False}],
    )

    # ---- Static TF: base_link → hesai_lidar ----
    # LiDAR mounting position. Adjust x/y/z to match actual LiDAR placement.
    # Typical: LiDAR centered on vehicle roof at ~0.3-0.5m height.
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.4',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'hesai_lidar'
        ],
        parameters=[{'use_sim_time': False}],
    )

    # ---- NeuPAN core node ----
    neupan_node = Node(
        package='neupan_ros2',
        executable='neupan_node',
        name='neupan_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            robot_config,
            {'robot_config_dir': robot_config_dir},
        ],
        # No topic remappings — robot.yaml sets cmd_vel_topic: /cmd_vel
    )

    # ---- RViz (optional) ----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'neupan_sim.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        static_tf_map_to_base,
        static_tf_base_to_lidar,
        neupan_node,
        rviz_node,
    ])
