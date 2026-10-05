"""Compose localization, global planning, scan conversion, command muxing, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ackermann_bringup')
    nav_status_share = get_package_share_directory('nav_status')
    arguments = [
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('map_pgm', default_value=''),
        DeclareLaunchArgument('globalmap_pcd', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                share, 'config', 'liorf_localization.yaml')),
    ]
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'localization.launch.py')),
        launch_arguments={
            'globalmap_pcd': LaunchConfiguration('globalmap_pcd'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
        }.items())
    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'planning.launch.py')),
        launch_arguments={'map': LaunchConfiguration('map'), 'map_pgm': LaunchConfiguration('map_pgm')}.items())
    scan = Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan', output='screen',
                parameters=[os.path.join(share, 'config', 'pcl_to_scan.yaml')],
                remappings=[('cloud_in', '/points_raw'), ('scan', '/scan')])
    adapter = Node(
        package='ackermann_control',
        executable='neupan_ackermann_adapter.py',
        name='neupan_ackermann_adapter',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wheelbase': 0.593,
            'input_topic': '/neupan_cmd_vel_raw',
            'output_topic': '/neupan_cmd_vel',
        }])
    mux = Node(package='ackermann_control', executable='cmd_vel_mux.py', name='cmd_vel_mux',
               output='screen', parameters=[{'use_sim_time': True}])
    nav_status = Node(
        package='nav_status', executable='nav_status_node', name='nav_status_node',
        output='screen',
        parameters=[
            os.path.join(nav_status_share, 'config', 'nav_status.yaml'),
            {'use_sim_time': True},
        ])
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', os.path.join(share, 'rviz', 'nav2_default_view.rviz')])
    return LaunchDescription(
        arguments + [localization, planning, scan, adapter, mux, nav_status, rviz])
