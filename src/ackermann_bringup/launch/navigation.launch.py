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
    localization_arguments = ['globalmap_pcd', 'specify_init_pose', 'init_pos_x', 'init_pos_y',
                              'init_pos_z', 'init_ori_w', 'init_ori_x', 'init_ori_y', 'init_ori_z']
    arguments = [
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('map_pgm', default_value=''),
        DeclareLaunchArgument('globalmap_pcd', default_value=''),
        DeclareLaunchArgument('specify_init_pose', default_value='false'),
    ]
    arguments += [DeclareLaunchArgument(name, default_value=value) for name, value in (
        ('init_pos_x', '0.0'), ('init_pos_y', '0.0'), ('init_pos_z', '0.0'),
        ('init_ori_w', '1.0'), ('init_ori_x', '0.0'), ('init_ori_y', '0.0'), ('init_ori_z', '0.0'))]
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'localization.launch.py')),
        launch_arguments={name: LaunchConfiguration(name) for name in localization_arguments}.items())
    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'planning.launch.py')),
        launch_arguments={'map': LaunchConfiguration('map'), 'map_pgm': LaunchConfiguration('map_pgm')}.items())
    scan = Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan', output='screen',
                parameters=[os.path.join(share, 'config', 'pcl_to_scan.yaml')],
                remappings=[('cloud_in', '/points_raw'), ('scan', '/scan')])
    mux = Node(package='ackermann_control', executable='cmd_vel_mux.py', name='cmd_vel_mux',
               output='screen', parameters=[{'use_sim_time': True, 'active_planner': 'neupan'}])
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', os.path.join(share, 'rviz', 'nav2_default_view.rviz')])
    return LaunchDescription(arguments + [localization, planning, scan, mux, rviz])
