"""Start Gazebo and, after robot initialization, only the Nav2 system."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    simulation_share = get_package_share_directory('ackermann_simulation')
    nav2_share = get_package_share_directory('ackermann_nav2')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'publish_ekf_tf': 'true',
            'use_rviz': 'false',
        }.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'nav2_navigation.launch.py')),
        launch_arguments={
            name: LaunchConfiguration(name) for name in (
                'map', 'globalmap_pcd', 'params_file', 'use_sim_time',
                'autostart', 'nav2_use_rviz', 'specify_init_pose', 'init_pos_x',
                'init_pos_y', 'init_pos_z', 'init_ori_w', 'init_ori_x',
                'init_ori_y', 'init_ori_z',
            )
        }.items(),
    )

    arguments = [
        DeclareLaunchArgument('map', description='Absolute path to the 2D map YAML'),
        DeclareLaunchArgument(
            'globalmap_pcd', description='Absolute path to the HDL localization PCD map'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav2_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('nav2_use_rviz', default_value='true'),
        DeclareLaunchArgument('specify_init_pose', default_value='false'),
    ]
    arguments += [DeclareLaunchArgument(name, default_value=value)
                  for name, value in (
                      ('init_pos_x', '0.0'), ('init_pos_y', '0.0'),
                      ('init_pos_z', '0.0'), ('init_ori_w', '1.0'),
                      ('init_ori_x', '0.0'), ('init_ori_y', '0.0'),
                      ('init_ori_z', '0.0'))]

    return LaunchDescription(arguments + [
        gazebo,
        TimerAction(period=5.0, actions=[nav2]),
    ])
