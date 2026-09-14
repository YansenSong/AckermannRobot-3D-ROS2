"""Compose real-vehicle hardware and optional navigation.

This bringup intentionally keeps hardware parameters local to each driver for
now. It does not depend on a shared vehicle_config package.

All hardware/navigation components are opt-in because the final real-vehicle
parameter strategy is intentionally deferred. Enable only the components whose
local driver parameters have been configured and verified.

In particular, enable actuation explicitly with ``enable_control:=true``.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('ackermann_bringup')
    lidar_share = get_package_share_directory('lidar_driver')
    motion_share = get_package_share_directory('motion_control')

    default_lidar_config = os.path.join(lidar_share, 'config', 'config.yaml')
    default_localization_params = os.path.join(
        bringup_share, 'config', 'liorf_localization.yaml'
    )

    arguments = [
        DeclareLaunchArgument(
            'enable_lidar',
            default_value='false',
            description='Start the Hesai LiDAR driver.',
        ),
        DeclareLaunchArgument(
            'lidar_config',
            default_value=default_lidar_config,
            description='Path to lidar_driver YAML configuration.',
        ),
        DeclareLaunchArgument(
            'lidar_rviz',
            default_value='false',
            description='Start the LiDAR driver RViz window.',
        ),
        DeclareLaunchArgument(
            'enable_imu',
            default_value='false',
            description='Start the LPMS-IG1 RS485 IMU driver.',
        ),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_baudrate', default_value='115200'),
        DeclareLaunchArgument('imu_rate', default_value='200'),
        DeclareLaunchArgument('imu_rs485_control_pin', default_value='-1'),
        DeclareLaunchArgument('imu_rs485_toggle_wait_ms', default_value='2'),
        DeclareLaunchArgument(
            'enable_control',
            default_value='false',
            description='Start the real-vehicle UDP motion-control backend.',
        ),
        DeclareLaunchArgument(
            'enable_navigation',
            default_value='false',
            description=(
                'Start the backend-neutral navigation stack. Keep disabled '
                'until real-vehicle localization topics/parameters are ready.'
            ),
        ),
        DeclareLaunchArgument(
            'points_topic',
            default_value='/lidar_points',
            description='Point cloud used by pointcloud_to_laserscan.',
        ),
        DeclareLaunchArgument('navigation_rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('map_pgm', default_value=''),
        DeclareLaunchArgument('globalmap_pcd', default_value=''),
        DeclareLaunchArgument(
            'localization_params_file',
            default_value=default_localization_params,
        ),
    ]

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_share, 'launch', 'start.py')
        ),
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
        launch_arguments={
            'config_path': LaunchConfiguration('lidar_config'),
            'with_rviz': LaunchConfiguration('lidar_rviz'),
        }.items(),
    )

    # Launch the sensor node directly rather than the legacy lpms launch file:
    # that file adds an ``imu`` namespace even though the node already publishes
    # relative ``imu/...`` topics, producing /imu/imu/... names.
    imu = Node(
        package='lpms_ig1',
        executable='lpms_ig1_rs485_node',
        name='lpms_ig1_rs485_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_imu')),
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baudrate': ParameterValue(
                LaunchConfiguration('imu_baudrate'), value_type=int
            ),
            'rate': ParameterValue(
                LaunchConfiguration('imu_rate'), value_type=int
            ),
            'rs485ControlPin': ParameterValue(
                LaunchConfiguration('imu_rs485_control_pin'), value_type=int
            ),
            'rs485ControlPinToggleWaitMs': ParameterValue(
                LaunchConfiguration('imu_rs485_toggle_wait_ms'), value_type=int
            ),
            'use_sim_time': False,
        }],
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(motion_share, 'launch', 'bridge.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('enable_control')),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'navigation.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('enable_navigation')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'map_pgm': LaunchConfiguration('map_pgm'),
            'globalmap_pcd': LaunchConfiguration('globalmap_pcd'),
            'params_file': LaunchConfiguration('localization_params_file'),
            'points_topic': LaunchConfiguration('points_topic'),
            'use_sim_time': 'false',
            'use_rviz': LaunchConfiguration('navigation_rviz'),
        }.items(),
    )

    return LaunchDescription(arguments + [lidar, imu, control, navigation])
