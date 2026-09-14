"""Compose real-vehicle hardware and optional navigation.

Project-level vehicle geometry and limits come from a root config/vehicle.yaml
file supplied through the ``vehicle_config`` launch argument. Device/network
settings remain local to their owning drivers.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in {
        '1', 'true', 'yes', 'on'
    }


def _load_vehicle_config(path):
    if not path:
        raise RuntimeError(
            "vehicle_config is required when control or navigation is enabled"
        )
    if not os.path.isfile(path):
        raise RuntimeError(f"Vehicle config does not exist: {path}")
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    vehicle = data.get('vehicle', {})
    limits = vehicle.get('control_limits', {})
    required = {
        'max_forward_speed': limits.get('max_forward_speed'),
        'max_reverse_speed': limits.get('max_reverse_speed'),
        'max_steering_angle_deg': limits.get('max_steering_angle_deg'),
    }
    missing = [name for name, value in required.items() if value is None]
    if missing:
        raise RuntimeError(
            f"Vehicle config is missing control limits: {', '.join(missing)}"
        )
    return required


def _configured_components(context, bringup_share, motion_share):
    enable_control = _enabled(context, 'enable_control')
    enable_navigation = _enabled(context, 'enable_navigation')
    if not enable_control and not enable_navigation:
        return []

    vehicle_config = LaunchConfiguration('vehicle_config').perform(context)
    limits = _load_vehicle_config(vehicle_config)
    actions = []

    if enable_control:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(motion_share, 'launch', 'bridge.launch.py')
                ),
                launch_arguments={
                    'max_speed': str(limits['max_forward_speed']),
                    'max_reverse_speed': str(limits['max_reverse_speed']),
                    'max_steer_deg': str(limits['max_steering_angle_deg']),
                }.items(),
            )
        )

    if enable_navigation:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'map_pgm': LaunchConfiguration('map_pgm'),
                    'globalmap_pcd': LaunchConfiguration('globalmap_pcd'),
                    'vehicle_config': vehicle_config,
                    'params_file': LaunchConfiguration('localization_params_file'),
                    'points_topic': LaunchConfiguration('points_topic'),
                    'use_sim_time': 'false',
                    'use_rviz': LaunchConfiguration('navigation_rviz'),
                }.items(),
            )
        )

    return actions


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
            'vehicle_config',
            default_value='',
            description='Absolute path to project-level config/vehicle.yaml.',
        ),
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
                'Start real-vehicle localization/planning/navigation. Keep disabled '
                'until localization and sensor calibration are verified.'
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

    configured = OpaqueFunction(
        function=_configured_components,
        args=[bringup_share, motion_share],
    )

    return LaunchDescription(arguments + [lidar, imu, configured])
