#!/usr/bin/env python3
"""Launch the real-vehicle command gate and STM32 hardware bridge."""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in {
        '1', 'true', 'yes', 'on'
    }


def _require_finite_number(mapping, key, section):
    if key not in mapping:
        raise RuntimeError(f'Missing {section}.{key}')
    value = float(mapping[key])
    if not math.isfinite(value):
        raise RuntimeError(f'{section}.{key} must be finite')
    return value


def _load_control_limits(path):
    if not path:
        raise RuntimeError(
            "vehicle_config is required when the STM32 bridge is enabled"
        )
    if not os.path.isfile(path):
        raise RuntimeError(f'Vehicle config does not exist: {path}')

    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}

    limits = data.get('vehicle', {}).get('control_limits', {})
    section = 'vehicle.control_limits'
    max_speed = _require_finite_number(
        limits, 'max_forward_speed', section
    )
    max_reverse_speed = _require_finite_number(
        limits, 'max_reverse_speed', section
    )
    max_steer_deg = _require_finite_number(
        limits, 'max_steering_angle_deg', section
    )

    if max_speed < 0.0:
        raise RuntimeError(f'{section}.max_forward_speed must be >= 0')
    if max_reverse_speed > 0.0:
        raise RuntimeError(f'{section}.max_reverse_speed must be <= 0')
    if max_steer_deg < 0.0:
        raise RuntimeError(f'{section}.max_steering_angle_deg must be >= 0')

    return {
        'max_speed': max_speed,
        'max_reverse_speed': max_reverse_speed,
        'max_steer_deg': max_steer_deg,
    }


def _resolve_bridge_params(context):
    path = LaunchConfiguration('bridge_params_file').perform(context)
    if not path:
        raise RuntimeError(
            'bridge_params_file is required when enable_stm32_bridge=true'
        )
    if not os.path.isfile(path):
        raise RuntimeError(f'Bridge params file does not exist: {path}')
    return path


def _configured_nodes(context):
    enable_gate = _enabled(context, 'enable_command_gate')
    enable_bridge = _enabled(context, 'enable_stm32_bridge')
    if not enable_gate and not enable_bridge:
        return []

    use_sim_time = LaunchConfiguration('use_sim_time')
    actions = []

    if enable_gate:
        actions.append(
            Node(
                package='motion_interface',
                executable='command_gate',
                name='motion_command_gate',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'input_topic': LaunchConfiguration('input_topic'),
                    'output_topic': LaunchConfiguration('output_topic'),
                }],
            )
        )

    if enable_bridge:
        vehicle_config = LaunchConfiguration('vehicle_config').perform(context)
        limits = _load_control_limits(vehicle_config)
        bridge_params = _resolve_bridge_params(context)
        actions.append(
            Node(
                package='motion_interface',
                executable='stm32_bridge',
                name='stm32_vehicle_bridge',
                output='screen',
                parameters=[
                    bridge_params,
                    {
                        'use_sim_time': use_sim_time,
                        **limits,
                    },
                ],
                emulate_tty=True,
            )
        )

    return actions


def generate_launch_description():
    package_share = get_package_share_directory('motion_interface')

    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_config',
            default_value='',
            description=(
                'Absolute path to project-level config/vehicle.yaml. Required '
                'when enable_stm32_bridge=true.'
            ),
        ),
        DeclareLaunchArgument('enable_command_gate', default_value='true'),
        DeclareLaunchArgument('enable_stm32_bridge', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/neupan_cmd_vel_raw',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/ackermann_cmd',
        ),
        DeclareLaunchArgument(
            'bridge_params_file',
            default_value=os.path.join(
                package_share, 'config', 'bridge_params.yaml'
            ),
            description=(
                'Path to the STM32 bridge YAML (UDP host/port, bind device, '
                'enable mask, control timing). Defaults to the installed copy; '
                'callers pass an explicit path to use a workspace-local file.'
            ),
        ),
        OpaqueFunction(
            function=_configured_nodes,
        ),
    ])
