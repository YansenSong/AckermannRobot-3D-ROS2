"""Start LIO-SAM mapping with project-level real-vehicle sensor extrinsics."""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _numeric_list(value, length, field):
    if not isinstance(value, list) or len(value) != length:
        raise RuntimeError(
            f"{field} must be a list with exactly {length} numeric values"
        )
    result = []
    for item in value:
        number = float(item)
        if not math.isfinite(number):
            raise RuntimeError(f"{field} must contain only finite numbers")
        result.append(number)
    return result


def _load_sensor_extrinsics(path):
    if not path:
        raise RuntimeError(
            "lio_sam run.launch.py requires a non-empty 'vehicle_config' path"
        )
    if not os.path.isfile(path):
        raise RuntimeError(f"Vehicle config does not exist: {path}")

    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}

    extrinsics = (
        data.get('vehicle', {})
        .get('sensor_extrinsics', {})
        .get('lidar_to_imu', {})
    )
    status = str(extrinsics.get('status', '')).strip().upper()
    if status != 'VERIFIED':
        raise RuntimeError(
            "LiDAR/IMU extrinsics are not VERIFIED in config/vehicle.yaml; "
            "LIO-SAM will not start with stale/default calibration"
        )

    return {
        'extrinsicTrans': _numeric_list(
            extrinsics.get('translation_xyz'), 3,
            'vehicle.sensor_extrinsics.lidar_to_imu.translation_xyz',
        ),
        'extrinsicRot': _numeric_list(
            extrinsics.get('rotation_matrix'), 9,
            'vehicle.sensor_extrinsics.lidar_to_imu.rotation_matrix',
        ),
        'extrinsicRPY': _numeric_list(
            extrinsics.get('orientation_matrix'), 9,
            'vehicle.sensor_extrinsics.lidar_to_imu.orientation_matrix',
        ),
    }


def _lio_sam_nodes(context, share_dir):
    parameter_file = LaunchConfiguration('params_file').perform(context)
    vehicle_config = LaunchConfiguration('vehicle_config').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    extrinsics = _load_sensor_extrinsics(vehicle_config)
    runtime_parameters = {
        'use_sim_time': use_sim_time,
        **extrinsics,
    }
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        # robot_state_publisher is provided by the vehicle bringup / TF setup.
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[parameter_file, runtime_parameters],
            output='screen',
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file, runtime_parameters],
            output='screen',
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file, runtime_parameters],
            output='screen',
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file, runtime_parameters],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ]


def generate_launch_description():
    share_dir = get_package_share_directory('lio_sam')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share_dir, 'config', 'params.yaml'),
            description='Path to the LIO-SAM algorithm/sensor parameter file.',
        ),
        DeclareLaunchArgument(
            'vehicle_config',
            default_value='',
            description=(
                'Absolute path to root config/vehicle.yaml. VERIFIED LiDAR/IMU '
                'extrinsics are required.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use ROS simulation clock. Real vehicle default is false.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start the LIO-SAM RViz window.',
        ),
        OpaqueFunction(
            function=_lio_sam_nodes,
            args=[share_dir],
        ),
    ])
