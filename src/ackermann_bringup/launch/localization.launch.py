"""Start real-vehicle LIORF prior-map localization and map-to-odom TF."""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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


def _load_vehicle_calibration(path):
    """Runtime LIORF overrides taken from config/vehicle.yaml.

    Covers the frame names as well as the LiDAR/IMU extrinsics, so the frames
    cannot drift between vehicle.yaml and the package-local parameter file.
    """
    if not path:
        raise RuntimeError(
            "localization.launch.py requires a non-empty 'vehicle_config' path"
        )
    if not os.path.isfile(path):
        raise RuntimeError(f"Vehicle config does not exist: {path}")

    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}

    vehicle = data.get('vehicle', {})
    extrinsics = vehicle.get('sensor_extrinsics', {}).get('lidar_to_imu', {})
    frames = vehicle.get('frames', {})

    status = str(extrinsics.get('status', '')).strip().upper()
    if status != 'VERIFIED':
        raise RuntimeError(
            "LiDAR/IMU extrinsics are not VERIFIED in config/vehicle.yaml; "
            "real localization will not start with stale/default calibration"
        )

    lidar_frame = str(frames.get('lidar', '')).strip()
    base_frame = str(frames.get('base', '')).strip()
    for label, name in (('lidar', lidar_frame), ('base', base_frame)):
        if not name:
            raise RuntimeError(f"vehicle.frames.{label} must not be empty")

    return {
        # LIORF publishes odometry in baselinkFrame and consumes the fixed
        # lidarFrame->baselinkFrame transform (see TransformFusion in
        # imuPreintegration.cpp). baselinkFrame is frames.base, NOT a separate
        # "base_link": nothing on the real vehicle defines where base_link
        # would be, and leaving it unset split the TF tree in two.
        'lidarFrame': lidar_frame,
        'baselinkFrame': base_frame,
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


def _localization_nodes(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    vehicle_config = LaunchConfiguration('vehicle_config').perform(context)
    globalmap_pcd = LaunchConfiguration('globalmap_pcd').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    calibration = _load_vehicle_calibration(vehicle_config)
    liorf_parameters = [
        params_file,
        {
            'use_sim_time': use_sim_time,
            'globalmap_pcd': globalmap_pcd,
            **calibration,
        },
    ]

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    image_projection = Node(
        package='liorf_localization',
        executable='liorf_localization_imageProjection',
        name='liorf_localization_imageProjection',
        parameters=liorf_parameters,
        output='screen',
    )
    imu_preintegration = Node(
        package='liorf_localization',
        executable='liorf_localization_imuPreintegration',
        name='liorf_localization_imuPreintegration',
        parameters=liorf_parameters,
        output='screen',
    )
    map_optimization = Node(
        package='liorf_localization',
        executable='liorf_localization_mapOptmization',
        name='liorf_localization_mapOptmization',
        parameters=liorf_parameters,
        output='screen',
    )

    return [
        map_to_odom,
        image_projection,
        imu_preintegration,
        map_optimization,
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('ackermann_bringup')
    default_params = os.path.join(
        bringup_share, 'config', 'liorf_localization.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'globalmap_pcd',
            default_value='',
            description='Absolute path to the real-vehicle GlobalMap.pcd',
        ),
        DeclareLaunchArgument(
            'vehicle_config',
            default_value='',
            description=(
                'Absolute path to root config/vehicle.yaml. VERIFIED LiDAR/IMU '
                'extrinsics are required.'
            ),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='LIORF real-vehicle algorithm parameter file',
        ),
        OpaqueFunction(function=_localization_nodes),
    ])
