#!/usr/bin/env python3
"""Launch the real-vehicle LPMS-IG1-RS485 IMU driver.

Parameters are injected from src/vehicle_config/config/real_vehicle.yaml
(the single source of truth) via vehicle_config.imu_parameters().

Unlike the upstream lpms_ig1_rs485_launch.py template, this launch file:
  * reads port / baudrate / RS485 control pin from real_vehicle.yaml,
  * publishes the base_link -> imu static TF from real_vehicle.yaml (mount),
  * uses no namespace, so the node's relative topic "imu/data" resolves
    to /imu/data — matching the topic declared in real_vehicle.yaml.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from vehicle_config import imu_parameters, imu_transform, load_real_vehicle_config


def generate_launch_description():
    vehicle = load_real_vehicle_config()
    imu_mount = imu_transform(vehicle)

    # ---- Static TF: base_link -> imu ----
    # Mirrors the base_link -> hesai_lidar publisher in navigation.launch.py.
    # LIO-SAM's extrinsic params are computed from the same mount in
    # vehicle_config.lio_sam_extrinsics(), so TF and preintegration agree.
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=[
            '--x', str(imu_mount['x']),
            '--y', str(imu_mount['y']),
            '--z', str(imu_mount['z']),
            '--yaw', str(imu_mount['yaw']),
            '--pitch', str(imu_mount['pitch']),
            '--roll', str(imu_mount['roll']),
            '--frame-id', 'base_link',
            '--child-frame-id', vehicle['sensors']['imu']['frame_id'],
        ],
        parameters=[{'use_sim_time': False}],
    )

    imu_node = Node(
        package='lpms_ig1',
        executable='lpms_ig1_rs485_node',
        name='lpms_ig1_rs485_node',
        output='screen',
        emulate_tty=True,
        parameters=[imu_parameters(vehicle)],
    )

    return LaunchDescription([
        static_tf_base_to_imu,
        imu_node,
    ])
