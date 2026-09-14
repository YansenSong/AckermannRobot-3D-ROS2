import math
from pathlib import Path

import yaml

from vehicle_config import (
    bridge_parameters,
    hybrid_astar_parameters,
    imu_transform,
    lidar_transform,
    lio_sam_extrinsics,
    load_real_vehicle_config,
    materialize_lidar_driver_config,
    minimum_turning_radius,
    neupan_robot_parameters,
    neupan_scan_parameters,
    pointcloud_to_laserscan_parameters,
)


def _rpy_matrix(roll, pitch, yaw):
    """Independent Rz(yaw) @ Ry(pitch) @ Rx(roll) reconstruction (row-major)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        cy * cp,                    cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp,                    sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp,                        cp * sr,                cp * cr,
    ]


def _mat_vec(m, v):
    """Multiply a row-major 3x3 matrix m by a length-3 vector v."""
    return [
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
        m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
    ]


CONFIG_FILE = Path(__file__).parents[1] / 'config' / 'real_vehicle.yaml'
WORKSPACE = Path(__file__).parents[3]


def test_real_vehicle_parameter_mappings():
    config = load_real_vehicle_config(str(CONFIG_FILE))
    vehicle = config['vehicle']

    neupan = neupan_robot_parameters(config)
    assert neupan['length'] == vehicle['length']
    assert neupan['width'] == vehicle['width']
    assert neupan['wheelbase'] == vehicle['wheelbase']
    assert neupan['max_speed'][0] == vehicle['max_forward_speed']
    assert math.isclose(
        neupan['max_speed'][1], math.radians(vehicle['max_steer_deg']))

    bridge = bridge_parameters(config)
    assert bridge['max_speed'] == vehicle['max_forward_speed']
    assert bridge['max_reverse_speed'] == vehicle['max_reverse_speed']
    assert bridge['max_steer_deg'] == vehicle['max_steer_deg']
    assert hybrid_astar_parameters(config)['vehicle_width'] == vehicle['width']
    assert math.isclose(
        minimum_turning_radius(config),
        vehicle['wheelbase'] / math.tan(math.radians(vehicle['max_steer_deg'])),
    )


def test_real_sensor_parameter_mappings():
    config = load_real_vehicle_config(str(CONFIG_FILE))
    lidar = config['sensors']['lidar']
    scan = lidar['scan']

    neupan = neupan_scan_parameters(config)
    pointcloud = pointcloud_to_laserscan_parameters(config)
    assert neupan['scan_topic'] == lidar['topics']['scan']
    assert neupan['scan_range_min'] == scan['range_min']
    assert neupan['scan_range_max'] == scan['range_max']
    assert pointcloud['target_frame'] == lidar['frames']['scan_target']
    assert pointcloud['angle_increment'] == scan['angle_increment']
    imu = config['sensors']['imu']
    assert imu['installed'] is True
    assert imu['model'] == 'LPMS-IG1-RS485'
    assert imu['frame_id'] == 'imu'
    assert imu['topic'] == '/imu/data'
    # 2026-08-08 实测安装位姿：x/y/z 卷尺测量，roll/pitch 5min 静止加速度标定
    assert imu['mount']['x'] == 0.13
    assert imu['mount']['y'] == 0.0
    assert imu['mount']['z'] == 0.26
    assert imu['mount']['roll'] == -0.006945
    assert imu['mount']['pitch'] == -0.006617
    assert imu['mount']['yaw'] == 0.0
    assert imu['parameters']['baudrate'] == 115200
    assert imu['parameters']['port'] == '/dev/ttyUSB0'

def test_imu_and_lidar_transforms():
    config = load_real_vehicle_config(str(CONFIG_FILE))
    imu = config['sensors']['imu']
    mount = imu['mount']

    imu_tf = imu_transform(config)
    assert set(imu_tf) == {'x', 'y', 'z', 'roll', 'pitch', 'yaw'}
    for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        assert isinstance(imu_tf[key], float)
        assert imu_tf[key] == mount[key]

    lidar_tf = lidar_transform(config)
    assert set(lidar_tf) == {'x', 'y', 'z', 'roll', 'pitch', 'yaw'}


def test_lio_sam_extrinsics():
    config = load_real_vehicle_config(str(CONFIG_FILE))
    imu = config['sensors']['imu']['mount']
    lidar = config['sensors']['lidar']['mount']

    ext = lio_sam_extrinsics(config)
    assert set(ext) == {'extrinsicTrans', 'extrinsicRot', 'extrinsicRPY'}
    assert len(ext['extrinsicTrans']) == 3
    assert len(ext['extrinsicRot']) == 9
    assert len(ext['extrinsicRPY']) == 9

    # extrinsicRot / extrinsicRPY must equal Rz(yaw) @ Ry(pitch) @ Rx(roll),
    # the same rpy convention the base_link -> imu static TF uses.
    expected = _rpy_matrix(imu['roll'], imu['pitch'], imu['yaw'])
    assert ext['extrinsicRot'] == expected
    assert ext['extrinsicRPY'] == expected

    # R must be orthonormal: R @ R^T == I and det(R) == 1.
    for r in (ext['extrinsicRot'],):
        for i in range(3):
            for j in range(3):
                dot = sum(r[i * 3 + k] * r[j * 3 + k] for k in range(3))
                assert math.isclose(dot, 1.0 if i == j else 0.0, abs_tol=1e-12)
    det = (
        expected[0] * (expected[4] * expected[8] - expected[5] * expected[7])
        - expected[1] * (expected[3] * expected[8] - expected[5] * expected[6])
        + expected[2] * (expected[3] * expected[7] - expected[4] * expected[6])
    )
    assert math.isclose(det, 1.0, abs_tol=1e-12)

    # extrinsicTrans = R_base_imu^T @ (lidar - imu), the lidar origin in IMU frame.
    diff = [lidar['x'] - imu['x'], lidar['y'] - imu['y'], lidar['z'] - imu['z']]
    r_transpose = [expected[i * 3 + j] for j in range(3) for i in range(3)]
    expected_trans = _mat_vec(r_transpose, diff)
    for got, want in zip(ext['extrinsicTrans'], expected_trans):
        assert math.isclose(got, want, abs_tol=1e-12)


def test_real_sensor_parameter_mappings():
    config = load_real_vehicle_config(str(CONFIG_FILE))
    lidar = config['sensors']['lidar']
    scan = lidar['scan']

    neupan = neupan_scan_parameters(config)
    pointcloud = pointcloud_to_laserscan_parameters(config)
    assert neupan['scan_topic'] == lidar['topics']['scan']
    assert neupan['scan_range_min'] == scan['range_min']
    assert neupan['scan_range_max'] == scan['range_max']
    assert pointcloud['target_frame'] == lidar['frames']['scan_target']
    assert pointcloud['angle_increment'] == scan['angle_increment']
    imu = config['sensors']['imu']
    assert imu['installed'] is True
    assert imu['model'] == 'LPMS-IG1-RS485'
    assert imu['frame_id'] == 'imu'
    assert imu['topic'] == '/imu/data'
    # 2026-08-08 实测安装位姿：x/y/z 卷尺测量，roll/pitch 5min 静止加速度标定
    assert imu['mount']['x'] == 0.13
    assert imu['mount']['y'] == 0.0
    assert imu['mount']['z'] == 0.26
    assert imu['mount']['roll'] == -0.006945
    assert imu['mount']['pitch'] == -0.006617
    assert imu['mount']['yaw'] == 0.0
    assert imu['parameters']['baudrate'] == 115200
    assert imu['parameters']['port'] == '/dev/ttyUSB0'

    base_config = WORKSPACE / 'src/lidar_driver/config/config.yaml'
    generated_path = materialize_lidar_driver_config(config, str(base_config))
    with open(generated_path, 'r', encoding='utf-8') as stream:
        generated = yaml.safe_load(stream)
    vendor_lidar = generated['lidar'][0]
    assert (
        vendor_lidar['driver']['lidar_udp_type']['device_ip_address'] ==
        lidar['driver']['device_ip_address']
    )
    assert vendor_lidar['ros']['ros_frame_id'] == lidar['frames']['sensor']
    assert vendor_lidar['ros']['send_imu_ros'] is False
