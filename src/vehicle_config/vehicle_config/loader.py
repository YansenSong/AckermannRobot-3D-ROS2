"""Load and map the single source of truth for the real vehicle."""

import copy
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory


REQUIRED_VEHICLE_FIELDS = (
    'length',
    'width',
    'wheelbase',
    'wheel_radius',
    'rear_axle_offset_x',
    'max_steer_deg',
    'max_forward_speed',
    'max_reverse_speed',
)
REQUIRED_LIDAR_DRIVER_FIELDS = (
    'device_ip_address',
    'udp_port',
    'ptc_port',
    'correction_file_path',
    'firetimes_path',
    'frame_frequency',
    'use_timestamp_type',
)
REQUIRED_LIDAR_SCAN_FIELDS = (
    'angle_min',
    'angle_max',
    'angle_increment',
    'scan_time',
    'range_min',
    'range_max',
    'downsample',
    'min_height',
    'max_height',
)
REQUIRED_MOUNT_FIELDS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def _default_config_path():
    package_dir = get_package_share_directory('vehicle_config')
    return os.path.join(package_dir, 'config', 'real_vehicle.yaml')


def load_real_vehicle_config(config_file=None):
    """Return the validated real-vehicle configuration dictionary."""
    path = config_file or _default_config_path()
    with open(path, 'r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    if not isinstance(config, dict):
        raise ValueError(f'Invalid vehicle configuration: {path}')

    vehicle = config.get('vehicle', {})
    sensors = config.get('sensors', {})
    lidar = sensors.get('lidar', {})
    driver = lidar.get('driver', {})
    scan = lidar.get('scan', {})
    mount = lidar.get('mount', {})
    frames = lidar.get('frames', {})
    topics = lidar.get('topics', {})
    missing_vehicle = [key for key in REQUIRED_VEHICLE_FIELDS if key not in vehicle]
    missing_lidar = [
        *[f'driver.{key}' for key in REQUIRED_LIDAR_DRIVER_FIELDS if key not in driver],
        *[f'scan.{key}' for key in REQUIRED_LIDAR_SCAN_FIELDS if key not in scan],
        *[f'mount.{key}' for key in REQUIRED_MOUNT_FIELDS if key not in mount],
        *[f'frames.{key}' for key in ('sensor', 'scan_target') if key not in frames],
        *[f'topics.{key}' for key in ('packets', 'points', 'packet_loss', 'scan')
          if key not in topics],
    ]
    if missing_vehicle or missing_lidar:
        missing = missing_vehicle + [f'sensors.lidar.{key}' for key in missing_lidar]
        raise ValueError(f'Missing vehicle configuration fields: {", ".join(missing)}')

    for key in REQUIRED_VEHICLE_FIELDS:
        vehicle[key] = float(vehicle[key])
    for key in REQUIRED_MOUNT_FIELDS:
        mount[key] = float(mount[key])
    for key in REQUIRED_LIDAR_SCAN_FIELDS:
        scan[key] = int(scan[key]) if key == 'downsample' else float(scan[key])
    for key in ('udp_port', 'ptc_port'):
        driver[key] = int(driver[key])
    driver['frame_frequency'] = float(driver['frame_frequency'])

    positive_fields = ('length', 'width', 'wheelbase', 'wheel_radius', 'max_steer_deg')
    if any(vehicle[key] <= 0.0 for key in positive_fields):
        raise ValueError(f'Physical dimensions and steering limit must be positive: {path}')
    if vehicle['max_forward_speed'] <= 0.0 or vehicle['max_reverse_speed'] >= 0.0:
        raise ValueError(f'Forward/reverse speed signs are invalid: {path}')
    if scan['range_min'] < 0.0 or scan['range_max'] <= scan['range_min']:
        raise ValueError(f'LiDAR scan range is invalid: {path}')
    if scan['angle_max'] <= scan['angle_min'] or scan['downsample'] < 1:
        raise ValueError(f'LiDAR scan angle/downsample is invalid: {path}')

    config['_config_file'] = path
    return config


def minimum_turning_radius(config):
    """Calculate the kinematic minimum turning radius in metres."""
    vehicle = config['vehicle']
    return vehicle['wheelbase'] / math.tan(math.radians(vehicle['max_steer_deg']))


def bridge_parameters(config):
    """Map shared vehicle fields to motion_control ROS parameters."""
    vehicle = config['vehicle']
    return {
        'max_speed': vehicle['max_forward_speed'],
        'max_reverse_speed': vehicle['max_reverse_speed'],
        'max_steer_deg': vehicle['max_steer_deg'],
    }


def hybrid_astar_parameters(config):
    """Map shared vehicle fields to Hybrid A* ROS parameters."""
    vehicle = config['vehicle']
    return {
        'minimum_turning_radius': minimum_turning_radius(config),
        'rear_axle_offset_x': vehicle['rear_axle_offset_x'],
        'vehicle_length': vehicle['length'],
        'vehicle_width': vehicle['width'],
    }


def neupan_robot_parameters(config):
    """Map shared vehicle fields to the NeuPAN robot configuration."""
    vehicle = config['vehicle']
    steer_rad = math.radians(vehicle['max_steer_deg'])
    return {
        'length': vehicle['length'],
        'width': vehicle['width'],
        'wheelbase': vehicle['wheelbase'],
        'max_speed': [vehicle['max_forward_speed'], steer_rad],
        'min_speed': [vehicle['max_reverse_speed'], -steer_rad],
    }


def neupan_ipath_parameters(config):
    """Map shared vehicle fields to the NeuPAN initial-path configuration."""
    return {'min_radius': minimum_turning_radius(config)}


def lidar_transform(config):
    """Return the base_link-to-LiDAR static transform fields."""
    return dict(config['sensors']['lidar']['mount'])


def imu_transform(config):
    """Return the base_link-to-IMU static transform fields."""
    return dict(config['sensors']['imu']['mount'])


def lio_sam_extrinsics(config):
    """Map the LiDAR/IMU mounts to LIO-SAM extrinsic parameters.

    LIO-SAM's imuConverter (include/lio_sam/utility.hpp) rotates IMU-frame
    vectors into the body/lidar frame with a 3x3 row-major matrix, and folds
    the same rotation into the IMU orientation via its quaternion:

        acc = extRot * acc;  gyr = extRot * gyr;
        q_body_world = q_imu_world * extQRPY   # extQRPY = quat(extrinsicRPY)

    So both extrinsicRot and extrinsicRPY must equal the base_link-to-IMU
    rotation R_base_imu = Rz(yaw) * Ry(pitch) * Rx(roll), which is the same
    rpy convention as static_transform_publisher. extrinsicTrans is the lidar
    origin expressed in the IMU frame, i.e. R_base_imu^T * (lidar - imu).
    """
    imu = config['sensors']['imu']['mount']
    lidar = config['sensors']['lidar']['mount']

    roll, pitch, yaw = imu['roll'], imu['pitch'], imu['yaw']
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # R_base_imu = Rz(yaw) @ Ry(pitch) @ Rx(roll), row-major
    rot = [
        cy * cp,                    cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp,                    sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp,                        cp * sr,                cp * cr,
    ]

    dx = lidar['x'] - imu['x']
    dy = lidar['y'] - imu['y']
    dz = lidar['z'] - imu['z']
    # extrinsicTrans = R_base_imu^T @ [dx, dy, dz]  (lidar origin in IMU frame)
    trans = [
        rot[0] * dx + rot[3] * dy + rot[6] * dz,
        rot[1] * dx + rot[4] * dy + rot[7] * dz,
        rot[2] * dx + rot[5] * dy + rot[8] * dz,
    ]

    return {
        'extrinsicTrans': trans,
        'extrinsicRot': rot,
        'extrinsicRPY': rot,
    }


def imu_parameters(config):
    """Map shared IMU fields to the LPMS-IG1-RS485 driver ROS parameters."""
    imu = config['sensors']['imu']
    params = imu['parameters']
    return {
        'port': params['port'],
        'baudrate': params['baudrate'],
        'frame_id': imu['frame_id'],
        'rate': params['rate'],
        'rs485ControlPin': params['rs485_control_pin'],
    }


def neupan_scan_parameters(config):
    """Map shared LiDAR scan fields to NeuPAN parameters."""
    lidar = config['sensors']['lidar']
    scan = lidar['scan']
    return {
        'lidar_frame': lidar['frames']['scan_target'],
        'scan_topic': lidar['topics']['scan'],
        'scan_angle_min': scan['angle_min'],
        'scan_angle_max': scan['angle_max'],
        'scan_downsample': scan['downsample'],
        'scan_range_min': scan['range_min'],
        'scan_range_max': scan['range_max'],
    }


def pointcloud_to_laserscan_parameters(config):
    """Map shared LiDAR fields to pointcloud_to_laserscan parameters."""
    lidar = config['sensors']['lidar']
    scan = lidar['scan']
    return {
        'target_frame': lidar['frames']['scan_target'],
        'min_height': scan['min_height'],
        'max_height': scan['max_height'],
        'angle_min': scan['angle_min'],
        'angle_max': scan['angle_max'],
        'angle_increment': scan['angle_increment'],
        'scan_time': scan['scan_time'],
        'range_min': scan['range_min'],
        'range_max': scan['range_max'],
    }


def materialize_lidar_driver_config(config, base_config_file):
    """Create the vendor driver YAML after injecting shared sensor fields."""
    with open(base_config_file, 'r', encoding='utf-8') as stream:
        driver_config = copy.deepcopy(yaml.safe_load(stream))

    lidar = config['sensors']['lidar']
    shared_driver = lidar['driver']
    vendor_lidar = driver_config['lidar'][0]
    vendor_driver = vendor_lidar['driver']
    vendor_udp = vendor_driver['lidar_udp_type']
    vendor_ros = vendor_lidar['ros']

    for key in (
        'device_ip_address', 'udp_port', 'ptc_port',
        'correction_file_path', 'firetimes_path',
    ):
        vendor_udp[key] = shared_driver[key]
    vendor_driver['default_frame_frequency'] = shared_driver['frame_frequency']
    vendor_driver['use_timestamp_type'] = shared_driver['use_timestamp_type']

    topics = lidar['topics']
    vendor_ros.update({
        'ros_frame_id': lidar['frames']['sensor'],
        'ros_recv_packet_topic': topics['packets'],
        'ros_send_packet_topic': topics['packets'],
        'ros_send_point_cloud_topic': topics['points'],
        'ros_send_packet_loss_topic': topics['packet_loss'],
        'ros_send_imu_topic': '',
        'send_imu_ros': False,
    })

    output_dir = '/tmp/ackermann_vehicle_config'
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, f'lidar_driver_{os.getpid()}.yaml')
    temporary_path = output_path + '.tmp'
    with open(temporary_path, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(driver_config, stream, sort_keys=False, allow_unicode=True)
    os.replace(temporary_path, output_path)
    return output_path
