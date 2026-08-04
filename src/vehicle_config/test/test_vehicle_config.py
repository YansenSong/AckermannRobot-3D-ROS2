import math
from pathlib import Path

import yaml

from vehicle_config import (
    bridge_parameters,
    hybrid_astar_parameters,
    load_real_vehicle_config,
    materialize_lidar_driver_config,
    minimum_turning_radius,
    neupan_robot_parameters,
    neupan_scan_parameters,
    pointcloud_to_laserscan_parameters,
)


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

    assert bridge_parameters(config)['wheelbase'] == vehicle['wheelbase']
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
    assert config['sensors']['imu']['installed'] is False
    assert config['sensors']['imu']['parameters'] == {}

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
