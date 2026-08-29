#!/usr/bin/env python3
"""Add Velodyne-style ring and per-point time fields to Gazebo point clouds.

gazebo_ros_ray_sensor publishes a flat PointCloud2 containing x/y/z/intensity.
LIO-SAM's Velodyne input path expects ring and time fields, so this node keeps
the original cloud on /points_raw and publishes an LIO-SAM compatible copy.

Gazebo does not expose the individual ray acquisition timestamps.  The time
field is therefore reconstructed from the point azimuth over one scan period.
This is a deterministic approximation for simulation, rather than hardware
timestamp data.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class GazeboLidarAdapter(Node):
    def __init__(self):
        super().__init__('gazebo_lidar_adapter')

        self.declare_parameter('input_topic', '/points_raw')
        self.declare_parameter('output_topic', '/points_lio')
        self.declare_parameter('scan_period', 0.05)  # 20 Hz Gazebo lidar
        self.declare_parameter('num_rings', 16)
        self.declare_parameter('vertical_min_deg', -15.0)
        self.declare_parameter('vertical_resolution_deg', 2.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.scan_period = float(self.get_parameter('scan_period').value)
        self.num_rings = int(self.get_parameter('num_rings').value)
        self.vertical_min_deg = float(self.get_parameter('vertical_min_deg').value)
        self.vertical_resolution_deg = float(
            self.get_parameter('vertical_resolution_deg').value)

        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Converting {input_topic} -> {output_topic}; '
            f'{self.num_rings} rings, scan period {self.scan_period:.3f}s')

    def cloud_callback(self, msg: PointCloud2):
        field_names = {field.name for field in msg.fields}
        required = {'x', 'y', 'z'}
        if not required.issubset(field_names):
            self.get_logger().error(
                'Input cloud must contain x, y and z fields; ignoring message')
            return

        names = ('x', 'y', 'z', 'intensity') if 'intensity' in field_names else (
            'x', 'y', 'z')
        converted = []

        for point in point_cloud2.read_points(
                msg, field_names=names, skip_nans=True):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            intensity = float(point[3]) if len(point) == 4 else 0.0

            horizontal_range = math.hypot(x, y)
            vertical_angle = math.degrees(math.atan2(z, horizontal_range))
            ring = int(round(
                (vertical_angle - self.vertical_min_deg) /
                self.vertical_resolution_deg))
            if ring < 0 or ring >= self.num_rings:
                continue

            # Reconstruct a scan-relative timestamp from azimuth.  The
            # modulo keeps the value in [0, scan_period), independent of the
            # angle convention used by the Gazebo sensor.
            azimuth = math.atan2(y, x)
            phase = ((azimuth + math.pi) / (2.0 * math.pi)) % 1.0
            relative_time = phase * self.scan_period
            converted.append((x, y, z, intensity, ring, relative_time))

        if not converted:
            return

        # Ensure the final point carries the largest timestamp.  LIO-SAM uses
        # the last point's time when determining the scan end time.
        converted.sort(key=lambda point: point[5])

        fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
            # PCL's PointXYZIRT layout places intensity after the 16-byte
            # XYZ/alignment block, followed by uint16 ring and float time.
            PointField(name='intensity', offset=16,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=20,
                       datatype=PointField.UINT16, count=1),
            PointField(name='time', offset=24,
                       datatype=PointField.FLOAT32, count=1),
        ]

        output = point_cloud2.create_cloud(msg.header, fields, converted)
        output.is_dense = True
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboLidarAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
