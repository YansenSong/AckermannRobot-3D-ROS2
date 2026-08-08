import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            namespace = 'imu',
            package = 'lpms_ig1',
            executable = 'lpms_ig1_rs485_node',
            output = 'screen',
            parameters=[{
                "port": "/dev/ttyTHS5",
                "baudrate": 115200,
                "rs485ControlPin": 388,
                "rs485ControlPinToggleWaitMs": 2
            }]
        ),
        Node(
            namespace = 'imu',
            package = 'lpms_ig1',
            executable = 'quat_to_euler_node'
        ),
        Node(
            namespace = 'imu',
            package = 'lpms_ig1',
            executable = 'imudata_rad_to_deg_node'
        )
    ])
