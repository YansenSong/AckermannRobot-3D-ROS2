import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("lpms_ig1_ros2"),
        "config",
        "lpms_ig1.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params_file),
        DeclareLaunchArgument("interface", default_value="can0"),
        DeclareLaunchArgument("node_id", default_value="5"),
        DeclareLaunchArgument("frame_id", default_value="imu_link"),
        DeclareLaunchArgument("invert_accel_for_ros", default_value="true"),
        DeclareLaunchArgument("convert_nwu_to_enu", default_value="true"),

        Node(
            package="lpms_ig1_ros2",
            executable="lpms_ig1_node",
            name="lpms_ig1_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "interface": LaunchConfiguration("interface"),
                    "node_id": LaunchConfiguration("node_id"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "invert_accel_for_ros": LaunchConfiguration(
                        "invert_accel_for_ros"
                    ),
                    "convert_nwu_to_enu": LaunchConfiguration(
                        "convert_nwu_to_enu"
                    ),
                },
            ],
        ),
    ])
