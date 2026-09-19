import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogWarning, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _mounting_tf(context):
    path = os.path.expanduser(LaunchConfiguration("mounting_file").perform(context))
    if not os.path.isfile(path):
        return [LogWarning(msg=f"IMU mounting file not found; static TF disabled: {path}")]
    values = {}
    section = None
    for line in open(path, encoding="utf-8"):
        stripped = line.strip()
        if stripped in ("translation_m:", "rotation_quaternion_xyzw:"):
            section = stripped[:-1]
            continue
        match = re.match(r"^\s{2}(parent_frame|child_frame):\s*(\S+)", line)
        if match:
            values[match.group(1)] = match.group(2)
            continue
        match = re.match(r"^\s{4}([xyzw]):\s*([-+0-9.eE]+)", line)
        if match and section:
            values[f"{section}.{match.group(1)}"] = match.group(2)
    required = ["parent_frame", "child_frame", "translation_m.x",
                "translation_m.y", "translation_m.z",
                "rotation_quaternion_xyzw.x", "rotation_quaternion_xyzw.y",
                "rotation_quaternion_xyzw.z", "rotation_quaternion_xyzw.w"]
    missing = [key for key in required if key not in values]
    if missing:
        return [LogWarning(msg="Invalid IMU mounting file; missing: " + ", ".join(missing))]
    return [Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="lpms_ig1_mounting_tf", output="screen",
        arguments=[
            "--x", values["translation_m.x"], "--y", values["translation_m.y"],
            "--z", values["translation_m.z"],
            "--qx", values["rotation_quaternion_xyzw.x"],
            "--qy", values["rotation_quaternion_xyzw.y"],
            "--qz", values["rotation_quaternion_xyzw.z"],
            "--qw", values["rotation_quaternion_xyzw.w"],
            "--frame-id", values["parent_frame"],
            "--child-frame-id", values["child_frame"],
        ],
    )]


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("lpms_ig1_ros2"),
        "config",
        "lpms_ig1_calibration.yaml",
    )
    default_mounting_file = os.path.join(
        get_package_share_directory("lpms_ig1_ros2"), "config", "lpms_ig1_mounting.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params_file),
        DeclareLaunchArgument("publish_static_tf", default_value="true"),
        DeclareLaunchArgument("mounting_file", default_value=default_mounting_file),
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
        OpaqueFunction(
            function=_mounting_tf,
            condition=IfCondition(LaunchConfiguration("publish_static_tf")),
        ),
    ])
