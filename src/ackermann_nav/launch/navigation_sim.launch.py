"""Start the current AckermannRobot simulation plus isolated Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    simulation_share = get_package_share_directory("ackermann_simulation")
    nav_share = get_package_share_directory("ackermann_nav")

    arguments = [
        DeclareLaunchArgument("map", description="Absolute path to map.yaml"),
        DeclareLaunchArgument(
            "globalmap_pcd", description="Absolute path to the LIORF GlobalMap.pcd"
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "liorf_params_file",
            default_value=os.path.join(
                get_package_share_directory("ackermann_bringup"),
                "config",
                "liorf_localization.yaml",
            ),
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=os.path.join(
                nav_share, "config", "nav2_params.yaml"
            ),
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_share, "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "publish_ekf_tf": "false",
            "use_rviz": "false",
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav_share, "launch", "navigation.launch.py"
            )
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "globalmap_pcd": LaunchConfiguration("globalmap_pcd"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "liorf_params_file": LaunchConfiguration("liorf_params_file"),
            "nav2_params_file": LaunchConfiguration("nav2_params_file"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    return LaunchDescription(
        arguments
        + [
            gazebo,
            TimerAction(period=5.0, actions=[navigation]),
        ]
    )
