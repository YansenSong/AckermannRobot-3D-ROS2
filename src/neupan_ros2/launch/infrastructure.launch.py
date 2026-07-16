"""
Start Gazebo + pointcloud_to_laserscan + Nav2 (no NeuPAN).

Run this first, then start neupan_node manually:
  conda activate neupan
  source install/setup.bash
  ros2 run neupan_ros2 neupan_node --ros-args \
    --params-file /home/young/my_robot/install/neupan_ros2/share/neupan_ros2/config/robots/big_vehicle/robot.yaml \
    -p robot_config_dir:=/home/young/my_robot/install/neupan_ros2/share/neupan_ros2/config/robots/big_vehicle \
    -r /neupan_cmd_vel:=/cmd_vel
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robot_gazebo'), '/launch/vehicle_gazebo.launch.py'
        ]),
        launch_arguments={'teleop': 'false'}.items(),
    )

    pcl_to_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robot_gazebo'), '/launch/pcl_to_scan.launch.py'
        ])
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_hybrid_planner'), '/nav2_hybrid_planner.launch.py'
        ])
    )

    return LaunchDescription([
        gazebo_launch, pcl_to_scan_launch, nav2_launch,
    ])
