"""
Nav2 navigation — AMCL + DWB local planner (pure Nav2, no NeuPAN)

Usage:
  ros2 launch robot_slam navigation_dwb.launch.py map:=/path/to/map.yaml
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_slam = 'robot_slam'
    pkg_robot = 'ackermann_robot'

    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to map.yaml file'
    )
    nav_param_file = os.path.join(
        get_package_share_directory(pkg_slam), 'config', 'nav2_params.yaml'
    )
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch'
    )

    # Nav2 full stack
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'bringup_launch.py')
        ),
        launch_arguments=[
            ('map', LaunchConfiguration('map')),
            ('use_sim_time', 'True'),
            ('params_file', nav_param_file),
            ('use_composition', 'False'),
        ],
    )

    # Twist → TwistStamped bridge
    bridge = Node(
        package='ackermann_robot',
        executable='cmd_vel_stamper.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Pointcloud to laserscan (3D → 2D for DWB obstacle layer + AMCL)
    pcl_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory(pkg_robot), 'config', 'pcl_to_scan.yaml'
        )],
        remappings=[
            ('cloud_in', '/points_raw'),
            ('scan', '/scan'),
        ],
    )

    # RViz
    rviz = Node(
        package='rviz2', executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory(pkg_robot), 'rviz', 'nav2_default_view.rviz'
        )],
        output='screen',
    )

    return LaunchDescription([map_arg, bringup, bridge, pcl_to_scan, rviz])
