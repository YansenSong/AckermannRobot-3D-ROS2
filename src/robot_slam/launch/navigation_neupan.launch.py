"""
Nav2 + NeuPAN navigation — AMCL + NeuPAN local planner + cmd_vel_mux

Components:
  - Nav2 bringup (AMCL, map_server, planner_server, bt_navigator, DWB)
  - NeuPAN local planner (subscribes /scan + /plan, publishes /neupan_cmd_vel)
  - cmd_vel_mux (selects NeuPAN by default)

Usage:
  Terminal 1: ros2 launch ackermann_robot gazebo.launch.py
  Terminal 2: ros2 launch robot_slam navigation_neupan.launch.py map:=src/maps/map.yaml
  Set goal via RViz "2D Goal Pose" tool
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
    pkg_neupan = 'neupan_ros2'

    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to map.yaml file'
    )

    # Nav2 params (use neupan-compatible: DWB runs but NeuPAN controls)
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

    # Pointcloud to laserscan
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

    # NeuPAN local planner
    robot_config_dir = os.path.join(
        get_package_share_directory(pkg_neupan),
        'config', 'robots', 'ackermann_robot'
    )
    neupan_node = Node(
        package='neupan_ros2',
        executable='neupan_node',
        name='neupan_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_type': 'ackermann_robot',
            'robot_description': 'Ackermann vehicle (0.70m, 0.593m wheelbase) in Gazebo',
            'robot_config_dir': robot_config_dir,
            'planner_config_file': 'planner.yaml',
            'dune_checkpoint_file': 'models/dune_model_5000.pth',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'lidar_frame': 'laser_link',
            'scan_topic': '/scan',
            'plan_input_topic': '/plan',
            'cmd_vel_topic': '/neupan_cmd_vel',
            'enable_visualization': False,
        }],
    )

    # cmd_vel mux: selects NeuPAN by default
    mux = Node(
        package='ackermann_robot',
        executable='cmd_vel_mux.py',
        name='cmd_vel_mux',
        output='screen',
        parameters=[{'use_sim_time': True, 'active_planner': 'neupan'}],
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

    return LaunchDescription([
        map_arg, bringup, pcl_to_scan, neupan_node, mux, rviz,
    ])
