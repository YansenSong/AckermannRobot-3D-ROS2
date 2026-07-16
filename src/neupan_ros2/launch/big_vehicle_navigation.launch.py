"""
NeuPAN Navigation System — Big Ackermann Vehicle (robot_gazebo)

Integrates:
  1. robot_gazebo       — Gazebo sim + vehicle + sensors
  2. pointcloud_to_laserscan — 3D Velodyne → 2D LaserScan
  3. nav2_hybrid_planner — Nav2 SmacPlannerHybrid (global planner)
  4. neupan_node        — NeuPAN local planner + controller
  5. RViz               — Visualization

Architecture:
  /goal_pose (RViz) → nav2_plan_bridge → /compute_path_to_pose (action)
      → planner_server (SmacPlannerHybrid) → /plan_path (global path)
      → neupan_node (PAN: DUNE + NRMP) → /neupan_cmd_vel → /cmd_vel
      → Gazebo vehicle plugin → /odom, /points_raw → pointcloud_to_laserscan → /scan
      → neupan_node (loop closed)

TF tree:
  map → odom (identity, nav2_plan_bridge)
  odom → base_link (from /odom, nav2_plan_bridge)
  base_link → rear_axle_link (static x=-1.545, nav2_plan_bridge)

Usage:
  ros2 launch neupan_ros2 big_vehicle_navigation.launch.py

  Then start RViz manually:
    rviz2 -d /home/young/my_robot/install/neupan_ros2/share/neupan_ros2/rviz/neupan_big_vehicle.rviz

  Use "2D Goal Pose" tool to set navigation target.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ================================================================
    # 1. Gazebo + Vehicle (teleop disabled — NeuPAN controls vehicle)
    # ================================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robot_gazebo'),
            '/launch/vehicle_gazebo.launch.py'
        ]),
        launch_arguments={'teleop': 'false'}.items(),
    )

    # ================================================================
    # 2. 3D PointCloud → 2D LaserScan
    # ================================================================
    pcl_to_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robot_gazebo'),
            '/launch/pcl_to_scan.launch.py'
        ])
    )

    # ================================================================
    # 3. Nav2 Global Planner (map_server + planner_server + bridge)
    # ================================================================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_hybrid_planner'),
            '/nav2_hybrid_planner.launch.py'
        ])
    )

    # ================================================================
    # 4. NeuPAN Local Planner + Controller
    # ================================================================
    pkg_share = FindPackageShare('neupan_ros2').find('neupan_ros2')
    robot_config_dir = os.path.join(pkg_share, 'config', 'robots', 'big_vehicle')
    robot_config = os.path.join(robot_config_dir, 'robot.yaml')

    neupan_node = Node(
        package='neupan_ros2',
        executable='neupan_node',
        name='neupan_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            robot_config,
            {'robot_config_dir': robot_config_dir},
        ],
        remappings=[
            # NeuPAN output → Gazebo vehicle input
            ('/neupan_cmd_vel', '/cmd_vel'),
        ],
    )

    # RViz — start manually:
    #   rviz2 -d /home/young/my_robot/install/neupan_ros2/share/neupan_ros2/rviz/neupan_big_vehicle.rviz

    # ================================================================
    return LaunchDescription([
        gazebo_launch,
        pcl_to_scan_launch,
        nav2_launch,
        neupan_node,
    ])
