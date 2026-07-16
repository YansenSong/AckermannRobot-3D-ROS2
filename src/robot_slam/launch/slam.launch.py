"""
LIO-SAM 3D SLAM — mapping mode with 16-line LiDAR + IMU

Usage:
  ros2 launch robot_slam slam.launch.py

After mapping, Ctrl+C to save GlobalMap.pcd, then convert:
  cd ~/AckermannRobot-3D
  ./src/pcd2pgm/build/pcd2gridmap src/maps/GlobalMap.pcd src/maps/map
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_lio = get_package_share_directory('lio_sam')
    pkg_robot = get_package_share_directory('ackermann_robot')
    params_file = os.path.join(pkg_lio, 'config', 'params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) time'
    )

    # Static TF: map → odom (identity, LIO-SAM publishes odom→base_link optimization)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # LIO-SAM pipeline nodes
    imu_preintegration = Node(
        package='lio_sam', executable='lio_sam_imuPreintegration',
        name='lio_sam_imuPreintegration',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    image_projection = Node(
        package='lio_sam', executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    feature_extraction = Node(
        package='lio_sam', executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    map_optimization = Node(
        package='lio_sam', executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # RViz for SLAM visualization
    rviz = Node(
        package='rviz2', executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_robot, 'rviz', 'slam_config.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_to_odom_tf,
        imu_preintegration,
        image_projection,
        feature_extraction,
        map_optimization,
        rviz,
    ])
