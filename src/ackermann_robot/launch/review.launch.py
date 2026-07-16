import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包路径
    pkg_name = 'ackermann_robot'
    pkg_share = get_package_share_directory(pkg_name)

    # 2. 路径定义
    xacro_file = os.path.join(pkg_share, 'xacro', 'robot.xacro')
    rviz_config_file = os.path.join(pkg_share, 'config', 'view_robot.rviz')

    # 3. 解析 URDF/Xacro
    robot_description = Command(['xacro ', xacro_file])

    # 4. 定义 Nodes

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher GUI (用于手动控制关节进行测试)
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # 键盘控制 (↑↓←→)
    keyboard_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ackermann_robot'),
                'launch',
                'keyboard_control.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz,
        keyboard_control
    ])
