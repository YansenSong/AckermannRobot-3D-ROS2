# gazebo.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    package_name = 'ackermann_robot'
    pkg_share = get_package_share_directory(package_name)

    # 1. 解析 URDF (XACRO)
    xacro_file = os.path.join(pkg_share, 'xacro', 'robot.xacro')
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    world_file_path = os.path.join(pkg_share, 'worlds', 'mini.world')

    # 设置 GAZEBO_MODEL_PATH 环境变量
    pkg_share_env = os.pathsep + os.path.join(get_package_prefix(package_name), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share_env
    else:
        os.environ['GAZEBO_MODEL_PATH'] = "/usr/share/gazebo-11/models" + pkg_share_env

    # 2. 启动 Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content
        }]
    )
    
    # 3. 启动 Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file_path,
            'verbose': 'true',
            'pause': 'false'
        }.items()
    )
    
    # 4. 在 Gazebo 中生成机器人模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ackermann_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # ================= NEW: 加载 ROS 2 Controllers =================
    
    # 加载关节状态广播器 (负责发布 /joint_states)
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # 加载阿克曼控制器 (负责底盘运动)
    load_ackermann_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # ================= 键盘控制 (↑↓←→) =================
    keyboard_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ackermann_robot'),
                'launch',
                'keyboard_control.launch.py'
            ])
        ])
    )

    # ================= RViz =================
    rviz_config_file = os.path.join(pkg_share, 'config', 'view_robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # ================= 返回 Launch Description =================
    return LaunchDescription([
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,

        # 使用事件处理器确保控制器在模型生成后启动
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_ackermann_controller],
            )
        ),

        # 键盘控制在阿克曼控制器加载后启动
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_ackermann_controller,
                on_exit=[keyboard_control],
            )
        ),

        rviz_node,
    ])