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

    world_file_path = os.path.join(pkg_share, 'worlds', 'empty.world')

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

    # ================= EKF 节点 =================
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf_config.yaml')
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[('/odometry/filtered', '/odometry/filtered')] 
    )

    # ================= 启动 cmd_vel_stamper 节点 =================
    cmd_vel_stamper_node = Node(
        package='ackermann_robot',
        executable='cmd_vel_stamper.py', # 如果报错找不到，请检查 CMakeLists.txt 是否安装了此脚本
        name='cmd_vel_stamper',
        output='screen',
        parameters=[{'use_sim_time': True}] # 关键：让节点使用仿真时间，保证时间戳与 Gazebo 同步
    )
    # ===================================================================
    
    # ================= 返回 Launch Description =================
    return LaunchDescription([
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        ekf_node,

        # 立即启动转换节点，不需要等待其他事件
        cmd_vel_stamper_node,

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
    ])