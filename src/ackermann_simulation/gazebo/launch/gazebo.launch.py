# gazebo.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    control_share = get_package_share_directory('ackermann_control')
    simulation_share = get_package_share_directory('ackermann_simulation')

    publish_ekf_tf_arg = DeclareLaunchArgument(
        'publish_ekf_tf',
        default_value='true',
        description='Publish odom -> base_link from EKF (disable when LIO-SAM owns this TF)',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start the robot RViz instance',
    )

    # 1. 解析 URDF (XACRO)
    xacro_file = os.path.join(simulation_share, 'robot', 'xacro', 'robot.xacro')
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    world_file_path = os.path.join(simulation_share, 'gazebo', 'worlds', 'mini', 'mini.world')

    # 设置 GAZEBO_MODEL_PATH 环境变量
    # Keep Gazebo's standard model path and add this package's parent.
    pkg_share_env = os.pathsep + os.path.dirname(simulation_share)
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

    # Gazebo's ray sensor does not publish Velodyne ring/time fields.  This
    # adapter adds them for LIO-SAM while preserving the original /points_raw.
    lidar_adapter_node = Node(
        package='ackermann_simulation',
        executable='gazebo_lidar_adapter.py',
        name='gazebo_lidar_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}],
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

    # ================= EKF 融合 (轮式里程计 + IMU → odom→base_link TF) =================
    ekf_config_path = os.path.join(control_share, 'config', 'ekf_config.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path,
            {
                'use_sim_time': True,
                'publish_tf': ParameterValue(
                    LaunchConfiguration('publish_ekf_tf'), value_type=bool),
            },
        ],
        remappings=[('/odometry/filtered', '/odometry/filtered')],
    )

    # ================= RViz =================
    rviz_config_file = os.path.join(simulation_share, 'robot', 'rviz', 'view_robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # ================= 返回 Launch Description =================
    return LaunchDescription([
        publish_ekf_tf_arg,
        use_rviz_arg,
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        lidar_adapter_node,

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

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_ackermann_controller,
                on_exit=[ekf_node],
            )
        ),

        rviz_node,
    ])
