"""Launch only the Nav2 navigation system and shared robot infrastructure."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


NAV2_NAMESPACE = 'nav2'


def _localization_nodes(context):
    """Create Nav2-owned localization nodes without HDL TF publication."""
    specify_pose = LaunchConfiguration('specify_init_pose').perform(context)
    initial_pose = {
        key: float(LaunchConfiguration(key).perform(context))
        for key in (
            'init_pos_x', 'init_pos_y', 'init_pos_z', 'init_ori_w',
            'init_ori_x', 'init_ori_y', 'init_ori_z')
    }
    globalmap_server = Node(
        package='hdl_localization',
        executable='hdl_localization_map_server',
        name='globalmap_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'globalmap_pcd': LaunchConfiguration('globalmap_pcd').perform(context),
            'convert_utm_to_local': True,
            'downsample_resolution': 0.2,
        }],
    )
    localization = Node(
        package='hdl_localization',
        executable='hdl_localization_node',
        name='hdl_localization',
        output='screen',
        remappings=[
            ('/velodyne_points', '/points_raw'),
            ('/gpsimu_driver/imu_data', '/imu/data'),
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_imu': True,
            'invert_acc': False,
            'invert_gyro': False,
            'cool_time_duration': 2.0,
            'enable_robot_odometry_prediction': False,
            # The Nav2-specific bridge publishes a fresh map->odom TF.
            'send_tf_transforms': False,
            'odom_child_frame_id': 'base_link',
            'robot_odom_frame_id': 'odom',
            'reg_method': 'NDT_OMP',
            'ndt_neighbor_search_method': 'DIRECT7',
            'ndt_neighbor_search_radius': 2.0,
            'ndt_resolution': 1.0,
            'downsample_resolution': 0.4,
            'specify_init_pose': specify_pose.lower() in ('true', '1'),
            'use_global_localization': False,
            'enable_auto_relocalize_monitor': False,
            **initial_pose,
        }],
    )
    tf_bridge = Node(
        package='ackermann_nav2',
        executable='nav2_localization_tf_bridge.py',
        name='nav2_localization_tf_bridge',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    return [globalmap_server, localization, tf_bridge]


def generate_launch_description():
    package_share = get_package_share_directory('ackermann_nav2')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=NAV2_NAMESPACE,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    common_remappings = [('/tf', '/tf'), ('/tf_static', '/tf_static')]

    localization = OpaqueFunction(function=_localization_nodes)

    scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='nav2_pointcloud_to_laserscan',
        output='screen',
        parameters=[
            os.path.join(package_share, 'config', 'pcl_to_scan.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('cloud_in', '/points_raw'), ('scan', '/scan')],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        namespace=NAV2_NAMESPACE,
        name='map_server',
        output='screen',
        parameters=[configured_params, {'yaml_filename': map_file}],
        remappings=common_remappings,
    )
    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        namespace=NAV2_NAMESPACE, name='controller_server', output='screen',
        parameters=[configured_params],
        remappings=common_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )
    smoother_server = Node(
        package='nav2_smoother', executable='smoother_server',
        namespace=NAV2_NAMESPACE, name='smoother_server', output='screen',
        parameters=[configured_params], remappings=common_remappings,
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        namespace=NAV2_NAMESPACE, name='planner_server', output='screen',
        parameters=[configured_params], remappings=common_remappings,
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        namespace=NAV2_NAMESPACE, name='behavior_server', output='screen',
        parameters=[configured_params], remappings=common_remappings,
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        namespace=NAV2_NAMESPACE, name='bt_navigator', output='screen',
        parameters=[configured_params], remappings=common_remappings,
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        namespace=NAV2_NAMESPACE, name='waypoint_follower', output='screen',
        parameters=[configured_params], remappings=common_remappings,
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        namespace=NAV2_NAMESPACE, name='velocity_smoother', output='screen',
        parameters=[configured_params],
        remappings=common_remappings + [
            ('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace=NAV2_NAMESPACE,
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'map_server', 'controller_server', 'smoother_server',
                'planner_server', 'behavior_server', 'bt_navigator',
                'waypoint_follower', 'velocity_smoother',
            ],
        }],
    )

    command_adapter = Node(
        package='ackermann_nav2',
        executable='nav2_cmd_adapter.py',
        name='nav2_cmd_adapter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', namespace=NAV2_NAMESPACE,
        name='rviz2', output='screen',
        arguments=['-d', os.path.join(package_share, 'rviz', 'nav2.rviz')],
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static'),
                    ('scan', '/scan'),
                    ('robot_description', '/robot_description')],
        condition=IfCondition(LaunchConfiguration('nav2_use_rviz')),
    )

    arguments = [
        DeclareLaunchArgument('map', description='Absolute path to the 2D map YAML'),
        DeclareLaunchArgument(
            'globalmap_pcd', description='Absolute path to the HDL localization PCD map'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('nav2_use_rviz', default_value='true'),
        DeclareLaunchArgument('specify_init_pose', default_value='false'),
    ]
    arguments += [DeclareLaunchArgument(name, default_value=value)
                  for name, value in (
                      ('init_pos_x', '0.0'), ('init_pos_y', '0.0'),
                      ('init_pos_z', '0.0'), ('init_ori_w', '1.0'),
                      ('init_ori_x', '0.0'), ('init_ori_y', '0.0'),
                      ('init_ori_z', '0.0'))]

    return LaunchDescription(arguments + [
        localization, scan, map_server, controller_server, smoother_server,
        planner_server, behavior_server, bt_navigator, waypoint_follower,
        velocity_smoother, lifecycle_manager, command_adapter, rviz,
    ])
