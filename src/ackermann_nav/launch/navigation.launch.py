#!/usr/bin/env python3
"""Start the independent real-vehicle Ackermann Nav2 stack."""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _finite(value, field):
    number = float(value)
    if not math.isfinite(number):
        raise RuntimeError(f'{field} must be finite')
    return number


def _lidar_mounting(vehicle):
    """LiDAR layout from vehicle.sensor_mounting.lidar.

    Returns the pointcloud_to_laserscan height slice plus the static transform
    placing laser_link in frames.base. Two different references are involved --
    see the comments in vehicle.yaml: height_above_ground is measured from the
    ground, the transform is relative to frames.base.
    """
    mounting = vehicle.get('sensor_mounting', {}).get('lidar', {})
    frames = vehicle.get('frames', {})
    base_frame = str(frames.get('base', '')).strip()
    lidar_frame = str(frames.get('lidar', '')).strip()
    for label, name in (('base', base_frame), ('lidar', lidar_frame)):
        if not name:
            raise RuntimeError(f'vehicle.frames.{label} must not be empty')

    status = str(mounting.get('status', '')).strip().upper()
    if status != 'VERIFIED':
        raise RuntimeError(
            'vehicle.sensor_mounting.lidar is not VERIFIED in '
            'config/vehicle.yaml (found '
            f'{status or "no status"}); the scan slice and the laser_link '
            'static transform would use unmeasured mounting values. Measure the '
            'LiDAR height above the ground and its mounting rotations, then set '
            'them there.'
        )

    numbers = {}
    for field in ('x', 'y', 'height_above_ground', 'roll_deg', 'pitch_deg',
                  'yaw_deg', 'ground_margin'):
        value = mounting.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RuntimeError(
                f'vehicle.sensor_mounting.lidar.{field} must be a number, '
                f'got: {value!r}'
            )
        numbers[field] = float(value)

    base_height = frames.get('base_height_above_ground')
    if isinstance(base_height, bool) or not isinstance(base_height, (int, float)):
        raise RuntimeError(
            'vehicle.frames.base_height_above_ground must be a number in '
            f'metres, got: {base_height!r}'
        )

    # The ground plane sits at -height_above_ground in laser_link.
    min_height = numbers['ground_margin'] - numbers['height_above_ground']

    # static_transform_publisher takes radians.
    return {
        'min_height': min_height,
        'tf': (
            f"{numbers['x']}",
            f"{numbers['y']}",
            f"{numbers['height_above_ground'] - float(base_height)}",
            f"{math.radians(numbers['roll_deg'])}",
            f"{math.radians(numbers['pitch_deg'])}",
            f"{math.radians(numbers['yaw_deg'])}",
        ),
        'base_frame': base_frame,
        'lidar_frame': lidar_frame,
    }


def _load_vehicle(path):
    if not path:
        raise RuntimeError('navigation.launch.py requires vehicle_config:=<vehicle.yaml>')
    if not os.path.isfile(path):
        raise RuntimeError(f'Vehicle config does not exist: {path}')

    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}

    vehicle = data.get('vehicle', {})
    geometry = vehicle.get('geometry', {})
    planning = vehicle.get('planning', {})
    footprint = planning.get('footprint', {})
    frames = vehicle.get('frames', {})

    values = {
        'wheelbase': _finite(geometry.get('wheelbase'), 'vehicle.geometry.wheelbase'),
        'base_frame': str(frames.get('base', '')).strip(),
        'max_speed': _finite(
            planning.get('max_forward_speed'), 'vehicle.planning.max_forward_speed'
        ),
        'max_acceleration': _finite(
            planning.get('max_acceleration'), 'vehicle.planning.max_acceleration'
        ),
        'min_radius': _finite(
            planning.get('minimum_turning_radius'),
            'vehicle.planning.minimum_turning_radius',
        ),
        'front_extent': _finite(
            footprint.get('front_extent'),
            'vehicle.planning.footprint.front_extent',
        ),
        'rear_extent': _finite(
            footprint.get('rear_extent'),
            'vehicle.planning.footprint.rear_extent',
        ),
        'half_width': _finite(
            footprint.get('half_width'),
            'vehicle.planning.footprint.half_width',
        ),
        'padding': _finite(
            footprint.get('padding'), 'vehicle.planning.footprint.padding'
        ),
        'lidar': _lidar_mounting(vehicle),
    }

    for key in ('wheelbase', 'max_speed', 'max_acceleration', 'min_radius'):
        if values[key] <= 0.0:
            raise RuntimeError(f'Vehicle parameter {key} must be > 0')
    for key in ('front_extent', 'rear_extent', 'half_width', 'padding'):
        if values[key] < 0.0:
            raise RuntimeError(f'Vehicle parameter {key} must be >= 0')
    if not values['base_frame']:
        raise RuntimeError('vehicle.frames.base must not be empty')

    values['max_yaw_rate'] = values['max_speed'] / values['min_radius']
    values['analytic_expansion_max_length'] = 5.0 * values['min_radius']
    values['footprint'] = (
        f"[[-{values['rear_extent']:.6f}, -{values['half_width']:.6f}], "
        f"[{values['front_extent']:.6f}, -{values['half_width']:.6f}], "
        f"[{values['front_extent']:.6f}, {values['half_width']:.6f}], "
        f"[-{values['rear_extent']:.6f}, {values['half_width']:.6f}]]"
    )
    return values


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in {
        '1', 'true', 'yes', 'on'
    }


def _build_navigation(context):
    map_yaml = LaunchConfiguration('map').perform(context)
    globalmap_pcd = LaunchConfiguration('globalmap_pcd').perform(context)
    vehicle_config = LaunchConfiguration('vehicle_config').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file').perform(context)
    liorf_params_file = LaunchConfiguration('liorf_params_file').perform(context)
    points_topic = LaunchConfiguration('points_topic')

    if not map_yaml:
        raise RuntimeError('navigation.launch.py requires map:=<map.yaml>')
    if not globalmap_pcd:
        raise RuntimeError('navigation.launch.py requires globalmap_pcd:=<GlobalMap.pcd>')
    for path, label in ((map_yaml, 'map'), (globalmap_pcd, 'globalmap_pcd')):
        if not os.path.isfile(path):
            raise RuntimeError(f'{label} file does not exist: {path}')

    with open(map_yaml, encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream) or {}
    resolution = _finite(metadata.get('resolution', 0.05), 'map.resolution')
    if resolution <= 0.0:
        raise RuntimeError(f'Map resolution must be positive, got {resolution}')

    vehicle = _load_vehicle(vehicle_config)

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
                'robot_base_frame': vehicle['base_frame'],
                'minimum_turning_radius': str(vehicle['min_radius']),
                'min_turning_r': str(vehicle['min_radius']),
                'vx_max': str(vehicle['max_speed']),
                'wz_max': str(vehicle['max_yaw_rate']),
                'footprint': vehicle['footprint'],
                'footprint_padding': str(vehicle['padding']),
                'analytic_expansion_max_length': str(
                    vehicle['analytic_expansion_max_length']
                ),
                'global_costmap.global_costmap.resolution': str(resolution),
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    bringup_share = get_package_share_directory('ackermann_bringup')
    nav_share = get_package_share_directory('ackermann_nav')
    nav_status_share = get_package_share_directory('nav_status')
    motion_share = get_package_share_directory('motion_interface')

    actions = []

    if _enabled(context, 'start_hardware'):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, 'launch', 'real_vehicle.launch.py')
                ),
                launch_arguments={
                    'vehicle_config': vehicle_config,
                    'enable_lidar': 'true',
                    'lidar_config': LaunchConfiguration('lidar_config'),
                    'lidar_rviz': 'false',
                    'enable_imu': LaunchConfiguration('enable_imu'),
                    'imu_port': LaunchConfiguration('imu_port'),
                    'enable_control': 'true',
                    'enable_navigation': 'false',
                }.items(),
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'localization.launch.py')
            ),
            launch_arguments={
                'globalmap_pcd': globalmap_pcd,
                'vehicle_config': vehicle_config,
                'use_sim_time': use_sim_time,
                'params_file': liorf_params_file,
            }.items(),
        )
    )

    scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='ackermann_nav_pointcloud_to_laserscan',
        output='screen',
        parameters=[
            os.path.join(nav_share, 'config', 'pcl_to_scan.yaml'),
            {
                'use_sim_time': use_sim_time,
                'min_height': vehicle['lidar']['min_height'],
            },
        ],
        remappings=[('cloud_in', points_topic), ('scan', '/scan')],
    )

    # Published unconditionally: this is the only publisher of laser_link in
    # the navigation modes, and without it anything looking up laser_link in the
    # TF tree blocks (NeuPAN does exactly that).
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ackermann_nav_laser_link_static_tf',
        output='screen',
        arguments=[
            *vehicle['lidar']['tf'],
            vehicle['lidar']['base_frame'],
            vehicle['lidar']['lidar_frame'],
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    common_parameters = [configured_params]
    bt_parameters = [
        configured_params,
        {
            'default_nav_to_pose_bt_xml': os.path.join(
                nav_share,
                'behavior_trees',
                'ackermann_navigate_to_pose_w_replanning_and_recovery.xml',
            ),
            'default_nav_through_poses_bt_xml': os.path.join(
                nav_share,
                'behavior_trees',
                'ackermann_navigate_through_poses_w_replanning_and_recovery.xml',
            ),
        },
    ]

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=common_parameters,
    )
    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[
            configured_params,
            {
                'FollowPath.vx_max': vehicle['max_speed'],
                'FollowPath.wz_max': vehicle['max_yaw_rate'],
                'FollowPath.AckermannConstraints.min_turning_r': vehicle['min_radius'],
            },
        ],
        remappings=[('cmd_vel', '/ackermann_nav/cmd_vel_raw')],
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen', parameters=common_parameters,
    )
    smoother_server = Node(
        package='nav2_smoother', executable='smoother_server', name='smoother_server',
        output='screen', parameters=common_parameters,
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server', name='behavior_server',
        output='screen', parameters=common_parameters,
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
        output='screen', parameters=bt_parameters,
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen', parameters=common_parameters,
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        parameters=[
            configured_params,
            {
                'max_velocity': [
                    vehicle['max_speed'], 0.0, vehicle['max_yaw_rate']
                ],
                'min_velocity': [0.0, 0.0, -vehicle['max_yaw_rate']],
                'max_accel': [vehicle['max_acceleration'], 0.0, 1.0],
                'max_decel': [-vehicle['max_acceleration'], 0.0, -1.0],
            },
        ],
        remappings=[
            ('cmd_vel', '/ackermann_nav/cmd_vel_raw'),
            ('cmd_vel_smoothed', '/ackermann_nav/cmd_vel_smoothed'),
        ],
    )
    command_adapter = Node(
        package='ackermann_nav', executable='nav2_cmd_adapter.py',
        name='ackermann_nav_cmd_adapter', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/ackermann_nav/cmd_vel_smoothed',
            'output_topic': '/ackermann_nav/ackermann_cmd_raw',
            'wheelbase': vehicle['wheelbase'],
            'max_speed': vehicle['max_speed'],
            'min_turning_radius': vehicle['min_radius'],
        }],
    )
    command_gate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(motion_share, 'launch', 'motion_interface.launch.py')
        ),
        launch_arguments={
            'vehicle_config': vehicle_config,
            'enable_command_gate': 'true',
            'enable_stm32_bridge': 'false',
            # No STM32 board is commanded on this path, so there is no sta__
            # feedback to listen for. Off here to keep nav2 and the
            # real_vehicle/nav path (which leaves it on) behaving alike.
            'enable_status_receiver': 'false',
            'use_sim_time': use_sim_time,
            'input_topic': '/ackermann_nav/ackermann_cmd_raw',
            'output_topic': '/ackermann_cmd',
        }.items(),
    )
    nav2_status = Node(
        package='nav_status', executable='nav2_status_node', name='nav2_status_node',
        output='screen',
        parameters=[
            os.path.join(nav_status_share, 'config', 'nav2_status.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': False,
            'node_names': lifecycle_nodes,
            'bond_timeout': 4.0,
        }],
    )
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 4.0,
        }],
    )
    startup_gate = Node(
        package='ackermann_nav', executable='nav2_startup_gate.py',
        name='nav2_startup_gate', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'startup_service': '/lifecycle_manager_navigation/manage_nodes',
            'required_topics': ['/odom', '/scan'],
            'target_frame': 'odom',
            'source_frame': vehicle['base_frame'],
            'startup_timeout': 90.0,
            'check_period': 0.5,
        }],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='ackermann_nav_rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    actions.extend([
        scan,
        lidar_tf,
        map_server,
        controller_server,
        planner_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        command_adapter,
        command_gate,
        nav2_status,
        map_lifecycle_manager,
        lifecycle_manager,
        startup_gate,
        rviz,
    ])
    return actions


def generate_launch_description():
    bringup_share = get_package_share_directory('ackermann_bringup')
    nav_share = get_package_share_directory('ackermann_nav')
    lidar_share = get_package_share_directory('lidar_driver')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='', description='Absolute path to map.yaml'),
        DeclareLaunchArgument(
            'globalmap_pcd', default_value='',
            description='Absolute path to LIORF GlobalMap.pcd',
        ),
        DeclareLaunchArgument(
            'vehicle_config', default_value='',
            description='Absolute path to repository-root config/vehicle.yaml',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('points_topic', default_value='/lidar_points'),
        DeclareLaunchArgument('start_hardware', default_value='false'),
        DeclareLaunchArgument(
            'lidar_config',
            default_value=os.path.join(lidar_share, 'config', 'config.yaml'),
        ),
        DeclareLaunchArgument('enable_imu', default_value='false'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument(
            'liorf_params_file',
            default_value=os.path.join(bringup_share, 'config', 'liorf_localization.yaml'),
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(nav_share, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(nav_share, 'rviz', 'nav2.rviz'),
        ),
        OpaqueFunction(function=_build_navigation),
    ])
