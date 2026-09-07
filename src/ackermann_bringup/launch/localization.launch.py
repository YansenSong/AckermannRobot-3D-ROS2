"""Start the 3D global-map server and HDL localization only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    bool_value = context.perform_substitution(LaunchConfiguration('specify_init_pose')).lower()
    initial_pose = {
        key: float(context.perform_substitution(LaunchConfiguration(key)))
        for key in ('init_pos_x', 'init_pos_y', 'init_pos_z', 'init_ori_w', 'init_ori_x', 'init_ori_y', 'init_ori_z')
    }
    globalmap_server = Node(
        package='hdl_localization', executable='hdl_localization_map_server',
        name='globalmap_server', output='screen', parameters=[{
            'use_sim_time': True,
            'globalmap_pcd': LaunchConfiguration('globalmap_pcd').perform(context),
            'convert_utm_to_local': True, 'downsample_resolution': 0.2,
        }])
    localization = Node(
        package='hdl_localization', executable='hdl_localization_node',
        name='hdl_localization', output='screen',
        remappings=[('/velodyne_points', '/points_raw'), ('/gpsimu_driver/imu_data', '/imu/data')],
        parameters=[{
            'use_sim_time': True, 'use_imu': True, 'invert_acc': False, 'invert_gyro': False,
            'cool_time_duration': 2.0, 'enable_robot_odometry_prediction': False,
            'send_tf_transforms': True, 'odom_child_frame_id': 'base_link',
            'robot_odom_frame_id': 'odom', 'reg_method': 'NDT_OMP',
            'ndt_neighbor_search_method': 'DIRECT7', 'ndt_neighbor_search_radius': 2.0,
            'ndt_resolution': 1.0, 'downsample_resolution': 0.4,
            'specify_init_pose': bool_value in ('true', '1'),
            'use_global_localization': False, 'enable_auto_relocalize_monitor': False,
            **initial_pose,
        }])
    return [globalmap_server, localization]


def generate_launch_description():
    arguments = [DeclareLaunchArgument('globalmap_pcd', default_value=''),
                 DeclareLaunchArgument('specify_init_pose', default_value='false')]
    arguments += [DeclareLaunchArgument(name, default_value=value) for name, value in (
        ('init_pos_x', '0.0'), ('init_pos_y', '0.0'), ('init_pos_z', '0.0'),
        ('init_ori_w', '1.0'), ('init_ori_x', '0.0'), ('init_ori_y', '0.0'), ('init_ori_z', '0.0'))]
    return LaunchDescription(arguments + [OpaqueFunction(function=_nodes)])
