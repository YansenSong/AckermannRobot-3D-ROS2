"""
hdl_localization (3D NDT) + Hybrid A* + NeuPAN 导航

用 hdl_localization 做 3D NDT 定位，Hybrid A* 做全局路径规划，
NeuPAN 做局部轨迹规划（不需要 Nav2 costmap/DWB）。

TF 链路:
    map ←(hdl NDT)← odom ←(EKF)← base_link ←(URDF)← laser_link

组件:
  - hybrid_astar_planner: 加载 PGM → /plan (Path) + /map (OccupancyGrid)
  - globalmap_server: 加载 PCD → /globalmap (hdl 匹配目标)
  - hdl_localization: NDT + UKF → map→odom TF + /odom 话题
  - pointcloud_to_laserscan: VLP-16 3D → /scan (NeuPAN 避障)
  - cmd_vel_mux: NeuPAN Twist → TwistStamped → 阿克曼控制器

Usage:
  ros2 launch robot_slam navigation_hdl.launch.py \
      map:=src/gazebo_worlds/worlds/mini/maps/map.yaml \
      map_pgm:=src/gazebo_worlds/worlds/mini/maps/map.pgm \
      globalmap_pcd:=src/gazebo_worlds/worlds/mini/maps/GlobalMap.pcd
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def parse_map_yaml(context):
    """在运行时解析 map.yaml，提取 resolution 和 origin。"""
    map_yaml_path = LaunchConfiguration('map').perform(context)
    map_pgm_path = LaunchConfiguration('map_pgm').perform(context)
    globalmap_pcd_path = LaunchConfiguration('globalmap_pcd').perform(context)

    resolution = 0.05
    origin_x = 0.0
    origin_y = 0.0

    # 如果提供了 map.yaml，从中读取 resolution 和 origin
    if map_yaml_path:
        try:
            with open(map_yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            resolution = float(data.get('resolution', resolution))
            origin = data.get('origin', [0.0, 0.0, 0.0])
            origin_x = float(origin[0])
            origin_y = float(origin[1])
        except Exception as e:
            print(f"[WARN] Failed to parse {map_yaml_path}: {e}, using defaults")

    # 如果没指定 map_pgm，从 map.yaml 同目录推断
    if not map_pgm_path and map_yaml_path:
        map_dir = os.path.dirname(map_yaml_path)
        with open(map_yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        image_rel = data.get('image', 'map.pgm')
        map_pgm_path = os.path.join(map_dir, image_rel)

    pkg_robot = get_package_share_directory('ackermann_robot')
    pkg_hybrid = get_package_share_directory('hybrid_astar_planner')

    # ====== Hybrid A* 全局规划器 ======
    hybrid_planner = Node(
        package='hybrid_astar_planner',
        executable='hybrid_astar_planner_node',
        name='hybrid_astar_planner',
        output='screen',
        parameters=[os.path.join(pkg_hybrid, 'config', 'planner_params.yaml'), {
            'use_sim_time': True,
            'map_path': map_pgm_path,
            'resolution': resolution,
            'origin_x': origin_x,
            'origin_y': origin_y,
        }],
    )

    # ====== hdl_localization ======
    globalmap_server = Node(
        package='hdl_localization',
        executable='hdl_localization_map_server',
        name='globalmap_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'globalmap_pcd': globalmap_pcd_path,
            'convert_utm_to_local': True,
            'downsample_resolution': 0.2,
        }],
    )

    # perform() 返回 string, 需要转为正确的 Python 类型
    specify_init_pose_str = context.perform_substitution(
        LaunchConfiguration('specify_init_pose'))
    specify_init_pose_val = specify_init_pose_str.lower() in ('true', '1')

    hdl_localization_node = Node(
        package='hdl_localization',
        executable='hdl_localization_node',
        name='hdl_localization',
        output='screen',
        remappings=[
            ('/velodyne_points', '/points_raw'),
            ('/gpsimu_driver/imu_data', '/imu/data'),
        ],
        parameters=[{
            'use_sim_time': True,
            'use_imu': True,
            'invert_acc': False,
            'invert_gyro': False,
            'cool_time_duration': 2.0,
            'enable_robot_odometry_prediction': False,
            'send_tf_transforms': True,
            'odom_child_frame_id': 'base_link',
            'robot_odom_frame_id': 'odom',
            'reg_method': 'NDT_OMP',
            'ndt_neighbor_search_method': 'DIRECT7',
            'ndt_neighbor_search_radius': 2.0,
            'ndt_resolution': 1.0,
            'downsample_resolution': 0.4,
            'specify_init_pose': specify_init_pose_val,
            'init_pos_x': float(context.perform_substitution(
                LaunchConfiguration('init_pos_x'))),
            'init_pos_y': float(context.perform_substitution(
                LaunchConfiguration('init_pos_y'))),
            'init_pos_z': float(context.perform_substitution(
                LaunchConfiguration('init_pos_z'))),
            'init_ori_w': float(context.perform_substitution(
                LaunchConfiguration('init_ori_w'))),
            'init_ori_x': float(context.perform_substitution(
                LaunchConfiguration('init_ori_x'))),
            'init_ori_y': float(context.perform_substitution(
                LaunchConfiguration('init_ori_y'))),
            'init_ori_z': float(context.perform_substitution(
                LaunchConfiguration('init_ori_z'))),
            'use_global_localization': False,
            'enable_auto_relocalize_monitor': False,
        }],
    )

    # ====== Pointcloud → Laserscan ======
    pcl_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[os.path.join(pkg_robot, 'config', 'pcl_to_scan.yaml')],
        remappings=[
            ('cloud_in', '/points_raw'),
            ('scan', '/scan'),
        ],
    )

    # ====== cmd_vel_mux ======
    mux = Node(
        package='ackermann_robot',
        executable='cmd_vel_mux.py',
        name='cmd_vel_mux',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'active_planner': 'neupan',
        }],
    )

    # ====== RViz ======
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_robot, 'rviz', 'nav2_default_view.rviz')],
    )

    return [
        hybrid_planner,
        globalmap_server,
        hdl_localization_node,
        pcl_to_scan,
        mux,
        rviz,
    ]


def generate_launch_description():
    map_yaml_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to map.yaml (读 resolution/origin)'
    )
    map_pgm_arg = DeclareLaunchArgument(
        'map_pgm', default_value='',
        description='Path to map.pgm (PGM 障碍物栅格地图, 不指定则从 map.yaml 推断)'
    )
    globalmap_pcd_arg = DeclareLaunchArgument(
        'globalmap_pcd', default_value='',
        description='Path to GlobalMap.pcd (3D 点云 for NDT)'
    )
    specify_init_pose_arg = DeclareLaunchArgument(
        'specify_init_pose', default_value='false',
    )
    init_pos_x_arg = DeclareLaunchArgument('init_pos_x', default_value='0.0')
    init_pos_y_arg = DeclareLaunchArgument('init_pos_y', default_value='0.0')
    init_pos_z_arg = DeclareLaunchArgument('init_pos_z', default_value='0.0')
    init_ori_w_arg = DeclareLaunchArgument('init_ori_w', default_value='1.0')
    init_ori_x_arg = DeclareLaunchArgument('init_ori_x', default_value='0.0')
    init_ori_y_arg = DeclareLaunchArgument('init_ori_y', default_value='0.0')
    init_ori_z_arg = DeclareLaunchArgument('init_ori_z', default_value='0.0')

    ld = LaunchDescription([
        map_yaml_arg,
        map_pgm_arg,
        globalmap_pcd_arg,
        specify_init_pose_arg,
        init_pos_x_arg, init_pos_y_arg, init_pos_z_arg,
        init_ori_w_arg, init_ori_x_arg, init_ori_y_arg, init_ori_z_arg,
        OpaqueFunction(function=parse_map_yaml),
    ])

    return ld
