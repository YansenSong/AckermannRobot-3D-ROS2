"""Start the isolated AckermannRobot Nav2 stack."""

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


def _build_navigation(context):
    map_yaml = LaunchConfiguration("map").perform(context)
    globalmap_pcd = LaunchConfiguration("globalmap_pcd").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params_file = LaunchConfiguration("nav2_params_file").perform(context)
    liorf_params_file = LaunchConfiguration("liorf_params_file").perform(context)

    if not map_yaml:
        raise RuntimeError("navigation.launch.py requires map:=<map.yaml>")
    if not globalmap_pcd:
        raise RuntimeError(
            "navigation.launch.py requires globalmap_pcd:=<GlobalMap.pcd>")
    for path, label in ((map_yaml, "map"), (globalmap_pcd, "globalmap_pcd")):
        if not os.path.isfile(path):
            raise RuntimeError("%s file does not exist: %s" % (label, path))

    with open(map_yaml, encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream) or {}
    resolution = float(metadata.get("resolution", 0.05))
    if resolution <= 0.0:
        raise RuntimeError("Map resolution must be positive, got %s" % resolution)

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "yaml_filename": map_yaml,
                "global_costmap.global_costmap.resolution": str(resolution),
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    bringup_share = get_package_share_directory("ackermann_bringup")
    nav_share = get_package_share_directory("ackermann_nav")
    nav_status_share = get_package_share_directory("nav_status")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "localization.launch.py")
        ),
        launch_arguments={
            "globalmap_pcd": globalmap_pcd,
            "use_sim_time": use_sim_time,
            "params_file": liorf_params_file,
        }.items(),
    )

    scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="ackermann_pointcloud_to_laserscan",
        output="screen",
        parameters=[
            os.path.join(nav_share, "config", "pcl_to_scan.yaml")
        ],
        remappings=[("cloud_in", "/points_raw"), ("scan", "/scan")],
    )

    common_parameters = [configured_params]
    bt_parameters = [
        configured_params,
        {
            "default_nav_to_pose_bt_xml": os.path.join(
                nav_share,
                "behavior_trees",
                "ackermann_navigate_to_pose_w_replanning_and_recovery.xml",
            ),
            "default_nav_through_poses_bt_xml": os.path.join(
                nav_share,
                "behavior_trees",
                "ackermann_navigate_through_poses_w_replanning_and_recovery.xml",
            ),
        },
    ]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=common_parameters,
    )
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=common_parameters,
        remappings=[("cmd_vel", "/ackermann_nav/cmd_vel_raw")],
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=common_parameters,
    )
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=common_parameters,
    )
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=common_parameters,
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=bt_parameters,
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=common_parameters,
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=common_parameters,
        remappings=[
            ("cmd_vel", "/ackermann_nav/cmd_vel_raw"),
            ("cmd_vel_smoothed", "/ackermann_nav/cmd_vel_smoothed"),
        ],
    )
    command_bridge = Node(
        package="ackermann_nav",
        executable="cmd_bridge.py",
        name="ackermann_cmd_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/ackermann_nav/cmd_vel_smoothed",
                "output_topic": "/ackermann_steering_controller/reference",
                "max_speed": 0.70,
                "min_speed": 0.0,
                "min_turning_radius": 1.320,
                "command_timeout": 0.35,
                "output_rate": 30.0,
                "frame_id": "base_link",
                "min_turning_speed": 0.02,
            }
        ],
    )
    nav2_status = Node(
        package="nav_status",
        executable="nav2_status_node",
        name="nav2_status_node",
        output="screen",
        parameters=[
            os.path.join(nav_status_share, "config", "nav2_status.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "smoother_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": False,
                "node_names": lifecycle_nodes,
                "bond_timeout": 4.0,
            }
        ],
    )
    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server"],
                "bond_timeout": 4.0,
            }
        ],
    )
    startup_gate = Node(
        package="ackermann_nav",
        executable="nav2_startup_gate.py",
        name="nav2_startup_gate",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "startup_service": "/lifecycle_manager_navigation/manage_nodes",
                "required_topics": ["/odom", "/scan"],
                "target_frame": "odom",
                "source_frame": "rear_axle_link",
                "startup_timeout": 90.0,
                "check_period": 0.5,
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="ackermann_nav_rviz",
        output="screen",
        arguments=[
            "-d",
            LaunchConfiguration(
                "rviz_config",
                default=os.path.join(nav_share, "rviz", "nav2.rviz"),
            ),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return [
        localization,
        scan,
        map_server,
        controller_server,
        planner_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        command_bridge,
        nav2_status,
        map_lifecycle_manager,
        lifecycle_manager,
        startup_gate,
        rviz,
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory("ackermann_bringup")
    nav_share = get_package_share_directory("ackermann_nav")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Absolute path to map.yaml",
            ),
            DeclareLaunchArgument(
                "globalmap_pcd",
                default_value="",
                description="Absolute path to the LIORF GlobalMap.pcd",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "liorf_params_file",
                default_value=os.path.join(
                    bringup_share, "config", "liorf_localization.yaml"
                ),
                description="Existing LIORF localization parameter file",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(
                    nav_share, "config", "nav2_params.yaml"
                ),
                description="Ackermann Nav2 parameter file",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(nav_share, "rviz", "nav2.rviz"),
            ),
            OpaqueFunction(function=_build_navigation),
        ]
    )
