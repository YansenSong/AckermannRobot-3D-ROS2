from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    rviz_config=get_package_share_directory('lidar_driver')+'/rviz/rviz2.rviz'
    # yaml_config=get_package_share_directory('lidar_driver')+'/config/config.yaml'
    return LaunchDescription([
        Node(
            package='lidar_driver',
            node_namespace='lidar_driver',
            node_name='lidar_driver_node',
            node_executable='lidar_driver_node',
            output='screen',
            # parameters=[{'config_path': yaml_config}]
        ),
        Node(
            package='rviz2',
            node_namespace='rviz2',
            node_name='rviz2',
            node_executable='rviz2',
            arguments=['-d',rviz_config]
        )
    ])
