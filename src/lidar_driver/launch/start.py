from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config=get_package_share_directory('lidar_driver')+'/rviz/rviz2.rviz'
    # yaml_config=get_package_share_directory('lidar_driver')+'/config/config.yaml'
    return LaunchDescription([
        Node(
            namespace='lidar_driver', 
            package='lidar_driver', 
            executable='lidar_driver_node', 
            output='screen',
            # parameters=[{'config_path': yaml_config}]
        ),
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config])
    ])
