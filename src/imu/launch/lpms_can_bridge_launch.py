from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lpms_ig1',
            executable='lpms_can_bridge_node',
            name='lpms_can_bridge',
            output='screen',
            parameters=[{
                'channel': 'can0',
                'publish_rate': 200.0,
                'timestamp_scale': 0.001,
                # 'interface' and 'bitrate' are NOT declared by the node.
                # Uncomment only after adding declare_parameter() calls for them,
                # or the node will throw ParameterNotDeclaredException on startup.
                # 'interface': 'socketcan',
                # 'bitrate': 500000,
            }],
        ),
    ])