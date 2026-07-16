# keyboard_control.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 箭头键控制节点（↑↓←→ 控制，参考 four_wheeled_vehicle 的设计）
    keyboard_control = Node(
        package='ackermann_robot',
        executable='arrow_key_control.py',
        name='arrow_key_teleop',
        output='screen',
        prefix='gnome-terminal --wait --',  # 在独立终端窗口中运行，捕获键盘输入
    )

    ld = LaunchDescription()
    ld.add_action(keyboard_control)

    return ld
