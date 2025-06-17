from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='shooter',
            executable='shooter_vesc_node',
            name='shooter_vesc_node',
            output='screen'
        ),
        Node(
            package='shooter',
            executable='shooter_control_node',
            name='shooter_control_node',
            output='screen'
        ),
    ])