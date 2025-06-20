from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bounce',
            executable='bounce_control_node',
            name='bounce_control_node',
            output='screen',
        ),
        Node(
            package='bounce',
            executable='bounce_relays_node',
            name='bounce_relays_node',
            output='screen',
        ),
    ])
