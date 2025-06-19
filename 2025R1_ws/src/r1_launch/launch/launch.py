import os
from launch import LaunchDescription
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory # Only needed if loading params from this pkg

def generate_launch_description():

    navigation_node = Node(
        package='navigation',
        executable='navigation_node',
        name='navigation_controller',
        output='screen',
    )

    active_caster_node = Node(
        package='navigation',
        executable='active_caster_node',
        name='active_caster_controller',
        output='screen',
    )

    joystick_node = Node(
        package='joystick_driver',
        executable='joystick_node',
        name='joystick_publisher',
        output='screen',
    )
   
    damiao_node = Node(
        package='active_caster',
        executable='damiao_node',
        name='damiao_motor_interface',
        output='screen',
    )
    
    vesc_node = Node(
        package='active_caster',
        executable='vesc_node',
        name='vesc_motor_interface',
        output='screen',
    )
    
    vesc_canbus_speed_control_node = Node(
        package='active_caster',
        executable='vesc_canbus_speed_control_node',
        name='vesc_canbus_speed_interface',
        output='screen',
    )
    
    shooter_control_node = Node(
        package='shooter',
        executable='shooter_control_node',
        name='shooter_controller',
        output='screen',
    )
    
    shooter_damiao_node = Node(
        package='shooter',
        executable='shooter_damiao_node',
        name='shooter_damiao_interface',
        output='screen',
    )
    shooter_vesc_node = Node(
        package='shooter',
        executable='shooter_vesc_node',
        name='shooter_vesc_interface',
        output='screen',
    )
    
    return LaunchDescription([
        navigation_node,
        active_caster_node,
        joystick_node,
        vesc_node,
        vesc_canbus_speed_control_node,
        shooter_control_node,
        shooter_damiao_node,
        shooter_vesc_node,
    ])
