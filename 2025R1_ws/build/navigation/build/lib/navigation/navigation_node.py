#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, String
import math

class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')
        self.subscription = self.create_subscription(Joy, 'ps4', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'driving', 10)
        self.deadzone = 0.1  # Define the deadzone threshold

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def listener_callback(self, msg):
        # Check if the triangle button (index 3) is pressed
        
        # auto_move_msg = String()
        # auto_move_msg.data = "move;front;1024"
        # self.auto_move_publisher.publish(auto_move_msg)
        
        # self.get_logger().info(f'Triangle pressed')
            
        if True:
            left_analog_horizontal = self.apply_deadzone(msg.axes[0])
            left_analog_vertical = self.apply_deadzone(msg.axes[1])
            right_analog_horizontal = self.apply_deadzone(msg.axes[3])

            # Calculate direction angle in degrees
            direction = math.atan2(left_analog_vertical, left_analog_horizontal) * 180 / math.pi
            
            if direction < 0:
                direction += 360
            
            # Calculate plane movement speed (hypotenuse of the joystick positions scaled to 0-8192)
            plane_speed = float(math.sqrt(left_analog_horizontal**2 + left_analog_vertical**2) * 8192)
            if plane_speed > 8192:
                plane_speed = 8192

            # Calculate rotation speed (right joystick horizontal axis scaled to -8192 to 8192)
            rotation_speed = float(right_analog_horizontal * 8192)
            if rotation_speed > 8192:
                rotation_speed = 8192
            elif rotation_speed < -8192:
                rotation_speed = -8192

            direction = (direction + 90) % 360

            if direction == 90 and plane_speed == 0:
                direction = 0
            
            # Prepare the message
            driving_msg = Float32MultiArray()
            driving_msg.data = [float(direction), float(plane_speed), float(rotation_speed)]

            # Publish the message
            self.publisher_.publish(driving_msg)

def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
