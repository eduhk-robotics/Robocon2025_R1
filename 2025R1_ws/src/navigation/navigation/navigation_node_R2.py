#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from joystick_msgs.msg import Joystick # Custom message from your joystick publisher
from std_msgs.msg import Float32MultiArray
import math

# Assumed maximum absolute value for raw joystick stick axes (e.g., for a 16-bit signed axis).
# This is used to normalize raw input from msg.lx, msg.ly, msg.rx to [-1.0, 1.0].
# Adjust if your joystick publisher sends different raw ranges.
MAX_STICK_VALUE = 32767.0

class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')
        self.subscription = self.create_subscription(
            Joystick,
            'joystick_input',  # Topic from your JoystickPublisher
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'driving', 10)
        self.get_logger().info("Navigation node started, subscribing to /joystick_input. Output speeds will be normalized (max 1.0).")

    def normalize_axis(self, raw_value, max_raw_value):
        """Normalizes a raw axis value to the range [-1.0, 1.0]."""
        normalized = float(raw_value) / max_raw_value
        return max(-1.0, min(1.0, normalized)) # Clamp to ensure range

    def listener_callback(self, msg: Joystick):
        # Normalize raw joystick inputs (which should have publisher's deadzone applied)
        # to the [-1.0, 1.0] range.
        left_analog_horizontal = self.normalize_axis(msg.lx, MAX_STICK_VALUE)
        # Ensure msg.ly has the correct sign for your setup (publisher might invert).
        left_analog_vertical = self.normalize_axis(msg.ly, MAX_STICK_VALUE)
        right_analog_horizontal = self.normalize_axis(msg.rx, MAX_STICK_VALUE)

        # Calculate direction angle in degrees (0-360)
        # atan2(y, x)
        direction = math.atan2(left_analog_vertical, left_analog_horizontal) * 180 / math.pi
        if direction < 0:
            direction += 360
        
        # Calculate plane movement speed (magnitude of left stick, normalized to 0.0-1.0)
        plane_speed = math.sqrt(left_analog_horizontal**2 + left_analog_vertical**2)
        if plane_speed > 1.0: # Clamp, though mathematically shouldn't exceed 1 if inputs are <=1
            plane_speed = 1.0

        # Calculate rotation speed (right stick horizontal, normalized to -1.0 to 1.0)
        # Negative sign inverts rotation direction, adjust if needed.
        rotation_speed = -right_analog_horizontal 

        # Custom direction adjustment specific to your application
        direction = (direction + 90) % 360
        if math.isclose(direction, 90.0) and math.isclose(plane_speed, 0.0):
            direction = 0.0
            
        # Prepare the message: [direction_degrees, plane_speed_0_to_1, rotation_speed_neg1_to_1]
        driving_msg = Float32MultiArray()
        driving_msg.data = [float(direction), float(plane_speed), float(rotation_speed)]

        self.publisher_.publish(driving_msg)
        #self.get_logger().info(f"Published - Dir: {direction:.2f}, PlaneSpd: {plane_speed:.2f}, RotSpd: {rotation_speed:.2f}")

def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    try:
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        navigation_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        if rclpy.ok(): # Check if context is still valid
            navigation_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()