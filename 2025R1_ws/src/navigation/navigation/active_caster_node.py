#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Combined Omni-Wheel Steering Controller
Subscribes to 'driving' (Float32MultiArray):
 - data[0]: heading in degrees
 - data[1]: speed (0 to 8192)
Publishes steering positions to 'damiao_control' and RPM to 'vesc_control'.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32

# Steering encoder/gear constants
ENCODER_RATIO = 19.20321
GEAR_RATIO = 35.0 / 61.0
TWO_PI = 2.0 * math.pi
PERIOD_RAD = TWO_PI * (ENCODER_RATIO / GEAR_RATIO)  # encoder rad per full wheel turn

# Motor speed constants
MAX_RPM = 6000  # VESC max motor RPM

class OmniWheelMotorController(Node):
    def __init__(self):
        super().__init__('omni_wheel_motor_controller')
        self.get_logger().info('OmniWheelMotorController starting')

        # Last commanded steering position (rad)
        self.last_pos_rad = 0.0

        # Steering command publisher
        self.pos_pub = self.create_publisher(
            Float32MultiArray, 'damiao_control', 10
        )

        # VESC RPM publisher
        self.rpm_pub = self.create_publisher(
            Float32, 'vesc_control', 10
        )

        # Subscribe to driving commands [heading_deg, speed]
        self.create_subscription(
            Float32MultiArray,
            'driving',
            self.driving_callback,
            10
        )

    def publish_steering(self, target_rad):
        """Publish steering commands to damiao_control for all 4 wheels."""
        for motor_id in range(1, 5):
            out = Float32MultiArray()
            out.data = [float(motor_id), 2.0, 20.0, target_rad]
            self.pos_pub.publish(out)

    def publish_rpm(self, desired_rpm):
        """Publish RPM command to vesc_control."""
        rpm_msg = Float32()
        rpm_msg.data = float(desired_rpm)
        self.rpm_pub.publish(rpm_msg)

    def driving_callback(self, msg: Float32MultiArray):
        # Heading in degrees
        direction_deg = float(msg.data[0])
        # Speed value (0..8192)
        raw_speed = float(msg.data[1])

        # 1) Steering: map heading to encoder rad
        wheel_rev   = direction_deg / 360.0
        motor_rev   = wheel_rev / GEAR_RATIO
        encoder_rev = motor_rev * ENCODER_RATIO
        target_rad  = encoder_rev * TWO_PI

        # Choose nearest equivalent to minimize travel
        delta = target_rad - self.last_pos_rad
        if delta >  PERIOD_RAD / 2.0:
            target_rad -= PERIOD_RAD
        elif delta < -PERIOD_RAD / 2.0:
            target_rad += PERIOD_RAD

        # Publish steering commands
        self.publish_steering(target_rad)

        # Update last steering pos
        self.last_pos_rad = target_rad
        self.get_logger().info(f"Steering {direction_deg:.1f}° -> {target_rad:.2f} rad")

        # 2) Speed: normalize raw_speed (0..8192) to [0, MAX_RPM]
        norm = max(min(raw_speed / 8192.0, 1.0), 0.0)
        desired_rpm = int(norm * MAX_RPM)
        self.get_logger().info(f"Speed raw={raw_speed:.1f} -> RPM={desired_rpm}")

        # Publish RPM command
        self.publish_rpm(desired_rpm)

    def destroy_node(self):
        # No VESC cleanup needed, just call parent
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OmniWheelMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()