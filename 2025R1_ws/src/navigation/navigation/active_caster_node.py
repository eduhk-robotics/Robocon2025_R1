#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 node OmniServoFollower
Subscribes to 'driving' topic for joystick angle (0-360 deg).
Publishes position commands to 'damiao' for four servos (IDs 1-4).
Keeps software-record of each servo's current angle for encoderless control.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class OmniServoFollower(Node):
    def __init__(self):
        super().__init__('omni_servo_follower')
        # Maximum speed in degrees per second
        self.max_speed_dps = 360.0
        # Servo IDs
        self.servo_ids = [1, 2, 3, 4]
        # Software-recorded current angles
        self.current_angles = {sid: 0.0 for sid in self.servo_ids}

        # Publisher to 'damiao_control'
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            'damiao_control',
            10
        )
        # Subscriber to 'driving'
        self.create_subscription(
            Float32MultiArray,
            'driving',
            self.driving_callback,
            10
        )

        self.get_logger().info('OmniServoFollower started, all angles = 0.0 deg')

    def driving_callback(self, msg: Float32MultiArray):
        # Read target angle
        try:
            target_angle = float(msg.data[0]) % 360.0
        except (IndexError, ValueError):
            self.get_logger().error('Invalid driving msg: data[0] not a float angle')
            return

        self.get_logger().info(f"Received target angle: {target_angle:.2f} deg")

        # For each servo, compute shortest delta and publish
        for sid in self.servo_ids:
            prev = self.current_angles[sid]
            delta = target_angle - prev
            if delta > 180.0:
                delta -= 360.0
            elif delta < -180.0:
                delta += 360.0

            new_angle = prev + delta

            cmd = Float32MultiArray()
            cmd.data = [
                float(sid),        # servo ID
                2.0,               # mode 2 = position
                new_angle,         # target angle
                self.max_speed_dps # max speed
            ]
            self.publisher_.publish(cmd)
            self.get_logger().info(
                f"Servo {sid}: from {prev:.2f} deg -> {new_angle:.2f} deg (delta {delta:.2f} deg)"
            )

            # Update stored angle
            self.current_angles[sid] = new_angle

    def destroy_node(self):
        super().destroy_node()
        self.get_logger().info('Node shutting down')

def main(args=None):
    rclpy.init(args=args)
    node = OmniServoFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
