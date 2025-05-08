#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

ENCODER_RATIO = 19.20321
GEAR_RATIO = 35.0 / 61.0
TWO_PI = 2.0 * math.pi

# 360 deg -> one full turn in encoder radians
PERIOD_RAD = TWO_PI * (ENCODER_RATIO / GEAR_RATIO)  # ~210 rad

class OmniWheelPositionMapper(Node):
    def __init__(self):
        super().__init__('omni_wheel_position_mapper')

        # Subscribe to desired heading (degrees)
        self.create_subscription(
            Float32MultiArray,
            'driving',
            self.driving_callback,
            10
        )

        # Publisher to steering servos
        self.pos_pub = self.create_publisher(
            Float32MultiArray,
            'damiao_control',
            10
        )

        # Last commanded position (shared by all motors, init 0)
        self.last_pos_rad = 0.0

    def driving_callback(self, msg: Float32MultiArray):
        """
        Convert heading (deg) to nearest encoder position (rad).
        """
        direction_deg = msg.data[0]

        # Basic mapping (no period adjustment yet)
        wheel_rev   = direction_deg / 360.0
        motor_rev   = wheel_rev / GEAR_RATIO
        encoder_rev = motor_rev * ENCODER_RATIO
        target_rad  = encoder_rev * TWO_PI

        # Choose the nearest equivalent angle to minimize travel
        delta = target_rad - self.last_pos_rad
        if delta >  PERIOD_RAD / 2.0:
            target_rad -= PERIOD_RAD
        elif delta < -PERIOD_RAD / 2.0:
            target_rad += PERIOD_RAD

        # Publish to all four motors
        for motor_id in range(1, 5):
            out = Float32MultiArray()
            out.data = [
                float(motor_id),  # motor ID 1-4
                2.0,              # mode: position + speed
                10.0,             # speed rad/s (adjust as needed)
                target_rad        # target position in rad
            ]
            self.pos_pub.publish(out)

        # Store last commanded position
        self.last_pos_rad = target_rad
        self.get_logger().info(
            f"Cmd {direction_deg:.1f} deg -> {target_rad:.2f} rad"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OmniWheelPositionMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
