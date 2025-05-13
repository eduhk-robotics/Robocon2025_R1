#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 node: read_motor_state.py
Subscribe to DaMiao driver, read 4 motor positions, and publish to 'motor_states' topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
from DM_CAN import Motor, DM_Motor_Type, MotorControl


class ReadMotorStateNode(Node):
    def __init__(self):
        super().__init__('read_motor_state')
        # Publisher for motor states
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            'motor_states',
            10
        )

        # Initialize CAN-serial interface
        ser = serial.Serial('/dev/ttyUSB0', 921600, timeout=0.5)
        # Motor control object
        self.mc = MotorControl(ser)

        # Register and enable four motors (IDs 1-4)
        self.motors = []
        for idx in (1, 2, 3, 4):
            m = Motor(DM_Motor_Type.DM4310, idx, 0x11 + idx)
            self.mc.addMotor(m)
            self.mc.enable(m)
            self.motors.append(m)

        # Timer: read and publish every 0.1 seconds
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        angles = []
        for m in self.motors:
            self.mc.refresh_motor_status(m)
            pos = m.getPosition()
            angles.append(float(pos))

        msg = Float32MultiArray()
        msg.data = angles
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published motor states: {angles}")


def main(args=None):
    rclpy.init(args=args)
    node = ReadMotorStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
