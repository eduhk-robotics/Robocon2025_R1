#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class PS4Subscriber(Node):

    def __init__(self):
        super().__init__('ps4_subscriber')
        self.subscription = self.create_subscription(Joy, 'ps4', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'Axes: {msg.axes}, Buttons: {msg.buttons}')

def main(args=None):
    rclpy.init(args=args)
    ps4_subscriber = PS4Subscriber()
    rclpy.spin(ps4_subscriber)
    ps4_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
