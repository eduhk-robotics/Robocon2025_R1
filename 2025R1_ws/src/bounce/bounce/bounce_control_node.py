#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from joystick_msgs.msg import Joystick
from std_msgs.msg import Bool

class BounceControlNode(Node):
    def __init__(self):
        super().__init__('bounce_control_node')
        self.publisher_ = self.create_publisher(Bool, 'bounce_control', 10)
        self.subscription = self.create_subscription(
            Joystick,
            'joystick_input',
            self.joystick_callback,
            10)
        self.get_logger().info('Bounce Control Node started')

    def joystick_callback(self, msg):
        lb = msg.l1
        # Publish to bounce_control
        bounce_msg = Bool()
        bounce_msg.data = lb
        self.publisher_.publish(bounce_msg)
        self.get_logger().info(f'Published start_bounce: {lb}')

def main(args=None):
    rclpy.init(args=args)
    node = BounceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
