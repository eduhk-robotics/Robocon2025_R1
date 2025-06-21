#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from joystick_msgs.msg import Joystick
from std_msgs.msg import Int32  # Changed to Int32 for publishing integer

class BounceControlNode(Node):
    def __init__(self):
        super().__init__('bounce_control_node')
        self.publisher_ = self.create_publisher(Int32, 'bounce_control', 10)  # Changed to Int32
        self.subscription = self.create_subscription(
            Joystick,
            'joystick_input',
            self.joystick_callback,
            10)
        self.button_press_count = 0  # Tracks number of L1 button presses (0, 1, 2)
        self.prev_lb_state = False  # Tracks previous L1 state to detect rising edge
        self.get_logger().info('Bounce Control Node started')

    def joystick_callback(self, msg):
        lb = msg.l1  # Unpack L1 button state (boolean)

        # Detect rising edge of L1 button press (from False to True)
        if lb and not self.prev_lb_state:
            # Increment press count and cap at 2 (0: prepare, 1: grip, 2: hit)
            self.button_press_count = (self.button_press_count + 1) % 3

            # Publish to bounce_control with integer value
            bounce_msg = Int32()
            bounce_msg.data = self.button_press_count
            self.publisher_.publish(bounce_msg)

            # Log the action based on press count
            if self.button_press_count == 0:
                self.get_logger().info('Published bounce_control: 0 (Prepare to release ball)')
            elif self.button_press_count == 1:
                self.get_logger().info('Published bounce_control: 1 (Grip ball)')
            else:  # self.button_press_count == 2
                self.get_logger().info('Published bounce_control: 2 (Hit ball)')

        # Update previous L1 state
        self.prev_lb_state = lb

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