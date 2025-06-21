#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import gpiod
import time

class BounceRelaysNode(Node):
    def __init__(self):
        super().__init__('bounce_relays_node')
        self.subscription = self.create_subscription(
            Int32,
            'bounce_control',
            self.bounce_control_callback,
            10)
        self.get_logger().info('Bounce Relays Node started')

    def hig_gpio(self, pin):
        try:
            with gpiod.Chip('gpiochip4') as chip:
                line = chip.get_line(pin)
                line.request(consumer='test', type=gpiod.LINE_REQ_DIR_OUT)
                line.set_value(1)  # High (3.3V)
                self.get_logger().info(f"GPIO {pin} set to high")
                self.get_logger().info(f"GPIO {pin} test successful")
        except Exception as e:
            self.get_logger().error(f"GPIO {pin} test failed: {e}")

    def low_gpio(self, pin):
        try:
            with gpiod.Chip('gpiochip4') as chip:
                line = chip.get_line(pin)
                line.request(consumer='test', type=gpiod.LINE_REQ_DIR_OUT)
                line.set_value(0)  # Low
                self.get_logger().info(f"GPIO {pin} set to low")
                self.get_logger().info(f"GPIO {pin} test successful")
        except Exception as e:
            self.get_logger().error(f"GPIO {pin} test failed: {e}")

    def bounce_control_callback(self, msg):
        state = msg.data
        self.get_logger().info(f'Received bounce_control: {state}')

        if state == 0:
            # Prepare to release ball
            self.get_logger().info('Preparing to release ball')
            self.hig_gpio(17)  # Track out
            time.sleep(0.5)
            self.low_gpio(27)  # Drill in
            time.sleep(0.01)
            self.hig_gpio(22)  # Craw release
        elif state == 1:
            # Grip ball
            time.sleep(0.1)
            self.get_logger().info('Gripping ball')
            self.low_gpio(22)  # Crawed
            time.sleep(0.1)
            self.low_gpio(17)  # Track in
        elif state == 2:
            # Hit ball (assuming a GPIO action for hitting, e.g., toggle a pin)
            self.get_logger().info('Hitting ball')
            # Placeholder: Toggle GPIO 23 (or another pin) to simulate hit action
            self.hig_gpio(17)
            self.time.sleep(1)
            self.hig_gpio(27)
            self.time.sleep(0.1)
            self.hig_gpio(22)
            self.time.sleep(0.1)
            self.low_gpio(27)
            self.time.sleep(0.4)
            self.low_gpio(22)
            self.time.sleep(0.5)
            self.low_gpio(17)

        else:
            self.get_logger().warn(f'Invalid bounce_control value: {state}')

def main(args=None):
    rclpy.init(args=args)
    node = BounceRelaysNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    