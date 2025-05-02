#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame
import time

class PS4Publisher(Node):

    def __init__(self):
        super().__init__('ps4_publisher')
        self.publisher_ = self.create_publisher(Joy, 'ps4', 10)
        self.last_received_time = time.time()
        self.timeout = 1.0  # Timeout period in seconds
        self.timer = None
        self.connected = False

        self.init_joystick()
        self.create_timer(0.1, self.check_timeout)  # Check for timeout every 100ms

    def init_joystick(self):
        """Initialize the joystick."""
        pygame.init()
        pygame.joystick.init()
        while pygame.joystick.get_count() == 0:
            self.get_logger().info('Waiting for PS4 controller to be connected...')
            time.sleep(1)
            pygame.joystick.quit()
            pygame.joystick.init()
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info('PS4 controller connected')
        self.connected = True
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if pygame.joystick.get_count() > 0 and self.connected:
            pygame.event.pump()
            msg = Joy()
            msg.axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
            msg.buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
            self.publisher_.publish(msg)
            self.last_received_time = time.time()  # Update the last received time
        else:
            self.connected = False
            self.publish_default_values()

    def check_timeout(self):
        """Check if the timeout period has been exceeded."""
        if time.time() - self.last_received_time > self.timeout:
            self.publish_default_values()

    def publish_default_values(self):
        """Publish default values for all axes and buttons."""
        msg = Joy()
        num_axes = self.joystick.get_numaxes() if self.connected else 0
        num_buttons = self.joystick.get_numbuttons() if self.connected else 0
        msg.axes = [0.0] * num_axes
        msg.buttons = [0] * num_buttons
        self.publisher_.publish(msg)
        self.get_logger().warn('PS4 controller signal lost or weak, publishing default values.')

def main(args=None):
    rclpy.init(args=args)
    ps4_publisher = PS4Publisher()
    rclpy.spin(ps4_publisher)
    ps4_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
