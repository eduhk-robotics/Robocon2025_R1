import rclpy
from rclpy.node import Node
from joystick_msgs.msg import Joystick  # Custom message from your joystick publisher
from std_msgs.msg import Float32MultiArray
import math

class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')
        self.subscription = self.create_subscription(
            Joystick,
            'joystick_input',  # Topic from your JoystickPublisher
            self.listener_callback, 
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'driving', 10)
        self.deadzone = 0.1

    def map_value(self, value):
        """Maps joystick input from [-32768, 32767] to [-1, 1]."""
        return ((value + 32768) / 65535) * 2.0 - 1.0

    def apply_deadzone(self, value):
        """Applies deadzone to joystick input."""
        return value if abs(value) >= self.deadzone else 0.0

    def listener_callback(self, msg):
        # Map joystick inputs to [-1, 1]
        left_x = self.map_value(self.apply_deadzone(msg.lx))
        left_y = self.map_value(self.apply_deadzone(msg.ly))
        right_x = self.map_value(self.apply_deadzone(msg.rx))

        # Calculate direction in degrees
        direction = math.degrees(math.atan2(left_y, left_x))
        if direction < 0.0:
            direction += 360.0
        direction = (direction + 90.0) % 360.0
        if direction == 90.0 and left_x == 0.0 and left_y == 0.0:
            direction = 0.0

        # Calculate plane speed (magnitude of joystick input, scaled to [0, 8192])
        magnitude = math.hypot(left_x, left_y)
        # Normalize to ensure magnitude <= 1 before scaling
        if magnitude > 1.0:
            magnitude = 1.0
        plane_speed = magnitude * 8192.0

        # Calculate rotation speed, scaled to [-8192, 8192]
        rotation_speed = -right_x * 8192.0
        rotation_speed = max(min(rotation_speed, 8192.0), -8192.0)

        # Package message
        driving_msg = Float32MultiArray()
        driving_msg.data = [
            float(direction),
            float(plane_speed),
            float(rotation_speed),
        ]

        # Log and publish if there's movement or rotation
        if plane_speed > 0.0 or abs(right_x) > 0.0:
            self.get_logger().info(
                f"Publishing driving: dir={direction:.1f} deg, "
                f"lin={plane_speed:.1f}, rot={rotation_speed:.1f}"
            )
            self.publisher_.publish(driving_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 