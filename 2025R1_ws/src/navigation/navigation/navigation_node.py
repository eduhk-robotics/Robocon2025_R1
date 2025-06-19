import rclpy
from rclpy.node import Node
from joystick_msgs.msg import Joystick
from std_msgs.msg import Float32MultiArray
import math

class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')
        self.subscription = self.create_subscription(
            Joystick,
            'joystick_input',
            self.listener_callback, 
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'driving', 10)

        # Deadzone threshold
        self.deadzone = 0.2

        # Store the last non-zero direction
        self.last_direction = 0.0

        # Declare parameters
        self.declare_parameter('dpad_erpm', 1300.0)
        self.declare_parameter('max_rpm', 10000.0)  # Default: 9000, alternative: 10000

        # Fixed speed for D-pad
        dpad_erpm = self.get_parameter('dpad_erpm').value
        max_rpm = self.get_parameter('max_rpm').value
        self.dpad_fixed_speed = (dpad_erpm / max_rpm) * 8192.0  # e.g., 1183.36 for 1300/9000

    def map_value(self, value):
        """Maps joystick input from [-32768, 32767] to [-1, 1]."""
        return ((value + 32768) / 65535) * 2.0 - 1.0

    def apply_deadzone(self, value):
        """Applies deadzone to joystick input."""
        return value if abs(value) >= self.deadzone else 0.0

    def listener_callback(self, msg):
        # Read joystick inputs
        try:
            raw_lx = msg.lx
            raw_ly = msg.ly
            raw_rx = msg.rx
            dpad_x = msg.dx
            dpad_y = msg.dy
        except AttributeError as e:
            self.get_logger().error(f"Invalid Joystick message: {e}")
            raw_lx = raw_ly = raw_rx = dpad_x = dpad_y = 0

        # Warn if D-pad inputs are invalid
        if dpad_x not in [-1, 0, 1] or dpad_y not in [-1, 0, 1]:
            self.get_logger().warn(f"Invalid D-pad input: dx={dpad_x}, dy={dpad_y}")

        # Map joystick inputs for left stick (primary control)
        left_x = self.apply_deadzone(self.map_value(raw_lx))
        left_y = self.apply_deadzone(self.map_value(raw_ly))
        right_x = self.apply_deadzone(self.map_value(raw_rx))

        # Initialize direction and speed
        direction = self.last_direction
        plane_speed = 0.0
        control_source = "none"

        # Primary control: Left joystick
        if left_x != 0.0 or left_y != 0.0:
            # Compute direction from joystick
            direction = math.degrees(math.atan2(left_y, left_x))
            if direction < 0.0:
                direction += 360.0
            direction = (direction + 90.0) % 360.0
            # Compute speed from joystick magnitude
            magnitude = math.hypot(left_x, left_y)
            if magnitude > 1.0:
                magnitude = 1.0
            plane_speed = magnitude * 8192.0
            self.last_direction = direction
            control_source = "joystick"
        # Backup control: D-pad (only if joystick is in deadzone)
        elif dpad_y != 0 or dpad_x != 0:
            # Y-axis priority (up/down over left/right)
            if dpad_y == -1:  # Up -> Forward
                direction = 0.0
            elif dpad_y == 1:  # Down -> Backward
                direction = 180.0
            elif dpad_x == -1:  # Left
                direction = 270.0
            elif dpad_x == 1:  # Right
                direction = 90.0
            # Set fixed speed for D-pad
            plane_speed = self.dpad_fixed_speed
            self.last_direction = direction
            control_source = "dpad"
        # No input: Keep last direction, speed = 0
        else:
            plane_speed = 0.0
            control_source = "none"

        # Calculate rotation speed, scaled to [-8192, 8192]
        rotation_speed = -right_x * 8192.0
        rotation_speed = max(min(rotation_speed, 8192.0), -8192.0)

        # Package message
        driving_msg = Float32MultiArray()
        driving_msg.data = [
            float(direction),
            float(plane_speed),
            float(rotation_speed)
        ]

        # Publish message
        self.publisher_.publish(driving_msg)

        # Debug logging
        self.get_logger().info(
            f"Control: {control_source}, "
            f"Raw: lx={raw_lx}, ly={raw_ly}, rx={raw_rx}, dx={dpad_x}, dy={dpad_y}, "
            f"Processed: lx={left_x:.2f}, ly={left_y:.2f}, rx={right_x:.2f}, "
            f"Driving: dir={direction:.1f} deg, lin={plane_speed:.1f}, rot={rotation_speed:.1f}"
        )

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