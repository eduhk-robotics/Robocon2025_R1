import rclpy
from rclpy.node import Node
from joystick_msgs.msg import Joystick
from std_msgs.msg import Float32, Float32MultiArray
import math

class ShooterControlNode(Node):
    def __init__(self):
        super().__init__('shooter_control_node')
        self.get_logger().info('ShooterControlNode starting')

        # Subscribe to joystick input
        self.subscription = self.create_subscription(
            Joystick,
            'joystick_input',
            self.joystick_callback,
            10
        )

        # Publisher for VESC speed control (shooter wheel)
        self.vesc_publisher = self.create_publisher(
            Float32, 
            'shooter_vesc_control', 
            10
        )
        
        # Publisher for motor position control
        self.damiao_publisher = self.create_publisher(
            Float32MultiArray, 
            'shooter_damiao_control', 
            10
        )

        # Variables to track button states
        self.last_a_state = False
        self.last_y_state = False

        # Constants for motor control
        self.default_speed = 0.0
        self.default_angle = 0.0
        self.move_speed = 0.16    # rad/s
        self.up_angle = 5.0       # radians (+5 rad)
        self.down_angle = -5.0    # radians (-5 rad)

    def joystick_callback(self, msg):
        """Handle joystick inputs including RT, A and Y buttons"""
        try:
            r2_value = msg.r2
            a_button = msg.a
            y_button = msg.y
        except AttributeError as e:
            self.get_logger().error(f"Invalid joystick message: {e}")
            r2_value = 0
            a_button = False
            y_button = False

        # ----- VESC SPEED CONTROL (RT Trigger) -----
        # Validate RT value (expected range: 0 to 255)
        if not (0 <= r2_value <= 255):
            self.get_logger().warn(f"Invalid r2 value: {r2_value}, expected 0 to 255")
            r2_value = max(min(r2_value, 255), 0)

        # Map RT from [0, 255] to [0, 30000] as eRPM
        erpm = (r2_value / 255.0) * 30000.0
        
        # Publish to shooter_vesc_control
        vesc_msg = Float32()
        vesc_msg.data = float(erpm)
        self.vesc_publisher.publish(vesc_msg)
        
        if r2_value > 0:
            self.get_logger().info(f"RT: {r2_value} -> eRPM: {erpm:.1f}")

        # ----- MOTOR POSITION CONTROL (A/Y Buttons) -----
        damiao_msg = Float32MultiArray()
        
        # Initialize angles and speed
        left_angle = self.default_angle   # Motor 10 (left)
        right_angle = self.default_angle  # Motor 12 (right)
        motor_speed = self.default_speed
        
        # Handle A button press (UP: left +, right -)
        if a_button and not self.last_a_state:
            self.get_logger().info("A button: UP (left +5 rad, right -5 rad) @ 0.16 rad/s")
            left_angle = self.up_angle    # +5 rad for left
            right_angle = -self.up_angle  # -5 rad for right
            motor_speed = self.move_speed
            
        # Handle Y button press (DOWN: left -, right +)
        elif y_button and not self.last_y_state:
            self.get_logger().info("Y button: DOWN (left -5 rad, right +5 rad) @ 0.16 rad/s")
            left_angle = self.down_angle    # -5 rad for left
            right_angle = -self.down_angle  # +5 rad for right
            motor_speed = self.move_speed
        
        # Create position control message
        # Format: [speed10, angle10, speed12, angle12] (angles in radians)
        damiao_msg.data = [
            motor_speed, left_angle,   # Motor 10 (left)
            motor_speed, right_angle   # Motor 12 (right)
        ]
        
        # Publish to shooter_damiao_control
        self.damiao_publisher.publish(damiao_msg)

        # Update last button states
        self.last_a_state = a_button
        self.last_y_state = y_button

def main(args=None):
    rclpy.init(args=args)
    node = ShooterControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node terminated by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
