import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Constants for Damiao motor direction control
ENCODER_RATIO = 19.20321  # Encoder ratio for motor revolutions
GEAR_RATIO = 35.0 / 61.0  # Gear ratio between motor and wheel
TWO_PI = 2.0 * math.pi    # Full circle in radians
PERIOD_RAD = TWO_PI * (ENCODER_RATIO / GEAR_RATIO)  # ~210 rad, one full encoder period

class ActiveCasterCtrl(Node):
    def __init__(self):
        """Initialize the ActiveCasterCtrl node for direction and speed control."""
        super().__init__('active_caster_ctrl')
        
        # Subscribe to driving commands (msg.data[0]: direction in degrees, msg.data[1]: speed)
        self.create_subscription(
            Float32MultiArray,
            'driving',
            self.driving_callback,
            10
        )
        
        # Publisher for Damiao motor control (direction: position + speed mode)
        self.pos_pub = self.create_publisher(
            Float32MultiArray,
            'damiao_control',
            10
        )
        
        # Publisher for VESC controller (speed: mapped to 0-10000 range)
        self.vesc_pub = self.create_publisher(
            Float32MultiArray,
            'vesc_control',
            10
        )
        
        # Track last commanded position for Damiao motors (shared across all motors)
        self.last_pos_rad = 0.0
        
        self.get_logger().info('ActiveCasterCtrl node initialized')

    def vesc_speed_control(self, speed: float):
        """
        Map speed from 0-8129 to VESC control range 0-10000 and publish to vesc_control topic.
        
        Args:
            speed: Input speed (0-8129 range) to map to 0-10000 for VESC.
        """
        # Map speed from 0-8129 to 0-10000
        target_rpm = (speed / 8129.0) * 10000.0
        # Clamp to ensure output stays within 0-10000
        target_rpm = max(0.0, min(10000.0, target_rpm))
        vesc_msg = Float32MultiArray()
        vesc_msg.data = [target_rpm]  # Forward mapped speed to VESC
        self.vesc_pub.publish(vesc_msg)
        self.get_logger().info(f"VESC: speed {speed:.2f} -> {target_rpm:.0f} RPM")

    def driving_callback(self, msg: Float32MultiArray):
        """
        Process driving commands: direction (deg) for Damiao motors and speed for VESC.
        
        Args:
            msg: Float32MultiArray with msg.data[0] as direction (degrees) and msg.data[1] as speed.
        """
        # Extract direction and speed from incoming message
        direction_deg = msg.data[0]
        speed = msg.data[1] if len(msg.data) > 1 else 0.0
        
        # --- Damiao Motor Control (Direction) ---
        # Convert direction (degrees) to encoder position (radians)
        wheel_rev = direction_deg / 360.0
        motor_rev = wheel_rev / GEAR_RATIO
        encoder_rev = motor_rev * ENCODER_RATIO
        target_rad = encoder_rev * TWO_PI
        
        # Adjust target to nearest equivalent angle to minimize motor travel
        delta = target_rad - self.last_pos_rad
        if delta > PERIOD_RAD / 2.0:
            target_rad -= PERIOD_RAD
        elif delta < -PERIOD_RAD / 2.0:
            target_rad += PERIOD_RAD
        
        # Publish position commands to all four Damiao motors
        for motor_id in range(1, 5):
            out = Float32MultiArray()
            out.data = [
                float(motor_id),  # Motor ID (1-4)
                2.0,              # Mode: position + speed control
                10.0,             # Speed (rad/s, adjustable)
                target_rad        # Target position (radians)
            ]
            self.pos_pub.publish(out)
        
        # Update last commanded position
        self.last_pos_rad = target_rad
        self.get_logger().info(
            f"Damiao: {direction_deg:.1f} deg -> {target_rad:.2f} rad"
        )
        
        # --- VESC Control (Speed) ---
        self.vesc_speed_control(speed)

def main(args=None):
    """Initialize and run the ActiveCasterCtrl node."""
    rclpy.init(args=args)
    node = ActiveCasterCtrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down ActiveCasterCtrl node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()