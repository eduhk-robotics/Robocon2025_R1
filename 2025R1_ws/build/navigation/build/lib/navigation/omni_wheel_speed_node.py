import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

class OmniWheelSpeedNode(Node):
    def __init__(self):
        super().__init__('omni_wheel_speed_node')

        # === Adjustable maximum speeds (in degrees per second) ===
        self.MAX_PLANE_SPEED_DPS = 360.0 * 20  # Max 20 rotations/sec
        self.MAX_ROTATION_SPEED_DPS = 360.0 * 10  # Max 10 rotations/sec

        # Subscribe to driving commands
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'driving',
            self.driving_callback,
            10
        )

        # Publish motor speed commands (to damiao)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'damiao', 10)

    def driving_callback(self, msg):
        direction = msg.data[0]  # degrees
        plane_speed_raw = msg.data[1]  # 0 to 8192
        rotation_speed_raw = msg.data[2]  # -8192 to 8192

        motor_speeds_dps = self.calculate_omni_wheel_speeds(direction, plane_speed_raw, rotation_speed_raw)

        for motor_id, speed in enumerate(motor_speeds_dps, start=1):
            message = Float32MultiArray()
            message.data = [
                float(motor_id),  # Motor ID (1,2,3,4...)
                1.0,              # Mode (always 1 for speed mode)
                float(speed),     # Speed (degrees per second)
                0.0               # Position (not used for mode 1)
            ]
            self.publisher_.publish(message)
            self.get_logger().info(f'Published to damiao: {message.data}')

    def calculate_omni_wheel_speeds(self, direction_deg, plane_speed_raw, rotation_speed_raw):
        # Scale joystick input (8192) to degrees per second
        plane_speed = (plane_speed_raw / 8192.0) * self.MAX_PLANE_SPEED_DPS
        rotation_speed = (rotation_speed_raw / 8192.0) * self.MAX_ROTATION_SPEED_DPS

        direction_rad = math.radians(direction_deg)

        # Decompose plane movement into vx and vy
        vx = plane_speed * math.cos(direction_rad)
        vy = plane_speed * math.sin(direction_rad)

        # Omni wheel inverse kinematics
        motor_speeds = [
            -vx + vy + rotation_speed,  # Motor 1
             vx + vy + rotation_speed,  # Motor 2
             vx - vy + rotation_speed,  # Motor 3
            -vx - vy + rotation_speed   # Motor 4
        ]

        return motor_speeds

def main(args=None):
    rclpy.init(args=args)
    node = OmniWheelSpeedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
