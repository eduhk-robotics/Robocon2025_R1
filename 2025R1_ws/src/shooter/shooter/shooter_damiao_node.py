import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from DM_can import Motor, MotorControl, DM_Motor_Type
import serial
import os
import time

# USB-CDC device ID
DEVICE_ID = "usb-HDSC_CDC_Device_00000000050C-if00"

def find_device_port(device_id):
    """
    Find the serial port for the given device ID in /dev/serial/by-id.
    Returns the port path if found, None otherwise.
    """
    by_id_dir = "/dev/serial/by-id/"
    try:
        for entry in os.listdir(by_id_dir):
            if device_id in entry:
                return os.path.realpath(os.path.join(by_id_dir, entry))
    except FileNotFoundError:
        pass
    return None

class ShooterDamiaoNode(Node):
    def __init__(self):
        super().__init__("shooter_damiao_node")

        # Find serial port
        self.get_logger().info(f"Searching for USB device with ID '{DEVICE_ID}'...")
        port = None
        while port is None:
            port = find_device_port(DEVICE_ID)
            if port is None:
                self.get_logger().warn("Device not found, retrying in 1 second...")
                time.sleep(1)
            else:
                self.get_logger().info(f"Found device at port: '{port}'")

        # Initialize serial connection
        self.serial_device = serial.Serial(port, 921600, timeout=0.5)
        if self.serial_device.is_open:
            self.get_logger().info("Successfully opened USB-CDC serial connection")
        else:
            self.get_logger().error("Failed to open serial port!")
            raise RuntimeError("Could not open serial port")

        # Initialize MotorControl
        self.motor_control = MotorControl(self.serial_device)

        # Initialize motors (DM4310, IDs=10 and 12, MasterID=0x00)
        self.motors = [
            Motor(DM_Motor_Type.DM4310, 10, 0x00),  # Motor 1, ID=10 (left)
            Motor(DM_Motor_Type.DM4310, 12, 0x00)   # Motor 2, ID=12 (right)
        ]

        # Configure motors
        for motor in self.motors:
            self.motor_control.addMotor(motor)
            self.motor_control.enable(motor)  # Enable motor
            self.motor_control.set_zero_position(motor)  # Set zero position

        # Status publisher
        self.status_publisher = self.create_publisher(Float32MultiArray, "shooter_damiao_status", 10)

        # Control subscriber
        self.control_subscriber = self.create_subscription(
            Float32MultiArray,
            "shooter_damiao_control",
            self.control_callback,
            10
        )

        # Timer for status updates at 10 Hz
        self.timer = self.create_timer(0.1, self.status_timer_callback)

        self.get_logger().info("ShooterDamiaoNode initialized: publishing to 'shooter_damiao_status', subscribing to 'shooter_damiao_control'")

    def status_timer_callback(self):
        """
        Publish motor status periodically (includes ID, position, velocity, torque).
        """
        status_msg = Float32MultiArray()
        for motor in self.motors:
            self.motor_control.refresh_motor_status(motor)  # Update motor status
            status_msg.data.extend([
                float(motor.SlaveID),  # Motor ID
                motor.getPosition(),   # Position
                motor.getVelocity(),   # Velocity
                motor.getTorque()      # Torque
            ])
        self.status_publisher.publish(status_msg)
        self.get_logger().debug(f"Published status: {status_msg.data}")

    def control_callback(self, msg):
        """
        Handle shooter_damiao_control messages, format: [speed10, angle10, speed12, angle12].
        """
        if len(msg.data) < 4:
            self.get_logger().warn("Invalid control message, too short!")
            return

        # Extract control values
        speed10, angle10, speed12, angle12 = msg.data[:4]
        self.get_logger().info(
            f"Received control: Motor 10 [speed: {speed10}, angle: {angle10}], Motor 12 [speed: {speed12}, angle: {angle12}]"
        )

        # Control Motor 10 (left)
        self.motor_control.control_Pos_Vel(self.motors[0], angle10, speed10)
        self.get_logger().info(f"Motor 10 set to angle: {angle10} rad, speed: {speed10} rad/s")

        # Control Motor 12 (right)
        self.motor_control.control_Pos_Vel(self.motors[1], angle12, speed12)
        self.get_logger().info(f"Motor 12 set to angle: {angle12} rad, speed: {speed12} rad/s")

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create and run node
    node = ShooterDamiaoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()