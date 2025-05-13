import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from damiao.DM_CAN import *  # Updated import statement (relative to the package structure) import *  # Import your motor control library (everything from DM_can.py)
import serial
import os

# USB-CDC Device ID (replace with the correct ID for your device)
DEVICE_ID = "usb-HDSC_CDC_Device_00000000050C-if00"

def find_device_port(device_id):
    """
    Scan /dev/serial/by-id for the device and return its real tty path, or None if not found.
    """
    by_id_dir = "/dev/serial/by-id/"
    try:
        for entry in os.listdir(by_id_dir):
            if device_id in entry:
                return os.path.realpath(os.path.join(by_id_dir, entry))
    except FileNotFoundError:
        pass
    return None

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__("motor_controller_node")

        # Try to locate the serial device
        self.get_logger().info(f"Looking for USB serial device with ID '{DEVICE_ID}'...")
        port = None
        while port is None:
            port = find_device_port(DEVICE_ID)
            if port is None:
                self.get_logger().warn("  Device not found, retrying in 1 second...")
                time.sleep(1)
            else:
                self.get_logger().info(f"Found device at '{port}'.")

        # Open serial connection and initialize MotorControl
        self.serial_device = serial.Serial(port, 921600, timeout=0.5)
        if self.serial_device.isOpen():
            self.get_logger().info("Successfully opened USB-CDC device.")
        else:
            self.get_logger().error("Failed to open the serial device!")
            raise RuntimeError("Could not open the serial port.")

        # Initialize MotorControl
        self.motor_control = MotorControl(self.serial_device)

        # Publisher for motor statuses
        self.status_publisher = self.create_publisher(Float32MultiArray, "damiao_status", 10)

        # Subscriber for motor control commands
        self.control_subscriber = self.create_subscription(
            Float32MultiArray,
            "damiao_control",  # Receive commands on this topic
            self.control_callback,
            10
        )

        # Timer to continuously publish motor statuses at 10 Hz
        self.timer = self.create_timer(0.1, self.status_timer_callback)  # 0.1 seconds = 10 Hz

        # Initialize motors with IDs 1 to 8
        self.motors = [Motor(DM_Motor_Type.DM4310, i, 0x00) for i in range(1, 5)]

        # Add and configure all motors
        for motor in self.motors:
            self.motor_control.addMotor(motor)
            self.motor_control.enable(motor)  # Enable the motor
            self.motor_control.set_zero_position(motor)  # Set the zero position

        self.get_logger().info("Motor controller node initialized: Publishing on 'damiao_status' and listening on 'damiao_control'.")

    def status_timer_callback(self):
        """Continuously publish motor statuses."""
        status_msg = Float32MultiArray()

        # Collect current status from all motors
        for motor in self.motors:
            self.motor_control.refresh_motor_status(motor)  # Refresh motor status from the hardware
            status_msg.data.extend([  # Format: [Motor ID, Position, Velocity, Torque]
                float(motor.SlaveID),
                motor.getPosition(),
                motor.getVelocity(),
                motor.getTorque()
            ])

        # Publish the status message
        self.status_publisher.publish(status_msg)

        # Optional: Log the data for debugging (can be removed in production for better performance)
        self.get_logger().debug(f"Published Motor Status: {status_msg.data}")

    def control_callback(self, msg):
        """Handle incoming motor control commands from 'damiao_control' topic."""
        if len(msg.data) < 4:
            self.get_logger().warn("Received invalid control message. Not enough data!")
            return

        # Extract motor control data
        motor_id = int(msg.data[0])  # Motor ID (1-based index)
        mode = int(msg.data[1])      # Control mode (0: Stop, 1: Speed, 2: Position & Speed)
        speed = float(msg.data[2])   # Desired speed
        position = float(msg.data[3])  # Desired position (only used in mode 2: position control)

        self.get_logger().info(f"Received Motor Control Command - Motor ID: {motor_id}, Mode: {mode}, Speed: {speed}, Position: {position}")

        # Validate motor ID
        if motor_id < 1 or motor_id > len(self.motors):
            self.get_logger().error(f"Motor ID {motor_id} is out of range (1-{len(self.motors)}). Ignoring command.")
            return

        # Get the corresponding motor object
        motor = self.motors[motor_id - 1]  # Motor IDs are 1-based, so subtract 1 for array index

        # Handle modes
        if mode == 0:  # Stop motor
            self.motor_control.disable(motor)
            self.get_logger().info(f"Motor {motor_id} stopped.")

        elif mode == 1:  # Speed control mode
            self.motor_control.control_Vel(motor, speed)
            self.get_logger().info(f"Motor {motor_id} set to speed {speed}.")

        elif mode == 2:  # Position and speed control mode
            self.motor_control.control_Pos_Vel(motor, position, speed)
            self.get_logger().info(f"Motor {motor_id} set to position {position} with speed {speed}.")
        else:
            self.get_logger().warn(f"Unknown mode {mode} received. Ignoring command.")


def main(args=None):
    # Initialize ROS 2 Node
    rclpy.init(args=args)

    # Create and spin the motor controller node
    motor_controller_node = MotorControllerNode()
    try:
        rclpy.spin(motor_controller_node)
    finally:
        motor_controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    
    #test
