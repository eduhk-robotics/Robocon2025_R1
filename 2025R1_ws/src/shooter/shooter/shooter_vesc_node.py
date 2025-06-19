import subprocess
import time
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Motor speed constants
HEARTBEAT_DT = 0.05    # seconds between RPM commands
MAX_RPM = 80000         # VESC max motor RPM
DESIRED_IDS = {10, 11, 12}  # CAN IDs of VESCs
CAN_CHANNEL = 'can0'   # CAN interface name

class VescCanNode(Node):
    def __init__(self):
        super().__init__('shooter_vesc_node')
        self.get_logger().info('VescCanNode starting')

        # Desired RPM set by incoming command
        self.desired_rpm = 0

        # Subscribe to RPM commands
        self.create_subscription(
            Float32,
            'shooter_vesc_control',
            self.rpm_callback,
            10
        )

        # Check CAN interface
        if not self.check_can_interface():
            self.get_logger().error(f'CAN interface {CAN_CHANNEL} not available')
            return

        # Check cansend availability
        if os.system("which cansend > /dev/null") != 0:
            self.get_logger().error("cansend not found. Install can-utils: 'sudo apt install can-utils'")
            return

        # Start heartbeat timer to send RPM
        self.create_timer(HEARTBEAT_DT, self.heartbeat)

    def check_can_interface(self):
        """Check if the CAN interface is available and up"""
        try:
            result = subprocess.run(f"ip link show {CAN_CHANNEL}", shell=True, capture_output=True, text=True, check=True)
            if "UP" not in result.stdout:
                self.get_logger().error(f"CAN interface {CAN_CHANNEL} is not up. Run 'sudo ip link set {CAN_CHANNEL} up type can bitrate 500000'")
                return False
            return True
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"CAN interface {CAN_CHANNEL} not found or inaccessible: {e}")
            return False

    def send_can_frame(self, can_id, rpm):
        """Send RPM command to VESC via CAN bus using cansend"""
        arb_id = f"{0x300 | can_id:08X}"  # Construct arbitration ID, e.g., 0000030A for ID 10
        rpm_data = f"{int(rpm):08X}"      # Convert RPM to hex, e.g., 00001388
        command = f"cansend {CAN_CHANNEL} {arb_id}#{rpm_data}"
        try:
            subprocess.run(command, shell=True, check=True)
            self.get_logger().info(f"Sent: {command}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to send CAN frame: {e}")

    def rpm_callback(self, msg: Float32):
        """Handle incoming RPM command from topic"""
        raw_rpm = float(msg.data)
        norm = max(min(raw_rpm / MAX_RPM, 1.0), -1.0)
        self.desired_rpm = int(norm * MAX_RPM)
        self.get_logger().info(f"Received RPM command: {self.desired_rpm}")

    def heartbeat(self):
        """Send RPM commands to all VESCs periodically"""
        for can_id in DESIRED_IDS:
            try:
                self.send_can_frame(can_id, self.desired_rpm)
            except Exception as e:
                self.get_logger().error(f"Error sending to VESC ID {can_id}: {e}")

    def destroy_node(self):
        """Stop all VESC motors on node shutdown"""
        self.get_logger().info("Stopping all VESC motors")
        for can_id in DESIRED_IDS:
            try:
                self.send_can_frame(can_id, 0)
            except Exception as e:
                self.get_logger().error(f"Failed to send stop command to VESC ID {can_id}: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VescCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Node error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


