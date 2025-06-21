#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import gpiod
import time
import os
import logging

class RelayControlNode(Node):
    def __init__(self):
        super().__init__('relay_control_node')
        logging.getLogger().setLevel(logging.DEBUG)  # Enable debug logging
        self.publisher_ = self.create_publisher(String, '/relay_status', 10)
        self.chip_path = '/dev/gpiochip4'
        self.chip = None
        self.lines = {}
        self.relay1_pin = 17  # GPIO 17 (Pin 11) for rack
        self.relay2_pin = 22  # GPIO 22 (Pin 15) for claw
        self.relay3_pin = 27  # GPIO 26 (Pin 36) for bouncer
        self.get_logger().info('Relay Control Node initialized')

        try:
            if not os.path.exists(self.chip_path):
                raise FileNotFoundError(f"GPIO chip device {self.chip_path} not found")
            self.get_logger().info(f"Accessing {self.chip_path} as user {os.getlogin()}")
            self.get_logger().info(f"Permissions: {os.stat(self.chip_path).st_mode & 0o777:03o}, Owner: {os.stat(self.chip_path).st_uid}, Group: {os.stat(self.chip_path).st_gid}")
            self.chip = gpiod.Chip('gpiochip4')
            for pin in [self.relay1_pin, self.relay2_pin, self.relay3_pin]:
                line = self.chip.get_line(pin)
                line.request(consumer='robotics', type=gpiod.LINE_REQ_DIR_OUT)
                line.set_value(0)  # Initialize to low
                self.lines[pin] = line
                self.get_logger().debug(f"Initialized GPIO {pin} successfully")
            self.get_logger().info("All GPIOs initialized successfully")
        except Exception as e:
            self.get_logger().error(f"GPIO setup failed: {e}")
            self.get_logger().error("提示：尝试以下备用方案：")
            self.get_logger().error("1. 使用 'gpiochip0' 代替（适用于树莓派4）")
            self.get_logger().error("2. 检查引脚映射（树莓派5使用不同编号）")
            self.cleanup()
            raise

    def set_relay(self, relay, value):
        pin = {1: self.relay1_pin, 2: self.relay2_pin, 3: self.relay3_pin}
        name = {1: "rack", 2: "claw", 3: "bouncer"}
        try:
            if pin[relay] not in self.lines:
                raise ValueError(f"Relay{relay} (GPIO {pin[relay]}) not initialized")
            self.lines[pin[relay]].set_value(1 if value else 0)  # High for True
            msg = String()
            msg.data = f'Relay{relay} ({name[relay]}) set to {value}'
            self.publisher_.publish(msg)
            self.get_logger().info(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to set Relay{relay} (GPIO {pin[relay]}): {e}")
            raise

    def run_sequence(self):
        try:
            self.get_logger().info("Initializing relays to low")
            self.set_relay(1, False)
            self.set_relay(2, False)
            self.set_relay(3, False)

            self.get_logger().info("Executing relay sequence")
            self.set_relay(1, True)  # Relay1 (rack) release
            time.sleep(1.5)  # Wait 1.5s
            self.set_relay(2, True)  # Relay2 (claw) release
            time.sleep(1.5)  # Wait 1.5s
            self.set_relay(3, False)  # Relay3 (bouncer) drill
            self.set_relay(3, True)  # Relay3 (bouncer) stop
            self.set_relay(2, False)  # Relay2 (claw) stop
            time.sleep(1.5)  # Wait 1.5s
            
            time.sleep(1.0)  # Wait 1.0s
            self.set_relay(1, False)  # Relay1 (rack) stop
            self.get_logger().info("Relay sequence completed")
        except Exception as e:
            self.get_logger().error(f"Relay sequence failed: {e}")
            raise

    def cleanup(self):
        try:
            for pin, line in self.lines.items():
                line.set_value(0)
                line.release()
                self.get_logger().debug(f"Released GPIO {pin}")
            if self.chip:
                self.chip.close()
                self.get_logger().debug("Closed chip")
            self.get_logger().info("GPIO cleaned up")
        except Exception as e:
            self.get_logger().error(f"GPIO cleanup failed: {e}")
        finally:
            self.chip = None
            self.lines = {}

    def destroy_node(self):
        self.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RelayControlNode()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
