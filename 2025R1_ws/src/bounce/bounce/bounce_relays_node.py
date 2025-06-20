#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time
try:
    import gpiod
except ImportError as e:
    print(f"Failed to import gpiod: {e}")
    raise

class BounceRelaysNode(Node):
    def __init__(self):
        super().__init__('bounce_relays_node')
        # Open GPIO chip
        try:
            self.chip = gpiod.Chip('gpiochip0')
            self.relay1_pin = 17  # GPIO 17 (Pin 11) for rack
            self.relay2_pin = 27  # GPIO 27 (Pin 13) for claw
            self.relay3_pin = 22  # GPIO 22 (Pin 15) for bouncer
            # Configure pins as outputs
            self.lines = {}
            for pin in [self.relay1_pin, self.relay2_pin, self.relay3_pin]:
                line = self.chip.get_line(pin)
                line.request(consumer='bounce_relays', type=gpiod.LINE_REQ_DIR_OUT)
                line.set_value(0)  # Initialize to low
                self.lines[pin] = line
        except Exception as e:
            self.get_logger().error(f"GPIO setup failed: {e}")
            raise
        # Subscriber for bounce_control
        self.subscription = self.create_subscription(
            Bool,
            'bounce_control',
            self.bounce_callback,
            10)
        self.get_logger().info('Bounce Relays Node started')

    def set_relay(self, relay, value):
        pin = {1: self.relay1_pin, 2: self.relay2_pin, 3: self.relay3_pin}
        try:
            self.lines[pin[relay]].set_value(1 if value else 0)
            self.get_logger().info(f'Relay{relay} ({"rack" if relay == 1 else "claw" if relay == 2 else "bouncer"}) set to {value}')
        except Exception as e:
            self.get_logger().error(f"Failed to set Relay{relay}: {e}")

    def bounce_callback(self, msg):
        if msg.data:
            self.get_logger().info('Received start_bounce: True, executing relay sequence')
            try:
                # Relay sequence
                self.set_relay(1, True)  # Relay1 (rack) high
                time.sleep(0.5)  # Wait 0.5s
                self.set_relay(2, True)  # Relay2 (claw) high
                self.set_relay(3, True)  # Relay3 (bouncer) high
                self.set_relay(3, False)  # Relay3 (bouncer) immediately low
                time.sleep(0.2)  # Wait 0.2s
                self.set_relay(2, False)  # Relay2 (claw) low
                time.sleep(0.2)  # Wait 0.2s
                self.set_relay(1, False)  # Relay1 (rack) low
            except Exception as e:
                self.get_logger().error(f"Relay sequence failed: {e}")
        else:
            self.get_logger().info('Received start_bounce: False, no action')

    def __del__(self):
        try:
            for pin in [self.relay1_pin, self.relay2_pin, self.relay3_pin]:
                self.lines[pin].set_value(0)
                self.lines[pin].release()
            self.chip.close()
            self.get_logger().info('GPIO cleaned up')
        except Exception as e:
            self.get_logger().error(f"GPIO cleanup failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = BounceRelaysNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in node: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()