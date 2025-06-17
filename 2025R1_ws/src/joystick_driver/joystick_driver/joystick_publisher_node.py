#!/usr/bin/env python3
import threading
import time
import rclpy
from rclpy.node import Node
from joystick_msgs.msg import Joystick  # your custom message type
from evdev import InputDevice, ecodes, list_devices

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher_node')

        # Declare parameter for device name (instead of path)
        self.declare_parameter('device_name', '8BitDo Pro 2 Wired Controller')
        self.device_name = self.get_parameter('device_name').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Joystick, 'joystick_input', 10)
        self.get_logger().info(f"Attempting to connect to joystick: {self.device_name}")

        # Initialize button and axis state dictionaries
        self.button_states_bool = {
            "a": False, "b": False, "x": False, "y": False,
            "lb": False, "rb": False, "l3": False, "r3": False,
            "minus": False, "plus": False,
        }
        self.axis_states = {
            "lx": 0, "ly": 0, "rx": 0, "ry": 0,
            "cx": 0, "cy": 0, "lt": 0, "rt": 0,
        }

        # EV_KEY codes → button name
        self.button_mapping = {
            304: "a", 305: "b", 307: "x", 308: "y",
            310: "lb", 311: "rb", 317: "l3", 318: "r3",
            314: "minus", 315: "plus",
        }
        # EV_ABS codes → axis name
        self.axis_mapping = {
            0: "lx", 1: "ly", 3: "rx", 4: "ry",
            16: "cx", 17: "cy", 2: "lt", 5: "rt",
        }

        # Flag to control read loop
        self.running = True

        # Start a background thread to read events (non-blocking for ROS)
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

        # Create a timer to publish at 20 Hz (every 0.05 s)
        self.publish_timer = self.create_timer(0.05, self._publish_current_state)

    def _try_connect(self):
        """Attempt to connect to the joystick device by name, return True if successful."""
        try:
            devices = [InputDevice(path) for path in list_devices()]
            for dev in devices:
                if self.device_name in dev.name:  # Match device by name
                    self.device_path = dev.path
                    self.gamepad = InputDevice(self.device_path)
                    self.get_logger().info(f"Connected to device: {self.gamepad.name} at {self.device_path}")
                    return True
            self.get_logger().warn(f"No matching device found for '{self.device_name}'. Retrying...")
            self._print_available_devices()
            return False
        except PermissionError:
            self.get_logger().error(f"Permission denied for input devices. Check user permissions (e.g., add your user to 'input' group).")
            return False
        except Exception as e:
            self.get_logger().error(f"Failed to connect to joystick: {e}")
            return False

    def _print_available_devices(self):
        """Log available input devices for debugging."""
        self.get_logger().info("Available input devices:")
        devices = [InputDevice(path) for path in list_devices()]
        for dev in devices:
            self.get_logger().info(f"  Path: {dev.path}, Name: {dev.name}, Phys: {dev.phys}")

    def _read_loop(self):
        """Background thread: continuously read joystick events and handle reconnection."""
        while self.running and rclpy.ok():
            # Attempt to connect if not already connected
            if not hasattr(self, 'gamepad') or self.gamepad is None:
                if not self._try_connect():
                    self._print_available_devices()
                    time.sleep(1)  # Wait before retrying
                    continue

            try:
                for event in self.gamepad.read_loop():
                    if not self.running or not rclpy.ok():
                        break

                    if event.type == ecodes.EV_KEY:
                        btn = self.button_mapping.get(event.code)
                        if btn is not None:
                            self.button_states_bool[btn] = (event.value == 1)
                    elif event.type == ecodes.EV_ABS:
                        axis = self.axis_mapping.get(event.code)
                        if axis is not None:
                            self.axis_states[axis] = event.value
            except OSError as e:
                self.get_logger().error(f"Joystick OSError in read loop: {e}. Device may have been disconnected.")
                if hasattr(self, 'gamepad') and self.gamepad:
                    self.gamepad.close()
                    self.gamepad = None
                time.sleep(2)  # Wait before attempting to reconnect
            except Exception as e:
                self.get_logger().error(f"Unexpected error in read loop: {e}")
                if hasattr(self, 'gamepad') and self.gamepad:
                    self.gamepad.close()
                    self.gamepad = None
                time.sleep(2)

        # Clean up on exit
        if hasattr(self, 'gamepad') and self.gamepad:
            self.gamepad.close()
            self.get_logger().info("Joystick device closed.")

    def _publish_current_state(self):
        """Timer callback: publish the latest axis/button states every 0.05 s."""
        msg = Joystick()

        # Axes
        msg.lx = self.axis_states['lx']
        msg.ly = self.axis_states['ly']
        msg.rx = self.axis_states['rx']
        msg.ry = self.axis_states['ry']
        msg.cx = self.axis_states['cx']
        msg.cy = self.axis_states['cy']
        msg.lt = self.axis_states['lt']
        msg.rt = self.axis_states['rt']

        # Buttons
        msg.a = self.button_states_bool['a']
        msg.b = self.button_states_bool['b']
        msg.x = self.button_states_bool['x']
        msg.y = self.button_states_bool['y']
        msg.lb = self.button_states_bool['lb']
        msg.rb = self.button_states_bool['rb']
        msg.l3 = self.button_states_bool['l3']
        msg.r3 = self.button_states_bool['r3']
        msg.minus = self.button_states_bool['minus']
        msg.plus = self.button_states_bool['plus']

        self.publisher_.publish(msg)

    def destroy_node(self):
        """Clean up resources when shutting down."""
        self.running = False
        if hasattr(self, 'publish_timer'):
            self.publish_timer.cancel()
        if hasattr(self, 'gamepad') and self.gamepad:
            self.gamepad.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    joystick_publisher = JoystickPublisher()

    try:
        rclpy.spin(joystick_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if joystick_publisher is not None:
            joystick_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()