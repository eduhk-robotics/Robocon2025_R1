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

        # Declare parameter for device path
        self.declare_parameter('device_path', '/dev/input/8bitdo_joystick_purple')
        self.device_path = self.get_parameter('device_path').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Joystick, 'joystick_input', 10)
        self.get_logger().info(f"Attempting to connect to joystick at: {self.device_path}")

        # Initialize button and axis state dictionaries
        self.button_states_bool = {
            "a": False, "b": False, "x": False, "y": False,
            "l1": False, "r1": False, "l3": False, "r3": False, 
            "select": False, "start": False,
        }

        self.axis_states = {
            "lx": 0, "ly": 0, "rx": 0, "ry": 0,
            "dx": 0, "dy": 0, "l2": 0, "r2": 0,
        }

        # EV_KEY codes ? button name
        self.button_mapping = {
            ecodes.BTN_SOUTH: "a",    # Or 304
            ecodes.BTN_EAST: "b",     # Or 305
            ecodes.BTN_WEST: "y",     # Or 307 (some use BTN_NORTH for Y)
            ecodes.BTN_NORTH: "x",    # Or 308 (some use BTN_WEST for X)
            ecodes.BTN_TL: "l1",      # Or 310
            ecodes.BTN_TR: "r1",      # Or 311
            ecodes.BTN_THUMBL: "l3",  # Or 317
            ecodes.BTN_THUMBR: "r3",  # Or 318
            ecodes.BTN_SELECT: "select",# Or 314
            ecodes.BTN_START: "start",  # Or 315
        }
        self.axis_mapping = {
            ecodes.ABS_X: "lx",       # Or 0
            ecodes.ABS_Y: "ly",       # Or 1
            ecodes.ABS_RX: "rx",      # Or 3
            ecodes.ABS_RY: "ry",      # Or 4
            ecodes.ABS_HAT0X: "dx",   # Or 16
            ecodes.ABS_HAT0Y: "dy",   # Or 17
            ecodes.ABS_Z: "l2",       # Or 2 (Often left trigger)
            ecodes.ABS_RZ: "r2",      # Or 5 (Often right trigger)
        }

        # Flag to control read loop
        self.running = True

        # Start a background thread to read events (non-blocking for ROS)
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

        # Create a timer to publish at 20 Hz (every 0.05 s)
        self.publish_timer = self.create_timer(0.05, self._publish_current_state)

    def _try_connect(self):
        """Attempt to connect to the joystick device, return True if successful."""
        try:
            self.gamepad = InputDevice(self.device_path)
            self.get_logger().info(f"Connected to device: {self.gamepad.name}")
            return True
        except FileNotFoundError:
            self.get_logger().warn(f"Device not found at {self.device_path}. Retrying...")
            return False
        except PermissionError:
            self.get_logger().error(f"Permission denied for {self.device_path}. Check user permissions (e.g., add your user to 'input' group).")
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
        msg.ly = self.axis_states['ly'] # evdev Y often inverted; negate if needed: -self.axis_states['ly']
        msg.rx = self.axis_states['rx']
        msg.ry = self.axis_states['ry'] # evdev Y often inverted; negate if needed: -self.axis_states['ry']
        msg.dx = self.axis_states['dx']
        msg.dy = self.axis_states['dy'] # evdev D-pad Y often inverted; negate if needed
        msg.l2 = self.axis_states['l2']
        msg.r2 = self.axis_states['r2']


        # Buttons
        msg.a = self.button_states_bool['a']
        msg.b = self.button_states_bool['b']
        msg.x = self.button_states_bool['x']
        msg.y = self.button_states_bool['y']
        msg.l1 = self.button_states_bool['l1']
        msg.r1 = self.button_states_bool['r1']
        msg.l3 = self.button_states_bool['l3']
        msg.r3 = self.button_states_bool['r3']
        msg.select = self.button_states_bool['select']
        msg.start = self.button_states_bool['start']

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
