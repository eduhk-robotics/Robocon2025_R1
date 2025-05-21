#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from joystick_msgs.msg import Joystick # Import the custom message (ensure this path is correct for your setup)
from evdev import InputDevice, ecodes, categorize, list_devices, util
import os
import time # For a small delay if device not found
        
class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher_node')

        # Declare parameters
        self.declare_parameter('device_path', '/dev/input/8bitdo_joystick') 
        self.declare_parameter('deadzone_lx', 0) # Deadzone for lx, 0 means no deadzone
        self.declare_parameter('deadzone_ly', 0)
        self.declare_parameter('deadzone_rx', 0)
        self.declare_parameter('deadzone_ry', 0)
        self.declare_parameter('deadzone_l2', 0) # Changed from deadzone_lt
        self.declare_parameter('deadzone_r2', 0) # Changed from deadzone_rt


        # Get parameters
        self.device_path = self.get_parameter('device_path').get_parameter_value().string_value
        self.deadzones = {
            "lx": self.get_parameter('deadzone_lx').get_parameter_value().integer_value,
            "ly": self.get_parameter('deadzone_ly').get_parameter_value().integer_value,
            "rx": self.get_parameter('deadzone_rx').get_parameter_value().integer_value,
            "ry": self.get_parameter('deadzone_ry').get_parameter_value().integer_value,
            "l2": self.get_parameter('deadzone_l2').get_parameter_value().integer_value, # Changed from lt
            "r2": self.get_parameter('deadzone_r2').get_parameter_value().integer_value, # Changed from rt
        }

        self.publisher_ = self.create_publisher(Joystick, 'joystick_input', 10)
        self.get_logger().info(f"Attempting to connect to joystick at: {self.device_path}")

        try:
            self.gamepad = InputDevice(self.device_path)
            self.get_logger().info(f"Connected to device: {self.gamepad.name}")
        except FileNotFoundError:
            self.get_logger().error(f"Device not found at {self.device_path}. Ensure it's connected and path is correct.")
            self.get_logger().info("Available input devices:")
            try:
                devices = [InputDevice(path) for path in list_devices()]
                for device in devices:
                    self.get_logger().info(f"  Path: {device.path}, Name: {device.name}, Phys: {device.phys}")
            except Exception as e:
                self.get_logger().warn(f"Could not list input devices: {e}")
            rclpy.shutdown()
            return
        except PermissionError:
            self.get_logger().error(f"Permission denied for {self.device_path}. Check user permissions (add to 'input' group?)")
            rclpy.shutdown()
            return
        except Exception as e:
            self.get_logger().error(f"Failed to connect to joystick: {e}")
            rclpy.shutdown()
            return


        # Initial states (maps to Joystick.msg fields)
        self.button_states_bool = {
            "a": False, "b": False, "x": False, "y": False,
            "l1": False, "r1": False, "l3": False, "r3": False, # Changed lb->l1, rb->r1
            "select": False, "start": False, # Changed minus->select, plus->start
        }
        self.axis_states = {
            "lx": 0, "ly": 0, "rx": 0, "ry": 0,
            "dx": 0, "dy": 0, "l2": 0, "r2": 0, # Changed cx->dx, cy->dy, lt->l2, rt->r2
        }

        # Event code mappings (adjust these based on your specific joystick)
        # Common XInput-like mappings:
        self.button_mapping = {
            ecodes.BTN_SOUTH: "a",    # Or 304
            ecodes.BTN_EAST: "b",     # Or 305
            ecodes.BTN_WEST: "x",     # Or 307 (some use BTN_NORTH for X)
            ecodes.BTN_NORTH: "y",    # Or 308 (some use BTN_WEST for Y)
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

        # Start the reading loop
        self.read_and_publish_loop()

    def apply_deadzone(self, value, deadzone_val):
        if abs(value) < deadzone_val:
            return 0
        return value

    def read_and_publish_loop(self):
        self.get_logger().info("Starting to read joystick events...")
        try:
            for event in self.gamepad.read_loop():
                if not rclpy.ok(): # Check if ROS is still running
                    break

                updated = False
                # Handle button events
                if event.type == ecodes.EV_KEY:
                    button_name = self.button_mapping.get(event.code)
                    if button_name:
                        self.button_states_bool[button_name] = (event.value == 1) # 1 for pressed, 0 for released
                        updated = True

                # Handle axis events
                elif event.type == ecodes.EV_ABS:
                    axis_name = self.axis_mapping.get(event.code)
                    if axis_name:
                        raw_value = event.value
                        # Apply deadzone if applicable and configured
                        if axis_name in self.deadzones:
                            deadzone_val = self.deadzones[axis_name]
                            self.axis_states[axis_name] = self.apply_deadzone(raw_value, deadzone_val)
                        else:
                            # For axes without configured deadzones (like d-pad dx, dy), use raw value
                            self.axis_states[axis_name] = raw_value
                        updated = True
                
                # If any state changed, create and publish message
                if updated:
                    msg = Joystick()
                    # Populate axis values from Joystick.msg
                    msg.lx = self.axis_states['lx']
                    msg.ly = self.axis_states['ly'] # evdev Y often inverted; negate if needed: -self.axis_states['ly']
                    msg.rx = self.axis_states['rx']
                    msg.ry = self.axis_states['ry'] # evdev Y often inverted; negate if needed: -self.axis_states['ry']
                    msg.dx = self.axis_states['dx']
                    msg.dy = self.axis_states['dy'] # evdev D-pad Y often inverted; negate if needed
                    msg.l2 = self.axis_states['l2']
                    msg.r2 = self.axis_states['r2']

                    # Populate button values from Joystick.msg
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
                    # self.get_logger().debug(f"Published: {msg}")

        except KeyboardInterrupt:
            self.get_logger().info("Joystick reader interrupted. Exiting...")
        except OSError as e: 
            self.get_logger().error(f"Joystick OSError: {e}. Device may have been disconnected.")
        finally:
            if hasattr(self, 'gamepad') and self.gamepad:
                self.gamepad.close()
            self.get_logger().info("Joystick publisher node shutting down.")


def main(args=None):
    rclpy.init(args=args)
    joystick_publisher = None # Initialize to None
    try:
        joystick_publisher = JoystickPublisher()
        # If __init__ calls rclpy.shutdown() due to error, rclpy.ok() will become false.
        # The read_and_publish_loop is blocking and called from __init__.
        # If it exits (e.g., device disconnect, Ctrl+C), the node should shut down.
        # This simple loop keeps main alive if read_and_publish_loop somehow exits
        # before rclpy.shutdown() is called or if it's run in a non-blocking way in future.
        while rclpy.ok() and hasattr(joystick_publisher, 'gamepad') and joystick_publisher.gamepad.fd != -1:
            # Check if the node is still valid and gamepad is connected
            if joystick_publisher is None or not rclpy.ok(): # joystick_publisher could be None if __init__ failed early
                break
            rclpy.spin_once(joystick_publisher, timeout_sec=0.1) # Allow callbacks if any, process shutdown events
    except KeyboardInterrupt:
        if joystick_publisher:
            joystick_publisher.get_logger().info("Main loop interrupted by KeyboardInterrupt.")
    except Exception as e:
        if joystick_publisher: # Check if joystick_publisher was successfully initialized
            joystick_publisher.get_logger().error(f"Unhandled exception in main: {e}")
        else:
            print(f"Unhandled exception before/during JoystickPublisher initialization: {e}")
    finally:
        if joystick_publisher and rclpy.ok() and joystick_publisher.gamepad.fd != -1: # Check fd as well
             # gamepad might be closed by read_and_publish_loop's finally block
            if hasattr(joystick_publisher, 'gamepad') and joystick_publisher.gamepad and joystick_publisher.gamepad.fd != -1:
                 joystick_publisher.gamepad.close()
        if joystick_publisher and rclpy.ok():
            joystick_publisher.destroy_node()
        if rclpy.ok(): # Ensure shutdown is only called if not already called
            rclpy.shutdown()

if __name__ == '__main__':
    main()
