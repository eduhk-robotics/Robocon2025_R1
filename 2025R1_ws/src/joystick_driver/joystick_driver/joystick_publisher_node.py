#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from joystick_msgs.msg import Joystick # Import your custom message
from evdev import InputDevice, ecodes, categorize, list_devices, util
import os
import time # For a small delay if device not found

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher_node')

        # Declare parameters
        self.declare_parameter('device_path', '/dev/input/8bitdo_joystick') # Default device path
        self.declare_parameter('deadzone_lx', 0) # Deadzone for lx, 0 means no deadzone
        self.declare_parameter('deadzone_ly', 0)
        self.declare_parameter('deadzone_rx', 0)
        self.declare_parameter('deadzone_ry', 0)
        self.declare_parameter('deadzone_lt', 0)
        self.declare_parameter('deadzone_rt', 0)


        # Get parameters
        self.device_path = self.get_parameter('device_path').get_parameter_value().string_value
        self.deadzones = {
            "lx": self.get_parameter('deadzone_lx').get_parameter_value().integer_value,
            "ly": self.get_parameter('deadzone_ly').get_parameter_value().integer_value,
            "rx": self.get_parameter('deadzone_rx').get_parameter_value().integer_value,
            "ry": self.get_parameter('deadzone_ry').get_parameter_value().integer_value,
            "lt": self.get_parameter('deadzone_lt').get_parameter_value().integer_value,
            "rt": self.get_parameter('deadzone_rt').get_parameter_value().integer_value,
        }

        self.publisher_ = self.create_publisher(Joystick, 'joystick_input', 10)
        self.get_logger().info(f"Attempting to connect to joystick at: {self.device_path}")

        try:
            self.gamepad = InputDevice(self.device_path)
            self.get_logger().info(f"Connected to device: {self.gamepad.name}")
        except FileNotFoundError:
            self.get_logger().error(f"Device not found at {self.device_path}. Ensure it's connected and path is correct.")
            self.get_logger().info("Available input devices:")
            devices = [InputDevice(path) for path in list_devices()]
            for device in devices:
                self.get_logger().info(f"  Path: {device.path}, Name: {device.name}, Phys: {device.phys}")
            # Allow rclpy to shutdown gracefully
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


        # Initial states (mirrors your script's structure)
        self.button_states_bool = {
            "a": False, "b": False, "x": False, "y": False,
            "lb": False, "rb": False, "l3": False, "r3": False,
            "minus": False, "plus": False,
        }
        self.axis_states = {
            "lx": 0, "ly": 0, "rx": 0, "ry": 0,
            "cx": 0, "cy": 0, "lt": 0, "rt": 0,
        }

        # Event code mappings (from your script)
        self.button_mapping = {
            304: "a", 305: "b", 307: "x", 308: "y",
            310: "lb", 311: "rb", 317: "l3", 318: "r3",
            314: "minus", 315: "plus",
        }
        self.axis_mapping = {
            0: "lx", 1: "ly", 3: "rx", 4: "ry",
            16: "cx", 17: "cy", 2: "lt", 5: "rt",
        }

        # Start the reading loop in a separate thread or use a timer
        # For simplicity with evdev's blocking read_loop, we'll run it directly
        # and publish immediately. This means the node will be busy in this loop.
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
                        # Apply deadzone if applicable
                        if axis_name in self.deadzones:
                            deadzone_val = self.deadzones[axis_name]
                            # For stick axes (lx, ly, rx, ry), often centered at 0
                            # For triggers (lt, rt), often start at 0
                            # Hat (cx, cy) are usually -1, 0, 1
                            # Assuming triggers (lt, rt) and analog sticks (lx, ly, rx, ry) might need deadzones.
                            # Hat axes (cx, cy) are usually precise and don't need deadzoning.
                            if axis_name not in ["cx", "cy"]:
                                self.axis_states[axis_name] = self.apply_deadzone(raw_value, deadzone_val)
                            else:
                                self.axis_states[axis_name] = raw_value # No deadzone for hat typically
                        else:
                            self.axis_states[axis_name] = raw_value
                        updated = True
                
                # If any state changed, create and publish message
                if updated:
                    msg = Joystick()
                    # Populate axis values
                    msg.lx = self.axis_states['lx']
                    msg.ly = self.axis_states['ly'] # evdev Y often inverted; adjust if needed by negating
                    msg.rx = self.axis_states['rx']
                    msg.ry = self.axis_states['ry'] # evdev Y often inverted
                    msg.cx = self.axis_states['cx']
                    msg.cy = self.axis_states['cy'] # evdev Y often inverted for hat
                    msg.lt = self.axis_states['lt']
                    msg.rt = self.axis_states['rt']

                    # Populate button values
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
                    # Optional: Log the published message for debugging
                    # self.get_logger().debug(f"Published: {msg}")

        except KeyboardInterrupt:
            self.get_logger().info("Joystick reader interrupted. Exiting...")
        except OSError as e: # Catches errors like device unplugged
            self.get_logger().error(f"Joystick OSError: {e}. Device may have been disconnected.")
        finally:
            if hasattr(self, 'gamepad') and self.gamepad:
                self.gamepad.close()
            self.get_logger().info("Joystick publisher node shutting down.")


def main(args=None):
    rclpy.init(args=args)
    joystick_publisher = JoystickPublisher()
    # rclpy.spin() is not used here because read_and_publish_loop is blocking
    # The loop itself will keep the node alive until Ctrl+C or error
    # However, if read_and_publish_loop finishes (e.g. device disconnect),
    # we need to ensure the node cleans up.
    # The try/except/finally in read_and_publish_loop handles KeyboardInterrupt.
    # If an error occurs causing read_loop to exit, the node might terminate.
    # For robustness, you might run read_and_publish_loop in a separate thread
    # and use rclpy.spin(joystick_publisher) here, but this adds complexity.

    # If joystick_publisher initialization failed (e.g. device not found)
    # rclpy.ok() might be false if rclpy.shutdown() was called in __init__
    if rclpy.ok():
        try:
            # The node's main work is done in read_and_publish_loop,
            # which is called in __init__. If it exits, we want to shut down.
            # A simple way to keep main alive until node is done or interrupted:
            while rclpy.ok() and hasattr(joystick_publisher, 'gamepad') and joystick_publisher.gamepad.fd != -1:
                rclpy.spin_once(joystick_publisher, timeout_sec=0.1) # Allow callbacks if any, process shutdown
        except KeyboardInterrupt:
            pass # Already handled in the loop or here
        except Exception as e:
            if joystick_publisher: # Check if joystick_publisher was successfully initialized
                joystick_publisher.get_logger().error(f"Unhandled exception in main: {e}")
        finally:
            if joystick_publisher and rclpy.ok():
                joystick_publisher.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

if __name__ == '__main__':
    main()

