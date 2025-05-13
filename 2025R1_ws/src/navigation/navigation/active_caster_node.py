#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Combined Omni-Wheel Steering and Motor Speed Controller
Subscribes to 'driving' (Float32MultiArray):
 - data[0]: heading in degrees
 - data[1]: speed (-8000 to 8000)
Publishes steering positions to 'damiao_control' and sends RPM commands via VESC CAN.
"""
import math
import time
import serial
from serial.tools import list_ports

import pyvesc
from pyvesc import GetValues, SetRPM

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Steering encoder/gear constants
ENCODER_RATIO = 19.20321
GEAR_RATIO = 35.0 / 61.0
TWO_PI = 2.0 * math.pi
PERIOD_RAD = TWO_PI * (ENCODER_RATIO / GEAR_RATIO)  # encoder rad per full wheel turn

# Motor speed constants
BAUDRATE = 115200
TIMEOUT = 0.1
HEARTBEAT_DT = 0.05    # seconds between RPM commands
MAX_RPM = 6000         # VESC max motor RPM
DESIRED_IDS = {1, 3, 4}  # CAN IDs of VESCs

class OmniWheelMotorController(Node):
    def __init__(self):
        super().__init__('omni_wheel_motor_controller')
        self.get_logger().info('OmniWheelMotorController starting')

        # Last commanded steering position (rad)
        self.last_pos_rad = 0.0
        # Desired RPM set by driving command
        self.desired_rpm = 0

        # Steering command publisher
        self.pos_pub = self.create_publisher(
            Float32MultiArray, 'damiao_control', 10
        )

        # Subscribe to driving commands [heading_deg, speed]
        self.create_subscription(
            Float32MultiArray,
            'driving',
            self.driving_callback,
            10
        )

        # Initialize VESC serial connections
        self.port_map = self.find_vescs()
        if not self.port_map:
            self.get_logger().error(f'No VESCs with CAN IDs {DESIRED_IDS} found')
        else:
            self.serial_map = {}
            for dev in self.port_map:
                ser = serial.Serial(dev, BAUDRATE, timeout=TIMEOUT)
                time.sleep(0.1)
                self.serial_map[dev] = ser

            # Start heartbeat timer to send RPM
            self.create_timer(HEARTBEAT_DT, self.heartbeat)

    def scan_ids_on_port(self, dev):
        found = []
        try:
            with serial.Serial(dev, BAUDRATE, timeout=TIMEOUT) as ser:
                time.sleep(0.1)
                for can_id in DESIRED_IDS:
                    setattr(GetValues, 'can_id', can_id)
                    ser.write(pyvesc.encode_request(GetValues))
                    time.sleep(HEARTBEAT_DT)
                    if ser.in_waiting:
                        data = ser.read(ser.in_waiting)
                        msg, _ = pyvesc.decode(data)
                        if msg:
                            found.append(can_id)
        except Exception as e:
            self.get_logger().warn(f'[scan] error on {dev}: {e}')
        return found

    def find_vescs(self):
        ports = [p.device for p in list_ports.comports() if 'ACM' in p.device or 'USB' in p.device]
        res = {}
        for dev in ports:
            ids = self.scan_ids_on_port(dev)
            if ids:
                res[dev] = ids
                self.get_logger().info(f'Found VESCs {ids} on {dev}')
        return res

    def driving_callback(self, msg: Float32MultiArray):
        # Heading in degrees
        direction_deg = float(msg.data[0])
        # Speed value (-8000..8000)
        raw_speed = float(msg.data[1])

        # 1) Steering: map heading to encoder rad
        wheel_rev   = direction_deg / 360.0
        motor_rev   = wheel_rev / GEAR_RATIO
        encoder_rev = motor_rev * ENCODER_RATIO
        target_rad  = encoder_rev * TWO_PI

        # Choose nearest equivalent to minimize travel
        delta = target_rad - self.last_pos_rad
        if delta >  PERIOD_RAD / 2.0:
            target_rad -= PERIOD_RAD
        elif delta < -PERIOD_RAD / 2.0:
            target_rad += PERIOD_RAD

        # Publish steering command to all 4 wheels
        for motor_id in range(1, 5):
            out = Float32MultiArray()
            out.data = [float(motor_id), 2.0, 40.0, target_rad]
            self.pos_pub.publish(out)

        # Update last steering pos
        self.last_pos_rad = target_rad
        self.get_logger().info(f"Steering {direction_deg:.1f}бу -> {target_rad:.2f} rad")

        # 2) Speed: normalize raw_speed to [-MAX_RPM, MAX_RPM]
        norm = max(min(raw_speed / 8000.0, 1.0), -1.0)
        self.desired_rpm = int(norm * MAX_RPM)
        self.get_logger().info(f"Speed raw={raw_speed:.1f} -> RPM={self.desired_rpm}")

    def heartbeat(self):
        # Periodically send RPM commands
        for dev, ids in self.port_map.items():
            ser = self.serial_map.get(dev)
            if not ser:
                continue
            for can_id in ids:
                cmd = SetRPM(self.desired_rpm)
                cmd.can_id = can_id
                ser.write(pyvesc.encode(cmd))

    def destroy_node(self):
        # Stop motors on shutdown
        for dev, ids in self.port_map.items():
            ser = self.serial_map.get(dev)
            if not ser:
                continue
            for can_id in ids:
                cmd = SetRPM(0)
                cmd.can_id = can_id
                ser.write(pyvesc.encode(cmd))
            ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OmniWheelMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
