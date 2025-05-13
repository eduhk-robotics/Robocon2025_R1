#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import serial
import numpy as np
from serial.tools import list_ports
import pyvesc
from pyvesc import GetValues, SetRPM, SetCurrentBrake

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Configuration
BAUDRATE = 115200
TIMEOUT = 0.1
CTRL_INTERVAL = 0.05  # Control loop interval (s)
MAX_RPM = 100000      # Maximum RPM (per your request)
BRAKE_CURRENT = 2000  # Braking current (mA)
BRAKE_THRESHOLD = 50  # Consider as stopped below this RPM

class DirectRPMController(Node):
    def __init__(self):
        super().__init__('direct_rpm_controller')
        self.get_logger().info('Initializing Direct RPM Controller')
        
        # Motor control parameters
        self.target_rpm = 0
        self.brake_active = False

        # Subscribe to driving commands
        self.create_subscription(
            Float32MultiArray,
            'driving',
            self.driving_callback,
            10
        )

        # Initialize VESC connections
        self.vesc_ports = self.find_vescs()
        if not self.vesc_ports:
            self.get_logger().error('Failed to find required VESC motors')
            return

        # Open serial ports
        self.ser_connections = {}
        for port in self.vesc_ports:
            ser = serial.Serial(port, BAUDRATE, timeout=TIMEOUT)
            time.sleep(0.1)
            self.ser_connections[port] = ser

        # Start control loop
        self.create_timer(CTRL_INTERVAL, self.control_loop)

    def scan_vesc_ids(self, port):
        found_ids = []
        try:
            with serial.Serial(port, BAUDRATE, timeout=TIMEOUT) as ser:
                time.sleep(0.1)
                for can_id in [1, 2, 3, 4]:
                    setattr(GetValues, 'can_id', can_id)
                    ser.write(pyvesc.encode_request(GetValues))
                    time.sleep(0.1)
                    if ser.in_waiting:
                        data = ser.read(ser.in_waiting)
                        msg, _ = pyvesc.decode(data)
                        if msg:
                            found_ids.append(can_id)
        except Exception as e:
            self.get_logger().warn(f'[Scan] Error on {port}: {str(e)}')
        return found_ids

    def find_vescs(self):
        ports = [p.device for p in list_ports.comports() if 'USB' in p.device]
        valid_ports = {}
        
        for port in ports:
            ids = self.scan_vesc_ids(port)
            if ids:
                valid_ports[port] = ids
                self.get_logger().info(f'Found VESC(s) {ids} at {port}')
        
        found_ids = set(sum(valid_ports.values(), []))
        if found_ids != {1, 2, 3, 4}:
            missing = {1, 2, 3, 4} - found_ids
            self.get_logger().error(f'Missing motor CAN IDs: {missing}')
            return None
        return valid_ports

    def driving_callback(self, msg: Float32MultiArray):
        try:
            # Directly use plane_speed as RPM value
            raw_rpm = msg.data[1]
            
            # Apply RPM limits
            self.target_rpm = int(np.clip(raw_rpm, -MAX_RPM, MAX_RPM))
            
            # Determine brake state
            self.brake_active = abs(self.target_rpm) < BRAKE_THRESHOLD

        except IndexError:
            self.get_logger().error('Invalid message format')

    def control_loop(self):
        for port, ids in self.vesc_ports.items():
            ser = self.ser_connections[port]
            for can_id in ids:
                if self.brake_active:
                    cmd = SetCurrentBrake(BRAKE_CURRENT)
                else:
                    cmd = SetRPM(self.target_rpm)
                
                cmd.can_id = can_id
                ser.write(pyvesc.encode(cmd))

    def destroy_node(self):
        # Emergency stop all motors
        cmd = SetCurrentBrake(BRAKE_CURRENT)
        for port, ids in self.vesc_ports.items():
            ser = self.ser_connections[port]
            for can_id in ids:
                cmd.can_id = can_id
                ser.write(pyvesc.encode(cmd))
        
        # Close serial connections
        for ser in self.ser_connections.values():
            ser.close()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = DirectRPMController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()