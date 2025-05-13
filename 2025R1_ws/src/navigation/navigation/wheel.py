#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import serial
from serial.tools import list_ports
import pyvesc
from pyvesc import GetValues, SetRPM

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

BAUDRATE = 115200
TIMEOUT = 0.1
HEARTBEAT_DT = 0.05   # heartbeat interval (s)
MAX_RPM = 20000       # max speed
DESIRED_IDS = {1, 3, 4}  # CAN IDs to control

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.get_logger().info('MotorControllerNode starting')
        self.desired_rpm = 0

        # 1) ????? ps4????????????
        self.create_subscription(
            Joy,
            'ps4',               # ?? ????????????????
            self.joy_callback,
            10
        )

        # 2) ????? VESC
        self.port_map = self.find_vescs()
        if not self.port_map:
            self.get_logger().error(f'No VESCs with CAN IDs {DESIRED_IDS} found')
            return

        # 3) ?????
        self.serial_map = {}
        for dev in self.port_map:
            ser = serial.Serial(dev, BAUDRATE, timeout=TIMEOUT)
            time.sleep(0.1)
            self.serial_map[dev] = ser

        # 4) ????????
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
        ports = [
            p.device for p in list_ports.comports()
            if 'ACM' in p.device or 'USB' in p.device
        ]
        res = {}
        for dev in ports:
            ids = self.scan_ids_on_port(dev)
            if ids:
                res[dev] = ids
                self.get_logger().info(f'Found VESCs {ids} on {dev}')
        return res

    def joy_callback(self, msg: Joy):
        # ????????????????????
        self.get_logger().info(f'[ps4] axes={msg.axes}')

        # ??????????4????5??????????????
        axis_val = msg.axes[4]
        self.desired_rpm = int(axis_val * MAX_RPM)

    def heartbeat(self):
        for dev, ids in self.port_map.items():
            ser = self.serial_map[dev]
            for can_id in ids:
                cmd = SetRPM(self.desired_rpm)
                cmd.can_id = can_id
                ser.write(pyvesc.encode(cmd))

    def destroy_node(self):
        # ?????
        for dev, ids in self.port_map.items():
            ser = self.serial_map[dev]
            for can_id in ids:
                cmd = SetRPM(0)
                cmd.can_id = can_id
                ser.write(pyvesc.encode(cmd))
        # ??????
        for ser in self.serial_map.values():
            ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
