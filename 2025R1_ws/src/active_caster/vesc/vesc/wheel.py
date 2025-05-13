#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import serial
from serial.tools import list_ports
import pyvesc
from pyvesc import GetValues, SetRPM, SetCurrentBrake

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
        """
        初始化 MotorControllerNode 节点。设置 PS4 手柄订阅，查找 VESC 设备，
        打开串口连接，并启动心跳定时器。
        """
        super().__init__('motor_controller')
        self.get_logger().info('MotorControllerNode starting')
        self.desired_rpm = 0

        # 1) 接收 PS4 手柄的控制信号
        self.create_subscription(
            Joy,
            'ps4',               # 确认手柄的主题名称
            self.joy_callback,
            10
        )

        # 2) 找到所有 VESC
        self.port_map = self.find_vescs()
        if not self.port_map:
            self.get_logger().error(f'No VESCs with CAN IDs {DESIRED_IDS} found')
            return

        # 3) 打开串口
        self.serial_map = {}
        for dev in self.port_map:
            ser = serial.Serial(dev, BAUDRATE, timeout=TIMEOUT)
            time.sleep(0.1)
            self.serial_map[dev] = ser

        # 4) 定时任务
        self.create_timer(HEARTBEAT_DT, self.heartbeat)

    def scan_ids_on_port(self, dev):
        """
        扫描指定串口上的 VESC 设备，查找目标 CAN ID。
        向每个 CAN ID 发送请求并检查响应。

        参数:
            dev (str): 要扫描的串口设备。

        返回:
            list: 在指定串口上找到的 CAN ID 列表。
        """
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
        """
        查找系统中所有连接的 VESC 设备，通过扫描可用的串口。

        返回:
            dict: 一个字典，映射串口到找到的 CAN ID 列表。
        """
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
        """
        PS4 手柄订阅的回调函数。
        根据手柄第 4 轴的值更新目标转速。

        参数:
            msg (Joy): 从 PS4 手柄接收到的消息。
        """
        # 打印消息，确认收到信号
        self.get_logger().info(f'[ps4] axes={msg.axes}')

        # 根据手柄 4 轴或 5 轴的值，设为目标转速
        axis_val = msg.axes[4]
        self.desired_rpm = int(axis_val * MAX_RPM)

    def heartbeat(self):
        """
        向所有连接的 VESC 设备发送周期性心跳消息。
        更新每个电机的转速为目标转速。
        """
        for dev, ids in self.port_map.items():
            ser = self.serial_map[dev]
            for can_id in ids:
                cmd = SetRPM(self.desired_rpm)
                cmd.can_id = can_id
                ser.write(pyvesc.encode(cmd))

    def destroy_node(self):
        """
        清理节点，停止所有电机并关闭串口连接。
        """
        # 停止电机
        for dev, ids in self.port_map.items():
            ser = self.serial_map[dev]
            for can_id in ids:
                cmd = SetCurrentBrake(10000)
                cmd.can_id = can_id
                ser.write(pyvesc.encode(cmd))
        # 关闭串口
        for ser in self.serial_map.values():
            ser.close()
        super().destroy_node()

def main(args=None):
    """
    主函数，初始化并运行 MotorControllerNode 节点。
    """
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

