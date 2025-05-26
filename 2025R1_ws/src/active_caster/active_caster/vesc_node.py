"""
VESC Node for Omni-Wheel Motor Control
Subscribes to 'vesc_control' (Float32): desired RPM for VESCs
Sends RPM commands via VESC CAN.
"""
import serial
from serial.tools import list_ports
import time

import pyvesc
from pyvesc import GetValues, SetRPM

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Motor speed constants
BAUDRATE = 115200
TIMEOUT = 0.1
HEARTBEAT_DT = 0.05    # seconds between RPM commands
MAX_RPM = 6000         # VESC max motor RPM
DESIRED_IDS = {1, 3, 4}  # CAN IDs of VESCs

class VescNode(Node):
    def __init__(self):
        super().__init__('vesc_node')
        self.get_logger().info('VescNode starting')

        # Desired RPM set by incoming command
        self.desired_rpm = 0

        # Subscribe to RPM commands
        self.create_subscription(
            Float32,
            'vesc_control',
            self.rpm_callback,
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

    def rpm_callback(self, msg: Float32):
        # Receive desired RPM and clamp to valid range
        raw_rpm = float(msg.data)
        norm = max(min(raw_rpm / MAX_RPM, 1.0), -1.0)
        self.desired_rpm = int(norm * MAX_RPM)
        self.get_logger().info(f"Received RPM command: {self.desired_rpm}")

    # 修改 heartbeat 函数，添加异常处理和重连机制
    def heartbeat(self):
        for dev, ids in self.port_map.items():
            ser = self.serial_map.get(dev)
            if not ser or not ser.is_open:
                try:
                    # 尝试重新打开串口
                    ser = serial.Serial(dev, BAUDRATE, timeout=TIMEOUT)
                    self.serial_map[dev] = ser
                    self.get_logger().warn(f"Reconnected to {dev}")
                except Exception as e:
                    self.get_logger().error(f"Failed to reconnect {dev}: {e}")
                    continue
        
        try:
            for can_id in ids:
                cmd = SetRPM(self.desired_rpm)
                cmd.can_id = can_id
                ser.write(pyvesc.encode(cmd))
                ser.flush()  # 强制刷新缓冲区
        except (OSError, serial.SerialException) as e:
            self.get_logger().error(f"Write failed on {dev}: {e}")
            ser.close()

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
    node = VescNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()